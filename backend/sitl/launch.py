"""Start and stop the ArduCopter SITL program (built by scripts/sitl_setup.sh)."""

from __future__ import annotations

import os
import signal
import socket
import subprocess
import time
from dataclasses import dataclass, field
from pathlib import Path

ROOT = Path(__file__).resolve().parents[2]
ARDUPILOT_DIR = ROOT / "external" / "ardupilot"
BINARY = ARDUPILOT_DIR / "build" / "sitl" / "bin" / "arducopter"
DEFAULTS = ARDUPILOT_DIR / "Tools" / "autotest" / "default_params" / "copter.parm"
RUN_DIR = ROOT / "external" / "sitl-run"  # SITL writes its parameter storage (eeprom.bin) and flight logs here

# ArduPilot's standard SITL test field near Canberra: latitude, longitude, ground height above sea level (m),
# heading (degrees). Any open spot works; this one matches ArduPilot's own docs and tests.
HOME = (-35.363261, 149.165230, 584.0, 353.0)
BASE_PORT = 5760  # instance N listens for a ground station on TCP 5760 + 10*N
KEEP_LOGS = 40  # SITL writes a ~4 MB flight log per flight; keep only the newest ones


def prune_logs(keep: int = KEEP_LOGS) -> int:
    """Delete all but the newest `keep` SITL flight logs (logs/NNNNNNNN.BIN, which SITL creates in RUN_DIR)."""
    logs = sorted((RUN_DIR / "logs").glob("[0-9]" * 8 + ".BIN"), key=lambda p: p.stat().st_mtime)
    old = logs[:-keep] if keep > 0 else logs
    for p in old:
        p.unlink()
    return len(old)


def is_built() -> bool:
    return BINARY.is_file()


def port_busy(port: int) -> bool:
    with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as s:
        s.settimeout(0.2)
        return s.connect_ex(("127.0.0.1", port)) == 0


def free_instance(max_instances: int = 10) -> int:
    """The first SITL instance number whose ports are all free (5760+10N .. 5763+10N, 5501+10N)."""
    for n in range(max_instances):
        if not any(port_busy(BASE_PORT + 10 * n + k) for k in range(4)):
            return n
    raise RuntimeError("No free SITL instance (ports 5760-5853 are all busy).")


@dataclass
class Sitl:
    """A running ArduCopter SITL. Use as a context manager so it always stops."""

    proc: subprocess.Popen
    instance: int
    log_path: Path
    _log: object = field(repr=False, default=None)
    own_group: bool = True

    @property
    def port(self) -> int:
        return BASE_PORT + 10 * self.instance

    @property
    def url(self) -> str:
        return f"tcp:127.0.0.1:{self.port}"

    @property
    def companion_url(self) -> str:
        """SERIAL2 (only when started with companion_port=True): where a companion-computer process connects."""
        return f"tcp:127.0.0.1:{self.port + 3}"

    def stop(self) -> None:
        if self.proc.poll() is None:
            if self.own_group:
                os.killpg(self.proc.pid, signal.SIGTERM)  # its own process group: only what we started
            else:
                self.proc.terminate()
            try:
                self.proc.wait(timeout=5)
            except subprocess.TimeoutExpired:
                if self.own_group:
                    os.killpg(self.proc.pid, signal.SIGKILL)
                else:
                    self.proc.kill()
                self.proc.wait()
        if self._log:
            self._log.close()

    def __enter__(self) -> "Sitl":
        return self

    def __exit__(self, *exc) -> None:
        self.stop()


FRAMES = {  # SITL model -> extra default parameters on top of copter.parm (as ArduPilot's sim_vehicle.py does)
    "quad": [],
    "hexa": ["copter-hexa.parm"],
}


def start(speedup: float = 1.0, wipe: bool = True, instance: int | None = None, timeout_s: float = 30.0,
          own_group: bool = True, frame: str = "quad", extra_params: dict[str, float] | None = None,
          companion_port: bool = False) -> Sitl:
    """Launch ArduCopter SITL (a "+" quad on its built-in physics) and wait until it accepts a ground station.

    speedup: run the simulation faster than real time (the autopilot doesn't notice).
    wipe: start from the default parameters instead of whatever the last run saved.
    frame: "quad" (4 motors) or "hexa" (6 motors: can survive one motor failing).
    extra_params: parameters set at boot (for ones that need a reboot, e.g. PRX1_TYPE, PLND_TYPE, OA_TYPE).
    companion_port: also open SERIAL2 as MAVLink on TCP port 5763 + 10N, for a companion-computer process.
    own_group: True = its own process group (Ctrl+C in a terminal doesn't hit it; we stop it ourselves).
      False = stay in the caller's group, so whatever stops the DroneLab server (./run.sh) stops SITL too.
    """
    if frame not in FRAMES:
        raise ValueError(f"frame must be one of {sorted(FRAMES)}")
    if not is_built():
        raise FileNotFoundError(f"ArduCopter SITL is not built yet ({BINARY}). Run scripts/sitl_setup.sh first.")
    n = free_instance() if instance is None else instance
    RUN_DIR.mkdir(parents=True, exist_ok=True)
    prune_logs()
    log_path = RUN_DIR / f"arducopter-{n}.log"
    log = open(log_path, "w")
    defaults = [DEFAULTS, *(DEFAULTS.parent / f for f in FRAMES[frame])]
    if extra_params:
        extra = RUN_DIR / f"extra-{n}.parm"
        extra.write_text("".join(f"{k} {v}\n" for k, v in extra_params.items()))
        defaults.append(extra)
    cmd = [
        str(BINARY),
        "--model", frame,
        "--speedup", str(speedup),
        "--home", ",".join(str(v) for v in HOME),
        "--defaults", ",".join(str(p) for p in defaults),
        "-I", str(n),
        "--serial0", "tcp:0:wait",  # SERIAL0 = the ground-station link (port 5760 + 10N)
    ]
    if companion_port:
        cmd += ["--serial2", "tcp:3"]  # SERIAL2 = the companion computer's link (port 5763 + 10N, MAVLink2)
    if wipe:
        cmd.append("--wipe")
    proc = subprocess.Popen(cmd, cwd=RUN_DIR, stdout=log, stderr=subprocess.STDOUT, start_new_session=own_group)
    sitl = Sitl(proc, n, log_path, log, own_group)
    deadline = time.monotonic() + timeout_s
    while time.monotonic() < deadline:
        if proc.poll() is not None:
            sitl.stop()
            raise RuntimeError(f"SITL exited early (code {proc.returncode}). See {log_path}")
        if "Waiting for connection" in log_path.read_text(errors="replace"):
            return sitl  # listening (we don't poke the port: SITL treats any connection as its ground station)
        time.sleep(0.2)
    sitl.stop()
    raise TimeoutError(f"SITL didn't open port {sitl.port} within {timeout_s} s. See {log_path}")
