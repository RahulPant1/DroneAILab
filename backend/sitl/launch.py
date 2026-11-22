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

    @property
    def port(self) -> int:
        return BASE_PORT + 10 * self.instance

    @property
    def url(self) -> str:
        return f"tcp:127.0.0.1:{self.port}"

    def stop(self) -> None:
        if self.proc.poll() is None:
            os.killpg(self.proc.pid, signal.SIGTERM)  # its own process group: only what we started
            try:
                self.proc.wait(timeout=5)
            except subprocess.TimeoutExpired:
                os.killpg(self.proc.pid, signal.SIGKILL)
                self.proc.wait()
        if self._log:
            self._log.close()

    def __enter__(self) -> "Sitl":
        return self

    def __exit__(self, *exc) -> None:
        self.stop()


def start(speedup: float = 1.0, wipe: bool = True, instance: int | None = None, timeout_s: float = 30.0) -> Sitl:
    """Launch ArduCopter SITL (a "+" quad on its built-in physics) and wait until it accepts a ground station.

    speedup: run the simulation faster than real time (the autopilot doesn't notice).
    wipe: start from the default parameters instead of whatever the last run saved.
    """
    if not is_built():
        raise FileNotFoundError(f"ArduCopter SITL is not built yet ({BINARY}). Run scripts/sitl_setup.sh first.")
    n = free_instance() if instance is None else instance
    RUN_DIR.mkdir(parents=True, exist_ok=True)
    log_path = RUN_DIR / f"arducopter-{n}.log"
    log = open(log_path, "w")
    cmd = [
        str(BINARY),
        "--model", "quad",
        "--speedup", str(speedup),
        "--home", ",".join(str(v) for v in HOME),
        "--defaults", str(DEFAULTS),
        "-I", str(n),
        "--serial0", "tcp:0:wait",  # SERIAL0 = the ground-station link (port 5760 + 10N)
    ]
    if wipe:
        cmd.append("--wipe")
    proc = subprocess.Popen(cmd, cwd=RUN_DIR, stdout=log, stderr=subprocess.STDOUT, start_new_session=True)
    sitl = Sitl(proc, n, log_path, log)
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
