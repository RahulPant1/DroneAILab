"""Start QGroundControl (a ground station with a map) connected to ArduCopter SITL. Used by scripts/qgc.sh.

- If DroneLab's 🛰 ArduPilot panel already runs SITL, QGroundControl joins it on SITL's second MAVLink link
  (TCP 127.0.0.1:5762), so you watch the same flight in both.
- Otherwise it starts its own SITL and connects on the first link (127.0.0.1:5760).

QGroundControl keeps all its files in external/qgc-home (its HOME for this program only), so nothing is written
to ~/.config or ~/Documents. Before each start we set:
  - auto-connect OFF for UDP (it would listen on 0.0.0.0:14550), ZeroConf/mDNS (0.0.0.0:5353) and every serial/USB
    radio or flight controller (no real-drone interfaces, PRD §7);
  - one TCP link to 127.0.0.1 that connects at start-up;
  - no automatic log uploads.
"""

from __future__ import annotations

import argparse
import os
import re
import signal
import subprocess
import sys
from pathlib import Path

from backend.sitl import launch
from backend.sitl.ground_station import stop_tree

QGC_DIR = launch.ROOT / "external" / "qgc"
QGC_BIN = QGC_DIR / "squashfs-root" / "QGroundControl"
QGC_HOME = launch.ROOT / "external" / "qgc-home"
INI = QGC_HOME / ".config" / "QGroundControl.org" / "QGroundControl.ini"
LINK_NAME = "DroneLab ArduPilot SITL"
TCP_LINK_TYPE = 2  # QGroundControl 4.x LinkConfiguration::TypeTcp (Serial=0, UDP=1, TCP=2)

AUTO_CONNECT_OFF = ["autoConnectUDP", "autoConnectZeroConf", "autoConnectPixhawk", "autoConnectSiKRadio",
                    "autoConnectLibrePilot", "autoConnectRTKGPS"]


def set_ini_values(text: str, section: str, values: dict[str, str]) -> str:
    """Set keys in one [section] of a Qt .ini file, keeping everything else (Qt's keys use backslashes, which
    Python's configparser would mangle, so this edits the lines directly)."""
    lines = text.splitlines()
    start = next((i for i, line in enumerate(lines) if line.strip() == f"[{section}]"), None)
    if start is None:
        if lines and lines[-1].strip():
            lines.append("")
        lines.append(f"[{section}]")
        lines += [f"{k}={v}" for k, v in values.items()]
        return "\n".join(lines) + "\n"
    end = next((i for i in range(start + 1, len(lines)) if lines[i].startswith("[")), len(lines))
    body = [line for line in lines[start + 1:end] if line.split("=", 1)[0] not in values]
    while body and not body[-1].strip():
        body.pop()
    body += [f"{k}={v}" for k, v in values.items()]
    return "\n".join(lines[: start + 1] + body + ([""] if end < len(lines) else []) + lines[end:]) + "\n"


def configure(port: int, ini: Path = INI) -> None:
    """Write our safety settings and the one TCP link (to 127.0.0.1:port) into QGroundControl's settings."""
    ini.parent.mkdir(parents=True, exist_ok=True)
    text = ini.read_text() if ini.exists() else ""
    # QGroundControl keeps its "AutoConnect" settings in the [LinkManager] section.
    text = re.sub(r"\[AutoConnectSettings\][^\[]*", "", text)  # a wrong guess an earlier version wrote
    text = set_ini_values(text, "LinkManager", {k: "false" for k in AUTO_CONNECT_OFF})
    # Replace any saved links with exactly one: ours.
    text = re.sub(r"\[LinkConfigurations\][^\[]*", "", text)
    text = set_ini_values(text, "LinkConfigurations", {
        "count": "1",
        "Link0\\name": LINK_NAME,
        "Link0\\type": str(TCP_LINK_TYPE),
        "Link0\\auto": "true",  # connect at start-up
        "Link0\\high_latency": "false",
        "Link0\\host": "127.0.0.1",
        "Link0\\port": str(port),
    })
    text = set_ini_values(text, "MAVLinkLogGroup", {"EnableAutoUpload": "false", "EnableAutoStart": "false"})
    ini.write_text(text)


def qgc_env() -> dict[str, str]:
    env = dict(os.environ)
    home = str(QGC_HOME)
    env.update(HOME=home, XDG_CONFIG_HOME=f"{home}/.config", XDG_DATA_HOME=f"{home}/.local/share",
               XDG_CACHE_HOME=f"{home}/.cache")  # XDG_RUNTIME_DIR stays: the WSLg display lives there
    return env


def main(argv: list[str] | None = None) -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("--speedup", type=float, default=1.0, help="only when we start our own SITL")
    args = ap.parse_args(argv)
    if not QGC_BIN.is_file():
        print("QGroundControl isn't set up yet: run ./scripts/qgc_setup.sh")
        return 1

    def stop_on_signal(signum, _frame):
        raise SystemExit(128 + signum)

    signal.signal(signal.SIGTERM, stop_on_signal)
    signal.signal(signal.SIGHUP, stop_on_signal)

    sitl = None
    second_link = launch.BASE_PORT + 2
    try:
        if launch.port_busy(second_link):
            port = second_link
            print(f"ArduPilot is already running (DroneLab's 🛰 panel?): QGroundControl joins it on 127.0.0.1:{port}.")
        else:
            print("Starting ArduCopter SITL for QGroundControl...")
            sitl = launch.start(speedup=args.speedup, wipe=False)
            port = sitl.port
            print(f"ArduCopter SITL is running on 127.0.0.1:{port} (log: {sitl.log_path}).")
        configure(port)
        print("Starting QGroundControl (its window opens on Windows). GPS + EKF need ~30–45 s before arming.")
        print("Close its window or press Ctrl+C here to stop.\n")
        qgc = subprocess.Popen([str(QGC_BIN)], env=qgc_env(), cwd=QGC_HOME, stdout=subprocess.DEVNULL,
                               stderr=subprocess.DEVNULL)
        try:
            qgc.wait()
        except KeyboardInterrupt:
            pass
        finally:
            stop_tree(qgc)
    finally:
        if sitl:
            sitl.stop()
            print("SITL stopped.")
    return 0


if __name__ == "__main__":
    sys.exit(main())
