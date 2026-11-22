"""Start ArduCopter SITL, then hand the terminal to MAVProxy (a command-line ground station). Used by scripts/sitl.sh."""

from __future__ import annotations

import argparse
import os
import signal
import subprocess
import sys

from backend.sitl import launch


def descendants(pid: int) -> list[int]:
    """All child processes of pid, and their children (read from /proc). MAVProxy forks helpers."""
    kids: dict[int, list[int]] = {}
    for entry in os.listdir("/proc"):
        if entry.isdigit():
            try:
                with open(f"/proc/{entry}/stat") as f:
                    ppid = int(f.read().rsplit(")", 1)[1].split()[1])
            except (OSError, ValueError, IndexError):
                continue
            kids.setdefault(ppid, []).append(int(entry))
    out, todo = [], [pid]
    while todo:
        for k in kids.get(todo.pop(), []):
            out.append(k)
            todo.append(k)
    return out


def stop_tree(proc: subprocess.Popen) -> None:
    """Stop MAVProxy and every helper process it started (only these: nothing else is touched)."""
    pids = descendants(proc.pid) + [proc.pid]
    for sig in (signal.SIGTERM, signal.SIGKILL):
        for p in pids:
            try:
                os.kill(p, sig)
            except ProcessLookupError:
                pass
        try:
            proc.wait(timeout=3)
        except subprocess.TimeoutExpired:
            pass
        if not any(os.path.exists(f"/proc/{p}") and _alive(p) for p in pids):
            return


def _alive(pid: int) -> bool:
    try:
        with open(f"/proc/{pid}/stat") as f:
            return f.read().rsplit(")", 1)[1].split()[0] != "Z"  # a zombie is already dead
    except OSError:
        return False


def main(argv: list[str] | None = None) -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("--speedup", type=float, default=1.0)
    args = ap.parse_args(argv)
    mavproxy = launch.ROOT / ".venv" / "bin" / "mavproxy.py"
    def stop_on_signal(signum, _frame):  # terminal closed or `kill`: still clean up both programs
        raise SystemExit(128 + signum)

    signal.signal(signal.SIGTERM, stop_on_signal)
    signal.signal(signal.SIGHUP, stop_on_signal)
    with launch.start(speedup=args.speedup, wipe=False) as sitl:
        print(f"ArduCopter SITL is running on {sitl.url} (log: {sitl.log_path}).")
        print("Starting MAVProxy. Wait about 30 s for 'EKF3 IMU0 is using GPS' before arming.")
        print("Try:  mode guided   arm throttle   takeoff 10   mode rtl      (Ctrl+C or Ctrl+D quits)\n")
        mav = subprocess.Popen([str(mavproxy), f"--master={sitl.url}", f"--state-basedir={launch.RUN_DIR}"])
        try:
            mav.wait()
        except KeyboardInterrupt:
            pass
        finally:
            stop_tree(mav)
    print("SITL stopped.")
    return 0


if __name__ == "__main__":
    sys.exit(main())
