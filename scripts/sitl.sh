#!/usr/bin/env bash
# Start ArduCopter SITL and connect MAVProxy (a text ground station) to it. Ctrl+C (or Ctrl+D) stops both. Everything talks over 127.0.0.1 only.
#
#   ./scripts/sitl.sh              # real time
#   ./scripts/sitl.sh --speedup 3  # 3x faster than real time
#
# In MAVProxy try:  mode guided  ·  arm throttle  ·  takeoff 10  ·  mode rtl   (see docs/learn/12-ardupilot-sitl.md)
set -euo pipefail
cd "$(dirname "$0")/.."
SPEEDUP=1
[ "${1:-}" = "--speedup" ] && SPEEDUP=${2:-1}
exec .venv/bin/python -m backend.sitl.ground_station --speedup "$SPEEDUP"
