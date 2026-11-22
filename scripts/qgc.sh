#!/usr/bin/env bash
# QGroundControl (map ground station) connected to ArduCopter SITL, over 127.0.0.1 only.
# If DroneLab's 🛰 ArduPilot panel is running SITL, it joins that flight; otherwise it starts its own SITL.
#
#   ./scripts/qgc.sh              # real time
#   ./scripts/qgc.sh --speedup 2  # (own SITL only)
set -euo pipefail
cd "$(dirname "$0")/.."
SPEEDUP=1
[ "${1:-}" = "--speedup" ] && SPEEDUP=${2:-1}
exec .venv/bin/python -m backend.sitl.qgc --speedup "$SPEEDUP"
