#!/usr/bin/env bash
# Phase 5: download and build ArduCopter SITL inside ~/DroneAILab/external/ (no sudo, no apt).
# Safe to re-run: it skips what's already done. Takes ~2-3 GB of disk; delete external/ to remove it all.
#
#   ./scripts/sitl_setup.sh
set -euo pipefail
cd "$(dirname "$0")/.."
ROOT=$PWD
TAG=Copter-4.7.3   # a stable ArduCopter release (pinned 2026-11-22)
AP=$ROOT/external/ardupilot

command -v g++ >/dev/null || { echo "g++ not found. It's needed to compile ArduPilot (ask before installing it)."; exit 1; }
[ -x .venv/bin/python ] || { echo "Project .venv missing: run the Python setup from the README first."; exit 1; }

echo "1/3 Python packages (into the project .venv only)..."
uv pip install --python .venv/bin/python -q -r backend/requirements-sitl.txt
uv pip install --python .venv/bin/python -q --no-deps mavproxy==1.8.75

if [ ! -d "$AP/.git" ]; then
  echo "2/3 Downloading ArduPilot $TAG (source only, no history)..."
  mkdir -p external
  git -c advice.detachedHead=false clone -q --depth 1 --branch "$TAG" https://github.com/ArduPilot/ardupilot.git "$AP"
else
  echo "2/3 ArduPilot already downloaded ($AP)."
fi
# Only the submodules the SITL build needs (skips ChibiOS etc., which are for real flight-controller boards).
git -C "$AP" -c advice.detachedHead=false submodule update --init --depth 1 --recommend-shallow \
  modules/waf modules/mavlink modules/lwip modules/littlefs \
  modules/DroneCAN/DSDL modules/DroneCAN/dronecan_dsdlc modules/DroneCAN/libcanard modules/DroneCAN/pydronecan

# DroneLab patch: SITL's TCP ports listen on 127.0.0.1 only (upstream listens on all interfaces).
PATCH=$ROOT/scripts/patches/ardupilot-sitl-loopback.patch
if git -C "$AP" apply --check "$PATCH" 2>/dev/null; then git -C "$AP" apply "$PATCH"; fi
git -C "$AP" apply --reverse --check "$PATCH" 2>/dev/null || { echo "Loopback patch didn't apply: stopping."; exit 1; }

echo "3/3 Building ArduCopter for SITL (first time ~10-20 min)..."
export PATH=$ROOT/.venv/bin:$PATH  # ArduPilot's build finds empy/pymavlink in our venv
cd "$AP"
[ -f build/sitl/ap_config.h ] || ./waf configure --board sitl >/dev/null
./waf copter -j3 >"$ROOT/external/build.log" 2>&1 || { echo "Build failed: see external/build.log"; exit 1; }
echo "Done: $AP/build/sitl/bin/arducopter"
echo "Try it:  ./scripts/sitl.sh          (fly it yourself with MAVProxy)"
echo "         .venv/bin/python -m backend.sitl.fly   (scripted demo flight)"
