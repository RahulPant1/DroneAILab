#!/usr/bin/env bash
# Phase 5 step 3: download QGroundControl 4.4.5 into ~/DroneAILab/external/qgc and unpack it (no install).
# Why 4.4.5 and not 5.x: 5.x needs glibc 2.38 (Ubuntu 24.04); this WSL is Ubuntu 22.04 (glibc 2.35).
# Safe to re-run. ~550 MB. Remove with: rm -rf external/qgc external/qgc-home
set -euo pipefail
cd "$(dirname "$0")/.."
DIR=external/qgc
URL=https://github.com/mavlink/qgroundcontrol/releases/download/v4.4.5/QGroundControl.AppImage
mkdir -p "$DIR"
if [ ! -x "$DIR/squashfs-root/QGroundControl" ]; then
  if [ ! -f "$DIR/QGroundControl-4.4.5.AppImage" ]; then
    echo "Downloading QGroundControl 4.4.5 (~170 MB)..."
    curl -fL --progress-bar -o "$DIR/QGroundControl-4.4.5.AppImage" "$URL"
  fi
  echo "Unpacking (no FUSE needed)..."
  chmod +x "$DIR/QGroundControl-4.4.5.AppImage"
  (cd "$DIR" && ./QGroundControl-4.4.5.AppImage --appimage-extract >/dev/null)
fi
missing=$(cd "$DIR/squashfs-root" && LD_LIBRARY_PATH=$PWD/Qt/libs ldd ./QGroundControl | awk '/not found/{print $1}')
if [ -n "$missing" ]; then
  echo "QGroundControl needs these system libraries:"; echo "$missing"
  echo "Install them yourself (DroneLab never runs sudo):"
  echo "  sudo apt install libgstreamer-gl1.0-0 libgstreamer-plugins-base1.0-0 libpulse-mainloop-glib0"
  exit 1
fi
echo "QGroundControl is ready. Start it with: ./scripts/qgc.sh"
