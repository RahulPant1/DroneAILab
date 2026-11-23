# Features to discuss later (not built, not approved)

Suggestions received on 2026-11-23 for moving from simulation toward building and selling drones. Only the
**simulation half of the companion computer** has been built (lesson 23, `backend/companion/`). Everything below
needs a decision first. Facts marked ✅ were checked in the ArduCopter 4.7.3 source in `external/ardupilot`.

## 1. Real-telemetry bridge and bench-test QA (needs: a decision to leave "simulation only", a Pixhawk)
- **What exists:** `Flight` opens its link with pymavlink, which already accepts `/dev/ttyACM0` (USB) or `udp:…`.
  The log doctor already reads `.BIN` logs uploaded from a real drone.
- **Work:** gate the SITL-only features on real hardware (ground truth `SIM_STATE`/`SIMSTATE`, fault injection via
  `SIM_*` parameters, offboard comparisons). Start **read-only** (live view + log doctor), commands only later.
- **WSL2 can't see USB** without `usbipd-win` installed on the Windows side (a Windows change: the user's call), or a
  telemetry radio through Mission Planner / MAVProxy on Windows forwarding UDP into WSL.
- **Bench QA runner:** sensor health and calibration status, failsafe parameters, props-off motor test
  (`MAV_CMD_DO_MOTOR_TEST`: spin order and direction), an RC-loss test, plus a signed JSON/PDF report per serial
  number. Name it a **factory test report**, not an "airworthiness certificate": airworthiness is the regulator's.
- **Safety rules to agree first:** props off on the bench, never auto-arm, a physical kill switch, and the
  companion/GCS never sends motor commands without a person confirming.

## 2. Firmware hardening for certification in India (needs: the actual QCI / DGCA requirements)
- ✅ Read-only built-in parameters: a defaults file can mark parameters `@READONLY` (`libraries/AP_Param/AP_Param.cpp`);
  a ground station then can't change them.
- ✅ Feature stripping: `Tools/scripts/build_options.py`, `extract_features.py`, and the board's `hwdef` files.
- ✅ Signed firmware + secure bootloader: `Tools/scripts/signing/` (`generate_keys.py`, `make_secure_bl.py`,
  `make_secure_fw.py`) and `libraries/AP_CheckFirmware`. The bootloader refuses firmware not signed by your key.
- ❌ Nothing in ArduPilot mentions NPNT or DGCA: that's integration work against the official spec.
- **Catches:** read-only parameters only lock settings if secure boot also stops reflashing. Building for a real
  board needs the ARM cross-compiler (a tarball in `external/`, no sudo). The **private signing key must never be in
  the public repo**. **GPL-3.0:** selling drones with modified ArduPilot means offering that source to buyers.
- **Certification:** DGCA issues type certificates on the recommendation of a QCI-approved certification body (e.g.
  NTH Ghaziabad, TQ Cert, SGS, ARAI). Hardening helps, but it doesn't certify by itself. Read the current
  certification scheme and NPNT documents before any design.

## 3b. Companion computer, hardware half (needs: a Jetson / Pi, camera, lidar; the decision in item 1)
- Replace `SimCamera` with a V4L2 / GStreamer driver (OpenCV `VideoCapture`) and `SimLidar` with an RPLidar / ToF
  driver: same `frame()` / `scan()` methods, everything else unchanged.
- A printed ArUco / AprilTag marker instead of our dark ring (OpenCV `aruco`): more robust and gives yaw.
  TensorRT / YOLO only if a learned detector is really needed.
- Packaging: a plain Python service (systemd) first. Docker / ROS 2 are heavy for this laptop (9 GB RAM, ~32 GB free
  on C:) and ROS 2 is on the PRD's Roadmap list (needs explicit approval).
- New real-world problems to plan for: lens calibration, motion blur, latency (`PLND_LAG`), glass and thin wires
  invisible to lidar, vibration.

## 4. DroneLab as a training / B2B product (business decision first)
- The repo is public and Apache-2.0: anyone can already use it. Anything sold would be support, packaging or private
  add-ons.
- **Remote-pilot training schools (RPTOs):** DGCA prescribes the remote-pilot syllabus. The 23 lessons here are
  engineering-focused and haven't been mapped to it; that mapping comes first.
- **White-label operator dashboards** (agriculture spraying, solar / tower inspection): technically easy forks of
  the frontend; spray-flow logging and tank-empty RTL only make sense with real sprayer hardware.

## Suggested order, when ready
1. Item 1, read-only link (live view + log doctor on a real Pixhawk), then the props-off QA runner.
2. Item 3b on the bench, reusing the companion service as-is.
3. Item 2 after reading the official requirements; build scripts in `external/`, keys outside the repo.
4. Item 4 once there's a product to sell.
