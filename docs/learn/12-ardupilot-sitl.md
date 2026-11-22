# 12 · Real autopilot software: ArduPilot SITL

**Big idea:** ArduPilot is the open-source autopilot that flies many real drones. **SITL** ("software in the loop") compiles that exact flight code for your laptop and connects it to a simulated drone. Nothing about the code changes when it later runs on a real flight controller: only the sensors and motors on the other end.

## What's running
```
 you ──► MAVProxy (ground station) ──MAVLink over 127.0.0.1──► ArduCopter (real autopilot code)
                                                                   │  motor commands ▲ fake IMU, GPS, baro
                                                                   ▼                 │
                                                             SITL's built-in quad physics
```
- **ArduCopter**: the copter firmware (here version 4.7.3), built from source into `external/ardupilot`.
- **MAVLink**: the message protocol between autopilots and ground stations (on a real drone it goes over a telemetry radio).
- **MAVProxy**: a text ground station. Mission Planner and QGroundControl are the graphical ones.

## Fly it yourself
```bash
./scripts/sitl.sh
```
Wait about 30–45 s for `EKF3 IMU0 is using GPS` (like DroneLab's GPS settle, but for real), then type:

| Type this | What it does |
|---|---|
| `mode guided` | GUIDED: the autopilot takes "go here" commands from the ground station |
| `arm throttle` | Arm the motors. Refused until the **pre-arm checks** pass (you'll see `PreArm: ...` reasons) |
| `takeoff 10` | Climb to 10 m and hold |
| `guided -35.36310 149.16523 10` | Fly to a latitude/longitude at 10 m (about 20 m north of home) |
| `mode loiter` | Hold position (the same idea as DroneLab's Hover) |
| `mode rtl` | Return To Launch: climb to 15 m (less if it's already near home), fly home, land, disarm |
| **Ctrl+C** | Quit (stops the simulator too) |

Harmless messages you may see: `Failed to load module: No module named 'PIL'` (a picture module we didn't install), and `time moved backwards` if you use `--speedup`.

## The scripted flight
```bash
.venv/bin/python -m backend.sitl.fly
```
This is what a ground station app does under the buttons: it waits for the pre-arm checks, arms, takes off, flies to a point, **uploads a 4-waypoint square mission** and flies it in AUTO, then RTL. It prints a report with arrival errors. The code is short and commented: [backend/sitl/fly.py](../../backend/sitl/fly.py).

What we measured (autopilot time; `--speedup 5` makes it 5x quicker on the clock):

| Step | Result |
|---|---|
| Pre-arm checks pass (GPS + EKF3 ready) | ~46 s after start. Before that: `PreArm: Need Position Estimate` |
| Take-off to 10 m | ~7 s |
| "Fly to" 20 m north | arrived **0.03 m** from the target |
| Square mission corners | 0.03, 1.81, 1.80, 0.23 m off: it "reaches" a corner within `WP_RADIUS_M` (2 m) and turns early instead of stopping, like DroneLab's carrot cutting corners |
| RTL | landed **0.02 m** from home, about 31 s |

## In DroneLab: the 🛰 ArduPilot panel
In the **Fly** tab, open **🛰 ArduPilot** (under the other bars) and press **▶ Start ArduPilot**. After about 30–45 s (GPS + EKF) it says *ready*, and an **orange drone** sits on its own pad 4 m south of yours. Your drone and the keyboard work as usual next to it.

- **🛫 Take off · Fly to · ⏸ Hover · ⬜ Square mission · 🏠 Return home · 🛬 Land**: the same commands a ground station sends. "Fly to" is measured from the orange pad.
- The line under the buttons shows ArduCopter's **flight mode** (GUIDED, AUTO, RTL...), arming, GPS and EKF, plus its own messages.
- **📈 Compare with DroneLab**: both autopilots do the same job, "fly 10 m north" from a hover at 5 m, and you get one chart and a table:

| "Fly 10 m north" | rise (10→90%) | overshoot | settles (within 0.2 m) | top speed |
|---|---|---|---|---|
| 🛰 ArduPilot (GUIDED) | 2.2 s | 0.19 m | 4.0 s | 4.5 m/s |
| DroneLab (basic preset) | 2.8 s | 0.10 m | 4.2 s | 3.0 m/s |

Same kind of controller (position → velocity → tilt), different limits: DroneLab caps horizontal speed at 3 m/s, so it rises more slowly but overshoots less; ArduPilot speeds up to 4.5 m/s and brakes with its own acceleration and jerk limits. In the end both settle at about the same time.

Only one ArduPilot runs at a time and every browser tab sees the same one. Stopping the app stops it too.

## DroneLab → ArduPilot
| DroneLab | ArduPilot |
|---|---|
| Take off / Hover / Land | `takeoff` in GUIDED · LOITER · LAND |
| "Fly to" box | GUIDED position target (`SET_POSITION_TARGET_LOCAL_NED`) |
| Mission autopilot | AUTO mode + a mission uploaded item by item |
| GPS settle before take-off | Pre-arm checks: "GPS: waiting for 3D fix", "EKF not ready" |
| Estimator | **EKF3**: a much bigger version of the same idea (blend IMU, GPS, baro) |
| Cascaded PID | Same layers. Parameters like `ATC_RAT_RLL_P` (roll-rate P) and `PSC_NE_POS_P` (horizontal position P). In MAVProxy: `param show ATC_RAT_RLL_*` |
| ENU: x east, y north, z **up** | NED: x north, y east, z **down**; headings 0° = north, clockwise |
| GPS-loss failsafe | `FS_EKF_ACTION`, plus battery, radio and geofence failsafes |

## Self-check
1. Why does `arm throttle` get refused for the first half-minute?
2. You want the drone 5 m east, 20 m north and 10 m up. What do you send in ArduPilot's NED frame?
3. Why does the same ArduCopter code work on a real drone later, when here it flies a simulated one?
4. In the comparison, ArduPilot gets there sooner but overshoots more. Which limit explains both?

## Answers
1. The pre-arm checks wait for a 3D GPS fix and for EKF3 to settle on a position. Arming before it knows where it is would be unsafe (DroneLab's GPS settle is the same idea).
2. North 20, east 5, down **−10** (up is negative "down").
3. The flight code only sees sensor readings and sends motor commands. In SITL those come from, and go to, a simulator; on a real drone they come from real chips and go to real motors.
4. The speed limit. ArduPilot is allowed to go faster (4.5 m/s against 3 m/s), so it covers the distance sooner, but it carries more speed into the stop, so it slides a little further past the target.
