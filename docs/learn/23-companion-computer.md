# 23 · The companion computer: perception next to the autopilot

**Big idea:** a real drone usually has **two computers**. The **autopilot** (a Pixhawk running ArduPilot) flies: fast control loops, the state estimate, failsafes. The **companion computer** (a Jetson or Raspberry Pi on a serial cable) runs the heavy perception (cameras, lidar, AI) and tells the autopilot only what it found, in standard MAVLink messages. If the companion crashes, the autopilot keeps flying.

DroneLab's own perception (the marker detector from lesson 18, the lidar from lesson 15) now runs that way, as a **separate process** talking MAVLink to real ArduCopter 4.7 in SITL.

```
 companion process (backend/companion/service.py)            ArduCopter SITL
 ┌──────────────────────────────────────────────┐   MAVLink   ┌─────────────────────────────────┐
 │ down camera ─> find marker ─> LANDING_TARGET  │ ──────────> │ precision landing (PLND_TYPE=1) │
 │ lidar 72 beams ─────────────> OBSTACLE_DISTANCE│ ──────────> │ proximity (PRX1_TYPE=2)         │
 │ <─ ATTITUDE, height (what the autopilot knows) │ <────────── │   -> BendyRuler planner (OA_TYPE=1)
 └──────────────────────────────────────────────┘  serial 2   └─────────────────────────────────┘
```

## The two messages
- **LANDING_TARGET**: "the pad is in this direction, this far away", in the drone's body frame (forward, right, down). The companion turns the marker's pixel into a line of sight and uses the autopilot's own height and tilt for the distance. ArduPilot does the rest: it slides over the pad while it descends.
- **OBSTACLE_DISTANCE**: 72 distances, one per 5° slice, clockwise from the nose. "Nothing there" is sent as *max range + 1*, never as a reading. ArduPilot's planner then steers round whatever is in the way.

## What we measured (ArduCopter 4.7 SITL, ground truth from the simulator)
| Job | Without the companion | With the companion |
|---|---|---|
| Land on a pad, told a spot 1.8 m off (a drifting GPS) | landed 1.82 m from the pad | **0.06 m** from the pad (310 LANDING_TARGET messages) |
| AUTO mission straight through a 10 m wall it doesn't know about | flew **through** it (a real drone would crash) | **went round**, never closer than 2.9 m (70 m path, 35 s: it hesitated left and right for ~20 s before picking the west end) |

Full table: `docs/results/companion.md` (`python -m backend.sitl.companion_flight --write`).

## Bugs we hit (each one is a real-world lesson)
1. **"No obstacle" must not look like an obstacle.** We added sensor noise to every beam, so "nothing within 8 m" became 7.99 m in every direction, and ArduPilot saw a ring of walls and danced on the spot. Noise now only goes on real hits.
2. **The companion shares the drone's system id.** It's component 191 of system 1, and ArduPilot forwards its heartbeats to the ground station. Our ground-station code took them for the autopilot's ("not armed!") until it listened to component 1 only.
3. **Home moves.** ArduPilot resets HOME wherever it arms, so "metres from home" shifts after a landing. Flights that re-arm keep the first home.
4. **ArduPilot's own limits again:** it ignores a landing target more than `PLND_XY_DIST_MAX` (2.5 m) away, and only uses precision landing below `PLND_ALT_MAX` (8 m). With the lidar configured (`PRX1_TYPE = 2`), it **refuses to arm** until the companion sends data (`PreArm: PRX1: No Data`). That's a good safety check.

## What's simulated, and what would change on a real drone
Only two small classes read the simulator's truth: `SimCamera` (renders the picture from the true pose) and `SimLidar` (ray-casts the virtual wall). On a real drone they become a camera driver (V4L2 / GStreamer) and a lidar driver (e.g. RPLidar over serial) with the same `frame()` / `scan()` methods. Everything else (detection, the MAVLink messages, the parameters) is what would run on a Jetson or Raspberry Pi. Real cameras bring lens distortion, blur, sun glare and latency; real lidars see through glass and miss thin wires. Those come with the hardware step (see `docs/discuss-later.md`).

## Try it
`.venv/bin/python -m backend.sitl.companion_flight` (both jobs, about 5 minutes), or `--job precland` / `--job avoid`. Run the companion by hand next to a SITL you started with `launch.start(companion_port=True)`: `.venv/bin/python -m backend.companion.service --connect tcp:127.0.0.1:5763 --world precland`.

## Self-check
1. Why not run the perception on the autopilot itself?
2. Why does the companion send the target in the *body* frame instead of as a GPS position?
3. Why is "nothing within range" sent as max range + 1?

## Answers
1. The autopilot's microcontroller is small and must never miss a control-loop deadline; image processing is heavy and can stall. Split up, a perception crash doesn't stop the drone flying.
2. The camera measures direction relative to the drone, and that's all precision landing needs. Turning it into GPS coordinates would add the very GPS error it's meant to remove.
3. A distance means "there IS something here". A value just under max range would tell the autopilot there are obstacles in every direction.
