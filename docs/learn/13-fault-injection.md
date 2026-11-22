# 13 · Fault injection: break the real autopilot on purpose

**Big idea:** a drone is only as safe as what it does when something goes wrong. Drone companies test that by *injecting faults* in simulation (GPS loss, a dead motor, a lost radio link) and checking the autopilot does the safe thing. That's what this lab does with the real ArduCopter 4.7.3.

## How a run works
1. Start a fresh ArduPilot (default settings), take off to 10 m, hover **20 m north of home** in GUIDED mode. (Not over home, so "landed where it was" and "flew home" look different.)
2. **Inject one fault** with SITL's own `SIM_*` switches, the same ones ArduPilot's developers use in their tests.
3. **Watch** and record: every flight-mode change, every autopilot message, where the EKF *believes* it is, and where the simulator says it *really* is.
4. **Judge** it against what ArduCopter is designed to do.

There are two kinds of scenarios:
- **Safety checks**: it must do the safe thing → **PASS / FAIL**.
- **Known limitations**: real weaknesses we reproduce on purpose → **CONFIRMED**. Knowing these is part of flying safely.

## Try it
- **In DroneLab:** Fly tab → 🛰 ArduPilot → **🧪 Fault injection**: pick a fault, press **▶ Run fault test** and watch the orange drone. A white wire ball appears when its **real** position differs from what it **believes** (try *GPS spoofing*). A report card shows expected vs observed and a timeline.
- **All of them, with a report:** `.venv/bin/python -m backend.sitl.faults --all` writes [docs/results/fault-injection.md](../results/fault-injection.md) with a chart per fault (≈7 min).

## What we found (ArduCopter 4.7.3, default settings)
| Fault | What ArduPilot did | Result |
|---|---|---|
| GPS lost | EKF failsafe after ~8 s → LAND where it was (5 m of drift on the way down) | PASS |
| GPS glitch, 100 m for 3 s | Rejected it: stayed within 0.1 m | PASS |
| Weak motor (80 %) | Other motors worked harder: within ~1 m | PASS |
| Radio link lost | RTL after ~1 s, landed exactly at home | PASS |
| Battery low | RTL after ~10 s below the limit (that's `BATT_LOW_TIMER`), landed at home | PASS |
| Strong gusty wind, 10 m/s | Leaned into it, within 0.4 m | PASS |
| Compass failure | Switched compass, within 0.1 m | PASS |
| Geofence | Refused a "fly to" outside the fence; pushed out anyway → RTL → home | PASS |
| GPS spoofing, steady 20 m offset | **Believed it**: really flew 20 m away while thinking it hadn't moved | limitation |
| Motor dead (quad) | Fell: hit the ground 1.7 s later at 10.7 m/s | limitation |
| Motor dead (hexa) | Stayed up ~20 s but lost heading control and swung 28 m around | limitation |

## The surprises (the best part of testing)
- **A small steady GPS error is believed.** The EKF rejects big or brief jumps (`EK3_GLITCH_RAD` = 25 m), but a steady offset looks exactly like real movement. ArduPilot even printed *"GPS Glitch or Compass error"*, then followed the false GPS anyway. That's why GPS spoofing is dangerous and why serious drones carry a second GPS or other position sensors.
- **The geofence has three layers.** GUIDED silently refuses a destination outside the fence; with default settings the drone also brakes at the edge (`AVOID_ENABLE`: we measured it stopping at 40.6 m for a 40 m fence); only if it really gets out does the breach failsafe (RTL) fire. To test that last layer we switched avoidance off.
- **"Weak motor" depends on the margin.** At 80 % thrust it holds; at 60 % it sinks, because hovering already needs ~59 % throttle. A heavier drone has less margin.
- **Six motors aren't automatically safe.** With default settings a hexacopter that loses a motor buys time (~20 s instead of 1.7 s) but can't hold its heading. Real hexa/octo builds tune for this.

## DroneLab ↔ ArduPilot
| DroneLab | ArduPilot |
|---|---|
| 📵 GPS outage + failsafe (level + altitude hold) | EKF failsafe (`FS_EKF_ACTION`): LAND |
| Pink ghost = belief, drone = truth | Orange drone = EKF belief, white ball = SITL truth |
| Wind sliders | `SIM_WIND_SPD`, `SIM_WIND_DIR`, `SIM_WIND_TURB` |
| — | Radio, battery and fence failsafes: `FS_THR_ENABLE`, `BATT_FS_LOW_ACT`, `FENCE_ACTION` |

## Self-check
1. Why do we hover 20 m from home before injecting the fault?
2. The GPS says "you're 20 m north" and keeps saying it. Why can't the EKF tell that it's wrong?
3. Why does a quad fall when one motor stops, while it copes with a motor at 80 %?

## Answers
1. So RTL (fly home) and LAND (land here) end in different places: we can tell which failsafe actually ran.
2. Its other sensors (IMU, barometer) only measure *changes* (acceleration, height), not absolute position. A steady offset doesn't conflict with them for long, so after a while the EKF accepts the GPS. Only an independent position source (a second GPS, optical flow, a camera) could disagree.
3. A quad needs all four motors to balance roll, pitch and yaw at once: opposite motors spin the same way, so losing one leaves an unbalanced spin it can't cancel. At 80 % the motor still works; the controller just asks it (and the others) for a different share.
