# 30 · Before and after the first real flight: the pre-flight check and the hardware checks

**Big idea:** most first-build crashes are caused by something a computer could have spotted. Before the flight, it's usually a missing setting: no failsafe, a calibration never done, an old parameter name. After the flight, it's what the log already shows: a compass that sees the motors, a drone that wobbles, a tired battery. DroneLab now checks both, and ends with a short list: **fix these first**.

## Before: ✈ the pre-flight check
- **🩺 Logs → ✈ Pre-flight check (.params):** load the parameter file saved from QGroundControl (*Parameters → Tools → Save to file*) or Mission Planner (*Full Parameter List → Save*). The file is only read, never stored.
- **🛠 Builder → ✈ first-flight checklist:** the same check on DroneLab's own starting file (lesson 28). It shows what's already set and, marked 📝, what can only be done on the real board.
- **Every log** carries the full parameter list, so the log report checks it too.

| It checks | Why |
|---|---|
| Frame type set | the wrong frame flips the drone at take-off |
| Accelerometer, compass, radio calibrated | offsets still 0 / radio still 1100-1900 = never done |
| Compass offsets not huge (> 600) | a magnet or power wire right next to it |
| Radio-loss, low-battery and bad-position failsafes, geofence | the safety nets |
| Return-home height ≥ 10 m | it comes home in a straight line at that height (trees!) |
| RTL on a switch | one flick brings it home |
| Arming checks not skipped | they catch the rest before take-off |
| Hover throttle 12.5–60 % | heavier = no margin in wind |
| Out-of-date names (`RTL_ALT`, `ARMING_CHECK`...) | ArduCopter 4.7 silently ignores them |
| Raw-gyro and fast-attitude logging | so the checks below can see the motor noise and the wobble |

## After: the log doctor's new hardware checks
| Check | What it looks at | Limit |
|---|---|---|
| **Compass interference from the motors** | the magnetic field's strength against the throttle, scaled to full throttle (like ArduPilot's COMPASSMOT) | < 30 % fine, > 60 % bad |
| **Motors fighting on yaw** | quad X: motors 1+2 spin one way, 3+4 the other; if one pair always works harder, the motors are twisted on their arms | < 50 µs |
| **Wobble** | the attitude error above 3 Hz (wind and stick moves are slower; a ringing controller is faster) | < 0.15° RMS |
| **Battery per cell** | cells guessed from the full-pack voltage; the lowest voltage per cell in flight; internal resistance = volts lost per amp | ≥ 3.5 V/cell, < 25 mΩ/cell |

Then **Fix these before the next flight**: the worst problems first, ordered safety → hardware → tuning, each with what to do.

## Do they work? Real ArduCopter 4.7.3, one problem built in per flight
`python -m backend.sitl.hardware_check --write` → `docs/results/hardware-checks.md`. The simulator can leak motor current into its compass (`SIM_MAG_MOT`, in mGauss per amp) and we can over-tune it:

| Flight | Compass vs motors | Wobble | Fix first |
|---|---|---|---|
| Nothing wrong | ✅ 0 % | ✅ 0.01° | nothing |
| Compass next to the power wires | ❌ 134 % | ✅ 0.01° | compass, EKF (it rejected the compass), position (it wandered) |
| Rate gains × 3.5 | ✅ 0 % | ❌ 0.49° at 23 Hz | wobble, attitude |

The compass flight shows why this check matters: the EKF started rejecting the compass and the position wandered 2 m. That's the start of a "toilet-bowl" circle in Loiter.

## Bugs we hit (lessons for real logs)
1. **In ArduCopter 4.7, `ATT` is always 10 Hz.** The controller's fast copy is a different message, `ANG`, logged at the loop rate (400 Hz) only when `LOG_BITMASK` includes bit 0 (ATTITUDE_FAST). With 10 Hz data the over-tuned drone's 23 Hz ringing was invisible: it looked *fine*. So for tuning flights, add 1 to `LOG_BITMASK`; the pre-flight check reminds you.
2. **The first wobble measure was wrong:** "remove a 1 s average, take the RMS" left the drone's own manoeuvres in: 0.22° for a perfect drone against 0.61° for the over-tuned one. Measuring only above 3 Hz gives 0.01° against 0.49°, a 50× difference. Look at the spectrum before choosing a threshold.
3. **SITL's default battery never sags,** so it can't test the battery check (it reads 4.20 V and ≈ 0 mΩ). That check is tested on synthetic data only; your real logs will be its first real test.
4. **A cached report hides new checks:** the log doctor caches each log's analysis, so the cache now carries a version number.

## Honest limits
- The limits for wobble come from the simulator. A real frame flexes and vibrates, so expect a little more; trust the *trend* between your own flights more than one number.
- "Motors fighting on yaw" and the per-cell battery check couldn't be reproduced in SITL; they're tested on synthetic logs.
- The cell count is a guess from the highest voltage (≈ 4 V/cell). A pack that starts half empty can fool it.

## Self-check
1. Why can't a 10 Hz log show a 23 Hz wobble?
2. In the compass flight nothing was wrong with the GPS, yet the position drifted 2 m. Why?
3. Why compare the two pairs of motors (1+2 against 3+4) rather than each motor on its own?

## Answers
1. You need more than 2 samples per wobble to see it (Nyquist: a 10 Hz log sees at most 5 Hz). The 23 Hz ringing falls between the samples and is either missed or folded into a fake slow signal.
2. The compass told the EKF the wrong heading whenever the throttle changed. With a wrong heading the EKF turns velocity into the wrong direction, until GPS corrections pull it back: the position wanders.
3. A twisted motor tilts its thrust sideways, and that becomes a yaw torque. The flight controller holds the heading by speeding up the pair that spins the other way, so the *pairs* stay unequal for the whole flight, even when the drone is level and balanced.
