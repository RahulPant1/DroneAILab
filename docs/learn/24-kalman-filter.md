# 24 · The Kalman filter (EKF): how autopilots really know where they are

**Big idea:** every reading lies a little, and so does dead-reckoning. A **Kalman filter** keeps a guess *and how sure it is of that guess*. Each time a reading arrives, it compares the two kinds of uncertainty and moves exactly as far as they justify. ArduPilot's **EKF3** and PX4's **EKF2** are Kalman filters. So is DroneLab's new `backend/sim/ekf.py` (about 300 lines, written from the textbook).

## Why our simple filter wasn't enough
Lesson 8's **complementary filter** blends sensors with **fixed** trust gains, and it believes "the accelerometer points up". That's true in a hover. In a long banked turn the accelerometer feels the *thrust*, so the filter slowly levels its estimate while the drone is really leaning. In experiment 11 the drone missed rings because of it.

## Three ideas that make a Kalman filter
1. **It tracks its own uncertainty** (the covariance, `P`). Dead-reckoning on the gyro and accelerometer makes it grow; every accepted reading shrinks it. The gain is worked out fresh each time: *my uncertainty ÷ (my uncertainty + the sensor's noise)*.
2. **It knows how errors are linked.** A tilt error makes a velocity error grow, because the accelerometer's push is turned the wrong way. So when the GPS **velocity** disagrees, the filter also corrects the **tilt**, and even the **gyro bias** (a gyro reading a little spin that isn't there). That's how it stays level in banked turns.
3. **It checks every reading before using it** (an *innovation gate*). If a reading is further from the prediction than 5 standard deviations, it's rejected, so a GPS glitch is ignored instead of followed. After 10 s of rejections it gives in and resets: maybe the filter is the one that's wrong. EKF3 uses the same numbers (`EK3_POS_I_GATE = 500` = 5σ, a 10 s retry).

```
 every IMU sample (250 Hz):  PREDICT  position, velocity, attitude  += gyro / accelerometer;   P grows
 when a reading arrives:     GATE     innovation = reading - prediction; too big for P + R? -> reject
                             CORRECT  move by gain × innovation (all 12 error states, linked through P); P shrinks
```

Ours is an *error-state* EKF with 12 error states: position, velocity, attitude (a small rotation) and gyro bias, 3 each. EKF3 has 24 (it adds accelerometer bias, the earth's and the drone's magnetic field, and wind) and runs two copies at once.

## Experiment 12: same flight, same noise, two filters (DroneLab sim, seed 0)
| Scenario | Complementary filter | EKF |
|---|---|---|
| Figure eight at 30° tilt | 6 of 8 rings, tilt error 2.6° | **8 of 8**, tilt error 0.3° |
| GPS jumps 5 m for 5 s | follows it: the drone flies **8.9 m** away | rejects it: the drone stays within **0.3 m** |
| Gyro reads 3°/s too much | estimate **13° off level** | learns the bias (2.99°/s), 0.2° off |
| GPS lost for 10 s (in wind) | drifts 4.0 m, and doesn't know it | drifts 3.4 m, and **knows**: its ±2σ grew to 7.5 m |

Two honest limits:
- **A long glitch beats it.** With a 15 s glitch the filter resets to the bad GPS after 10 s. When the glitch ends, the *correct* fixes now look like the glitch and are rejected for another 10 s. With one GPS no filter can tell a long, steady error from the truth; that's why serious drones carry two receivers or a camera.
- **In an outage it still drifts.** It can't make up information it doesn't have; it only knows how lost it is. Autopilots use that number to decide when to stop trusting the position (ArduPilot's EKF failsafe).

## Checked against ArduPilot's EKF3 on the same data
We flew real ArduCopter 4.7 in SITL with `LOG_REPLAY = 1`, which logs every input its EKF3 used. Then we fed those inputs through *our* filter and scored both against the simulator's truth. The flight: hover; a 2°/s gyro bias appears; the figure eight at 30°; a 5 m GPS glitch. Full table: `docs/results/ekf-compare.md` (`python -m backend.sitl.ekf_replay --write`).

- **Both rejected the glitch** for its full 5 s.
- **The sudden gyro bias is where EKF3 showed its class.** It lets its bias estimate change slowly (biases usually drift with temperature): it sat about a metre off for a while, but it **never got lost**. Our filter with a slow setting (EKF3-like, or our default) **did** get lost: up to 23–27 m off before its reset rule pulled it back. With a fast setting it learnt the bias in seconds and stayed within 0.6 m. But in experiment 12, the fast setting makes the estimate wander more in a GPS outage. So there is no free choice, and EKF3's extra machinery (more states, two copies, careful variance limits) buys robustness our 12-state filter doesn't have.
- **During the lap ours was closer to the truth** (0.07 m vs 0.42 m). Don't read much into that: SITL's GPS has no horizontal noise, which rewards trusting it. EKF3 is built to be hard to fool on real, messy hardware.

## Bugs we hit (each one is a real lesson)
1. **Overconfidence kills a filter.** Our first replay diverged by hundreds of metres. The filter was so sure of its gyro bias that it couldn't explain the growing tilt error; it started rejecting *every* GPS reading, and from then on it only dead-reckoned. Fix: let the bias change faster, and add a "lost" rule: if GPS velocity is rejected for 5 s, jump to the GPS and **widen** the uncertainty, so it listens again.
2. **GPS readings arrive late.** A receiver reports where the drone *was* about 0.1 s ago. At 7 m/s that's 0.7 m. The filter compares each fix with its own estimate from that moment (ArduPilot does the same with a "delayed horizon").
3. **SITL's first GPS messages come before a fix**, with a position about 144 km off. Our filter now waits for EKF3's origin before it starts.
4. **Axes, again.** ArduPilot uses forward-right-down and north-east-down; DroneLab uses forward-left-up and east-north-up. Every vector is converted at one place (`frd_to_flu`, `ned_to_enu`), with a test for each.

## Try it
- **Lab → experiment 12**: four scenarios, both filters on the same plots. The dashed line is the EKF's own claim of how wrong it might be.
- **Fly view → 📡 Sensor noise → filter: Kalman (EKF)**: the 🧮 line shows its uncertainty, the bias it learnt and the GPS test ratio. Tick **⚡ GPS glitch** and watch the ratio go above 1, then switch back to the complementary filter and watch the drone fly away.
- `.venv/bin/python -m backend.sitl.ekf_replay` flies the ArduPilot comparison (about 3 minutes); `--log file.BIN` replays any log flown with `LOG_REPLAY=1`.

## Self-check
1. Why does a GPS *velocity* reading change the filter's estimate of the drone's *tilt*?
2. A reading's innovation is 4 m and the filter's predicted standard deviation is 0.5 m. With a 5σ gate, is it used?
3. Why widen the uncertainty after a reset, instead of keeping the small one?

## Answers
1. Because a tilt error turns the accelerometer's push into a sideways acceleration that isn't really there. The covariance has recorded that link, so a velocity mismatch is partly blamed on the tilt.
2. No. The test ratio is (4 / (5 × 0.5))² = 2.56, which is above 1, so it's rejected.
3. A filter that is very sure of itself gives new readings almost no weight. After a reset it *should* be unsure, so that the next readings can pull it into place.
