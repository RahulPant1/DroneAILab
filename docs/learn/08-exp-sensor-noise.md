# 08 · Experiment: Sensor noise (truth vs estimate)

**Question:** the controller never sees the truth. What does it see, and how do we clean it up?

## Sensors (all imperfect)
- **Gyroscope:** spin rates. Smooth, but a small constant *bias* integrates into a growing angle error.
- **Accelerometer:** feels gravity + acceleration. At rest it points "up", but while the drone accelerates it mostly feels its own thrust and **lies** about which way is down.
- **Altimeter / GPS:** absolute height and position. They don't drift, but they're noisy and slow.

## The estimator (complementary filter)
1. **Predict** with the fast sensors (gyro, accelerometer): smooth, but drifts.
2. **Correct** with the slow, absolute ones (altimeter, GPS, compass, gravity direction).
3. **Trust** = how much each correction moves the estimate.
   - Too high: the estimate copies the noise, and the drone chases it.
   - Too low: smooth, but drift creeps in.

## What to look for
- **Altitude plot:** the scattered dots (altimeter) vs the smooth estimate vs the truth. The gap between the estimate and the truth is what the drone *doesn't know*.
- **Altitude trust 1.0:** the estimate jumps with every reading, and the true height bobs.
- **Gyro bias:** the tilt estimate settles off by about *bias ÷ tilt correction*. The drone then truly tilts to "fix" a tilt it doesn't have, and drifts.
- **Tilt correction 1.0:** fixes bias faster, but trusts the lying accelerometer during accelerations. Look at the roll gap.

## Self-check
1. Why combine two sensors instead of using the best one?
2. With a 1 °/s gyro bias and a tilt correction of 0.5 /s, how big is the steady tilt error?
3. Why does the drone wander even though its *estimated* position sits at 0?

## Try it live
In the **Fly** tab, tick **📡 Sensor noise** and hover. The pink ghost outline is the drone's *estimate* of itself.
The controller keeps the ghost on target, so the real drone wanders by about the estimate error. Turn the level up
to 3× and watch the gap grow. Then fly a mission through the slalom. The mission autopilot also only knows the estimate, so it can say "Reached
waypoint 1" while really being 0.6 m away (the message tells you both). At 1x the safety margin holds; at 3x the
estimate is often wrong by more than the margin, and the drone can clip a box. That's why real drones with poor
GPS fly slower and with wider margins.
