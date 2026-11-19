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

Everything that flies the drone (the controller, the buttons and the mission autopilot) only knows the estimate.
Try **Fly to** x 4, y 3 at 3x noise: it stops where it *thinks* the point is, often half a metre or more away.

### GPS settle (pre-arm check)
With noise on, the drone won't take off for the first 5 seconds. While it sits still on the ground it **knows** it
isn't moving, so it averages every GPS fix instead of filtering. One fix can be half a metre off; the average of 50
is about 7 times better (error shrinks with the square root of the number of fixes). Watch the pink ghost jump
around at first and then settle onto the real drone. Real flight controllers do the same: they won't arm until the
position estimate is steady.

### Which sensor causes the drift?
Untick **🛰 GPS noise** (with sensor noise still on): the GPS becomes perfect while the gyro, accelerometer,
altimeter and compass stay noisy. Sideways wander drops about 5 times (0.5 m → 0.1 m at 3x), but up-down wobble
stays the same, because height comes from the altimeter, not the GPS. That's why precise drones add RTK GPS
(centimetre-level) or cameras for position.

### GPS outage (try it!)
Hover with sensor noise on, then tick **📵 GPS outage**. Without GPS, the drone can only work out its position by
adding up its accelerometer readings (*dead reckoning*). Tiny errors add up into speed errors, and speed errors
into position errors, so the drift grows with time squared: after 30 s, anything from 3 m to 30 m at 1x.
That's why real drones have a **failsafe**: after 1 s without GPS, ours stops trusting its position and just holds
level and keeps its height (the altimeter still works). It still drifts a few metres, but doesn't run away.
Untick **failsafe** to see the difference, then untick the outage and watch it fly home. A mission is aborted on
GPS loss, and take-off is refused without a GPS fix.
In failsafe, **W/A/S/D tilt the drone** instead of moving a target, like flying a real drone in "altitude mode".
Tilt forward and it speeds up forward; let go and it levels out but **keeps coasting**. To stop, tilt the other way
for a moment. That's exactly what makes flying without GPS hard. When GPS returns, it flies back to where it was.

To make this less frustrating, the simulator adds some **extra air drag during the failsafe** (a "training
wheels" helper, not real physics): a 1.5 s press of W now carries the drone about 3 m and it stops by itself within
about 6 s, instead of about 12 m and still moving. A real drone has no such help: it keeps coasting until you stop it.
