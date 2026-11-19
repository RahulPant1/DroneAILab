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

## Try it live (Fly tab)
Each step says what to do, then what you should see.

### 1. Truth vs estimate
1. Tick **📡 Sensor noise** and press **Take off**. The button counts down "GPS settling" for 5 s first (see part 2), then the drone takes off.
2. Hover and watch the **pink ghost outline**: that's where the drone *thinks* it is. The controller keeps the ghost on target, so the real drone wanders by about the estimate error (about 0.15 m at 1×). Telemetry shows "estimate off by …".
3. Turn **level** up to 3×. The gap grows to about 0.5 m.
4. Use **Fly to** x 4, y 3 at 3×. The drone stops where it *thinks* the point is, often half a metre or more away.

Everything that flies the drone (the controller, the buttons and the mission autopilot) only knows the estimate.
Only physical facts, like touching the ground or a box, use the truth.

### 2. GPS settle (pre-arm check)
With noise on, the drone won't take off for the first 5 seconds. While it sits still on the ground it **knows** it
isn't moving, so it averages every GPS fix instead of filtering. One fix can be half a metre off; the average of 50
is about 7 times better (error shrinks with the square root of the number of fixes). Watch the ghost jump around at
first and then settle onto the real drone. Real flight controllers do the same: they won't arm until the position
estimate is steady.

### 3. Which sensor causes the drift?
1. With sensor noise on at 3×, untick **🛰 GPS noise**. The GPS becomes perfect; the gyro, accelerometer, altimeter and compass stay noisy.
2. Sideways wander drops about 5 times (0.5 m → 0.1 m), but the up-down wobble stays the same: height comes from the altimeter, not the GPS.

That's why precise drones add RTK GPS (centimetre-level) or cameras for position.

### 4. A mission on an estimate
1. Pick the **Slalom** course and press **▶ Fly mission** (A*) at 1×. The messages say things like "Reached waypoint 1, by its estimate. Really 0.6 m away."
2. Try 3×. The estimate is now often wrong by more than the safety margin, and the drone can clip a box (3 crashes in 5 tries in our tests).

That's why real drones with poor GPS fly slower and with wider margins.

### 5. GPS outage and the failsafe
1. Hover with sensor noise on, then tick **📵 GPS outage**. Telemetry shows **GPS LOST**, and after 1 s the mode turns to **failsafe**.
2. The drone now holds level and height, but **not position**: it drifts a few metres over 30 s. The ghost wanders off on its own.
3. Press **W** for a second and let go. The drone tilts, speeds up, levels out, and **keeps going**. Press **S** briefly to stop it.
4. Untick the outage. The drone flies back to its old target (Telemetry shows it as "Target (frozen)" during the failsafe).
5. Untick **🛟 Failsafe** and repeat step 1. Now the drone keeps trusting its drifting estimate: after 30 s it can be anywhere from 3 to 30 m away at 1× (35 to 100 m at 3×).

**Why it drifts so much.** Without GPS, the drone can only work out its position by adding up its accelerometer
readings (*dead reckoning*). A tiny acceleration error adds up into a speed error, and the speed error adds up into a
position error, so the drift grows with **time squared**.

**Why the failsafe helps.** After 1 s without GPS, it stops trusting its position and just holds level and keeps
its height (the altimeter still works). It still drifts, but it doesn't chase a position that's wrong. A mission is
aborted on GPS loss, and take-off is refused without a GPS fix. Real drones call this "altitude mode".

**Why it keeps coasting.** In failsafe, W/A/S/D tilt the drone directly instead of moving a target. Tilting makes
it *accelerate*; levelling out stops the acceleration, not the motion (Newton's first law, see lesson 05). To stop,
you tilt the other way for a moment. That's exactly what makes flying without GPS hard.

**The two failsafe sliders** (in the 🛟 Failsafe box):
- **drag** adds extra air drag *only during the failsafe*, so the drone coasts to a stop by itself. It's a "training wheels" helper, **not real physics**. At the default 0.4, a 1.5 s press of W carries the drone about 3 m and it stops within about 6 s; at **0** (real physics) it's about 12 m and it's still moving. A real drone has no such help.
- **tilt** sets how far a full key press tilts the drone (default 8°). More tilt speeds it up faster (20° is about 2.5 times 8°), but makes it harder to stop.

## Self-check
1. Why combine two sensors instead of using the best one?
2. With a 1 °/s gyro bias and a tilt correction of 0.5 /s, how big is the steady tilt error?
3. Why does the drone wander even though its *estimated* position sits at 0?
4. Why is the average of 50 GPS fixes about 7 times better than one, and why may the drone only do this on the ground?
5. Without GPS, the drift after 20 s is much more than twice the drift after 10 s. Why?
6. In failsafe you press W, then let go. The drone is level again but still moving. Why, and how do you stop it?

## Answers
1. Each sensor is good at something different. The gyro is smooth and fast but slowly drifts; the accelerometer, altimeter and GPS never drift but are noisy or slow. Combining them keeps the smoothness of one and the long-term accuracy of the other.
2. Error ≈ bias ÷ tilt correction = 1 °/s ÷ 0.5 /s = **2°**.
3. The controller holds the *estimate* on target, but the estimate is a little wrong, and the error keeps changing. The real drone moves by exactly that error.
4. Random errors partly cancel when you average, and the leftover error shrinks with the square root of the number of readings: √50 ≈ 7. Averaging only works if the drone isn't moving; in flight, a change in the GPS reading might be real motion.
5. An acceleration error adds up into a speed error (growing with time), and that speed error adds up into a position error (growing with time squared). Twice the time gives about four times the drift.
6. Tilting makes the drone accelerate; levelling stops the acceleration, but the speed it built up stays (only air drag slows it). Tilt the other way briefly (S after W) to brake.
