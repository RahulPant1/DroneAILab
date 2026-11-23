# 21 · Agile flight: smooth trajectories and controllers that keep up

**Big idea:** our missions fly from waypoint to waypoint: a "carrot" slides along straight lines and the drone chases it. That's safe, but slow and always a little behind. Racing, inspection and delivery drones plan **one smooth curve** through every point *before* they fly, and the controller is told not just *where* to be but **how fast and how hard to accelerate** at every moment.

## 1. The trajectory: minimum snap
Between two waypoints, each axis (x, y, z) follows a polynomial in time. Among all the curves that go through every waypoint (and start and end at rest), pick the one with the smallest total **snap**: the 4th derivative of position (position → velocity → acceleration → jerk → **snap**).

Why snap? A quadcopter's tilt follows its acceleration, and how fast it tilts follows the jerk. Keeping snap small keeps the motor commands smooth. (Mellinger & Kumar, *Minimum Snap Trajectory Generation and Control for Quadrotors*, 2011.)

It's a quadratic cost with straight-line (linear) rules: "be at waypoint 3 at time t₃", "velocity, acceleration, jerk and snap match where two pieces join". So one linear solve gives the answer, with no iterative optimiser. Race rings add one more rule: **the velocity at a ring must point through it** (along its arrow).

**Timing:** a first guess per piece (distance ÷ speed), then time is moved between pieces to lower the snap, then the *whole* trajectory is stretched or squeezed until it just fits the drone's limits. Stretching time by k divides every speed by k and every acceleration by k², so the shape stays the same and only the pace changes. The **max tilt** you allow decides the pace: tilt = how hard it may accelerate sideways (a = g·tan(tilt)).

We checked our solver against an independent open-source implementation ([minsnap-trajectories](https://github.com/Hs293Go/minsnap_trajectories), MIT). The same waypoints and times give the same curve, to within a millionth of a millimetre. That check runs in every test run.

## 2. Feedforward: don't wait for an error
A feedback controller only reacts once something is wrong. The trajectory already knows the speed and acceleration needed at every moment, so we **add them in directly** (feedforward), plus the air drag it will meet at that speed. Feedback then only corrects the small leftover errors. ArduPilot and PX4 follow their own trajectories this way.

## 3. A geometric controller
Our cascaded PID turns "accelerate east" into "pitch this much" with a small-angle shortcut, one axis at a time, and handles height in a separate loop. A **geometric (SE(3)) controller** does it in one 3D step:
1. wanted acceleration = plan + drag compensation + P·(position error) + D·(velocity error) + I·(integral);
2. add gravity → the thrust **vector**. Its direction *is* the attitude the drone needs (exact at any tilt); the yaw only picks the rotation about it;
3. **spin-rate feedforward** from the plan's *jerk*: to change acceleration, the thrust axis must swing, and the jerk says how fast. The inner loop gets that turn rate before any error appears.

Same gains as the PID: only the structure differs.

## What we measured (figure eight, 8 rings, calm air unless stated)
| How it flies | Max tilt | Rings | Lap | Off the planned path (mean / worst) |
|---|---|---|---|---|
| Waypoints (carrot, capped at 3 m/s) | – | 8/8 | 23.5 s | 0.08 / 0.39 m |
| Trajectory + PID feedforward | 15° | 8/8 | 20.9 s | 0.06 / 0.41 m |
| Trajectory + geometric | 15° | 8/8 | 20.9 s | 0.02 / 0.04 m |
| Trajectory + PID feedforward | 30° | 8/8 | 14.7 s | 0.20 / 0.81 m |
| Trajectory + geometric | 30° | 8/8 | 14.7 s | **0.03 / 0.05 m** |
| Trajectory + PID feedforward | 45° | 8/8 | 11.7 s | 0.38 / 1.39 m |
| Trajectory + geometric | 45° | 8/8 | 11.6 s | 0.04 / 0.08 m |
| *Gusty 5 m/s crosswind*: PID feedforward | 30° | 8/8 | 14.8 s | 0.31 / 0.85 m |
| *Gusty 5 m/s crosswind*: geometric | 30° | 8/8 | 14.7 s | 0.20 / 0.47 m |

- **The plan sets the lap time;** the controller decides how *closely* it's flown. The faster the plan, the more the PID's shortcuts show: about 7× more error than the geometric controller at 30°, and 9× at 45°.
- **Wind isn't in the plan**, so feedforward can't help with it: both controllers fall back on feedback, and the geometric one's lead shrinks.

## The surprise: real sensors
With real (noisy) sensors, the trajectory flights **missed 2 of the 8 rings** (0.61 m off the path on average), while slow waypoint flying was fine. The cause is in the estimator, not the controller. To keep its tilt estimate honest, it trusts the accelerometer as "which way is down" whenever that reads about 1 g. In a long banked turn the accelerometer feels the **thrust**, not gravity, so the estimate slowly "levels out" while the drone is really leaning. The controller then flies the wrong acceleration.

| Real sensors, 30° | Rings | Off the path (mean) |
|---|---|---|
| Geometric, our simple filter | 6/8 | 0.61 m |
| Geometric, gyro-only tilt (accelerometer correction off) | 8/8 | 0.14 m |

Gyro-only works here because this gyro has no bias and a lap is short. Real autopilots (ArduPilot's EKF3, PX4's EKF2) fix it properly by fusing **GPS velocity**, which tells them the real acceleration. We tried a quick GPS-*position* version and it didn't help (position noise leaked into the tilt), so we kept the honest finding instead.

## Try it
- **Lab → experiment 11:** pick a track, a method, the max tilt and the conditions. The map shows the plan (dashed), the flight, and each ring (green = through, red = missed).
- **Fly tab → 🏁 Race → any track → ✈ Autopilot:** the orange line is the planned trajectory. Pick the method and max tilt. Any flying button takes over. Autopilot laps don't count as best times.

## Self-check
1. Why minimise snap rather than, say, distance?
2. The plan says "accelerate 4 m/s² east now". What does the controller do with that before any error appears?
3. Why did faster plans hurt the PID more than the geometric controller?
4. Why can't feedforward help against wind?

## Answers
1. The shortest path has sharp corners: infinite acceleration, which no drone can fly. Small snap means the tilt changes smoothly, so the motors can actually follow it.
2. It adds those 4 m/s² straight into its acceleration command (and turns it into tilt and thrust). Feedback only has to fix what's left over.
3. The PID converts acceleration to tilt with a small-angle approximation, one axis at a time, and handles height separately. Big tilts and combined climb-and-turn moves break those shortcuts; the geometric controller computes the thrust direction exactly.
4. Feedforward only contains what you know in advance. Wind is a surprise, so only feedback (and the slow integral) can fight it.
