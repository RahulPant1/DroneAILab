# 22 · Offboard control: your code flies a real autopilot

**Big idea:** companies rarely rewrite the flight controller. They run their own planning on a companion computer (or a ground station) and **stream setpoints** to ArduPilot or PX4, many times a second. The autopilot keeps doing what it's best at: estimation, the fast attitude loops, failsafes. That's called **offboard control**. Here DroneLab's own minimum-snap trajectory (lesson 21) is streamed to real ArduCopter 4.7 running in SITL.

## How it works (MAVLink)
1. Take off in **GUIDED** mode (the mode that takes orders from outside).
2. 25 times a second, send `SET_POSITION_TARGET_LOCAL_NED`: position, velocity **and** acceleration, in metres from home, in ArduPilot's **NED** frame (north, east, *down*). DroneLab uses ENU (east, north, up), so swap x/y and flip z.
3. A **type_mask** says which fields to use. Its bits mean *ignore*: bits 0–2 position, 3–5 velocity, 6–8 acceleration, 10 yaw, 11 yaw rate. "Use position + velocity + acceleration, ignore yaw" = `0b1100_0000_0000`.
4. Keep sending. If nothing arrives for **GUID_TIMEOUT** (3 s), ArduCopter stops and holds.

Older docs (and the DroneKit guide) say ArduCopter ignores the acceleration field. That's out of date. We checked the 4.7 source (`ArduCopter/GCS_MAVLink_Copter.cpp` → `ModeGuided::set_pos_vel_accel_NED_m`) and it uses all three.

## What we measured (figure eight, 8 rings, plan at 30° max tilt, calm air)
| Who flies it | Rings | Lap | Off the planned path (mean) | Behind the plan (mean) |
|---|---|---|---|---|
| DroneLab: trajectory + geometric controller | 8/8 | 14.7 s | 0.03 m | 0.04 m |
| **ArduCopter: our trajectory as pos + vel + accel** | 8/8 | **14.9 s** | **0.14 m** | 0.38 m |
| ArduCopter: our trajectory as positions only | 8/8 | 16.0 s | 0.31 m | 4.06 m |
| ArduCopter: AUTO mission through the ring centres (its own planner) | 8/8 | 27.6 s | 0.07 m | – |

Measured on SITL's ground truth (`SIM_STATE`), not on what ArduPilot believes. Full table: `docs/results/agile-compare.md` (`python -m backend.sitl.offboard --write`).

- **Feedforward matters on a real autopilot too.** Positions alone arrive "late": ArduPilot treats each one as a new destination and is about 4 m behind schedule. With velocity and acceleration it flies the plan, 1.4 s faster per lap.
- **AUTO is safe and slow.** ArduPilot's own S-curve planner only knows the ring centres, not which way they face or what comes next, so it keeps the speed low (2.8 m/s top).

## The lesson we learned the hard way: the autopilot's limits must fit your plan
The first 30° run diverged by metres. ArduPilot smooths every GUIDED target with a **jerk limit** (`PSC_NE_JERK`, default 5 m/s³). Our 30° plan peaks at 8.4 m/s³ of jerk, so ArduPilot's input shaper fell behind, overshot, and never recovered:

| 30° square-loop plan | Rings | Off the path (mean / worst) |
|---|---|---|
| ArduPilot defaults (`PSC_NE_JERK` 5) | 1/4 | 2.3 / 5.2 m |
| `PSC_NE_JERK` = 1.5 × the plan's peak jerk (12.7), `ATC_ANGLE_MAX` = tilt + 10°, `WP_ACC` = the plan's 5.66 m/s² | 4/4 | 0.10 / 0.26 m (lap 6.34 s; DroneLab 6.31 s) |

A gentler 20° plan (4.2 m/s³) flew fine with the defaults. So `offboard.py` reads the plan's peaks and sets `PSC_NE_JERK`, `PSC_D_JERK`, `ATC_ANGLE_MAX` and `WP_ACC` before flying. Real offboard work is like this: you have to know the autopilot's parameters as well as your own code.

## Open-source you can learn from (checked for this lesson)
| Project | License | What's in it | How DroneLab uses it |
|---|---|---|---|
| [ArduPilot](https://github.com/ArduPilot/ardupilot) (in `external/`) | GPL-3.0 | The real autopilot; GUIDED handling in `ArduCopter/mode_guided.cpp`, input shaping in `libraries/AC_AttitudeControl/AC_PosControl.cpp` | We **read** it to find what GUIDED accepts and the jerk limit. Never copied into our repo (Apache-2.0); it's built in the gitignored `external/` folder. |
| ArduPilot Lua examples (`libraries/AP_Scripting/examples/`) | GPL-3.0 | `set_target_posvel_circle.lua`: the autopilot itself streams position + velocity to fly a circle | A good next step: offboard logic running **on** the autopilot. Not run here: it needs `SCR_ENABLE=1` and the script in SITL's `scripts/` folder, and it takes over any GUIDED flight above 5 m. |
| [minsnap-trajectories](https://github.com/Hs293Go/minsnap_trajectories) | MIT | Minimum-snap in Python (Bry & Roy's closed form, plus Mellinger & Kumar's QP) | **Added** as a test-only dependency (`backend/requirements-dev.txt`): our solver is checked against it to 10⁻⁶ m on every CI run. |
| [RotorPy](https://github.com/spencerfolk/rotorpy) (UPenn) | MIT | A teaching quadrotor simulator: `trajectories/minsnap.py`, an SE(3) controller in `controllers/quadrotor_control.py` | Worth reading next to our `trajectory.py` and `GeometricController`. Not added: it brings its own simulator and heavy dependencies (gymnasium, pandas, matplotlib, cvxopt). |
| [mav_trajectory_generation](https://github.com/ethz-asl/mav_trajectory_generation) (ETH Zürich) | Apache-2.0 | The classic C++/ROS implementation, with nonlinear time optimisation | Reference reading (C++ and ROS: too heavy for this laptop's setup). |
| [uav_trajectories](https://github.com/whoenig/uav_trajectories) (Crazyflie) | MIT | Polynomial trajectories for tiny real drones | Reference: the same idea flown on real Crazyflies. |
| [mrasl_mav_traj](https://github.com/MRASL/mrasl_mav_traj) | GPL-3.0 | Another Mellinger & Kumar re-implementation | Not used: GPL code can't be copied into an Apache-2.0 repo. |

## Try it
- `.venv/bin/python -m backend.sitl.offboard`: the figure eight on DroneLab (three ways) and on ArduCopter SITL (three ways). Add `--track square_loop --tilt 20` to try other plans, and `--write` to save the table.
- In CI, `test_real_arducopter_flies_our_trajectory_offboard` streams the trajectory to ArduCopter on every push and checks every ring.

## Self-check
1. Why does streaming positions only make the drone fall behind?
2. Your plan works in DroneLab but ArduPilot overshoots by metres. Where do you look first?
3. Why stream at 25 Hz instead of sending the whole trajectory once?

## Answers
1. Each position is a new "go there" with no idea how fast the target is moving. The autopilot accelerates toward it from scratch, so it always arrives late. Velocity and acceleration tell it the motion, not just the point.
2. The autopilot's own limits: lean angle, acceleration and jerk (`ATC_ANGLE_MAX`, `WP_ACC`, `PSC_NE_JERK`). If the plan asks for more than those, the autopilot's shaping can't follow.
3. So your code stays in charge: it can change the plan at any moment (a new obstacle, a moved ring), and if it stops talking, the autopilot's timeout makes the drone stop safely.
