# 06 · Experiment: PID tuning (step response)

**Question:** what do P, I and D each do? We ask for a sudden jump from 1 m to 3 m and watch.

## The three terms
- **P (Kp): a spring.** It pushes in proportion to the error. Stronger means faster, but a strong spring bounces.
- **D (Kd): a shock absorber.** It pushes *against* the vertical speed, braking the approach so it doesn't overshoot.
- **I (Ki): a memory.** It adds up error over time, so it can cancel a steady push that P alone can't, like a payload the controller doesn't know about.

## Reading a step response
- **Rise time:** how fast it gets 10 % → 90 % of the way.
- **Overshoot:** how far past the target it goes.
- **Settling time:** when it stops wobbling (stays within ±5 cm).
- **Steady-state error:** where it ends up vs where it should be.

## What to look for
- **Kp 25:** fast rise, big overshoot, several bounces. Too much spring for the damping.
- **Kd 0:** nothing brakes it: about 100 % overshoot and long wobbles.
- **Hidden payload with Ki 0:** it settles too low *for good*. P needs an error to push, so an error remains. Turn Ki on and watch it slowly climb back.
- Flat motor lines at 100 % mean **saturation**: the gains ask for more than the motors can give.

## Self-check
1. Overshoot is too big. Name two knobs that reduce it.
2. Why can't a P-only controller hold the exact height with an unknown payload?
3. What's the downside of a very large Ki?

## Answers
1. Lower **Kp** (a softer spring) or raise **Kd** (more braking). A smaller Ki also helps.
2. P only pushes when there's an error. Holding up an unknown payload needs a steady extra push, so P needs a steady error to produce it: the drone settles a bit too low.
3. The I term keeps adding up error during big moves (*wind-up*), then overshoots and swings slowly back and forth before settling.
