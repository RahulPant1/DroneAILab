# 05 · Experiment: Roll, pitch and yaw

**Question:** which motors make each rotation, and why does tilting move the drone?

## The idea
- **Roll / pitch:** push one side harder than the other. The thrust *difference* times the arm length is a turning torque.
- **Yaw:** each propeller drags the body the opposite way to its spin. Speed up the clockwise pair and slow the counter-clockwise pair (keeping total thrust the same), and the body turns.
- Once tilted, part of the thrust points sideways, so the drone **accelerates** that way. Levelling out stops the acceleration, not the motion; only air drag slows it.

## What to look for
- **Motor plot at the step:** one pair jumps up and the other drops. Right after, the split briefly *reverses*: that's the controller braking the rotation so it stops at the right angle.
- **Yaw:** the split is huge (often 100 % vs 0 %) for a small turn. Propeller drag torque is weak compared with thrust torque.
- **Position plot:** roll and pitch steps send the drone sliding; yaw doesn't move it at all.

## Gain
A higher angle gain gives a faster rotation, but past a point it overshoots. A lower gain feels lazy.

## Self-check
1. Which motors speed up for a nose-down pitch? Why those?
2. Why is yaw the slowest of the three rotations?
3. After a 2-second 15° roll, the drone is level again but still moving sideways. Why?

## Answers
1. The **rear** motors (M2, M4). Pushing the back up tips the nose down.
2. Yaw torque comes from the propellers' air drag, which is much weaker than thrust × arm length. Total thrust must also stay the same while yawing, and the drone is hardest to turn about its vertical axis.
3. Levelling stops the **acceleration**, not the motion. The drone keeps the speed it built up (Newton's first law); only air drag slowly takes it away.
