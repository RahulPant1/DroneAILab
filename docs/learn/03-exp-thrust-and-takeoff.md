# 03 · Experiment: Thrust and take-off

**Question:** how much thrust does a drone need, and what does *spare* thrust buy you?

## The idea
- To leave the ground, total thrust must be **more** than weight. To hover, it must be **exactly** equal.
- **T/W** (thrust-to-weight) = max thrust ÷ weight. Hover throttle ≈ 100 % ÷ T/W.
- Everything above hover thrust is **spare**. It sets how fast you can climb, brake, and fight wind.

## What to look for
- **Thrust vs weight plot:** while thrust is above the dashed weight line, the drone speeds up upward.
- **Motor plot:** at T/W ≈ 1.2 the motors sit at 100 % (saturated) for most of the climb, so the climb is slow.
- **T/W 0.9:** thrust never beats weight, so the drone never moves. No controller can fix physics.

## Rules of thumb
Real multirotors are designed for T/W ≈ 2 (a camera drone) up to 5+ (a racing drone). Below about 1.5, they feel sluggish and struggle in wind.

## Self-check
1. A drone with T/W 2.5 hovers at what throttle?
2. Why does a low-T/W drone climb slowly even though its controller asks for a fast climb?
3. The drone weighs 12 N. What is the least max thrust that lets it climb at all?

## Answers
1. 100 % ÷ 2.5 = **40 %**.
2. The motors hit 100 % (saturate). The upward acceleration is (max thrust − weight) ÷ mass, which is small when there's little spare thrust, whatever the controller asks for.
3. **More than 12 N.** Exactly 12 N only balances the weight (hover); it needs a little more to climb.
