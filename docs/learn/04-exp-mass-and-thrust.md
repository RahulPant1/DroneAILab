# 04 · Experiment: Mass vs required thrust

**Question:** how does the thrust needed to hover depend on mass?

## The idea
- Hover means **thrust = weight = m · g**. Nothing else matters for a steady hover: not arm length, not gains.
- So hover thrust is **proportional** to mass. Double the mass, and you need double the thrust (and double the throttle %).
- **Headroom** = 100 % − hover %. It's the spare capacity for climbing and manoeuvring.

## What to look for
- **Thrust vs weight plot:** once hovering, the two lines sit on top of each other, for any mass.
- **Motor plot:** the dashed "expected hover" line (100 ÷ T/W) predicts where the motors settle.
- Save two runs (e.g. 0.8 kg and 1.6 kg) and **compare** them in History: the hover throttle doubles.

## Why it matters
Every gram you add (a bigger battery, a camera) eats headroom. A bigger battery gives more energy but more weight, so flight time doesn't grow as much as you'd hope.

## Self-check
1. A 1.5 kg drone hovers at 60 % throttle. What will it hover at with a 0.5 kg payload added?
2. Why doesn't arm length change the hover throttle?
3. What happens to headroom as mass goes up, and why does that make the drone feel sluggish?

## Answers
1. Hover throttle is proportional to mass: 60 % × 2.0 ÷ 1.5 = **80 %**.
2. Hover only needs thrust = weight (m · g). Arm length changes the *leverage* for tilting (how fast it can rotate), not how much lift is needed.
3. Headroom (100 % − hover %) shrinks. Less spare thrust means slower climbs, weaker braking and less tilt, and the motors saturate sooner.
