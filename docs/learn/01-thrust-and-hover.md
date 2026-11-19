# 01 · Thrust and hover

**Big idea:** a drone hovers when its propellers push up exactly as hard as gravity pulls down.

## Thrust vs gravity
- **Weight** W = m · g. A 1 kg drone weighs 1 × 9.81 = **9.81 N**.
- Each propeller makes **thrust** T = kT · ω² (ω = spin speed). Spin twice as fast and you get **4×** the thrust.
- Net vertical force = total thrust − weight. If it's positive the drone accelerates up (F = m·a). Zero: it holds its speed. Negative: it slows or falls.

## Thrust-to-weight (T/W)
T/W = (max thrust of all 4 motors) / weight.
- **T/W < 1**: it can never take off. No controller can fix that. Try `--tw 0.9`.
- **T/W = 2** (our default): plenty of spare power for climbing and for fighting wind.
- **Hover throttle ≈ 100 / (T/W)**: T/W 2 → 50 %, T/W 4 → 25 %. The sim measures this: see "hover motor %".
- More mass → higher hover throttle, in direct proportion (twice the mass → twice the %).

## What a control loop is
The drone can't "know" the right thrust in advance, so it keeps correcting, 250 times a second:
1. **Measure** altitude (and vertical speed).
2. **Compare** with the target: error = target − altitude.
3. **Correct** with PID: Kp × error (push harder when far away), Kd × speed (brake when moving fast), Ki × accumulated error (fix small, lasting offsets).
4. Thrust = m·(g + correction). The **m·g** part is the hover "baseline"; the PID only adds or removes a little.

Remove the damping (`--alt-kd 0`) and the drone overshoots to 4 m and keeps bouncing: P alone acts like a spring with no shock absorber.

## Try it
```bash
.venv/bin/python -m backend.cli hover --alt 2                  # normal flight
.venv/bin/python -m backend.cli hover --alt 2 --tw 0.9         # too weak to fly
.venv/bin/python -m backend.cli hover --alt 2 --mass-kg 1.6    # heavier: watch hover %
.venv/bin/python -m backend.cli hover --alt 2 --alt-kd 0       # no damping
.venv/bin/python -m backend.cli freefall --height 20           # gravity check
```

## Code to read
`backend/sim/config.py` (`thrust_to_weight`, `hover_throttle_pct`) → `physics.py` (`step`, steps 1–6) → `control.py` (`_altitude_loop`, `PID.update`).

## Self-check
1. A 1.5 kg drone has four motors of 6 N max each. What is its T/W, and roughly what hover throttle will it need?
2. Why does doubling a propeller's spin speed give four times the thrust, not twice?
3. With Kd = 0 the drone bounces around the target. Which term was "braking" it, and what does it measure?
