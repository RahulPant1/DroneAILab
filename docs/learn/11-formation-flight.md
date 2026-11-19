# 11 · Formation flight (multi-drone)

**Big idea:** a swarm has no single brain. Each drone runs its own autopilot and only needs one piece of shared information: where the leader is.

## How our formation works
1. **The leader broadcasts.** Many times a second, your drone tells the others where it *believes* it is and which way it faces (real swarms do this over radio).
2. **Each wingman has a slot**, measured from the leader: "1.5 m behind and 1.5 m to the left". The shape is lined up with the leader's nose, so turning with **Q/E** turns the whole formation. (On a mission the autopilot doesn't turn the nose, so there the shape lines up with the direction of travel.)
3. **Each wingman flies to its slot** with its own autopilot: the same position controller as your drone. It takes off, lands and turns when the leader does.

Shapes: **V** (like geese), **line** (side by side) and **column** (single file).

## Why swarms need collision avoidance
A wingman can't teleport to its slot: it lags behind, just like your drone lags behind a "Fly to" target. Fly **backwards (S)** into a column and you run into the wingman behind you before it can get out of the way. Two drones closer than **0.6 m** (centre to centre) touch propellers, and both fall.

With **🛡 avoid crashes** on, each wingman:
- **looks ahead** one second: "at our current speeds, will we get too close?"
- **steps sideways** and **dodges up or down**. Between two wingmen, the lower-numbered one climbs and the other ducks, so they never dodge into each other.

That's the potential-field idea from lesson 09, used between moving drones instead of fixed boxes.

## Obstacles: breaking formation
A V or a line is wider than a gap in the slalom. With avoidance on, a wingman whose way to its slot is blocked by a box **breaks formation**: it follows the leader's own path (the leader got through, so its path is safe), queueing in the order the wingmen are already in. Once it's clear, it spreads out to its slot again. Telemetry says "breaking formation" while it happens.

## What we measured
- Backing into your formation, changing shape mid-flight, calm or 6 m/s wind, spacing 1.5–4 m (24 cases): **6 crashes with avoidance off, 0 with it on**.
- A* missions on all 4 courses with 2 or 4 wingmen in each shape (24 runs): crashes in **14 runs with avoidance off, 1 with it on**. That one is the widest formation (a line of 5 drones, 6 m across) in the narrow box canyon.

## Try it (Fly tab)
1. Pick **🛩 Wingmen: 4**, shape **V**, and press **Take off**. Fly around, and turn with Q/E: the V turns with you.
2. Switch to **column**, untick **🛡 avoid crashes**, then hold **S**. Crash! Press Reset, tick it again, and try once more: the wingman sidesteps and climbs out of your way.
3. Load the **Slalom** course and press **▶ Fly mission** with a V of 2. Watch the wingmen fall in behind the leader between the boxes, then spread out again.
4. Turn on **📡 Sensor noise** and a **📵 GPS outage**. The wingmen follow where your drone *thinks* it is (the pink ghost), so they drift away from the real drone with it: about 10 m after 30 s, or 27 m without the failsafe. One drone's bad position misleads the whole swarm.

## Real drones
Light-show swarms (hundreds of drones) mostly fly pre-planned paths, each with precise RTK GPS. Research swarms share positions over radio and use avoidance rules like ours (the classic "boids" rules: separation, alignment, cohesion). Both need every drone to know its own position well.

## Self-check
1. Why does a wingman lag behind its slot when the leader speeds up?
2. Why is flying backwards into a column more dangerous than flying forwards away from it?
3. Why do the wingmen drift away from your real drone during a GPS outage, even though *their own* sensors are perfect?

## Answers
1. Its autopilot only starts moving once the slot has moved away (there's an error to correct), and it needs time to tilt and speed up. So it's always a little behind a moving slot.
2. Going forwards, the lag makes the gap bigger. Going backwards, the leader rushes at the wingman behind it, and the wingman's lag makes the gap smaller: there's only a fraction of a second before they touch.
3. They don't measure where the leader is. They fly to slots around the position the leader *broadcasts*, and during an outage that's the leader's drifting estimate. A swarm is only as good as the information it shares.
