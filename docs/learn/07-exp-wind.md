# 07 · Experiment: Wind

**Question:** what does wind do to a hovering drone, and how does the controller fight back?

## The idea
- Drag depends on speed **relative to the air**. In a 5 m/s wind, a drone holding still feels the same drag as one flying at 5 m/s in calm air.
- To stay put, the drone must **lean into the wind**, so part of its thrust pushes back against the drag.
- The position controller is a chain: position error → wanted speed (P) → wanted acceleration (P + **I**) → tilt.
  - **P** only reacts once the drone has been pushed off its spot.
  - **I** slowly "learns" the steady push and adds the lean on its own, bringing the drone back to exactly zero error.

## What to look for
- **Distance plot:** a peak (how far the gust of wind pushed it) and then a decay back to 0.
- **Speed I-gain 0:** it stops drifting but stays **downwind**. P needs an error to produce a lean.
- **Gusts:** random wind changes faster than the controller reacts, so the drone wanders and the motors never rest.
- **Tilt plot:** the lean angle grows with wind speed squared (drag ∝ v + v²), so strong wind costs a lot of tilt and thrust.

## Self-check
1. Why does a hovering drone tilt in steady wind even though it isn't moving?
2. What does the I term add that P can't?
3. Why can't the controller cancel gusts completely?
