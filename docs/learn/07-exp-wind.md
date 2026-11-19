# 07 · Experiment: Wind

**Question:** what does wind do to a hovering drone, and how does the controller fight back?

## The idea
- Drag depends on speed **relative to the air**. In a 5 m/s wind, a drone holding still feels the same drag as one flying at 5 m/s in calm air.
- To stay put, the drone must **lean into the wind**, so part of its thrust pushes back against the drag.
- The position controller is a chain: position error → wanted speed (P) → wanted acceleration (P + **I**) → tilt.
  - **P** only reacts once the drone has been pushed off its spot.
  - **I** slowly "learns" the steady push and adds the lean on its own, bringing the drone back to exactly zero error.

## Wind direction: this app vs weather reports
The app gives the direction the wind blows **toward**, as a maths angle: 0° = east, 90° = north, counting
counter-clockwise (the same as the x/y axes).
Weather reports and pilots do the opposite: they give where the wind comes **from**, as a compass bearing
(0° = north, 90° = east, counting clockwise). To convert a report into the app: **toward = 270° − from**
(add 360° if it comes out negative).
- A "north wind" (from 0°) blows toward the south: app **270°**.
- A "west wind" (from 270°) blows toward the east: app **0°**.
- A "south-west wind" (from 225°) blows toward the north-east: app **45°**.

## What to look for
- **Distance plot:** a peak (how far the gust of wind pushed it) and then a decay back to 0.
- **Speed I-gain 0:** it stops drifting but stays **downwind**. P needs an error to produce a lean.
- **Gusts:** random wind changes faster than the controller reacts, so the drone wanders and the motors never rest.
- **Tilt plot:** the lean angle grows with wind speed squared (drag ∝ v + v²), so strong wind costs a lot of tilt and thrust.

## Self-check
1. Why does a hovering drone tilt in steady wind even though it isn't moving?
2. What does the I term add that P can't?
3. Why can't the controller cancel gusts completely?

## Answers
1. Drag depends on speed relative to the **air**. Standing still in wind feels like flying into it, so the drone must lean into the wind to make a sideways push that cancels the drag.
2. I adds up the error over time, so it builds the lean on its own and brings the drone back to exactly zero error. P needs a lasting error to push, so with P alone the drone stays a little downwind.
3. The controller only reacts **after** a gust has pushed the drone off, and tilting takes time. Random gusts change faster than that loop can respond (and the drone has no wind sensor to see them coming).
