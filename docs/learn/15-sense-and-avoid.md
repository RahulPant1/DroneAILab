# 15 · Sense and avoid: flying without a map

**Big idea:** the A* planner (lesson 09) knows every obstacle before take-off. A real drone usually doesn't: it has to **sense** obstacles, **map** them and **replan**, all while flying. That's what the lidar planner does.

## How it works
1. **Lidar.** 72 laser beams sweep a full circle at the drone's height, 10 times a second, out to 8 m. Each beam measures the distance to the first obstacle it hits (with ~2 cm of noise). Obstacles lower than the drone are flown over; one obstacle can hide another behind it.
2. **Map.** Every hit becomes a red dot in an occupancy grid (20 cm squares). Cells within the drone's size plus a safety margin (0.7 m) of a hit are *blocked*. Hits are placed where the drone **thinks** it is, so with noisy sensors the map gets smeared, as on a real drone.
3. **Plan.** A* on that grid. Space it hasn't seen is assumed **free**: it heads straight for the goal until something is in the way (the "optimistic" approach real robots use).
4. **Replan.** Whenever the next 4 m of its path runs into newly blocked cells, it plans again from where it is. If something is suddenly close in the direction it's flying, it brakes first.

## Try it
- **Fly tab:** pick the **Hidden walls** course, planner **Lidar: sense & replan**, ▶ Fly mission. Watch the beams (red = hit), the red map growing, and the route changing. Tick **📡 Lidar** to see the scanner while you fly yourself.
- **Lab → experiment 7:** same course, planner *Lidar*. The map plot shows the red map, the plans it dropped (faint) and the path it flew.

## What we measured (Hidden walls: two walls, gaps at opposite ends, the second one 9.5 m away and hidden behind the first)
| Planner | Knows the obstacles? | Result |
|---|---|---|
| Straight lines | no (ignores them) | crashed into the first wall |
| Potential field | only nearby ones | stuck in front of the first wall |
| A* | **yes, all of them** | done: 23.9 m in 16.6 s |
| **Lidar** | **no: senses them** | **done: 25.7 m in 21.8 s**, 5 replans, 311 map cells |

The lidar planner flew about 2 m (8 %) further and took 5 s longer than A*: that's **the price of not having a map**. It also completes every other course with no collisions.

## Real drones
- Real obstacle avoidance uses lidar, stereo cameras or depth cameras, often all three. ArduPilot supports proximity sensors ("simple avoidance" brakes before obstacles, "BendyRuler" and "Dijkstra" route around them).
- Real maps are usually 3D (voxels), and they also have to *forget* things (a person who walked away).
- The hard part isn't the planning, it's trusting the sensors: reflections, glass, thin wires and sunlight all fool them.

## Self-check
1. Why does the lidar planner assume unseen space is free instead of blocked?
2. The second wall is 9.5 m away and the lidar reaches 8 m. Why does it still find the wall in time?
3. With noisy sensors (📡 Sensor noise on) the map looks smeared. Why?

## Answers
1. If unseen space counted as blocked, it couldn't plan anywhere at the start (it has seen almost nothing). Assuming free lets it head for the goal and fix the plan as it learns, which is fine as long as it can see far enough to stop in time.
2. It only needs to see an obstacle before it gets too close: at 1.5 m/s, 8 m of range is more than 5 s of warning. It replans the moment the wall appears in range.
3. Each hit is placed using the drone's *estimated* position. When the estimate wanders, the same wall gets drawn in slightly different places.
