# 26 · Autonomous exploration: let the drone map an unknown place by itself

**Big idea:** until now the drone always had a goal: a waypoint, a field, a ring. Here it gets none. It's dropped into a building or a forest it has never seen, with only its lidar, and it must decide by itself where to go until the whole place is mapped. It does this without crashing, even when someone walks past, and even without GPS.

These are the ideas inside ROS 2's navigation stack (Nav2) and its SLAM packages (slam_toolbox), written in plain NumPy in `backend/autonomy/explore.py`, with no ROS install needed.

## 1 · A map that knows what it doesn't know
Each lidar beam says two things: *something is there* (where it stopped) and *nothing is in between* (every cell it passed through). A **log-odds grid** adds up that evidence for every 0.2 m cell, so each cell ends up **free**, **occupied** or still **unknown**. The lidar from lesson 15 only remembered hits, so it couldn't tell "empty" from "never looked".

## 2 · Frontiers: fly to the edge of the known
A **frontier** is a free cell next to an unknown one: the edge of what the drone has seen. The explorer:
1. groups frontier cells into clusters;
2. measures how far each cluster is by flying only through known-free space (Dijkstra), never through the unknown;
3. picks the best cluster, one that is near and long. It keeps its current choice unless another is clearly better; otherwise it flips between two similar ones and wastes time;
4. flies there, looks, and repeats.

When no frontier is left that it can reach, it's done.

## 3 · Local planner: how to move right now
The path says *where* to go; the local planner decides *how to move* this instant.
- **Chase the path + brake:** aim at a point 1.5 m ahead along the path, and stop if something is close in front. Simple, but aiming ahead cuts corners, and door frames and tree trunks are corners.
- **DWA (Dynamic Window Approach):** 10 times a second, it tries about 100 velocities the drone can reach within half a second. It imagines flying each for 1.5 s, throws out any that pass too close to what the lidar sees right now, and picks the best mix of progress, room and speed.
- **Moving things.** A wall is never seen *through*; the spot where a person now stands was seen empty many times before. So such hits are treated as moving: grouped, tracked from scan to scan, and given a velocity. DWA checks each imagined path against where the person **will be**, with a bigger margin (0.6 m). If nothing is safe, it doesn't freeze; it moves to wherever leaves the most room.

## 4 · Without GPS: SLAM
Indoors there is no GPS, and the EKF from lesson 24 can only dead-reckon. **Scan matching** fixes that. It tries small shifts and turns of each new scan against the map built so far, and keeps the one where the most beams land on walls already in the map. That position and heading go into the EKF like a GPS fix. Map and position are estimated together: **SLAM** (simultaneous localisation and mapping).

## What we measured (42 flights, `docs/results/exploration.md`)
| Setup | Building (4 rooms + hall) | Forest (22 trees) |
|---|---|---|
| DWA + GPS | 3/3 finished, 90–93 % mapped, 90 % in about 30 s | 3/3 finished, 99–100 % |
| Chase the path + brake | crashed or got stuck every time (54–65 % mapped) | crashed 6/6 |
| No GPS, motion sensors only | crashed 3/3 within 10 s (up to 8 m wrong) | crashed 3/3 (up to 21 m wrong) |
| **No GPS + lidar SLAM** | **3/3 finished, error 0.08 m mean, 0.14 m worst** | **3/3 finished, 0.17 m mean, 0.31 m worst** |
| Someone walking through (DWA) | never hit them | never hit them |

DWA never touched anything in 24 runs. The building's walls make scan matching sharper than the forest's thin tree trunks.

Honest limits:
- With someone walking around, *finishing* can take much longer. The person blocks doorways in the map, so frontiers behind them keep flickering between reachable and not.
- The world is flat: one height, a 2D lidar, and walls that stand straight up.

## Bugs we hit (each one is a real lesson)
1. **Doors too narrow for the plan.** A 1.6 m door with a 0.7 m keep-out on each side left 0.2 m, about one grid cell, and the explorer declared itself done at 75 %. Indoors needs a tighter margin (0.5 m), which DWA then enforces moment by moment.
2. **A map "in cells" is half a cell off.** Our walls sit exactly on grid lines, so every hit landed on a cell edge and the matcher pulled the drone 0.1 m sideways. Sharing each hit between 4 cells fixed a test but made real flights worse. What worked is the model ROS's AMCL uses: remember *where in the cell* the hits really were, and score each beam by its distance to that surface. Error dropped from 0.14 m to 0.08 m in the building, and from 0.38 m to 0.15 m in the forest.
3. **"Everything ahead is new" is not a reason to distrust a match.** Entering a new room, half the scan sees walls not mapped yet. Rejecting matches on the *average* score threw away good ones; counting how many beams land on *mapped* walls works.
4. **Freezing gets you hit.** A hovering drone that finds no safe move and stops gets walked into. It must step aside, and it must keep watching even while it is thinking where to go next.
5. **Walls aren't moving just because a beam grazed them.** A cell counts as "moving" only if it was seen empty far more often than it was hit.

## Try it
**Fly view → 🗺 Explore.** Pick the world, the local planner, GPS or not, tick 🚶 *someone walking*, and press ▶ Explore. The floor shows its own map filling in: dark = unknown, light = seen free, red = walls it mapped. The yellow line is its path to the next frontier (green ring). Orange = the person, where they *really* are; yellow rings with arrows = what the drone *thinks* is moving, and where it will be in a second. Any flying button takes over.

**Lab → experiment 13.** Pick a world, a local planner, GPS or not, and someone walking through. The map shows the real walls in grey, the map it built in red, where it flew, and where it *thought* it flew.

## Self-check
1. Why does the explorer plan only through cells known to be free, never through unknown ones?
2. Why does chasing a point 1.5 m ahead cut corners?
3. Scan matching builds the map from its own estimate and then uses that map to correct the estimate. Why doesn't that just go round in a circle?

## Answers
1. Unknown might be a wall. It flies to the *edge* of the unknown and looks before going in.
2. It steers straight at a point that is already round the corner, so it takes the shortcut across the inside of the bend.
3. It does drift slowly over long distances (that's why big SLAM systems look for *loop closures*, places seen before). But from moment to moment, the walls it saw a few seconds ago are a much better reference than integrating the accelerometer twice.
