# 09 · Experiment: Missions and obstacle avoidance

**Question:** how does a drone get from A to B on its own without hitting things?

## A mission
A list of **waypoints**: take off at the first, visit the rest in order, land at the last. The drone's position controller only knows "go to this point". The **mission follower** feeds it a moving target, a "carrot" that slides along the route at the cruise speed.

## Three ways past obstacles
- **Straight lines:** connect the waypoints. Zero thinking, so it crashes into anything in the way.
- **Potential field (reactive):** while flying, the goal **pulls** and nearby obstacles **push**. Simple, and works when obstacles are off to the side. But where pull and push cancel (a wall straight ahead, or a U-shaped trap), the drone stops: a **local minimum**. It can't see the way out because it only looks nearby.
- **A\* (planned):** before take-off, cover the map with a grid of small cells, block every cell too close to an obstacle (drone size + a safety margin), and search for the shortest chain of free cells. Then straighten the zig-zags into a few straight legs. It sees the whole map, so it finds a way if one exists.

## Planned vs flown
Even a perfect plan isn't followed perfectly:
- **Corners:** the drone can't turn instantly, so at speed it cuts or swings wide (see "max distance from the plan").
- **Wind:** a crosswind pushes it off the line. This is why planners keep a **safety margin**.

## What to look for
- **Map:** dashed yellow is the plan, solid is the real path. Faint outlines are the "keep-out" zones A* respects.
- **Clearance plot:** the gap between the drone's edge and the nearest obstacle. 0 means a collision.
- **Box canyon:** potential field (stuck) vs A* (around). **Patrol at 3 m/s:** corner cutting.

## Real drones
PX4 and ArduPilot fly waypoint missions exactly like this (take-off, waypoints, land). Real obstacle avoidance adds sensors (cameras, lidar) to *discover* obstacles. Here the map is known in advance.

## Self-check
1. Why does a potential field get stuck in a U-shaped trap, while A* doesn't?
2. A* keeps a safety margin. Why is that needed if the plan already avoids the obstacles?
3. Flying faster makes the mission shorter but the corner errors larger. Why?
