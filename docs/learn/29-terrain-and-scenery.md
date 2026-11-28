# 29 · Real ground: hills, real places, and a world that looks like outside

**Big idea:** a real drone's "altitude" is its height above **where it took off**, measured by a barometer. It does *not* know how high the ground below it is. Over flat grass that makes no difference. Next to a hill, it is how drones crash.

## What's new in the 3D view
- **🌳 Realistic scenery** (button bottom-left of the 3D view; **▦ Simple** brings the grid back): grass, sky, fog, trees and hills on the horizon, and the sun's shadow under the drone. Everything is drawn from code, with no downloads, so it works offline. The trees on the horizon are only scenery: they are more than 95 m away, outside every course, and the drone can't hit them.
- **🏔 Terrain** (Engineer: next to 🗺 Course; Play: the **🏔 Hills** card): the ground has real height, *for the physics too*. The drone lands on a slope and can fly into a hillside.

## Altitude is measured from home
```
         drone holds "10 m"  ─────────────────────────╳  💥 hits the ridge (22 m)
 home ●──────────────────────────────╱‾‾‾‾‾‾╲
 (0 m)        flat valley          ╱  ridge  ╲
```
The Play screen shows both numbers when they differ: *"10 m up · 3.2 m above the ground ⚠ hill!"*. A real ground station shows the same pair: **Alt** (from home) and, if the drone has a downward rangefinder or terrain data, the height above the ground.

How real drones deal with it:
- **RTL_ALT_M** is set above the tallest thing around (lesson 28), because the drone comes home in a straight line at that height.
- **Terrain following:** ArduPilot can download terrain heights through the ground station (`TERRAIN_ENABLE`) and fly missions "10 m above the ground", or hold height with a **downward rangefinder** (`RNGFND1_*`, "surface tracking").
- **Plan from a map:** look at the elevations along the route before the flight.

## Fly over a real place
```
.venv/bin/python -m backend.terrain_fetch --lat -35.363261 --lon 149.165230 --name cmac --title "CMAC field"
```
This downloads the shape of the land once, from open elevation data (AWS Terrain Tiles), plus an open satellite picture (Sentinel-2 cloudless, 10 m pixels). The result is saved in `data/terrain/<name>/`, which is not uploaded to GitHub: a real place is often near someone's home. After that it works offline and appears in the 🏔 list and as a Play card. The example is CMAC, ArduPilot's own test field near Canberra, the default home of its simulator.

Options: `--size 400` (metres, square), `--cell 3` (grid spacing), `--no-imagery`, or your own picture: `--image field.jpg --image-width-m 300` (north up, centred on home: for example the map from your own drone, later).

Honest limits:
- **The elevation is the shape of the land, not what stands on it.** Outside the USA it comes from ~30 m data: no trees, hedges, power lines or houses. Never trust it for clearance; look at the real site.
- **10 m satellite pixels are blurry** close up: fields and roads, not single trees.
- The coordinates you type are sent to the two tile servers (as tile numbers). Nothing else is.
- Lidar, click-to-go and the exploration planner still think the ground is flat, so they can steer into a slope. Courses, races and exploration are on flat ground; picking one switches the terrain off.

## How it's built
- `backend/sim/terrain.py`: a height grid (bilinear lookup). The take-off spot is flattened to exactly 0 (a 3 m "mat", blended out to 6 m), so the drone always starts level.
- `QuadcopterPhysics.ground_height_fn`: the touch-down test is `z < ground(x, y)` instead of `z < 0` (None = the old flat world; every earlier test is unchanged).
- A touch-down faster than 2.5 m/s, away from the mat, counts as a crash: the motors stop.
- The 3D view draws the same grid, plus a "skirt" that carries the border heights outwards, which is exactly what the physics uses beyond the square.

## Self-check
1. The drone hovers at "10 m" and flies 80 m north in the Hill valley. What happens, and why doesn't the drone see it coming?
2. Why is the take-off spot flattened?
3. Why aren't the downloaded heights good enough to plan how close to fly to a tree line?

## Answers
1. It hits the slope about 50 m north, where the ground has risen to 10 m (the ridge top is 22 m). Its altitude is measured from home by the barometer; nothing measures the ground ahead or below.
2. So that z = 0 means "sitting at home", exactly like a barometer zeroed at take-off. Every controller and failsafe in the sim relies on that.
3. Trees aren't in the data (it's bare ground), and the grid is ~30 m: anything smaller is smoothed away.
