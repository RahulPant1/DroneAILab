# Autonomous exploration: 42 flights

DroneLab sim (`backend/autonomy/explore.py`), seeds 0, 1 and 2 of each setup, 240 s limit. Measured on truth: *mapped* = share of the real floor the map knows; *crash* = the real drone touched a wall, tree or the walking person (the flight ends). Position error = estimate vs truth. Re-run 2026-11-28 after two fixes: the building's right-hand door was split in half by a wall (too narrow to fly through), and the planner now prefers the middle of doors and corridors (a clearance costmap).

| World | Setup | Someone walking | Outcome (3 runs) | Mapped | Time to 90 % | Closest gap (all 3) | Position error mean / worst (worst run) |
|---|---|---|---|---|---|---|---|
| building | DWA, GPS | no | 3 finished | 99–99 % | 13–15 s | 0.19 m | 0.07 / 0.20 m |
| building | DWA, GPS | yes | 3 finished | 98–99 % | 14–17 s | 0.22 m | 0.06 / 0.20 m |
| building | Chase the path + brake, GPS | no | 3 finished | 99–99 % | 11–16 s | 0.21 m | 0.07 / 0.20 m |
| building | Chase the path + brake, GPS | yes | 2 crashed, 1 finished | 96–99 % | 10–12 s | -0.17 m | 0.08 / 0.20 m |
| building | DWA, no GPS (motion sensors only) | no | 3 crashed | 46–94 % | 9–9 s (1/3) | -0.30 m | 5.09 / 17.51 m |
| building | DWA, no GPS + lidar SLAM | no | 3 finished | 99–99 % | 14–15 s | 0.33 m | 0.10 / 0.23 m |
| building | DWA, no GPS + lidar SLAM | yes | 3 finished | 99–100 % | 15–15 s | 0.27 m | 0.10 / 0.17 m |
| forest | DWA, GPS | no | 3 finished | 99–100 % | 21–26 s | 0.20 m | 0.08 / 0.26 m |
| forest | DWA, GPS | yes | 2 finished, 1 crashed | 99–100 % | 21–26 s | -0.02 m | 0.07 / 0.26 m |
| forest | Chase the path + brake, GPS | no | 2 finished, 1 crashed | 87–100 % | 21–25 s (2/3) | -0.02 m | 0.08 / 0.20 m |
| forest | Chase the path + brake, GPS | yes | 1 crashed, 2 finished | 96–100 % | 18–25 s | -0.19 m | 0.07 / 0.20 m |
| forest | DWA, no GPS (motion sensors only) | no | 3 crashed | 41–62 % | – | -0.30 m | 7.67 / 19.60 m |
| forest | DWA, no GPS + lidar SLAM | no | 3 finished | 100–100 % | 23–27 s | 0.25 m | 0.37 / 0.81 m |
| forest | DWA, no GPS + lidar SLAM | yes | 3 finished | 100–100 % | 25–25 s | 0.09 m | 0.16 / 0.23 m |

- A closest gap below 0 is a crash (how far into the obstacle the drone's edge was when it ended).
- DWA (GPS or SLAM): 1 crash in 24 runs (a 0.02 m graze by the walking deer in the forest). Chase the path + brake: 4 crashes in 12 runs, mostly with something moving: braking can't get out of the way.
- Without GPS the EKF dead-reckons and is metres off within 10 s: the map it draws is wrong, and it flies into walls. Matching each scan against its own map (SLAM) keeps it within 0.23 m at worst in the building, 0.81 m in the forest (thin trunks give the matcher less to hold on to).

Regenerate: run every setup with `backend.autonomy.explore.run_explore(world, local, localisation, movers, seed)` (Lab experiment 13 runs one).
