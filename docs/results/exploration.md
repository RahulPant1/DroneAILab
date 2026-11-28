# Autonomous exploration: 42 flights

DroneLab sim (`backend/autonomy/explore.py`), seeds 0, 1 and 2 of each setup, 240 s limit. Measured on truth: *mapped* = share of the real floor the map knows; *crash* = the real drone touched a wall, tree or the walking person (the flight ends). Position error = estimate vs truth.

| World | Setup | Someone walking | Outcome (3 runs) | Mapped | Time to 90 % | Closest gap (all 3) | Position error mean / worst (worst run) |
|---|---|---|---|---|---|---|---|
| building | DWA, GPS | no | 3 finished | 90–93 % | 28–30 s | 0.21 m | 0.07 / 0.20 m |
| building | DWA, GPS | yes | 2 finished, 1 out of time | 90–94 % | 30–45 s | 0.15 m | 0.07 / 0.26 m |
| building | Chase the path + brake, GPS | no | 2 crashed, 1 out of time | 57–62 % | – | -0.06 m | 0.07 / 0.26 m |
| building | Chase the path + brake, GPS | yes | 2 crashed, 1 out of time | 54–65 % | – | -0.08 m | 0.07 / 0.26 m |
| building | DWA, no GPS (motion sensors only) | no | 3 crashed | 52–77 % | – | -0.30 m | 2.08 / 8.07 m |
| building | DWA, no GPS + lidar SLAM | no | 3 finished | 92–94 % | 20–31 s | 0.20 m | 0.08 / 0.14 m |
| building | DWA, no GPS + lidar SLAM | yes | 2 finished, 1 out of time | 84–95 % | 28–41 s (2/3) | 0.05 m | 0.07 / 0.15 m |
| forest | DWA, GPS | no | 3 finished | 99–100 % | 26–31 s | 0.13 m | 0.08 / 0.26 m |
| forest | DWA, GPS | yes | 3 finished | 99–100 % | 26–33 s | 0.13 m | 0.07 / 0.26 m |
| forest | Chase the path + brake, GPS | no | 3 crashed | 41–76 % | – | -0.03 m | 0.06 / 0.20 m |
| forest | Chase the path + brake, GPS | yes | 3 crashed | 41–65 % | – | -0.24 m | 0.06 / 0.20 m |
| forest | DWA, no GPS (motion sensors only) | no | 3 crashed | 36–84 % | – | -0.30 m | 5.04 / 20.71 m |
| forest | DWA, no GPS + lidar SLAM | no | 3 finished | 100–100 % | 28–44 s | 0.22 m | 0.17 / 0.31 m |
| forest | DWA, no GPS + lidar SLAM | yes | 3 finished | 99–100 % | 23–28 s | 0.15 m | 0.16 / 0.26 m |

- A closest gap below 0 is a crash (negative = how far into the obstacle the drone's edge was when it ended).
- Chasing a point 1.5 m along the path cuts corners at door frames and trees; DWA looks at the lidar before every move and never touched anything in these 24 runs.
- Without GPS the EKF dead-reckons and is metres off within 10 s: the map it draws is wrong, and it flies into walls. Matching each scan against its own map (SLAM) keeps it within 0.31 m at worst.
- With someone walking around, finishing can take much longer (two building runs ran 210–240 s): the person keeps blocking doorways in the map, so frontiers behind them flicker between reachable and not.

Regenerate: run every setup with `backend.autonomy.explore.run_explore(world, local, localisation, movers, seed)` (Lab experiment 13 runs one).
