# DroneLab progress

## Phase 0: Environment check (done 2026-11-19)

**Machine:** Ubuntu 22.04 on WSL2 3.0.5 (mirrored networking), 4 CPUs, 9 GB RAM cap, **swap 0**
(the user chose to keep this). RTX 4060 8 GB, with ~1.1 GB already used by ClipForge when it runs. 794 GB disk free.

**Tools reused:** uv 0.8.3 (Python 3.12.11 already downloaded), Node v23.7.0 via NVM 0.39.5, git 2.34.1
(global identity already set), Ollama 0.40.3 at `http://localhost:11434`.

**Decisions:**
| Topic | Choice |
|---|---|
| Project path | `~/DroneAILab` (not `~/projects/dronelab`) |
| Python env | `uv venv --python 3.12 .venv` (no conda env) |
| Ports | backend 8001, frontend 5174 (8000/5173 = ClipForge) |
| LLM | `gemma4:e4b-it-q4_K_M`, `think: false`; fallback `qwen3.5:4b` only if it doesn't fit (ask first) |
| Extra deps approved | `websockets`; frontend `react-dom` + Vite/TypeScript build tools |
| Download caches | shared `~/.cache/uv`, `~/.npm` (not removed on uninstall) |
| `.wslconfig` | unchanged (9 GB, swap 0) |

**Done:** updated the docs to match the machine; created `.env`; created `.venv` (Python 3.12.11)
with the pinned deps in `backend/requirements.txt`; `git init`.

**Verified:** all deps import; uvicorn has WebSocket support; Ollama answers over HTTP and the
model tag exists; `ollama ps` is empty (no model loaded).

**Footprint:** project 83 MB (all of it `.venv`).

## Phase 1a: Headless physics core (done 2026-11-19)

**Built:** `backend/sim/` (`config`, `frames`, `physics`, `control`, `runner`, `metrics`), `backend/cli.py`,
23 tests in `backend/tests/`, lesson `docs/learn/01-thrust-and-hover.md`, `README.md`.

**Acceptance:** all pass (`.venv/bin/pytest` → 23 passed in ~2.3 s).
| Requirement | Result |
|---|---|
| Hover at 2.0 m, ±0.05 m from 5 to 10 s | max error 0.014 m (overshoot 0.8 %, settled at 2.4 s) |
| T/W < 1 never leaves the ground + explains why | yes (rule-based summary) |
| Doubling the mass raises hover % | 0.6 → 1.2 kg: ratio 2.00 |
| Same seed gives identical telemetry | yes |
| Free fall vs ½·g·t² within 1 % | ≤ 0.1 % at 2 s, max abs error 2 cm (from t ≥ 0.25 s; see the test docstring) |
| 20 s headless run < 3 s | ~0.3 s (120 s run: 1.9 s) |

**Footprint:** peak RAM of a run ~39 MB; project 86 MB on disk.

**Design notes:**
- Motor % maps to thrust linearly (speed = max · √%), so hover % = 100 / (T/W).
- The mixer keeps attitude torques when a motor saturates by shifting the collective.
- The altitude PID uses an integral zone (0.3 m) to stop windup on take-off (the overshoot was 8 % without it).
- The controller sees true state for now (no sensors/estimator until Phase 2).

**Known limitations:** no position control yet (Phase 1b); simple ground contact (no tipping, drone is
levelled on touchdown); a 120 s run's JSON is ~1.1 MB. Floats should be rounded before saving runs in Phase 2.

## Phase 1b: Fly it in the browser (done 2026-11-19)

**Built:**
- Backend: position control (`control.py`), `pilot.py` (take-off/hover/land modes + keyboard sticks),
  `RealtimeSim` (wall-clock paced, pause, 30 Hz snapshots) in `runner.py`, `api.py` (FastAPI + `/ws/sim`).
- Frontend (React 19 + Vite 8 + TS 7 + r3f/drei, hand-written): header with Sim/AI status, Drone Studio
  (presets, 5 key params with plain-English help, live T/W, Advanced = more physics + all PID gains),
  a real-scale 3D drone, ENU axes, grid, trail, target marker, ground shadow, camera follow, on-demand rendering
  (~30 fps, none while the tab is hidden), telemetry (position/target/velocity, roll/pitch/yaw in °, motor bars),
  buttons, WASD/RF/QE keys, "fly to x/y/alt", and a lesson viewer.
- `run.sh`: reads `.env` safely, picks free ports, binds 127.0.0.1, and Ctrl+C stops everything it started.
- Lesson `docs/learn/02-roll-pitch-yaw.md`. 17 new tests (40 total).

**Acceptance:**
| Requirement | Result |
|---|---|
| First flight within 2 clicks | 1 click (Take off); the sim auto-connects on page load |
| UI smooth | renders only on new data (~30 fps); telemetry text at 10 Hz |
| Backend < 500 MB RAM | 64 MB |
| Total app < 1.5 GB | ~545 MB (Vite dev server ~480 MB of that) |
| Works with Ollama stopped | sim never calls Ollama; health shows "AI offline" (tested with an unreachable URL) |
| Ctrl+C leaves no processes | verified, both during start-up and after |

**Fixes found while testing:**
- Heavy preset crashed on a 90° turn: yaw now gets only leftover thrust, and tilt torque requests are capped
  to the motors' real authority.
- `run.sh` hung for up to 20 s if Ctrl+C came during start-up: the cleanup handler now exits.
- React 19 unmount warnings from drei `<Html>` labels: replaced with canvas sprites.

**Footprint:** project 347 MB on disk (node_modules 241 MB, .venv 83 MB).

**Not verified by me:** how the page looks and feels in a real browser (I had no browser to look at it).
I tested everything through the Vite proxy with a WebSocket client, plus the type-check and build.

**Known limitations:** the Vite dev server costs ~480 MB RAM; the trail keeps the last ~1500 points;
no live plots yet (Phase 2); a sim restarts when the page reloads or the backend restarts.

## Phase 2: Engineering Lab (done 2026-11-19)

**Built:**
- Sim: `environment.py` (steady wind + seeded gusts, drag uses airspeed, hidden payload), `sensors.py`
  (gyro/accel 250 Hz, altimeter + compass 50 Hz, GPS 10 Hz, optional gyro bias), `estimator.py`
  (complementary filters: Mahony-style tilt, alpha-beta position/altitude). With sensors off the controller sees the truth.
- `backend/experiments/`: 6 experiments, each with an objective, a 3-line concept, 2–4 validated sliders,
  a headless run, plots, a rule-based summary and one-click "try next" ideas.
- `backend/storage.py`: JSON runs + index, 100-run cap, delete-old, compare. Saves re-run on the server (genuine data).
- API: `/api/experiments`, `/api/experiments/{id}/run`, `/api/runs` (list/save/get/delete/delete-old/compare).
- Frontend: Lab tab, History tab (view, compare 2 runs with A solid / B dashed overlays + metric diff, delete),
  uPlot charts with units and a "how to read" line, CSV/JSON export.
- Lessons `docs/learn/03`–`08` (one per experiment). 24 new tests (64 total).

**Acceptance:**
| Requirement | Result |
|---|---|
| Too-high Kp visibly oscillates | Kp 25: 64 % overshoot, 6 target crossings |
| Kd = 0 overshoots more than the default | 104 % vs 0.6 % |
| Wind: measurable drift, controller recovers | 5 m/s: 0.39 m drift, back within 10 cm after 7.3 s (offset 0.5 cm) |
| Noise: visible truth vs estimate gap | yes (altitude, x, roll plots); the estimate removes ~75 % of altimeter noise |
| Save → reload → compare | verified via API, storage tests and end-to-end through run.sh |

**Design changes found while testing (documented in code):**
- Tilt-correction default 1.0 → 0.2: the accelerometer "lies" during accelerations, and at 1.0 the drone wandered 3 m.
- Altitude I-term: replaced the 0.3 m integral zone with a speed gate (|vz| < 0.5 m/s), so Ki can fix a hidden payload.
- PID I-limits are now "max output from the I-term" (independent of Ki).
- Roll/pitch motor split is reported at its peak (averaging hid it).

**Footprint:** backend 72 MB, Vite dev 533 MB, total ~605 MB RAM. Each run takes 0.3–0.7 s CPU and 47–268 KB.
Project 379 MB on disk.

**Not verified by me:** the look of the Lab/History pages and charts in a real browser.

**Known limitations:** the Vite dev server is the biggest RAM user; the estimator is a teaching-grade complementary
filter (not an EKF); live flight (Fly tab) used perfect sensors and no wind (both were added later as toggles).

## Phase 3: Local LLM assistant (done 2026-11-19)

**Built:**
- `backend/ai/ollama_client.py`: HTTP only; per request `keep_alive 2m`, `num_ctx 8192`, `temperature 0.3`,
  `think false`; readable offline/error messages.
- `backend/ai/prompts.py`: system prompts ([Measured]/[Hypothesis], never invent numbers, "I don't know"),
  a telemetry-free compact run summary (≤ 2 runs, trimmed to ≤ 2k tokens), and the experiment catalogue for suggestions.
- `backend/ai/schemas.py`: JSON schema for Ollama `format`, plus pydantic shape check and the same range checks
  as the UI, giving readable rejections.
- `backend/ai/assistant.py`: ask/suggest, a number check that flags numbers not found in the run data,
  LaTeX tidy-up, and a JSONL log (rotates at 5 MB).
- API: `POST /api/assistant/ask`, `POST /api/assistant/suggest`. The run context is rebuilt server-side
  from saved ids or (experiment, params, seed). One LLM request at a time.
- Frontend: Assistant side panel on every tab (shows exactly which runs the AI sees), claim badges,
  unverified-number warnings, a suggestion card with a diff and "Apply & run in Lab" (nothing runs without a click).
- 26 new tests against a fake Ollama (90 total).

**Acceptance (live, real model `gemma4:e4b-it-q4_K_M`):**
| Requirement | Result |
|---|---|
| Malformed / out-of-range AI configs rejected readably | 10 rejection cases tested (bad JSON, wrong shape, unknown experiment/setting, out of range, no-op, too many changes) |
| Answers quote real metrics | "[Measured] overshoot 64.27 % and oscillations 6" (exact run values); 0 unverified numbers in 3 live answers |
| Model unloads after ~2 min idle | unloaded 113 s after the last request (`ollama ps` empty; GPU back to 1.3 GB) |
| App fully usable with Ollama off | tested with an unreachable Ollama: assistant → "AI offline" (503), the Lab still runs |

**Live performance:** first answer 58 s (includes the model load), then 2–17 s. 100 % GPU at num_ctx 8192
(no CPU split, so no need for 4096 or the fallback model). RAM available stayed ≥ 4.7 GB while loaded.

**Known limitations:** the model sometimes suggests a different experiment than the one discussed (it's still
validated and safe); the first answer after idle waits for the model to load; no streaming (the answer appears all at once).

## Extra: `./run.sh --lite` (2026-11-19)
Builds the frontend only when its files changed, then one uvicorn process serves the page, API and WebSocket on
the frontend port (5174). RAM: ~70 MB vs ~280–530 MB in dev mode. Verified: first start builds (2.5 s), later starts
skip the build (0.6 s), a source change triggers a rebuild, live flight and Lab work, Ctrl+C leaves nothing running.

## Phase 4: Basic autonomy (done 2026-11-19)

**Built:**
- `backend/autonomy/`: `world.py` (box obstacles, drone footprint 0.3 m, clearance), `planner.py` (straight,
  A* on a 0.2 m grid with a 0.4 m safety margin + line-of-sight shortcutting, readable "no path" errors),
  `follower.py` (take-off → carrot along the route / potential field → land; collision, stuck and waypoint events),
  `mission.py` (4 scenarios, custom-mission validation, metrics).
- Mission metrics: completed, collisions, waypoints reached, mission time, planned vs flown length,
  cross-track RMS/max, closest approach to obstacles and to each waypoint, A* cells and planning time.
- Lab experiment 7 (scenario, planner, cruise speed, crosswind) with an SVG top-down map (planned vs flown,
  keep-out zones, crash/stuck markers); it works in History compare (A/B paths), export, and the AI assistant.
- API: `GET /api/missions/scenarios`, `POST /api/missions/run` (custom missions). CLI: `mission` with an ASCII map.
- Lesson `docs/learn/09-exp-missions.md`. 30 new tests (120 total).

**Results (each mission 0.2–0.6 s CPU):**
| Scenario | Straight | Potential field | A* |
|---|---|---|---|
| Around the block | collision | completes (0.1 m clearance) | completes (0.39 m) |
| Slalom | collision | stuck | completes |
| Box canyon (U trap) | collision | stuck (local minimum) | completes |
| Square patrol | completes | completes | completes |
Faster flight cuts corners: waypoint miss 0.18 m at 0.8 m/s vs 0.55 m at 3 m/s. A 6 m/s crosswind reduces the slalom clearance 0.38 → 0.25 m.

**Fixes found while testing:** stuck detection wasn't reset per waypoint; fast flights never "reached" a cut corner
(waypoints now count as passed, and the miss is measured); the head-on "around the block" layout trapped the
potential field (made the approach angled); a NaN metric in early-ending runs.

**Not verified by me:** the look of the map in a real browser.

**Limitations:** 2D planning at one cruise altitude (low boxes can be flown over, but there's no climbing over tall
ones); the obstacles are known in advance (no sensing).

## Extra: missions in the live 3D Fly view (done 2026-11-19)
- `backend/autonomy/live.py` `MissionSim`: the live sim plus an optional course. WebSocket commands
  `{"cmd": "course", "scenario": id|null}` and `{"cmd": "mission", "planner", "cruise_speed_m_s"}`.
  The same MissionFollower as the Lab flies it in real time; when it lands, crashes or gets stuck, control
  returns to the user (stuck = hover). Hover / Land / Fly-to take over mid-mission.
- Manual flight on a course: touching a box stops the motors; sideways motion stops at the wall so the drone
  drops beside it (a crude impact model, live view only; Lab metrics are unchanged).
- 3D: obstacles (translucent boxes), keep-out outlines on the ground, waypoint rings at cruise height
  (green done / yellow next), dashed planned route, camera framed on the course when it loads.
- Live outcomes match the Lab matrix for all 12 scenario/planner pairs (tests). A mission costs ~2.5 % of one core.
- Tests: 137 pass (+17 in `test_live_missions.py`). **Not verified by me:** the look in a real browser.

## Extra: wind in the live view (done 2026-11-19)
- WebSocket `{"cmd": "wind", "speed_m_s", "direction_deg", "gust_m_s"}` applies at once, mid-flight
  (`LightweightSim.set_env`); the wind is kept through Reset and new missions. State frames carry `wind_m_s`.
- UI: 🌬 Wind sliders (0–12 m/s, direction in 15° steps, gusts 0–4 m/s, Calm button), a wind arrow above the
  drone (live, including gusts), and a wind line in Telemetry. Settings are re-sent if the connection drops.
- Live check: slalom with A*, crosswind north: min gap 0.38 m (calm), 0.25 m (6 m/s, same as the Lab), 0.09 m (10 m/s).
- Tests: 140 pass (+3). **Not verified by me:** the look in a real browser.

## Extra: sensor noise toggle in the live view (done 2026-11-19)
- WebSocket `{"cmd": "sensors", "enabled", "noise_scale"}` switches noisy sensors + estimator on/off mid-flight
  (`LightweightSim.set_sensors`; a new estimator starts at the true state). Kept through Reset. Off by default,
  so the live view is unchanged unless you tick it. State frames carry `est_pos_m` / `est_quat`.
- The controller flies on the estimate; collision checks use the truth (the mission follower and the buttons were
  moved to the estimate afterwards, see below).
- UI: 📡 Sensor noise checkbox + level (0.5–3×), pink ghost outline at the estimated position/tilt, "estimate off by" line.
- Hover at 2 m, 15 s: wander std 0.15 m (1×) / 0.4 m (3×); estimate error mean 0.16 m / 0.47 m (max 1.1 m at 3×).
  A* still completes the slalom at 3×.
- Tests: 143 pass (+3). **Not verified by me:** the look in a real browser.

## Extra: the mission autopilot navigates on the estimate (done 2026-11-19)
- `MissionFollower.setpoint_for(estimate, t, truth)`: steering, waypoint and stuck decisions use the estimate;
  obstacle contact and ground contact use the truth (the estimator doesn't track ground contact, so using it
  would cut the motors at altitude). With sensors off the two are identical, so Lab results are unchanged.
- "Passed waypoint" now needs the distance to grow by 0.3 m (was 0.05 m): with noise, estimate jitter alone
  was triggering it (patrol at 1x: "missed by 1.27 m" for a waypoint it actually reached).
- Live messages with noise on: "Reached waypoint 1, by its estimate. Really 0.65 m away."
- A* over 5 seeds: patrol lands 5/5 at 0/1/3x; slalom lands 5/5 at 1x (gap 0.23–0.48 m) but **crashes 3/5 at 3x**:
  the estimate error then exceeds the 0.4 m safety margin.
- Tests: 145 pass (+2).

## Extra: the buttons act on the estimate too (done 2026-11-19)
- Take off / Hover here / Land / Fly to / keyboard nudges now use the estimate (`Pilot.update(estimate, dt, truth)`;
  `RealtimeSim.command` passes `sim.estimate`). Only ground contact (landing → motors off) uses the truth.
  With sensors off nothing changes. Now the whole autopilot (controller, buttons, missions) flies on the estimate;
  only physics facts (obstacle contact, ground contact) come from the truth.
- Measured (3 seeds): Fly-to true miss 0.01 m (perfect) / 0.12–0.45 m (1x) / 0.41–0.95 m (3x). Every landing ends
  on the ground with motors off; landing takes 5–6 s instead of 4.5 s (altitude estimate is a bit off).
  The estimate drifts on the ground before take-off (up to ~0.7 m at 3x), so take-off aims at the drifted spot.
- Tests: 146 pass (+1).

## Extra: GPS settle before arming (done 2026-11-19)
- Live view only (`LightweightSim.ground_align`; Lab runs unchanged). With sensor noise on, while the drone is
  disarmed on the ground the estimator knows it is still: position = average of all GPS fixes (and altimeter
  readings) so far, velocity = 0. Take off is refused until 5 s of fixes (50) are averaged; Fly mission waits in a
  "gps" phase. No wait with perfect sensors, when noise is switched on in the air, or after a landing.
- Estimate error at take-off: before, up to ~0.7 m at 3x (drifted filter); now 0.15 m at 3x, 0.05 m at 1x
  (single fix 1.6 m / 0.54 m; error shrinks about 1/sqrt(fixes)). In-flight error is unchanged, so Fly-to
  misses are still 0.15–0.25 m (1x) / 0.45–0.8 m (3x): settling fixes the start, not the flight.
- UI: Take off button shows "🛰 GPS settling 3.2 s"; Telemetry and mission status show it too.
- Tests: 149 pass (+3; three older noisy tests now wait for the settle).

## Extra: GPS noise toggle in the live view (done 2026-11-19)
- `SensorConfig.gps_noise_scale` (default 1.0, so the Lab is unchanged); WS `sensors` takes `gps_noise: true|false`.
  UI: "🛰 GPS noise" checkbox next to the sensor noise level (only GPS goes perfect; other sensors stay noisy).
- Hover at 2 m, 3 seeds: horizontal wander 0.18 → 0.03 m (1x) and 0.50 → 0.10 m (3x) when GPS noise is off;
  estimate error 0.16 → 0.01 m / 0.48 → 0.04 m. Vertical wander unchanged (altimeter). Slalom at 3x: 3/5 crashes
  with GPS noise, 0/5 without. GPS noise is most of the sideways drift.
- Tests: 151 pass (+2).

## Extra: GPS outage toggle + GPS-loss failsafe (done 2026-11-19)
- WS `{"cmd": "gps", "outage": bool, "failsafe": bool}` (live view, matters only with sensor noise on). Outage =
  no fixes (`Sensors.gps_ok`); position comes from dead reckoning on the motion sensors. Kept across resets.
- Failsafe (default on): GPS lost ≥ 1 s while airborne -> hold level + altitude (attitude mode, no position hold),
  like PX4's altitude-mode fallback. A running mission is aborted. When GPS returns, position hold resumes.
  On the ground with no GPS, take-off is refused ("No GPS fix").
- Measured, hover at 2 m, 1x noise, 30 s outage (3 seeds): no failsafe 2.8–29 m true drift (grows with t²);
  failsafe 2.5–4.3 m. At 3x: 35–100 m vs 6.6–18 m. The failsafe still drifts because "level" is level by the
  estimate (small tilt errors). After GPS returns the drone flies back within ~10 s.
- UI: 📵 GPS outage + failsafe checkboxes, Take off shows "📵 No GPS fix", Telemetry "GPS LOST n s" /
  "FAILSAFE", mode shows "failsafe". State frames carry `gps_outage`, `gps_lost_s`, `failsafe`.
- Tests: 155 pass (+4).

## Extra: W/A/S/D tilt the drone in failsafe (done 2026-11-19)
- In the GPS-loss failsafe (attitude mode), W/S = pitch ±8°, A/D = roll ∓8° (`Pilot.tilt_from_sticks`); let go
  and it levels out but keeps coasting, so you counter-steer to stop. R/F and Q/E work as before.
- The x/y target is frozen in failsafe (the 2 m stick leash used to drag it along with the drifting estimate), so
  when GPS returns the drone flies back to it (checked: 27 m away -> 0.37 m after 15 s).
- Tilt lowered 12° -> 8°, plus extra failsafe-only air drag (`FAILSAFE_EXTRA_DRAG_N_S_M = 0.4`, the drone's own is
  0.10; `QuadcopterPhysics.extra_drag_n_s_m`, 0 outside failsafe). A 1.5 s W press + 2 s coast: 7.9 m (12°) ->
  5.3 m (8°) -> 3.3 m (8° + drag); holding W: top speed 7.2 -> 2.7 m/s; it stops by itself within ~6 s.
- Drag slider (0–1 N·s/m, default 0.4) next to the failsafe box; WS `gps` takes `drag_n_s_m`; kept across resets.
  0 = real physics (still coasting > 1 m/s 6 s after letting go).
- Tilt slider (2–20°, default 8°) beside it; WS `gps` takes `tilt_deg`; `Pilot.tilt_from_sticks(max_tilt_rad)`;
  kept across resets.
- Directions checked at headings 0° and 90° (W = nose direction, A = its left). The keys hint under the 3D view
  switches to the tilt meaning in failsafe.
- Tests: 158 pass (+3).

## Next: Roadmap items (PX4/Gazebo, ROS 2, vision, …) only with explicit approval.

## Extra: learner review fixes (done 2026-11-19)
Reviewed the app as a beginner would (a full Fly-view session over the WebSocket, plus the screens and lessons). All flows worked; fixed:
- Failsafe key hint was wrong at drag 0 (always said "slowed by drag"); it now reads the server's `failsafe_drag_n_s_m` /
  `failsafe_tilt_deg` (new snapshot fields) and shows the tilt.
- Telemetry shows "Target (frozen)" in failsafe (the controller isn't flying to it; it returns there when GPS is back).
- Sensor bar split in two rows; failsafe + drag + tilt boxed together; a visible line says why controls are greyed out.
- 📖 Learn links in the Fly view for wind (07), sensors/GPS/failsafe (08) and missions (09).
- Lesson 08 "Try it live" rewritten as numbered steps with "what you should see"; 3 new self-check questions; 1x -> 1×.
- Every lesson has an `## Answers` section; the lesson renderer folds it behind "Show answers".
- Lesson renderer fixes: `###` headings and *italic* used to show raw; escaped `A\*` now works.
- Convention notes: wind "toward, maths angle" vs weather-report "from, compass bearing" (lesson 07); yaw vs compass
  heading and NED (lesson 02).

## Extra: rings race game (done 2026-11-19)
- `backend/games/race.py`: tracks (first_flight, up_and_down, zig_zag, square_loop), upright rings (radius 0.9 m),
  pass = the TRUE centre crosses the ring plane in the arrow direction within the radius, in order. Clock starts at
  take-off (armed), stops at the last ring. Best times in `data/race_best.json` (gitignored); tests use a temp file.
- Live: `MissionSim.load_race` (WS `race` cmd, reply carries `race` info); a race and a course replace each other;
  Reset restarts the race. Snapshot `race` status. `GET /api/race/tracks`.
- UI: 🏁 Race bar (track list with best times, Race again, tip), hoops in 3D (yellow glowing = next, green = done,
  arrow cone, number, pole), start pad, big timer top-right. Lesson 10 "Racing tips" (kid-friendly, with answers).
- Every track is finished by a simple "aim at each ring" autopilot in the tests: 5.2 / 7.1 / 8.1 / 8.8 s.
- Tests: +8 (170).

## Extra: multi-drone formation flight (done 2026-11-20)
- `backend/autonomy/formation.py`: up to 4 wingmen, each its own LightweightSim + Pilot (perfect sensors, own gusts).
  Slots (V / line / column, spacing 1.5–4 m) in the leader's frame, around the position the leader BROADCASTS (its
  estimate). Shape heading = leader's nose; on a mission (autopilot doesn't turn the nose) = direction of travel,
  turning <= 60°/s. Wingmen copy the leader's phase (take off / fly / land).
- Avoidance (one switch): look 1 s ahead when closing > 0.3 m/s (or < 1.2 m now), push away + sidestep + vertical
  dodge 2 m (vs the leader: odd/even split; between wingmen: lower number climbs). Obstacles: if the way to its slot
  is blocked, a wingman breaks formation and follows the leader's trail (queue in current order, 1 m carrot), plus
  a push away from nearby boxes. Collisions (< 0.6 m centre to centre, truth) stop both drones (incl. yours).
- Measured: 24 drone-drone stress cases (backing into a column, shape switches, calm / 6 m/s wind, spacing 1.5–4):
  6 crashes off, 0 on. 24 A* missions (4 courses x 3 shapes x 2/4 wingmen): crashes in 14 off, 1 on (line of 5 in
  box canyon). GPS outage: wingmen follow the leader's drifting estimate (10 m off after 30 s with failsafe).
  CPU: ~170 ms per simulated second with 4 wingmen (~17 % of one core).
- Things that didn't work (kept here so we don't retry them): a trail-following column (wingmen fly through the
  leader when it reverses); 1 m minimum spacing (steady formations sat inside the avoidance zone and kept dodging);
  odd/even vertical dodging between wingmen (same-side wingmen dodged into each other).
- UI: 🛩 Wingmen bar (count, shape, spacing, 🛡 avoid crashes), coloured numbered wingmen in 3D (grey = crashed),
  Telemetry "closest gap". Lesson 11. WS `formation` cmd; snapshot `formation`.
- Tests: +7 (177).


## Phase 5 steps 1–2: ArduPilot SITL (done 2026-11-22, user-approved; steps 3–4 need a separate OK)
- PRD updated: Phase 5 section (footprint exception for `external/`), §7 allows MAVLink only to a SIMULATED
  autopilot on 127.0.0.1. Roadmap line no longer lists multi-drone (done).
- `scripts/sitl_setup.sh` (re-runnable, no sudo/apt; does NOT run ArduPilot's install-prereqs): shallow clone of
  Copter-4.7.3 into `external/ardupilot`, only the SITL submodules (waf, mavlink, lwip, littlefs, DroneCAN x4),
  applies `scripts/patches/ardupilot-sitl-loopback.patch`, `waf configure --board sitl && waf copter -j3`.
  Measured: build ~3 min total on 4 cores, peak ~280 MB RAM for the waf process; `external/` = 965 MB.
- Loopback patch: upstream SITL listens on 0.0.0.0 (TCP 5760/5762/5763 in UARTDriver, UDP 5501 RC input in
  AP_RCProtocol_UDP). Patched both to 127.0.0.1; verified with `ss` mid-flight.
- Python extras (`backend/requirements-sitl.txt`): empy 3.3.4, pexpect, pymavlink, dronecan, lxml, pyserial,
  fastcrc; MAVProxy 1.8.75 with `--no-deps` (its full deps pull opencv, matplotlib, vtk: not needed for the text
  ground station; harmless "No module named 'PIL'" warning).
- `backend/sitl/launch.py`: start/stop arducopter (own process group, free instance, ready = "Waiting for
  connection" in its log; we don't poke the port because SITL takes the first connection as its ground station).
- `backend/sitl/fly.py` (pymavlink): pre-arm wait, GUIDED arm + takeoff 10 m, fly-to 20 m N (ENU -> NED), upload a
  4-WP square mission (MISSION_COUNT / MISSION_REQUEST_INT / MISSION_ITEM_INT), AUTO, RTL, report.
  Measured (speedup 5): armed at ~46 s, fly-to error 0.03 m, corners 0.03 / 1.81 / 1.80 / 0.23 m (WP_RADIUS_M = 2 m,
  it turns early), landed 0.02 m from home. RTL skips its 15 m climb when already near home (RTL cone).
- `scripts/sitl.sh` -> `backend/sitl/ground_station.py`: SITL + MAVProxy in the terminal; verified guided / arm /
  takeoff / fly-to / RTL by piping commands. Ctrl+C quits (MAVProxy's `exit` needs `requireexit`).
- Bugs found and fixed: (1) waiting with recv_match(type=X) dropped position messages, so corner errors were
  measured from a stale position (20 / 28 m); now one reader records acks and "reached" events. (2) MAVProxy forks a
  helper; stopping only the parent left it running: `stop_tree` stops MAVProxy's whole process tree.
- Tests: +3 unit (ENU->NED, lat/lon offsets, square mission) + 1 opt-in real flight (`DRONELAB_SITL=1`, ~1 min).
  180 passed, 1 skipped. Lesson 12 (terminal-based; not linked from the UI yet).

## Phase 5 step 4: ArduPilot in DroneLab's Fly view (done 2026-11-22)
- `fly.Flight` rewritten around a background MAVLink reader thread (state under a lock; acks, "reached" and
  mission-protocol messages recorded by the reader; `cancel` event so a new panel command replaces the old one).
- `backend/sitl/bridge.py` (`BRIDGE`, one per server): start/stop SITL (server's process group, so run.sh's stop
  kills it; plus atexit), actions in a worker thread: take-off, fly-to (relative to ArduPilot's pad), hover
  (GUIDED hold: LOITER would need RC throttle), square mission (AUTO), RTL, land, compare. Status in every WS frame
  (`sitl`): mode, armed, GPS, EKF, ENU pos on its pad (4 m south of DroneLab's), quat (NED->ENU: roll same, pitch
  negated, yaw = 90° - heading), motors from SERVO_OUTPUT_RAW, recent STATUSTEXT.
- Compare: same "fly 10 m north" from a 5 m hover. ArduPilot rise 2.2 s / overshoot 0.19 m / settle 4.0 s / top
  4.5 m/s; DroneLab 2.8 s / 0.10 m / 4.2 s / 3.0 m/s.
- UI: 🛰 ArduPilot panel (SitlPanel.tsx, own uPlot chart), orange drone + pad + trail in Scene.
- Bugs found: take-off right after arming is sometimes refused (retry up to 5x); repeated timestamps made the speed
  NaN (dedupe); demo test flaky at 5x speed (position samples ~10/real s => corner error up to 2 m + 0.5 s of
  flight): run at 3x, limit 3.5 m.
- Verified: whole panel flow over the real WebSocket; `./run.sh --lite` serves the panel; stopping run.sh (Ctrl+C)
  stops SITL; stopping uvicorn alone stops it (atexit). NOT verified in a browser.
- Tests: +6 offline, +1 opt-in real (186 passed, 2 skipped; with DRONELAB_SITL=1 all 11 SITL tests pass, twice).

## Phase 5 step 3: QGroundControl (done 2026-11-22)
- 5.1.5 AppImage (sha256 verified) needs glibc 2.38 / GLIBCXX_3.4.32: Ubuntu 22.04 has 2.35. Deleted.
- 4.4.5 AppImage (glibc 2.35 OK; GitHub publishes no checksum for that old release) unpacked in `external/qgc/`;
  we run the binary directly (its AppRun/start script copies an icon into ~/.icons). It needed libgstgl-1.0,
  libgstvideo-1.0 and libpulse-mainloop-glib: the user ran `sudo apt install libgstreamer-gl1.0-0
  libgstreamer-plugins-base1.0-0 libpulse-mainloop-glib0` (we never run apt).
- `backend/sitl/qgc.py` / `scripts/qgc.sh`: HOME (+XDG dirs) = external/qgc-home, so nothing lands in ~/.config or
  ~/Documents (checked). Before each start: [LinkManager] autoConnectUDP/ZeroConf/Pixhawk/SiKRadio/LibrePilot/RTKGPS
  = false (first guess `[AutoConnectSettings]` was ignored: QGC still listened on 0.0.0.0:14550 and :5353), one TCP
  link to 127.0.0.1 (auto-connect at start), no log auto-upload. Verified: no listening sockets; link ESTAB.
- Join mode: if 127.0.0.1:5762 is listening (the 🛰 panel's SITL), QGC uses it (139 KB of MAVLink received in 35 s
  while DroneLab's bridge held 5760); otherwise it starts its own SITL on 5760. QGC uses ~290 MB RAM.
- Outbound internet: map tiles over HTTP (~40 connections to a CDN) + one HTTPS connection. Documented in lesson
  12; map is blank offline. Not verified visually (the window opens on the Windows desktop).
- Test: +1 (settings writer). `scripts/qgc_setup.sh` re-downloads/unpacks and prints the apt line if libs are missing.

## Showcase + fault-injection lab (2026-11-22, user-approved)
- Privacy before going public: full history scanned (no secrets); the personal email was in all 28 commits, so the
  user chose to rewrite it to the GitHub noreply address (filter-branch, bundle backup first, one force-push with
  lease). Repo-local git config uses the noreply address from now on.
- CI (GitHub Actions): pytest, frontend build, and a job that builds ArduCopter (cached on the setup script + patch
  hash) and runs the real SITL flights + all fault scenarios.
- README rewritten as a front page (architecture SVG, measured results, skills map, honest limits, licence note:
  ArduPilot GPLv3 isn't in the repo; the 2-line patch stays GPLv3). LinkedIn draft in private/ (gitignored).
- Fault lab (`backend/sitl/faults.py`, lesson 13, docs/results/): 11 scenarios on real ArduCopter 4.7.3, default
  params, hold 20 m N of home at 10 m in GUIDED, inject via SIM_*, sample EKF belief + SIMSTATE truth at 5 Hz,
  judge. Safety 8/8 PASS, limitations 3/3 CONFIRMED.
- Things that surprised us (expectations corrected after measuring, all kept honest in the report):
  - 20 m sustained GPS glitch: EKF BELIEVED it (drone truly moved 20.5 m). EK3_GLITCH_RAD (25 m) only rejects
    big/brief jumps; a 100 m jump for 3 s was rejected (0.1 m). Split into "glitch" (safety) and "spoofing"
    (limitation).
  - Geofence: GUIDED silently refuses a destination outside the fence; with defaults AVOID_ENABLE brakes at the
    edge (stopped at 40.6 m for 40 m, RTL only 88 s later). Breach test pushes with velocity commands and turns
    avoidance off; the judge verifies the "fly to" was refused (no movement in the first 2 s).
  - Weak motor at 60 % sinks (hover needs ~59 % throttle): use 80 % (holds within ~1 m).
  - Hexa with a dead motor (default '+' hexa): stays up ~10–20 s, loses heading control, crash check disarms
    (AngErr > 30°). Same at speedup 1, so not a speedup artifact. Documented as a limitation.
  - Quad dead motor: no "Crash" message (it lands upright and bounces): judge on "SIM Hit ground at X m/s".
- Panel: 🧪 Fault injection row (fresh SITL per run, other actions blocked while it runs), report card, and a white
  wire ball at the SITL truth position when it differs from the EKF belief (> 0.5 m).
- Tests: test_faults.py: 6 offline + 11 real (opt-in).

## 🩺 Flight-log doctor (2026-11-22, user-approved "#3")
- `backend/logdoctor/analyze.py`: reads ArduPilot DataFlash logs with pymavlink (0.3 s / ~70 MB RAM for a 4.4 MB
  SITL log, filtering 16 message types), checks vibration, EKF test ratios, GPS, attitude/altitude/position
  tracking (only while armed and > 1 m up), motor balance/saturation, battery, failsafes/errors (decoded from
  ArduPilot 4.7's AP_Logger tables), and belief-vs-truth (SIM vs POS, simulator only). Timeline of modes, events,
  errors and messages (start-up chatter filtered), charts thinned to <= 400 points.
- `explain.py`: local Gemma debrief ("What happened / Most likely cause / What to check next", [Measured] /
  [Hypothesis]) on the compact summary only (~2.7 k chars); numbers checked with the assistant's
  `unverified_numbers`; logged. Measured: ~43 s per debrief, 0 unverified numbers on the GPS-loss and spoofing logs.
  Sent the summary with ensure_ascii=False (the model copied "\u00d7" literally before).
- `store.py`: SITL logs + uploads (`data/logs/`, .BIN only, DataFlash magic bytes checked, <= 100 MB), ids only
  map to files in those two folders (path tricks -> 404); analysis cached per file size+mtime.
- API: GET /api/logs, POST /api/logs/upload (raw body + X-Filename: no python-multipart needed),
  GET /api/logs/{id}/analysis, POST /api/logs/{id}/explain. UI: 🩺 Logs tab; fault cards link to their log
  (SITL's LASTLOG.TXT). SITL logs pruned to the newest 40 (they were 278 MB).
- Comparison recorder: one showcase run had a 2.6 s hole in the telemetry (rise time came out as an impossible
  0.8 s). Not reproducible in 2 instrumented runs (likely a WSL scheduling stall). Now the recorder measures its
  largest gap, retries up to 3x if > 0.3 s, warns in the panel, and the showcase refuses to write a chart from it.
- Geofence real test was flaky: peak distance 44.9–45.0 m vs a 45 m limit. Limit now fence + 10 m (braking at
  4 m/s), with the reason in the code.
- Tests: test_logdoctor.py: 5 offline (synthetic logs, store safety, fake-Ollama debrief) + 1 real (GPS-loss log).

## 📡 Lidar sense-and-avoid, 🔋 battery, 🛠 Drone Builder (2026-11-22, user-approved)
- Lidar (`backend/autonomy/lidar.py`): 72 beams / 8 m / 10 Hz, slab ray casting vs boxes (vectorised), 2 cm noise,
  hits mapped at the ESTIMATED pose into a 0.2 m occupancy grid, inflated by drone radius + margin (0.7 m);
  optimistic A* (unseen = free), straightened by line of sight; SenseFollower replans when the next 4 m is blocked,
  brakes when a beam within ±35° of travel is closer than radius + 0.45 m, gives up (stuck) after 3 s without a path.
  New course "hidden_walls" (second wall out of range and occluded). Results: straight crashes, potential stuck,
  A* 23.9 m / 16.6 s, lidar 25.7 m / 21.8 s with 5 replans and 311 cells. Lidar completes all 5 courses, no
  collisions (~0.5 s CPU per mission). Live: planner + 📡 toggle; map sent only when its version changes (avg frame
  2.3 KB, max 9 KB). Lab map plot draws hits (red) and dropped plans (faint).
- Battery (`backend/sim/battery.py`): LiPo OCV table, exact sag solution, current from shaft power / 0.75 + 4 W
  avionics, motor top speed x V/(3.85 V x cells). Off by default (all old tests unchanged). Measured: 1 kg / 3S
  3000 mAh hover 152 W, 12.4 A, ~12 min to low. Failsafe tuning: at 3.5 V/cell (under load ~12 % left) the trip
  home at 10x drain landed with 0.5 %; 3.6 V/cell -> landed home with 11 %. At 60x/30x drain the failsafe can't
  get home (also the 3 s wait had to be scaled by the drain speed): demo drain capped at 10x.
- Builder (`backend/sim/builder.py`): catalogue of 4 frames, 5 motors, 4 props, 2S–6S packs; momentum-theory hover
  power with figure of merit by prop size (0.45 for 3" .. 0.62 for 15"; a single 0.6 made 5" racers hover 14.5 min,
  unrealistic); the sim's kQ/kT is set so hover burns exactly that power (verified: sim power = estimate for 5
  builds). Flight-time estimate matches a simulated hover to the low point within 20 %. Every build flies a 5 m
  step with ~0.1 m overshoot with the default gains (181 g micro to 3.6 kg heavy-lift).
- UI: 🛠 Build your own (Studio), 🔋 Battery bar + Telemetry battery panel, 📡 lidar rays + red map in 3D.
- Tests: +9 battery/builder, +5 lidar, 2 updated for min mass 0.1 kg, phase-4 matrix gains hidden_walls + lidar.
  226 passed.

## 🌾 Farm surveys, 🎯 precision landing, 🧭 AI mission planner (2026-11-22/23, user-approved: "Take 1 2 and 3")
- Farm site (`site.py`): 3 convex fields, 3 buildings (9/16/8 m), 2 no-fly zones, pads home + pad B, 3 places, 90 m
  geofence, 60 m ceiling. Task plans (`plan.py`) compile to waypoints + an A* route (0.5 m grid; the default 0.2 m grid
  is too small for a 130 m farm) and 6-8 checks. Estimates vs simulated flights: time within ~3-12 %, battery within
  ~1-7 % (hover power measured with a 3 s simulated hover). Fixes on the way: line spacing first spread lines evenly
  (10 % overlap really flew at 50 %: no lesson) -> now exact spacing; follower lands at 1.5 m/s above 5 m (was 0.6:
  25 s from 15 m).
- Survey numbers (north field, 20 m): 4 lines, 36 photos, 0.7 cm/pixel, 142 s, 100 % covered. Gaps only appear with
  0 % side overlap in a gusty 6 m/s crosswind + GPS drift (94.8 / 97.6 / 98.8 %); 10 % overlap was already enough.
- Same survey on ArduCopter 4.7.3 SITL in AUTO: 100 % covered, 34 photos vs 36, 146 s vs 143 s, landed 0.04 m from
  its pad. Gotcha: 4.7 renamed WPNAV_SPEED (cm/s) to WP_SPD (m/s); set_param timed out until fixed.
- Precision landing: first version only improved 1.6 m -> 0.86 m. Logging showed the camera target was right to
  ~0.1 m; the drone wandered because the GPS drift model (first-order Gauss-Markov) jumped ~0.3 m every second. Real
  GPS error wanders smoothly: added a 10 s smoothing stage. Now, 8 seeds, 1.5 m drift: GPS 1.61 m mean (3.57 worst)
  vs camera 0.31 m (0.67); gusty 6 m/s: 1.53 vs 0.56 m (1.12 worst). Detection: lone-pixel noise filter (zero false
  detections on 100 grass frames), edge-touching markers ignored, blind final descent below 2 m (at 1.4 m: no gain).
- AI planner with the real gemma4 e4b (Ollama, HTTP): first try ignored "from 12 m"/"at 25 m" (kept 20 m) and added
  a goto to each field before its survey. Prompt now says altitude covers surveys; the compiler also drops a goto to
  the field right before its survey. After that, 4/4 example requests planned as asked; "school" -> not_on_map (fail),
  horse paddock -> left_out (warn), 150 m -> altitude fail, three fields at 8 m -> battery fail (79 %). ~2 s per plan
  once loaded (first load ~50 s).
- Live: ⏩ time speed-up (1/2/4/8x; 8x survey = ~3 % of a CPU core), photo footprints (sent only when a new photo is
  taken), down-camera picture-in-picture at 5 Hz (12 KB base64 per picture), no-fly warnings when flown by hand.
- Tests: +12 survey/plan, +5 precision landing, +6 mission planner (fake Ollama), +1 wind conversion, +1 opt-in real
  ArduPilot survey; phase-2 test updated for experiments 8-9. UI not verified in a browser.

## 🌱 Crop-health maps (2026-11-23, user-approved: "go ahead with #3 and #1")
- `cropmap.py`: NDVI ground truth per field (crop texture + planted patches: north = dry corner + pest damage, east =
  waterlogged dip, orchard = diseased trees); photos sampled on a 0.25 m grid from the TRUE camera position and pasted
  at their geotag (the estimate); smoothing, NDVI < 0.6, 2 m headland, patches >= 2 m².
- What didn't work: (1) image matching between photos (±3 m search) made placement WORSE (1.4 -> 3.2 m): with slow
  GPS drift neighbouring photos already agree, the error is a map-wide shift. (2) Ground control points (two dark
  panels per field, one global shift): unreliable, panels often blurred out by photo-to-photo jitter, and one shift
  can't fix a drift that changes over a 2-3 minute flight. Removed both; the lesson explains RTK / GCPs instead.
  (3) The "true area" counted soil outside the field; fixed. (4) GCP panels showed up as false patches (moot now).
- Results (4 seeds each, 20 m): every planted patch found in all 24 runs (3 fields x calm/real/windy), 1 false
  alarm. Position error = the GPS error: 0.1-0.4 m perfect GPS, 0.4-2.4 m with 1.5 m drift.
- Real gemma4 farmer's report: ~10 s first call, ~1-2 s after; 0 unverified numbers in 2 reports; causes labelled
  as hypotheses.
- Live: map built in a background thread after a survey plan lands (~0.1-1 s), sent once; 🗺 Field map modal with
  NDVI colours, findings, "show the truth" toggle, 🤖 report. Lab experiment 10.

## ✈ Agile flight: trajectories, a geometric controller, offboard ArduPilot (2026-11-23, user-approved: "go ahead with #1 and #5")
- Open-source check first (licenses verified from the repos/PyPI): minsnap-trajectories (MIT, numpy+scipy) added as a
  test-only dependency (`backend/requirements-dev.txt`) to cross-check our solver: same waypoints + times -> same curve
  to 1e-10 m. RotorPy (MIT; minsnap + SE(3) controller) and mav_trajectory_generation (Apache-2.0) / uav_trajectories
  (MIT) are reading references only (heavy deps / C++ + ROS). mrasl_mav_traj is GPL-3.0: not usable in this Apache
  repo. ArduPilot (GPL-3.0) is read in `external/`, never copied; its Lua `set_target_posvel_circle.lua` is pointed to
  in lesson 22 but NOT run (it would sit in SITL's shared scripts folder and hijack GUIDED flights above 5 m).
- `trajectory.py`: min-snap KKT solve (7th order, snap continuous), x/y/z solved together so "pass through the ring
  along its arrow" is two linear constraints; time = trapezoid guess -> share optimised (fixed total, -2 % lap) ->
  exact time scaling to speed/accel/tilt/thrust limits. `sample_one` fast path (a figure-eight run: ~5 s).
- Controller: Setpoint feedforward fields (None = old behaviour, bit-for-bit); cascade adds vel/acc/drag feedforward
  (agile tilt cap 50°); `GeometricController` (Lee/Mellinger): 3D accel -> exact thrust direction, projected thrust,
  jerk -> body-rate feedforward, same stiffness as the cascade.
- Results (figure eight, calm): lap is set by the plan (30°: 14.7 s for both, waypoints 23.5 s); path error PID-ff
  0.20 m vs geometric 0.03 m (45°: 0.38 vs 0.04 m, lap 11.6 s). Gusty 5 m/s: 0.31 vs 0.20 m (wind isn't in the plan).
- Found: with real sensors the trajectory flights missed 2/8 rings (0.61 m off). Cause = the estimator's accelerometer
  tilt correction ("|f| about g = down") is fooled in long banked turns (attitude error 2.3°, max 4.5°). Tilt
  correction off: 0.1° and 8/8 (0.14 m). Tried a GPS-position-aided tilt correction: worse (GPS noise leaks into
  tilt); not kept. Real fix = fuse GPS velocity (EKF). Exp 11 shows both conditions.
- Scoring bug fixed: rings judged one by one counted the figure eight's last (low, middle) ring on the way out from
  the start; a ring now counts only once the plan is heading for it.
- Live: ✈ Autopilot on any race track (orange planned path). My first live test SAVED the autopilot's lap as a human
  best time in data/race_best.json; removed that entry (the figure_eight track was new, so it was mine) and autopilot
  laps are never saved now.
- ArduPilot offboard (`backend/sitl/offboard.py`): GUIDED SET_POSITION_TARGET_LOCAL_NED with pos+vel+acc at 25 Hz
  (4.7 source checked: acceleration IS used; old docs say it isn't). First 30° runs diverged by metres: ArduPilot's
  input shaper has PSC_NE_JERK = 5 m/s³ and the plan peaks at 8.4. 20° (4.2 m/s³) was fine with defaults.
  `ap_limits` now sets PSC_NE_JERK/PSC_D_JERK = 1.5 x plan peak, ATC_ANGLE_MAX = tilt + 10, WP_ACC = plan accel.
  Figure eight, 30°: ArduPilot pos+vel+acc 8/8, 14.9 s, 0.14 m off the path; positions only 16.0 s, 0.31 m (4 m behind
  schedule); its own AUTO planner 27.6 s, 0.07 m. DroneLab geometric 14.7 s, 0.03 m. docs/results/agile-compare.md.
  Square loop 30° with these limits: 4/4, 6.34 s (DroneLab 6.31 s), 0.10 m mean / 0.26 m worst (defaults: 1/4, 2.3 / 5.2 m).
- Tests: test_agile.py (maths, cross-check, controllers, exp 11, live autopilot) + test_sitl.py (limits/mask unit test,
  opt-in real offboard flight: passed locally). CI installs requirements-dev.txt.

## 🧠 Companion computer, simulation half (2026-11-23, user-approved: "go ahead with #3 sim half")
- `backend/companion/service.py`: a separate process on SITL's SERIAL2 (tcp 5763) with DroneLab's own detect() and
  lidar scan(); sends LANDING_TARGET + OBSTACLE_DISTANCE. Only SimCamera / SimLidar read SITL truth (SIM_STATE).
- Results (ArduCopter 4.7 SITL, truth): told to land 1.8 m off the pad: 1.82 m without, 0.06 m with (310
  LANDING_TARGET). AUTO mission through a 10 m wall: through it without; round it with (closest 2.9 m, but 70 m path /
  35 s: ~20 s of left-right hesitation in front of the wall before BendyRuler took the west end).
- What went wrong first: (1) noise on "no hit" beams made 8 m into 7.99 m everywhere = a ring of walls, the drone
  danced near home; noise now on hits only. (2) Companion heartbeats (sysid 1, comp 191) are forwarded to the GCS link
  and `Flight` read them as "disarmed" (and pymavlink's target_component is 0), so it now accepts component 1 only.
  (3) ArduPilot moved HOME on re-arming; `keep_first_home`. (4) A 2.5 m GPS offset = PLND_XY_DIST_MAX: ignored; used
  1.8 m. (5) With PRX1_TYPE=2 it refuses to arm without lidar data, so "without the companion" = stock params on a
  fresh SITL. (6) At the default 10 m/s WP speed an 8 m lidar is too short to react; the avoid mission uses 3 m/s.
- Parked for later discussion (not approved): docs/discuss-later.md (real telemetry + bench QA, firmware hardening /
  certification, companion hardware half, training / B2B product).
