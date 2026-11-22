# 14 · Reading flight logs: the 🩺 log doctor

**Big idea:** after every flight, an ArduPilot drone writes a detailed log (a `.BIN` "DataFlash" file): every sensor, every controller target, every mode change and error, many times a second. When something goes wrong, the log is how engineers find out why. The same skill keeps a home-built drone flying.

## What's inside a log
| Message | What it records | Used for |
|---|---|---|
| `VIBE` | how much the flight controller is shaken | vibration problems (props, motors, mounting) |
| `XKF4` | EKF "test ratios": how much each sensor disagrees with the estimate | GPS glitches, compass interference, failing sensors |
| `GPS` | satellites, HDOP, fix | position quality |
| `ATT` / `CTUN` / `PSCN`/`PSCE` | what the controller *wanted* vs what happened (tilt, altitude, position) | tuning, weak motors, wind |
| `RCOU` | the output to each motor (PWM 1000–2000 µs) | an unbalanced or failing motor, no power margin |
| `BAT` | voltage and current | tired battery, battery failsafe |
| `MODE`, `EV`, `ERR`, `MSG` | mode changes, events, failsafes, text messages | the story of the flight |
| `SIM` | *simulator only*: where the drone really was | belief vs truth |

## Use it
- **🩺 Logs** tab: pick a log from the simulator (every ArduPilot flight in the 🛰 panel or the fault lab makes one) or **⬆ Upload a .BIN** from a real drone. You get a table of checks (✅ ⚠️ ❌; hover a row for what it means), the timeline, and charts.
- After a fault test, the result card has **🩺 Analyse this flight's log**.
- **🤖 Explain this flight** asks the local AI for a debrief: *What happened / Most likely cause / What to check next*. It only sees the summary (checks + timeline), never the raw log, and every number it writes is checked against the data. Numbers it can't back up are flagged.
- In the terminal: `.venv/bin/python -m backend.logdoctor.analyze path/to/log.BIN`

## The checks and their limits
| Check | Healthy if | Why that limit |
|---|---|---|
| Vibration | < 30 m/s², no clipping | ArduPilot's guidance: under 30 normally fine, over 60 nearly always a problem |
| EKF test ratios | < 0.5 (over 1.0 = rejected) | 1.0 is the EKF's own gate |
| GPS | ≥ 9 satellites, HDOP ≤ 1.4, fix never lost | 1.4 is ArduPilot's own arming limit |
| Attitude tracking | max error < 5° | bigger means the inner loop can't keep up |
| Altitude / position tracking | < 1 m / < 2 m | in hover and gentle flight |
| Motors | spread < 75 µs, at full power < 2 % of the time | one motor working much harder = CG, prop or motor problem |
| Battery | sag < 1.5 V, no failsafe | |
| Failsafes and errors | none | every failsafe means something went wrong |

These are sensible defaults for a small quad, not laws: a racing drone vibrates more, a heavy-lift drone runs its motors harder.

## Example
The log of a GPS-spoofing flight from the fault lab, with its checks, timeline and the AI's debrief: [docs/results/log-doctor-example.md](../results/log-doctor-example.md). The EKF position test ratio hit **5.37** (it did fight the false GPS at first), then a lane switch, and a 20.3 m gap between belief and truth.

## Self-check
1. In a log, motor 1's average output is 1750 µs and the other three are around 1550 µs. What would you check?
2. Why can the log doctor show "Belief vs truth" for a simulator log but never for a real drone's log?
3. The AI's debrief says "the drone drifted 57 m" and the panel flags 57 as unverified. What should you do?

## Answers
1. That one motor is doing much more work: the centre of gravity may be off towards it, its prop may be damaged or on the wrong way, or the motor/ESC is weaker. Check the build before flying again.
2. Only the simulator knows where the drone really is. A real drone only has its own estimate (the EKF), which is exactly what a log records.
3. Don't trust that number: it isn't in the data. Look at the checks and timeline yourself, or ask again. The flag exists because language models can sound sure about things they made up.
