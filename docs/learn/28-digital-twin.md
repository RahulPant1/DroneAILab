# 28 · From the simulator to your real drone: the digital twin and the starting parameter file

**Big idea:** before a real drone flies, two things should already be right: its **parameters** (the ~1300 settings ArduPilot reads) and your **expectations** (how it should hover, how long it should fly, what it does when the radio or the battery fails). DroneLab now produces both from a 🛠 Drone Builder design.

## 1 · The digital twin: ArduPilot flies *your* build
ArduPilot's simulator (SITL) normally flies a generic quad. It can also load a **frame model**: a small JSON file with the airframe's mass, size, hover throttle, battery and rotor area. DroneLab writes that file from your design, so ArduPilot's own physics and flight code fly something like your drone: same weight, same thrust margin, same battery.

**🛠 Drone Builder → 🛰 Fly its twin on ArduPilot.** Then fly it from the ArduPilot panel (Engineer mode), or connect QGroundControl, exactly as you would the real one.

Or run the automatic test flight: `.venv/bin/python -m backend.sitl.twin` (add `--autotune` for ArduPilot's tuning flight, `--write` for the results table).

## 2 · The starting parameter file
**🛠 Drone Builder → ⬇ Parameters** gives a file to load on the real flight controller (QGroundControl's format, or Mission Planner's). It sets:

| What | Why it matters |
|---|---|
| Frame: quad, X | the wrong frame type flips the drone at take-off |
| Battery: capacity, low/critical voltage (3.5 / 3.3 V per cell) and mAh reserves | so the low-battery failsafe triggers *before* the pack is empty |
| Low battery → come home; critical → land | the most common first-build crash is an empty battery |
| Radio lost → come home | needs your receiver set to "no pulses" on signal loss too |
| Bad position estimate → land | |
| Geofence: 60 m up, 100 m out → come home | keeps it in sight and under the legal ceiling |
| Hover throttle (from the Builder's thrust-to-weight) | a good first guess; ArduPilot keeps learning the real one |
| Gyro filter and acceleration limits for your prop size | ArduPilot's own table: 80 Hz at 5″, 40 Hz at 10″, 20 Hz at 20″ |
| Raw-gyro logging | so the 🩺 log doctor can find the motor noise and suggest a notch (lesson 25) |

It **doesn't** set what only your hardware knows: calibrations (accelerometer, compass, radio, ESCs), the ESC protocol and motor outputs, the battery monitor's sensor scaling, flight modes. You do those in QGroundControl. It's a safe start, not a tune.

## What the twin measured (ArduCopter 4.7.3 SITL, `docs/results/twin.md`)
| Check | Builder predicted | ArduPilot measured |
|---|---|---|
| Hover throttle, 450 mm / 10″ / 3S build | 42 % | 40.5 % |
| Hover throttle, 5″ racer / 4S | 19 % | 18 % |
| Radio lost 30 m away | come home | RTL in 0.8 s, landed 0.2 m from the take-off spot |
| Battery low 30 m away | come home | RTL after 10.5 s (ArduPilot waits 10 s to be sure) |
| AUTOTUNE | | success: roll rate P 0.135 → 0.228 |

Honest limits:
- **Hover current and flight time are not an independent check.** ArduPilot's simulator is *given* the Builder's hover current as its hover power, so it gives it back. The real number comes from your first flight's log (BAT.Curr while hovering).
- **The AUTOTUNE gains are for the simulated airframe,** which responds perfectly crisply: no flexing arms, no vibration. Don't copy them. Fly AUTOTUNE on the real drone once the basics are proven.
- It can't check motor order and direction, ESC calibration, vibration or compass interference. Use the props-off motor test and the first flight log for those.

## Bugs we hit (each one is a real lesson)
1. **Old names fail silently.** Most guides still say `RTL_ALT` (cm), `ATC_ACCEL_R_MAX` (centidegrees) and `ARMING_CHECK`. ArduPilot 4.7 calls them `RTL_ALT_M` (metres), `ATC_ACC_R_MAX` (deg/s²) and `ARMING_SKIPCHK` (which works the other way round: checks to *skip*). An unknown name in a parameter file is just ignored, so the drone would fly with the default. Every name here is checked by booting real ArduPilot and reading each value back.
2. **QGroundControl needs each parameter's type** (integer or decimal) in its file. The types were read from ArduPilot itself.
3. **A comment isn't the code.** The simulator's example files say the reference current is "from a test at 45°"; the code uses it as the *hover* power. Scaling it for 45° made the twin draw 60 % too much current. Read the source.
4. **The simulator only loads a frame file from its own folder** (an absolute path makes it panic at start).
5. **A held transmitter matters.** During AUTOTUNE our "centred sticks" were sent now and then. ArduPilot dropped them, the simulator's own radio had the throttle at zero, and the twin fell out of the sky. A real transmitter sends 50 times a second; ours now sends 10.
6. **The battery failsafe stays on until a reboot** and then refuses to arm. That's correct, so AUTOTUNE now gets its own fresh boot, like a separate tuning flight.

## Your first real flight, in order
1. Build it; flash ArduCopter; load **⬇ Parameters**.
2. In QGroundControl: frame check, accelerometer, compass, radio calibration, ESC calibration, flight modes (Stabilize, AltHold, Loiter; RTL on a switch).
3. **Props off:** the motor test (order and direction), failsafe tests (switch off the transmitter: does it go to RTL?).
4. Props on, open field, a low hover in Stabilize / AltHold for a minute. Land.
5. Upload the log to the **🩺 log doctor**: vibration, motor noise (it suggests a notch), hover throttle, current.
6. Set the notch, then AUTOTUNE on a calm day.

## Self-check
1. Why is a parameter with an out-of-date name dangerous, even though nothing complains?
2. The twin's flight time matched the Builder's. Why doesn't that prove the real flight time?
3. Why test the radio-loss failsafe with the props *off* first?

## Answers
1. It's silently ignored, so the drone flies with the default value you thought you had changed: for example the wrong return-home height.
2. The simulator was given the Builder's hover current, so it is the same model twice. Only a real log measures the real current.
3. If it's wrong, the drone does something unexpected on the bench, not in the air: you learn that it doesn't come home without losing the drone.
