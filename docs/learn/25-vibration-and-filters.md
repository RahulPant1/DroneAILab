# 25 · Vibration and gyro filters: the first thing to fix on a real build

**Big idea:** motors and props shake the flight controller at the rotor's spin frequency, usually 100 to 300 Hz. The gyro can't tell shaking from real rotation. The rate controller then reacts to it, and the motors get a fast buzz on top of their commands. The result is hot motors, wasted battery, and a drone that flies "twitchy" or can't be tuned. On a new build this is often the first problem you hit.

## Two filters, one trade-off
- A **low-pass filter** (`INS_GYRO_FILTER`, ArduPilot's default 20 Hz) removes everything fast. But every filter also **delays** the signal: the controller sees the drone's rotation late, and a late controller must be tuned gently.
- A **harmonic notch** (`INS_HNTCH_*`) removes only a narrow band around the motor frequency, and follows it as the throttle changes. With a notch doing the heavy work, the low-pass can be raised (e.g. to 80 Hz) for much less delay.

## The workflow, done on real ArduCopter in SITL
1. **Measure.** Turn on the batch sampler: `INS_LOG_BAT_MASK = 1` logs raw gyro samples at 1 kHz, and `INS_LOG_BAT_OPT = 4` logs them both before and after the filters. Fly a hover.
2. **Find the peak.** DroneLab's log doctor runs an FFT over every block (Welch's method) and finds the biggest peak above 30 Hz (`backend/logdoctor/spectrum.py`).
3. **Set the notch** at that frequency. `INS_HNTCH_FREQ` = the peak, `INS_HNTCH_BW` = half of it, `INS_HNTCH_MODE = 1` follows the throttle, and `INS_HNTCH_REF` = the hover throttle. These rules come from the parameter descriptions in ArduPilot's source.
4. **Fly again and check** that the peak is gone.

SITL shook the gyro at each motor's real spin rate (`SIM_VIB_MOT_MAX = 350`, `SIM_GYR1_RND = 20`, the same as ArduPilot's own notch test). Measured (`docs/results/vibration.md`):

| Filters | Motor noise left | Gyro delay at 10 Hz |
|---|---|---|
| Low-pass 20 Hz (default) | 0.28 deg/s | 12.0 ms |
| Low-pass 80 Hz | 4.34 deg/s | 2.8 ms |
| **Low-pass 80 Hz + notch at 192 Hz** | **0.12 deg/s** | **3.1 ms** |

The notch gives the best of both: less noise than the 20 Hz filter, with a quarter of its delay.

**An honest limit:** attitude tracking in a 10 m dash was the same with all three (about 1.1° RMS). ArduPilot's default PID gains are gentle enough to cope with 12 ms of delay. The saved delay is *room* to tune the PIDs sharper (ArduPilot's AUTOTUNE does that); we didn't retune, so it doesn't show yet.

## Why this one is done on ArduPilot, not in DroneLab's own physics
DroneLab's simulated gyro samples at 250 Hz. By the sampling theorem, a 250 Hz sample rate can only represent vibrations up to 125 Hz, half of it. A 192 Hz buzz would fold down ("alias") to a fake 58 Hz wobble. Real flight controllers sample the gyro at 1 to 8 kHz and filter *before* they slow down. ArduPilot's SITL simulates exactly that, so we reuse it.

## Do it on your own drone
- Fly a hover with `INS_LOG_BAT_MASK = 1` and `INS_LOG_BAT_OPT = 4`, then upload the `.BIN` file in **Logs → 🩺 Flight-log doctor**. The new **Gyro noise** check shows the peak and a suggested notch, and the spectrum chart shows the peak before and after the filters.
- Fix the hardware first: balance the props, tighten the motors, soft-mount the flight controller. A notch hides vibration from the gyro; it doesn't stop the accelerometers being shaken (the **Vibration** check, `VIBE`, above 30 m/s² is bad).
- With ESC telemetry or an RPM sensor, `INS_HNTCH_MODE = 3` (or 2) follows each motor's real speed instead of guessing it from the throttle.

## Try it
`.venv/bin/python -m backend.sitl.vibration` flies all three setups on fresh SITLs (about 4 minutes); `--write` saves the table.

## Self-check
1. Why not just set the low-pass filter to 10 Hz and be done with it?
2. Why does the notch need to follow the throttle?
3. A gyro sampled at 250 Hz sees a motor buzz at 192 Hz. What frequency does it seem to be?

## Answers
1. The delay. A 10 Hz low-pass delays the gyro by tens of milliseconds, and the rate loop would have to be tuned so gently that the drone feels sluggish and fights wind badly.
2. Motor speed changes with throttle: climbing or carrying more weight moves the vibration up. A fixed notch would miss it.
3. 250 − 192 = 58 Hz: a fake slow wobble the controller would try to fight. That's why real gyros sample at 1 kHz or more and filter first.
