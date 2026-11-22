# Example: the 🩺 log doctor on a GPS-spoofing flight

A real ArduCopter 4.7.3 flight log from the fault lab (*GPS spoofing*: the GPS reports a steady 20 m offset for 15 s). The checks are rule-based; the debrief is written by a local Gemma model (Ollama, offline) that only saw the summary below, never the raw log. Every number it wrote was checked against the data (none flagged).

Log: 00000070.BIN, flew 50.9 s, modes GUIDED.

## Checks

| | Check | Measured | Healthy if |
|---|---|---|---|
| ✅ | Vibration | peak 0.9 m/s², 0 clipping events | < 30 m/s², no clipping |
| ❌ | EKF health (innovation test ratios) | velocity 0.06, position 5.37, height 0.01, compass 0.04 | < 0.5 normal, > 1.0 = reading rejected |
| ✅ | GPS quality | min 10 satellites, max HDOP 1.21, lost fix 0× | ≥ 9 satellites, HDOP ≤ 1.4, no lost fix |
| ✅ | Attitude tracking (roll/pitch) | max 1.8°, RMS 0.26° | max < 5° |
| ✅ | Altitude tracking | max error 0.09 m | < 1 m |
| ✅ | Position tracking | max error 0.59 m | < 2 m |
| ✅ | Motor balance and saturation | average outputs 1594, 1594, 1597, 1597 µs (spread 3); at full power 0 % of the time | spread < 75 µs, full power < 2 % of the time |
| ✅ | Battery | 12.60 V → min 12.60 V (sag 0.00 V) | no failsafe, sag < 1.5 V |
| ⚠️ | Failsafes and errors | 25 s GPS: GPS glitch; 34 s EKF primary lane: switched to lane 1 | none |
| ❌ | Belief vs truth (simulator only) | max 20.3 m | < 2 m |

## Timeline

- **0.0 s** mode GUIDED
- **0.0 s** Arming motors
- **0.0 s** mode GUIDED
- **3.1 s** auto-armed
- **4.1 s** took off
- **6.5 s** EKF yaw reset
- **6.5 s** EKF3 IMU0 MAG0 in-flight yaw alignment complete
- **6.5 s** EKF3 IMU1 MAG0 in-flight yaw alignment complete
- **25.3 s** GPS: GPS glitch
- **25.3 s** GPS Glitch or Compass error
- **33.9 s** EKF yaw reset
- **33.9 s** EKF primary lane: switched to lane 1
- **33.9 s** EKF3 lane switch 1
- **33.9 s** EKF primary changed:1

## AI debrief (gemma4:e4b-it-q4_K_M, 21.3 s)

```
What happened:
* [Measured] EKF health (innovation test ratios) status was "bad" with velocity 0.06 and position 5.37.
* [Measured] The flight experienced a "GPS: GPS glitch" at t=25.3s.
* [Measured] An EKF primary lane switch occurred at t=33.9s, indicated by "EKF primary lane: switched to lane 1".
* [Measured] The "Belief vs truth (simulator only)" status was "bad" with a max value of 20.3 m.

Most likely cause:
* [Hypothesis] The "bad" EKF health and the "Belief vs truth" error suggest significant state estimation issues.
* [Hypothesis] The GPS glitch at t=25.3s likely contributed to the subsequent EKF instability and lane switching.

What to check next:
* [Hypothesis] Review the sensor data leading up to t=25.3s to diagnose the GPS glitch source.
* [Hypothesis] Examine the EKF tuning parameters, especially related to position and velocity innovation thresholds.
```
