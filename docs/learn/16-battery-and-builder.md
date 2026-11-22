# 16 · Batteries and building your own drone

**Big idea:** before you buy parts, a few numbers tell you whether a drone will fly and for how long: **weight**, **thrust-to-weight**, **hover power**, and the **battery**. The 🛠 Drone Builder works them out, then lets you fly the design on its own battery.

## The battery (🔋)
- A drone battery is several LiPo cells in series: **3S** = 3 cells = 11.1 V nominal. **mAh** is how much charge it holds.
- A cell's resting voltage follows its charge: **4.2 V** full, ~3.8 V in the middle, dropping fast below ~3.5 V.
- Under load the voltage **sags** (current × the pack's internal resistance). Watch the voltage dip in Telemetry when you climb.
- Motor speed ≈ Kv × volts, so as the battery drains the motors lose top speed and thrust. To keep hovering, the controller has to ask for more throttle.
- **Failsafe** (like ArduPilot's battery failsafe): 3 s below **3.6 V per cell** → fly home and land; below **3.4 V** → land where it is; empty → the motors stop.

**Measured** (1 kg learner drone, 3S 3000 mAh): hover takes ~152 W (12.4 A), about 12 minutes to the low-battery point. With the battery draining 10× faster, flown 8 m away: low battery → it flew home and landed 0.65 m from the pad with 11 % left.

## Building (🛠 in the Drone Studio panel)
Pick a frame, motors, props, battery and payload:
| Number | What it means | Rule of thumb |
|---|---|---|
| **All-up weight** | everything together | every gram costs flight time |
| **Thrust-to-weight (T/W)** | max thrust ÷ weight | over 1 to fly at all; **2+** for a normal drone; 8–12 for racers |
| **Hover throttle** | 100 ÷ T/W | under ~60 % leaves room to manoeuvre |
| **Hover power** | from momentum theory: lifting with a bigger prop takes **less** power | |
| **Flight time** | usable charge ÷ hover current | gentle hovering; aggressive flying uses 2–3× more |
| **Full-throttle current** | vs the pack's C rating × capacity | over it: the pack sags and heats up |

It also refuses impossible builds: props too big for the frame, a motor on too many cells (it would burn out), a motor on the wrong prop size.

## Example builds (as the Builder computes them)
| Build | Weight | T/W | Hover | Flight time |
|---|---|---|---|---|
| 450 mm, 2212 920 Kv, 10" props, 3S 3000 mAh | 898 g | 3.8 | 26 % | ~17 min |
| 250 mm racer, 2306 2450 Kv, 5", 4S 1300 mAh | 435 g | 11.5 | 9 % | ~12 min hover (2–4 min racing) |
| 180 mm micro, 1404 3800 Kv, 3", 4S 450 mAh | 181 g | 8.4 | 12 % | ~8 min |
| 650 mm heavy-lift, 3510 360 Kv, 15", 6S 10 Ah + 1 kg payload | 3638 g | 2.5 | 40 % | ~22 min |

The simulator then flies exactly that drone: its motors are set up so that in hover they burn the same power the Builder estimated (checked by a test), and the flight-time estimate matches a simulated flight to the low-battery point within 20 %.

## What it doesn't model
Motor heating, prop flex, battery temperature and ageing, ESC limits, and aerodynamic effects at speed. Treat the numbers as a careful first estimate, the way builders use tools like eCalc, and leave margin.

## Self-check
1. Doubling the battery capacity doesn't double the flight time. Why?
2. Why does the same drone hover longer with 10" props than with 8" props (same motors, same weight)?
3. Your design shows T/W 1.4. It hovers fine in the simulator on a calm day. Why is it still risky?

## Answers
1. The bigger battery is heavier, so hovering needs more power. You get more energy, but you spend it faster.
2. Hover power ≈ T^1.5 / √(2ρA): a bigger prop disc (A) moves more air more slowly, which takes less power for the same thrust.
3. At T/W 1.4 hover needs ~70 % throttle: almost nothing is left to climb, brake, turn or fight a gust, and as the battery sags there's even less.
