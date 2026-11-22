# 18 · Precision landing: finding the pad with a camera

**Big idea:** GPS is good to a metre or two, and its error **wanders slowly**: for a minute it might say you're 2 m east of where you really are. A drone that lands on the pad's GPS coordinate misses by that much. A delivery drone, or one landing on a charging station, has to hit a 1 m pad. So it uses a **camera** to find a marker on the pad.

## How it works
1. **Picture.** A 128 × 96 grey-scale camera under the drone looks down (60° wide). The pad is a light mat with a black square ring: the **marker**.
2. **Detect.** Pixels much darker than grass are marker. Their centre is the pad's centre. Lone dark pixels (noise) are ignored. If the marker touches the picture's edge, only part of it is visible, so it is ignored too (its centre would be wrong).
3. **Where is it?** That pixel becomes a direction from the camera. Rotate it with the drone's **tilt and heading**, stretch it down to the ground using the **height**, and you get the pad's offset in metres east/north **from the drone**.
4. **Why GPS stops mattering.** Target = *where I think I am* + *offset*. If GPS says you're 2 m east of the truth, the target moves 2 m east too. The drone flies to it, which puts it really over the pad. The error cancels.
5. **Land.** Descend while the marker is in view, slowing down when not centred. Marker lost high up: **stop and search**; not found in 6 s: land on GPS. Below 2 m the marker may not fit in the picture, so it keeps the last target for the touch-down.

## What we measured (pad B, from 10 m, GPS with 1.5 m of drift, 8 runs each)
| Landing | Calm: average (worst) | Gusty 6 m/s wind: average (worst) |
|---|---|---|
| GPS only | 1.61 m (3.57 m) | 1.53 m (2.67 m) |
| **Camera** | **0.31 m (0.67 m)** | **0.56 m (1.12 m)** |

The camera made landings about **5× more accurate** in calm air. The marker is 1 m wide, so GPS alone often lands off the pad. What's left with the camera is the controller chasing its jittery position estimate. Gusts make it worse; real systems add better sensor fusion (ArduPilot runs a separate "landing target" estimator) and a range finder.

## Try it
- **Lab → experiment 9:** camera vs GPS only, GPS drift, wind. The camera pictures on the way down are shown with the detected marker boxed.
- **Fly tab:** course **🌾 Hillside farm**. Fly near a pad 🅿, then **🎯 Land on pad (camera)**; the picture-in-picture shows what the camera sees. Turn on **📡 Sensor noise** to get a GPS error; compare with **GPS only**.
- **🧭 Mission planner:** "…then land on pad B" uses precision landing at the end.

## Real drones
ArduPilot's precision landing (`PLND_*` parameters) works with an IR beacon (IR-Lock) or a companion computer that detects ArUco/AprilTag markers and sends `LANDING_TARGET` messages. PX4 does the same. Real cameras also have lens distortion, motion blur and changing light, and markers can be dirty or in shadow.

## Self-check
1. Why doesn't a GPS error matter once the camera sees the marker?
2. Why does the drone ignore a marker that touches the edge of the picture?
3. Why keep the last target instead of looking for the marker in the last 2 m?

## Answers
1. The camera measures the pad's position *relative to the drone*. The target is "my GPS position + that offset", so if GPS is off by 2 m the target is off by the same 2 m, and flying to it puts the drone over the real pad.
2. Then only part of the marker is visible, and the centre of the visible part is not the centre of the marker. Steering to it would pull the drone the wrong way.
3. Close to the ground the 1 m marker no longer fits in the camera's view (at 1.5 m the picture covers only about 1.7 m of ground), so detections stop. The last good target is still right, and the drone barely moves in those last seconds.
