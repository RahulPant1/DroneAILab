# 02 · Roll, pitch and yaw

**Big idea:** a quadcopter can only push straight "up" out of its own propellers. To move sideways it must **tilt**, so part of that push points the way it wants to go.

## The three angles
In DroneLab, z points up, x is the drone's nose, and y is its left side.
- **Roll** is rotation about the nose axis. **Roll + = right side down**, which pushes the drone to the **right**.
- **Pitch** is rotation about the left axis. **Pitch + = nose down**, which pushes the drone **forward**.
- **Yaw** is the heading, rotation about the vertical axis. **Yaw + = turn left** (counter-clockwise seen from above). Yaw 0 = nose pointing east.

(Aircraft books often use "pitch + = nose up". We follow the robotics convention with z up, so watch the sign.)

**Yaw vs compass heading.** Our yaw is a maths angle: 0 = east, counting counter-clockwise (90° = north).
A compass heading is the other way round: 0° = north, counting clockwise (90° = east). To convert:
**heading = 90° − yaw** (add 360° if negative). So yaw 0 = heading 90° (east), and yaw 90° = heading 0° (north).
Real flight software (PX4, ArduPilot) uses compass headings, and often a frame with z pointing **down** (called
NED: north, east, down), so expect the signs to flip when you move to real tools.

## How 4 motors make each move
Motors: M1 front-right, M2 rear-left (both spin counter-clockwise), M3 front-left, M4 rear-right (both clockwise).
- **Climb:** all four speed up equally.
- **Roll right:** the left motors (M2, M3) push harder than the right ones (M1, M4).
- **Pitch nose-down:** the rear motors (M2, M4) push harder than the front ones (M1, M3).
- **Yaw left:** the clockwise props (M3, M4) speed up, and the counter-clockwise ones slow down. Each prop drags the body the *opposite* way to its spin, so the body turns left. Yaw is the weakest move, so the mixer gives it last priority.

## Why tilting costs height
When tilted by angle θ, only cos θ of the thrust holds the drone up. At 25° that is 91 %, so the controller adds about 10 % more thrust while moving. Watch the motor bars rise when you hold **W**.

## The control chain
Target position → wanted speed → wanted acceleration → **tilt angles** → spin rates → motor commands. Every arrow is one small P or PID controller (see `backend/sim/control.py`, `_position_loop`).

## Try it (in the app)
1. Take off, then hold **W**. Pitch goes positive (nose down) and the rear motor bars rise.
2. Hold **A**. Roll goes negative (left side down) and the drone slides left.
3. Hold **Q**. Yaw increases, and M3/M4 work harder than M1/M2.
4. Studio → **Heavy** preset → Apply. Fly to x = 5. Notice the slower, saggier moves (T/W 1.3 leaves little spare thrust).

## Self-check
1. The drone faces east (yaw 0) and you want it to go north. Which angle must change, and with which sign?
2. Why can't the drone just "push sideways" without tilting?
3. While hovering, M3 and M4 speed up and M1 and M2 slow down by the same amount. What happens?

## Answers
1. North is on the drone's **left**, so it must roll left: **roll negative** (left side down). Pitch and yaw stay the same.
2. Propellers can only push along the drone's own "up" axis; there's no sideways thruster. Tilting points part of that push sideways.
3. Total thrust stays the same, so the height holds. The clockwise pair (M3, M4) wins the drag-torque contest, so the drone **turns left** (yaw increases).
