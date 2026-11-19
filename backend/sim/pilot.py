"""The 'autopilot buttons': turns Take-off / Hover / Land and keyboard nudges into setpoints.

Modes:
  idle    : motors off, sitting on the ground
  takeoff : armed, climbing to the take-off altitude; switches to hover when it gets there
  hover   : holding a position; keyboard nudges move that position around
  land    : sinking slowly; motors switch off once it has been on the ground briefly

The pilot never touches the motors directly. It only says WHERE the drone should be,
and the controller (control.py) works out how to get there.

What it can see: like the controller, it works from the drone's ESTIMATE of itself (the truth when sensors are
perfect). Only "am I on the ground?" comes from the true state, because our estimator doesn't track ground contact.
"""

from __future__ import annotations

import math
from dataclasses import dataclass

from .control import Setpoint
from .physics import State

DEFAULT_TAKEOFF_ALT_M = 2.0
TAKEOFF_REACHED_M = 0.1
LAND_SPEED_M_S = 0.6
LAND_TARGET_BELOW_GROUND_M = 0.3  # aim slightly below ground so it settles firmly
DISARM_AFTER_GROUND_S = 0.3
NUDGE_SPEED_M_S = 1.5
NUDGE_CLIMB_M_S = 0.8
NUDGE_YAW_RAD_S = math.radians(60.0)
FAILSAFE_TILT_RAD = math.radians(12.0)  # attitude mode: a full W/A/S/D press tilts this much
MAX_LEAD_M = 2.0  # the target can run at most this far ahead of the drone
MIN_FLY_ALT_M = 0.3
MAX_FLY_ALT_M = 50.0
MAX_RANGE_M = 200.0


@dataclass
class Sticks:
    """Keyboard nudges, each -1, 0 or +1 (or anything in between)."""

    forward: float = 0.0  # W / S
    left: float = 0.0  # A / D (A = +left)
    up: float = 0.0  # R / F
    yaw: float = 0.0  # Q / E (Q = +yaw = turn left)

    def clamped(self) -> "Sticks":
        c = lambda v: max(-1.0, min(1.0, float(v)))  # noqa: E731
        return Sticks(c(self.forward), c(self.left), c(self.up), c(self.yaw))


class Pilot:
    def __init__(self, takeoff_alt_m: float = DEFAULT_TAKEOFF_ALT_M):
        self.takeoff_alt_m = takeoff_alt_m
        self.mode = "idle"
        self.sp = Setpoint(armed=False, position_control=True)
        self.sticks = Sticks()
        self.message = "Ready. Press Take off."
        self.attitude_mode = False  # GPS-loss failsafe: W/A/S/D tilt the drone; the x/y target stays frozen
        self._ground_time_s = 0.0

    # ---- Button commands (return a short message for the UI) ----

    def takeoff(self, state: State, alt_m: float | None = None) -> str:
        if self.mode in ("takeoff", "hover"):
            return self._say("Already flying.")
        alt = self.takeoff_alt_m if alt_m is None else max(MIN_FLY_ALT_M, min(MAX_FLY_ALT_M, alt_m))
        x, y, _ = state.pos_m
        _, _, yaw = state.euler_rad()
        self.sp = Setpoint(armed=True, alt_m=alt, yaw_rad=yaw, position_control=True, x_m=x, y_m=y)
        self.mode = "takeoff"
        return self._say(f"Taking off to {alt:.1f} m.")

    def hover(self, state: State) -> str:
        if self.mode == "idle":
            return self._say("On the ground. Take off first.")
        x, y, z = state.pos_m
        _, _, yaw = state.euler_rad()
        self.sp = Setpoint(armed=True, alt_m=max(MIN_FLY_ALT_M, z), yaw_rad=yaw, position_control=True, x_m=x, y_m=y)
        self.sticks = Sticks()
        self.mode = "hover"
        return self._say(f"Holding position at {self.sp.alt_m:.1f} m.")

    def land(self, state: State) -> str:
        if self.mode == "idle":
            return self._say("Already on the ground.")
        self.sp.x_m, self.sp.y_m = float(state.pos_m[0]), float(state.pos_m[1])
        self.sp.alt_m = min(self.sp.alt_m, float(state.pos_m[2]))
        self.mode = "land"
        self._ground_time_s = 0.0
        return self._say("Landing.")

    def set_target(self, x_m=None, y_m=None, alt_m=None, yaw_rad=None) -> str:
        if self.mode not in ("takeoff", "hover"):
            return self._say("Take off first, then set a target.")
        if x_m is not None:
            self.sp.x_m = max(-MAX_RANGE_M, min(MAX_RANGE_M, float(x_m)))
        if y_m is not None:
            self.sp.y_m = max(-MAX_RANGE_M, min(MAX_RANGE_M, float(y_m)))
        if alt_m is not None:
            self.sp.alt_m = max(MIN_FLY_ALT_M, min(MAX_FLY_ALT_M, float(alt_m)))
        if yaw_rad is not None:
            self.sp.yaw_rad = float(yaw_rad)
        self.mode = "hover"
        return self._say(f"Flying to x={self.sp.x_m:.1f}, y={self.sp.y_m:.1f}, alt={self.sp.alt_m:.1f} m.")

    def set_sticks(self, sticks: Sticks) -> None:
        self.sticks = sticks.clamped()

    # ---- Called every physics step ----

    def update(self, state: State, dt_s: float, truth: State | None = None) -> Setpoint:
        """Advance the mode logic by dt and return the setpoint for the controller.
        `state` = what the drone believes (the estimate); `truth` = the real state, used only for ground contact."""
        truth = truth or state
        if self.mode == "takeoff" and abs(state.pos_m[2] - self.sp.alt_m) < TAKEOFF_REACHED_M:
            self.mode = "hover"
            self._say(f"Hovering at {self.sp.alt_m:.1f} m. Use W/A/S/D, R/F, Q/E to move.")

        if self.mode in ("takeoff", "hover"):
            self._apply_sticks(state, dt_s)
        elif self.mode == "land":
            floor = -LAND_TARGET_BELOW_GROUND_M
            self.sp.alt_m = max(floor, self.sp.alt_m - LAND_SPEED_M_S * dt_s)
            self._ground_time_s = self._ground_time_s + dt_s if truth.on_ground else 0.0
            if self._ground_time_s >= DISARM_AFTER_GROUND_S:
                self.mode = "idle"
                self.sp.armed = False
                self._say("Landed. Motors off.")
        return self.sp

    def _apply_sticks(self, state: State, dt_s: float) -> None:
        """Move the target in the drone's heading frame (W = the way the nose points)."""
        s = self.sticks
        if s.forward == 0 and s.left == 0 and s.up == 0 and s.yaw == 0:
            return
        yaw = self.sp.yaw_rad
        # Attitude mode (GPS-loss failsafe): the x/y target stays frozen, so that when GPS returns the drone goes
        # back to it. W/A/S/D tilt the drone instead (see tilt_from_sticks); only height and heading change here.
        if not self.attitude_mode:
            fwd, left = s.forward * NUDGE_SPEED_M_S * dt_s, s.left * NUDGE_SPEED_M_S * dt_s
            x = self.sp.x_m + math.cos(yaw) * fwd - math.sin(yaw) * left
            y = self.sp.y_m + math.sin(yaw) * fwd + math.cos(yaw) * left
            # Keep the target on a short leash so it can't run away from the drone.
            dx, dy = x - state.pos_m[0], y - state.pos_m[1]
            lead = math.hypot(dx, dy)
            if lead > MAX_LEAD_M:
                x, y = state.pos_m[0] + dx * MAX_LEAD_M / lead, state.pos_m[1] + dy * MAX_LEAD_M / lead
            self.sp.x_m = max(-MAX_RANGE_M, min(MAX_RANGE_M, x))
            self.sp.y_m = max(-MAX_RANGE_M, min(MAX_RANGE_M, y))
        alt = self.sp.alt_m + s.up * NUDGE_CLIMB_M_S * dt_s
        alt = min(alt, state.pos_m[2] + MAX_LEAD_M) if s.up > 0 else max(alt, state.pos_m[2] - MAX_LEAD_M)
        self.sp.alt_m = max(MIN_FLY_ALT_M, min(MAX_FLY_ALT_M, alt))
        self.sp.yaw_rad = math.remainder(self.sp.yaw_rad + s.yaw * NUDGE_YAW_RAD_S * dt_s, 2 * math.pi)

    def tilt_from_sticks(self) -> tuple[float, float]:
        """Attitude mode: (roll, pitch) from the keys. W = nose down = go forward (+pitch);
        A = left side down = go left (-roll). Let go and the drone levels out, but keeps coasting."""
        return -self.sticks.left * FAILSAFE_TILT_RAD, self.sticks.forward * FAILSAFE_TILT_RAD

    def _say(self, message: str) -> str:
        self.message = message
        return message
