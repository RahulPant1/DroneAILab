"""Smooth trajectories: minimum-snap polynomials through waypoints (Mellinger & Kumar, ICRA 2011).

Why: flying straight lines between waypoints means stopping (or cutting) at every corner. Racing, inspection and
delivery drones instead fly ONE smooth curve through all the points, planned before take-off, and the controller
is told not just where to be but how fast and how hard to accelerate at every moment ("feedforward").

How:
  * Each piece between two waypoints is a 7th-order polynomial per axis (x, y, z), in normalised time tau in [0, 1].
  * Among all curves that pass through the waypoints (and start and end at rest), pick the one with the smallest
    total SNAP squared (snap = 4th derivative of position). A drone's thrust and spin follow the acceleration and
    jerk; keeping snap small keeps the motor commands smooth.
  * That's a quadratic cost with linear constraints, solved in one go (a "KKT" linear system).
  * Gates (race rings) can also say "go through me along this direction": the velocity there must be parallel to
    the gate's arrow, which is two more linear constraints. That's why x, y and z are solved together here.
  * Timing: first a guess per piece (distance / speed), optionally improved by moving time between pieces
    (gradient descent on the snap cost, total time fixed), then the WHOLE trajectory is stretched or squeezed in
    time until it just fits the drone's limits (speed, acceleration, tilt). Stretching time by k divides every
    speed by k and every acceleration by k^2, so the shape stays the same: only how fast it is flown changes.

Units: metres, seconds, world frame ENU (z up). Everything is NumPy; no optimisation library is needed.
"""

from __future__ import annotations

import math
from dataclasses import dataclass

import numpy as np

from backend.sim.config import GRAVITY_M_S2

ORDER = 8  # coefficients per axis per piece (7th-order polynomial)
CONTINUOUS_DERIVS = 4  # velocity, acceleration, jerk and snap are continuous at each waypoint
END_DERIVS = 3  # start and end at rest: velocity, acceleration, jerk = 0
_FACT = [math.factorial(k) for k in range(ORDER)]


class TrajectoryError(ValueError):
    """The waypoints can't make a trajectory (too few, repeated, bad gate)."""


def _deriv_row(r: int, tau: float) -> np.ndarray:
    """Coefficients' contribution to the r-th derivative (w.r.t. tau) at tau: d^r/dtau^r sum c_k tau^k."""
    row = np.zeros(ORDER)
    for k in range(r, ORDER):
        row[k] = _FACT[k] / _FACT[k - r] * tau ** (k - r)
    return row


def _snap_cost_unit() -> np.ndarray:
    """Integral over tau in [0, 1] of (4th derivative)^2, as a matrix on the coefficients."""
    q = np.zeros((ORDER, ORDER))
    for i in range(4, ORDER):
        for j in range(4, ORDER):
            ci = _FACT[i] / _FACT[i - 4]
            cj = _FACT[j] / _FACT[j - 4]
            q[i, j] = ci * cj / (i + j - 7)
    return q


Q_UNIT = _snap_cost_unit()
ROW_0 = [_deriv_row(r, 0.0) for r in range(ORDER)]
ROW_1 = [_deriv_row(r, 1.0) for r in range(ORDER)]


def _perpendiculars(d: np.ndarray) -> tuple[np.ndarray, np.ndarray]:
    """Two unit vectors at right angles to d and to each other."""
    d = d / np.linalg.norm(d)
    helper = np.array([0.0, 0.0, 1.0]) if abs(d[2]) < 0.9 else np.array([1.0, 0.0, 0.0])
    e1 = np.cross(d, helper)
    e1 /= np.linalg.norm(e1)
    return e1, np.cross(d, e1)


def solve_min_snap(points: np.ndarray, times_s: np.ndarray, directions: dict[int, np.ndarray] | None = None) -> np.ndarray:
    """The minimum-snap coefficients, shape (pieces, 3 axes, 8), in normalised time per piece.

    points: (N, 3) waypoints; times_s: (N-1,) piece durations; directions: {waypoint index: unit vector} for
    interior waypoints that must be flown through along that direction.
    """
    n = len(times_s)
    nv = n * 3 * ORDER

    def idx(seg: int, axis: int) -> slice:
        start = (seg * 3 + axis) * ORDER
        return slice(start, start + ORDER)

    rows: list[np.ndarray] = []
    rhs: list[float] = []

    def add(entries: list[tuple[int, int, np.ndarray]], value: float) -> None:
        row = np.zeros(nv)
        for seg, axis, coeffs in entries:
            row[idx(seg, axis)] += coeffs
        rows.append(row)
        rhs.append(value)

    for i in range(n):
        for a in range(3):
            add([(i, a, ROW_0[0])], points[i, a])  # starts at waypoint i
            add([(i, a, ROW_1[0])], points[i + 1, a])  # ends at waypoint i+1
    for a in range(3):
        for r in range(1, END_DERIVS + 1):
            add([(0, a, ROW_0[r])], 0.0)  # at rest at the start ...
            add([(n - 1, a, ROW_1[r])], 0.0)  # ... and at the end
    for j in range(1, n):  # smooth joins: derivative r matches (in real time, so divide by T^r)
        t0, t1 = times_s[j - 1], times_s[j]
        for a in range(3):
            for r in range(1, CONTINUOUS_DERIVS + 1):
                add([(j - 1, a, ROW_1[r] / t0**r), (j, a, -ROW_0[r] / t1**r)], 0.0)
    for j, d in (directions or {}).items():
        if not 0 < j < n:
            raise TrajectoryError("Only waypoints in the middle of a trajectory can be gates.")
        for e in _perpendiculars(np.asarray(d, dtype=float)):  # velocity has no component across the gate's arrow
            add([(j - 1, a, ROW_1[1] * e[a]) for a in range(3)], 0.0)

    a_mat = np.array(rows)
    b = np.array(rhs)
    q = np.zeros((nv, nv))
    for i in range(n):
        for a in range(3):
            q[idx(i, a), idx(i, a)] = Q_UNIT / times_s[i] ** 7
    m = len(rows)
    kkt = np.zeros((nv + m, nv + m))
    kkt[:nv, :nv] = 2.0 * q
    kkt[:nv, nv:] = a_mat.T
    kkt[nv:, :nv] = a_mat
    sol = np.linalg.lstsq(kkt, np.concatenate([np.zeros(nv), b]), rcond=None)[0]
    return sol[:nv].reshape(n, 3, ORDER)


def snap_cost(coeffs: np.ndarray, times_s: np.ndarray) -> float:
    """Total snap squared over the whole trajectory (real time)."""
    return float(sum(c @ Q_UNIT @ c / t**7 for seg, t in zip(coeffs, times_s) for c in seg))


@dataclass
class Trajectory:
    """A piecewise-polynomial trajectory. `sample(t)` gives position, velocity, acceleration, jerk at time t."""

    points: np.ndarray  # (N, 3) waypoints
    times_s: np.ndarray  # (N-1,) piece durations
    coeffs: np.ndarray  # (N-1, 3, 8)
    directions: dict[int, np.ndarray]

    @property
    def duration_s(self) -> float:
        return float(self.times_s.sum())

    def knots_s(self) -> np.ndarray:
        """When each waypoint is reached."""
        return np.concatenate([[0.0], np.cumsum(self.times_s)])

    def sample(self, t_s) -> dict[str, np.ndarray]:
        """Position / velocity / acceleration / jerk at time(s) t (clamped to the trajectory). Arrays of shape (k, 3)."""
        t = np.atleast_1d(np.asarray(t_s, dtype=float))
        t = np.clip(t, 0.0, self.duration_s)
        knots = self.knots_s()
        seg = np.clip(np.searchsorted(knots, t, side="right") - 1, 0, len(self.times_s) - 1)
        T = self.times_s[seg]
        tau = (t - knots[seg]) / T
        out = {}
        for r, name in enumerate(("pos", "vel", "acc", "jerk", "snap")):
            # d^r/dt^r = (1/T^r) d^r/dtau^r
            k = np.arange(r, ORDER)
            fac = np.array([_FACT[kk] / _FACT[kk - r] for kk in k])
            powers = tau[:, None] ** (k - r)[None, :] * fac[None, :]  # (samples, ORDER-r)
            c = self.coeffs[seg][:, :, r:]  # (samples, 3, ORDER-r)
            out[name] = np.einsum("sk,sak->sa", powers, c) / (T**r)[:, None]
        return out

    def sample_one(self, t_s: float) -> tuple[np.ndarray, np.ndarray, np.ndarray, np.ndarray]:
        """Fast version of sample() for one time: (position, velocity, acceleration, jerk)."""
        cache = self.__dict__.get("_fast")
        if cache is None:
            knots = self.knots_s()
            # per piece and derivative r: coefficients already divided by T^r and multiplied by k!/(k-r)!
            d = np.zeros((len(self.times_s), 4, 3, ORDER))
            for r in range(4):
                fac = np.array([_FACT[k] / _FACT[k - r] if k >= r else 0.0 for k in range(ORDER)])
                for i, T in enumerate(self.times_s):
                    shifted = np.zeros((3, ORDER))
                    shifted[:, : ORDER - r] = (self.coeffs[i] * fac)[:, r:]
                    d[i, r] = shifted / T**r
            cache = self.__dict__["_fast"] = (knots, d)
        knots, d = cache
        t = min(max(float(t_s), 0.0), float(knots[-1]))
        i = min(int(np.searchsorted(knots, t, side="right")) - 1, len(self.times_s) - 1)
        tau = (t - knots[i]) / self.times_s[i]
        powers = tau ** np.arange(ORDER)
        out = d[i] @ powers  # (4 derivatives, 3 axes)
        return out[0], out[1], out[2], out[3]

    def scaled(self, k: float) -> "Trajectory":
        """The same curve flown k times slower (k < 1: faster). The coefficients (in normalised time) don't change."""
        return Trajectory(self.points, self.times_s * k, self.coeffs, self.directions)

    def peaks(self, samples_per_piece: int = 60) -> dict[str, float]:
        """Largest speed, acceleration, tilt and thrust (per kg) along the trajectory."""
        t = np.concatenate([np.linspace(k0, k0 + T, samples_per_piece, endpoint=False)
                            for k0, T in zip(self.knots_s()[:-1], self.times_s)] + [[self.duration_s]])  # fmt: skip
        s = self.sample(t)
        v, a, j = s["vel"], s["acc"], s["jerk"]
        thrust = a + np.array([0.0, 0.0, GRAVITY_M_S2])  # what the propellers must provide, per kg
        tilt = np.degrees(np.arctan2(np.hypot(thrust[:, 0], thrust[:, 1]), thrust[:, 2]))
        return {
            "speed_m_s": float(np.linalg.norm(v, axis=1).max()),
            "accel_m_s2": float(np.linalg.norm(a, axis=1).max()),
            "jerk_m_s3": float(np.linalg.norm(j, axis=1).max()),
            "tilt_deg": float(tilt.max()),
            "thrust_per_kg": float(np.linalg.norm(thrust, axis=1).max()),
            "min_thrust_per_kg": float(np.linalg.norm(thrust, axis=1).min()),
        }

    def length_m(self) -> float:
        p = self.sample(np.linspace(0.0, self.duration_s, 600))["pos"]
        return float(np.linalg.norm(np.diff(p, axis=0), axis=1).sum())

    def polyline(self, n: int = 200) -> list[list[float]]:
        p = self.sample(np.linspace(0.0, self.duration_s, n))["pos"]
        return np.round(p, 3).tolist()


@dataclass(frozen=True)
class Limits:
    """What the drone can do. The trajectory is timed so it never asks for more."""

    speed_m_s: float = 5.0
    accel_m_s2: float = 6.0
    tilt_deg: float = 35.0
    thrust_per_kg: float = 1.7 * GRAVITY_M_S2  # leave 15 % of a 2:1 drone's thrust for corrections


def initial_times(points: np.ndarray, speed_m_s: float, accel_m_s2: float) -> np.ndarray:
    """A first guess per piece: the time to cover it accelerating and braking at the limits (a trapezoid)."""
    d = np.linalg.norm(np.diff(points, axis=0), axis=1)
    if np.any(d < 1e-3):
        raise TrajectoryError("Two waypoints in a row are in the same place.")
    ramp = speed_m_s**2 / accel_m_s2  # distance to speed up and slow down again
    return np.where(d > ramp, d / speed_m_s + speed_m_s / accel_m_s2, 2.0 * np.sqrt(d / accel_m_s2))


def optimise_times(points: np.ndarray, times_s: np.ndarray, directions: dict, iterations: int = 40) -> np.ndarray:
    """Share a FIXED total time between the pieces so the snap cost is lowest (Mellinger & Kumar, section IV).

    Gradient descent on the shares (softmax of free numbers), with a numerical gradient: each piece in turn gets
    a little more time and we see whether the cost drops. Cheap: a solve is a few milliseconds.
    """
    total = float(times_s.sum())
    theta = np.log(times_s / total)

    def cost(th: np.ndarray) -> float:
        w = np.exp(th - th.max())
        t = total * w / w.sum()
        return snap_cost(solve_min_snap(points, t, directions), t)

    c = cost(theta)
    step = 0.3
    for _ in range(iterations):
        grad = np.zeros_like(theta)
        for i in range(len(theta)):
            th = theta.copy()
            th[i] += 1e-4
            grad[i] = (cost(th) - c) / 1e-4
        norm = float(np.linalg.norm(grad))
        if norm < 1e-12:
            break
        while step > 1e-4:
            trial = theta - step * grad / norm
            ct = cost(trial)
            if ct < c:
                theta, c = trial, ct
                step *= 1.5
                break
            step *= 0.5
        else:
            break
    w = np.exp(theta - theta.max())
    return total * w / w.sum()


def fit_to_limits(traj: Trajectory, limits: Limits) -> Trajectory:
    """Stretch (or squeeze) time until the trajectory just respects every limit.

    Speed and acceleration scale exactly (1/k and 1/k^2); tilt and thrust include gravity, so they're found by
    bisection. The tightest limit decides how fast the drone can fly this shape.
    """
    p = traj.peaks()
    k = max(p["speed_m_s"] / limits.speed_m_s, math.sqrt(p["accel_m_s2"] / limits.accel_m_s2))

    def ok(kk: float) -> bool:
        q = traj.scaled(kk).peaks()
        return q["tilt_deg"] <= limits.tilt_deg and q["thrust_per_kg"] <= limits.thrust_per_kg \
            and q["min_thrust_per_kg"] >= 0.3 * GRAVITY_M_S2  # never close to free fall
    if not ok(k):
        lo, hi = k, k * 2.0
        while not ok(hi):
            lo, hi = hi, hi * 2.0
            if hi > 1e3:
                raise TrajectoryError("This shape can't be flown within the drone's limits.")
        for _ in range(30):
            mid = 0.5 * (lo + hi)
            lo, hi = (lo, mid) if ok(mid) else (mid, hi)
        k = hi
    return traj.scaled(k)


def plan_trajectory(points, limits: Limits = Limits(), directions: dict[int, np.ndarray] | None = None,
                    optimise: bool = True) -> Trajectory:
    """Waypoints (N, 3) -> the fastest minimum-snap trajectory within the limits."""
    pts = np.asarray(points, dtype=float)
    if pts.ndim != 2 or pts.shape[1] != 3 or len(pts) < 2:
        raise TrajectoryError("Need at least two 3D waypoints.")
    dirs = {int(k): np.asarray(v, dtype=float) / np.linalg.norm(v) for k, v in (directions or {}).items()}
    times = initial_times(pts, limits.speed_m_s, limits.accel_m_s2)
    if optimise and len(times) > 1:
        times = optimise_times(pts, times, dirs)
    traj = Trajectory(pts, times, solve_min_snap(pts, times, dirs), dirs)
    return fit_to_limits(traj, limits)
