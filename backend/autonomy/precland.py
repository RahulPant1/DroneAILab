"""Precision landing: find a printed marker with a down-looking camera and land on it.

Why: GPS is often 1-2 m off, and the error wanders over a minute or so (sensors.gps_drift_m). A drone that
lands "on the GPS coordinate of the pad" can miss it by that much. A camera sees where the pad really is
RELATIVE to the drone, and that is all it needs: GPS error doesn't matter any more.

1. Picture. A 128 × 96 grey-scale camera, fixed under the drone, looking down (60° across). Each pixel is a
   ray from the camera; we follow it to the ground (z = 0) using the TRUE pose and look up what is there:
   grass (a textured pattern) or a pad (light mat, black square ring = the marker). Plus pixel noise.
2. Detect. Pixels much darker than grass are marker. Their centre (centroid) is the pad's centre. If the dark
   blob touches the picture's edge, only part of the marker is visible: ignore it (the centre would be wrong).
3. Where is it? Turn that pixel back into a ray, rotate it with the drone's ESTIMATED tilt and heading, and
   stretch it down to the ground using the ESTIMATED height: that gives the pad's offset from the drone, in
   metres east/north. The target = (where I think I am) + offset, so a GPS error cancels out.
4. Land. Descend while the marker is in view, slowing down when not yet above it. Marker lost high up: stop
   and look (hold height). Not found for a while: give up and land on GPS. Below 2 m the marker may not fit in the
   picture, so the last target is kept for the final touch-down (ArduPilot does the same).
"""

from __future__ import annotations

import base64
import math

import numpy as np

from backend.sim.frames import quat_to_rotation_matrix
from backend.sim.physics import State

IMAGE_W, IMAGE_H = 128, 96
HFOV_RAD = math.radians(60.0)
FOCAL_PX = (IMAGE_W / 2) / math.tan(HFOV_RAD / 2)
CAMERA_HZ = 15.0
PIXEL_NOISE = 0.03
DARK_LEVEL = 0.13  # below this = marker (grass never gets this dark)
MIN_MARKER_PIXELS = 6
MAT_SCALE = 1.5  # the light mat around the marker is 1.5 × the marker's size
RING_INNER = 0.5  # the marker is a black square ring; its hole is half its size

FILTER = 0.5  # how much each new sighting moves the target (0-1)
SEEN_RECENTLY_S = 0.5
FINAL_ALT_M = 2.0  # below this the marker may not fit in the picture: keep the last target
SEARCH_GIVE_UP_S = 6.0
DESCENT_FAST_M_S, DESCENT_SLOW_M_S, DESCENT_CREEP_M_S = 1.0, 0.5, 0.15


def _rays_body() -> np.ndarray:
    """Unit-length ray per pixel, in the body frame (x forward, y left, z up). Top of the picture = forward."""
    u, v = np.meshgrid(np.arange(IMAGE_W) + 0.5 - IMAGE_W / 2, np.arange(IMAGE_H) + 0.5 - IMAGE_H / 2)
    rays = np.stack([-v / FOCAL_PX, -u / FOCAL_PX, -np.ones_like(u)], axis=-1)
    return rays / np.linalg.norm(rays, axis=-1, keepdims=True)


RAYS_BODY = _rays_body()  # (H, W, 3)


def grass(x: np.ndarray, y: np.ndarray) -> np.ndarray:
    return (0.44 + 0.06 * np.sin(1.7 * x + 0.3 * y) + 0.05 * np.sin(0.9 * y - 2.3 * x)
            + 0.04 * np.sin(5.1 * x + 4.3 * y) + 0.03 * np.sin(11.0 * y - 7.0 * x))


def ground_shade(x: np.ndarray, y: np.ndarray, pads) -> np.ndarray:
    """Brightness (0 = black, 1 = white) of the ground at (x, y): grass, mats and markers."""
    shade = grass(x, y)
    for px, py, size in pads:
        dx, dy = np.abs(x - px), np.abs(y - py)
        half = size / 2
        shade = np.where(np.maximum(dx, dy) <= half * MAT_SCALE, 0.8, shade)  # light mat
        ring = (np.maximum(dx, dy) <= half) & (np.maximum(dx, dy) >= half * RING_INNER)
        shade = np.where(ring, 0.05, shade)  # black square ring = the marker
    return shade


def render(truth: State, pads, rng: np.random.Generator | None = None) -> np.ndarray:
    """What the down camera sees from the TRUE pose: (H, W) brightness 0-1."""
    rot = quat_to_rotation_matrix(truth.quat)
    rays = RAYS_BODY @ rot.T  # into the world frame
    z = max(float(truth.pos_m[2]), 0.02)
    rz = rays[..., 2]
    t = np.where(rz < -1e-6, z / np.maximum(-rz, 1e-6), np.inf)
    gx = truth.pos_m[0] + t * rays[..., 0]
    gy = truth.pos_m[1] + t * rays[..., 1]
    img = np.where(np.isfinite(t), ground_shade(np.nan_to_num(gx), np.nan_to_num(gy), pads), 0.6)  # sky: grey
    if rng is not None:
        img = img + PIXEL_NOISE * rng.standard_normal(img.shape)
    return np.clip(img, 0.0, 1.0)


def detect(img: np.ndarray) -> dict | None:
    """Find the marker: the centre of the dark pixels. None if too few, or if it is cut off by the picture's edge."""
    dark = img < DARK_LEVEL
    # A real marker pixel has dark neighbours; a lone dark pixel is just noise.
    near = np.zeros_like(dark)
    near[1:, :] |= dark[:-1, :]
    near[:-1, :] |= dark[1:, :]
    near[:, 1:] |= dark[:, :-1]
    near[:, :-1] |= dark[:, 1:]
    dark &= near
    n = int(dark.sum())
    if n < MIN_MARKER_PIXELS:
        return None
    if dark[0, :].any() or dark[-1, :].any() or dark[:, 0].any() or dark[:, -1].any():
        return None  # partly outside the picture
    rows, cols = np.nonzero(dark)
    return {"u": float(cols.mean()) + 0.5, "v": float(rows.mean()) + 0.5, "pixels": n,
            "box": [int(cols.min()), int(rows.min()), int(cols.max()), int(rows.max())]}


def pixel_to_offset(u: float, v: float, est: State) -> tuple[float, float] | None:
    """The ground point seen at pixel (u, v), as an (east, north) offset from the drone, using the drone's
    ESTIMATED attitude and height. None if that pixel looks at the sky."""
    ray_b = np.array([-(v - IMAGE_H / 2) / FOCAL_PX, -(u - IMAGE_W / 2) / FOCAL_PX, -1.0])
    ray = quat_to_rotation_matrix(est.quat) @ ray_b
    if ray[2] > -1e-3:
        return None
    t = max(float(est.pos_m[2]), 0.05) / -ray[2]
    return float(t * ray[0]), float(t * ray[1])


def image_b64(img: np.ndarray) -> str:
    return base64.b64encode((np.clip(img, 0, 1) * 255).astype(np.uint8).tobytes()).decode("ascii")


class PrecisionLander:
    """Controls the landing once the drone is over the pad (by GPS). Gives (x, y, alt) setpoints in the
    estimate's frame. mode "vision" uses the camera; mode "gps" just descends onto the pad's map coordinate."""

    def __init__(self, pad: tuple[float, float, float], mode: str = "vision", seed: int = 0, pads=None):
        self.pad = pad  # (x, y, marker size)
        self.pads = pads or [pad]  # everything the camera can see
        self.mode = mode
        self.rng = np.random.default_rng(seed + 4242)
        self.target = [pad[0], pad[1]]  # where it thinks the pad is (estimate frame)
        self.alt_sp: float | None = None
        self.state = "descend"
        self.last_seen_s = -math.inf
        self.search_since: float | None = None
        self.sightings = 0
        self.first_fix_alt_m: float | None = None
        self.first_correction_m: float | None = None
        self.events: list[tuple[float, str, str]] = []  # (t, kind, message)
        self.image: np.ndarray | None = None
        self.detection: dict | None = None
        self._next_frame_s = 0.0
        self._t_last: float | None = None
        self.keyframes: list[dict] = []  # a few pictures on the way down (for the Lab), each with what was detected
        self._keyframe_alts = [8.0, 4.0, 2.2]

    def _say(self, t: float, kind: str, msg: str) -> None:
        self.events.append((t, kind, msg))

    def update(self, t: float, est: State, truth: State) -> tuple[float, float, float]:
        dt = 0.0 if self._t_last is None else max(0.0, t - self._t_last)
        self._t_last = t
        h = float(est.pos_m[2])
        if self.alt_sp is None:
            self.alt_sp = h
        if t >= self._next_frame_s:  # GPS mode records the pictures too, but doesn't use them
            self._next_frame_s = t + 1.0 / CAMERA_HZ
            self.image = render(truth, self.pads, self.rng)
            self.detection = detect(self.image) if h > 0.8 else None
            if self._keyframe_alts and (h < self._keyframe_alts[0] or (not self.keyframes and self.detection)):
                while self._keyframe_alts and h < self._keyframe_alts[0]:
                    self._keyframe_alts.pop(0)
                if len(self.keyframes) < 4:
                    self.keyframes.append({"alt_m": round(h, 1), "t_s": round(t, 2), "image": image_b64(self.image),
                                           "w": IMAGE_W, "h": IMAGE_H, "detection": self.detection})
            if self.mode == "gps":
                self.detection = None
            if self.detection:
                off = pixel_to_offset(self.detection["u"], self.detection["v"], est)
                if off:
                    meas = (float(est.pos_m[0]) + off[0], float(est.pos_m[1]) + off[1])
                    if self.sightings == 0:
                        self.first_fix_alt_m = h
                        self.first_correction_m = math.dist(meas, self.target)
                        self.target = list(meas)
                        self._say(t, "marker", f"Marker spotted from {h:.1f} m up: the pad is {self.first_correction_m:.2f} m "
                                               f"from where GPS put it. Correcting.")
                    else:
                        self.target = [self.target[0] + FILTER * (meas[0] - self.target[0]),
                                       self.target[1] + FILTER * (meas[1] - self.target[1])]
                    self.sightings += 1
                    self.last_seen_s = t

        if self.mode == "gps" or self.state == "gps":
            rate = 1.5 if self.alt_sp > 5.0 else 0.6
        elif h < FINAL_ALT_M and self.sightings:
            rate = DESCENT_SLOW_M_S  # too close to see the whole marker: trust the last target
        elif t - self.last_seen_s < SEEN_RECENTLY_S:
            self.search_since = None
            off_m = math.hypot(self.target[0] - est.pos_m[0], self.target[1] - est.pos_m[1])
            rate = DESCENT_FAST_M_S if off_m < 0.3 and h > 4 else DESCENT_SLOW_M_S if off_m < 0.4 else DESCENT_CREEP_M_S
        else:
            if self.search_since is None:
                self.search_since = t
                self.alt_sp = h  # stop where it is and look
                self._say(t, "search", f"No marker in view at {h:.1f} m: holding to search.")
            if t - self.search_since > SEARCH_GIVE_UP_S:
                self.state = "gps"
                self._say(t, "fallback", "Marker not found: landing on the GPS position instead.")
            rate = 0.0
        self.alt_sp = max(-0.3, self.alt_sp - rate * dt)
        return self.target[0], self.target[1], self.alt_sp

    def snapshot(self) -> dict:
        return {
            "mode": self.mode,
            "state": self.state,
            "sightings": self.sightings,
            "seen": self.detection is not None,
            "detection": self.detection,
            "target": [round(self.target[0], 3), round(self.target[1], 3)],
            "image": image_b64(self.image) if self.image is not None else None,
            "w": IMAGE_W,
            "h": IMAGE_H,
        }
