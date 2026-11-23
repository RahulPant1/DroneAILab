"""Companion computer: perception as a separate process feeding ArduPilot (LANDING_TARGET, OBSTACLE_DISTANCE).

The conventions are checked offline with a fake vehicle. The real flights (ArduCopter SITL + the companion process)
take about 4 minutes, so they only run when asked:
    DRONELAB_SITL=1 .venv/bin/pytest -q backend/tests/test_companion.py
"""

import math
import os
import threading

import numpy as np
import pytest

from backend.companion import service as S
from backend.sitl import launch
from backend.sitl.fly import offset_latlon

HOME = (-35.36, 149.16, 584.0)


class FakeView:
    """Stands in for VehicleView: a drone at (east, north, alt) metres from home, nose on a compass heading."""

    def __init__(self, e: float, n: float, alt: float, heading_deg: float, roll: float = 0.0, pitch: float = 0.0):
        self.lock = threading.Lock()
        self.home = HOME
        lat, lon = offset_latlon(HOME[0], HOME[1], n, e)
        yaw = math.radians(heading_deg)
        self.sim = {"lat": lat, "lon": lon, "alt": HOME[2] + alt, "roll": roll, "pitch": pitch, "yaw": yaw}
        self.att = (roll, pitch, yaw)
        self.rel_alt_m = alt


def body_frd_to_ne(x: float, y: float, heading_deg: float) -> tuple[float, float]:
    c, s = math.cos(math.radians(heading_deg)), math.sin(math.radians(heading_deg))
    return c * x - s * y, s * x + c * y


@pytest.mark.parametrize("heading", [0.0, 90.0, 200.0, 315.0])
def test_landing_target_points_at_the_pad_whatever_the_heading(heading):
    pad = (6.0, 4.0, 1.0)
    v = FakeView(7.5, 3.0, 10.0, heading)
    det = S.detect(S.SimCamera(v, [pad]).frame())
    assert det is not None
    lt = S.landing_target(det, v.att, v.rel_alt_m)
    n, e = body_frd_to_ne(lt["x"], lt["y"], heading)
    assert (n, e) == pytest.approx((4.0 - 3.0, 6.0 - 7.5), abs=0.15)  # pad minus drone, in north / east
    assert lt["z"] == pytest.approx(10.0, abs=0.2) and lt["distance"] > lt["z"]


def test_landing_target_uses_the_autopilots_tilt():
    """Tilted 10° nose-down, the pad straight below appears 'behind' in the picture; the target must still be below."""
    pitch = math.radians(-10.0)  # NED pitch: negative = nose down
    v = FakeView(6.0, 4.0, 10.0, 0.0, pitch=pitch)
    det = S.detect(S.SimCamera(v, [(6.0, 4.0, 1.0)]).frame())
    lt = S.landing_target(det, v.att, v.rel_alt_m)
    # rotate body FRD -> NED with the pitch: the line of sight should be (nearly) straight down
    x, z = lt["x"], lt["z"]
    north = math.cos(pitch) * x + math.sin(pitch) * z
    assert abs(north) < 0.2


@pytest.mark.parametrize("heading,sector", [(0.0, 0), (90.0, 270), (180.0, 180), (270.0, 90)])
def test_lidar_sectors_are_clockwise_from_the_nose(heading, sector):
    v = FakeView(0.0, 5.0, 5.0, heading)  # the wall is 6 m north
    r = S.SimLidar(v, S.WORLDS["avoid"]["boxes"]).scan()
    j = int(np.argmin(r))
    assert j * 5 == sector and r[j] == pytest.approx(6.0, abs=0.1)


def test_no_obstacle_is_sent_as_out_of_range_never_as_a_reading():
    v = FakeView(0.0, -20.0, 5.0, 0.0)  # far from the wall: nothing within 8 m
    cm = S.obstacle_distance_cm(S.SimLidar(v, S.WORLDS["avoid"]["boxes"]).scan())
    assert cm == [801] * 72  # max_distance + 1 = "nothing here" (a noisy 7.99 m would look like a ring of walls)
    assert S.obstacle_distance_cm(np.array([0.05, 3.0, 8.0])) == [20, 300, 801]


@pytest.mark.skipif(os.environ.get("DRONELAB_SITL") != "1", reason="real SITL flight: set DRONELAB_SITL=1")
@pytest.mark.skipif(not launch.is_built(), reason="ArduCopter SITL not built (scripts/sitl_setup.sh)")
def test_real_ardupilot_lands_on_the_pad_with_the_companion():
    from backend.sitl.companion_flight import run_job

    res = run_job("precland", say=lambda _t: None)
    assert res["without"]["landed_from_pad_m"] > 1.4
    assert res["with"]["landed_from_pad_m"] < 0.5 and res["with"]["companion"]["landing_target_sent"] > 20


@pytest.mark.skipif(os.environ.get("DRONELAB_SITL") != "1", reason="real SITL flight: set DRONELAB_SITL=1")
@pytest.mark.skipif(not launch.is_built(), reason="ArduCopter SITL not built (scripts/sitl_setup.sh)")
def test_real_ardupilot_goes_round_the_wall_with_the_companion():
    from backend.sitl.companion_flight import run_job

    res = run_job("avoid", say=lambda _t: None)
    assert res["without"]["flew_through_wall"]
    assert not res["with"]["flew_through_wall"] and res["with"]["min_clearance_m"] > 1.0
