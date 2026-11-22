"""Phase 5: ArduPilot SITL helpers, plus one real flight against ArduCopter SITL.

The real flight takes about a minute, so it only runs when asked:
    DRONELAB_SITL=1 .venv/bin/pytest -q backend/tests/test_sitl.py
(and needs scripts/sitl_setup.sh to have built ArduCopter).
"""

import math
import os

import pytest

from backend.sitl.fly import EARTH_RADIUS_M, enu_to_ned, offset_latlon, square_mission
from backend.sitl import launch


def test_enu_to_ned_swaps_east_and_north_and_flips_up():
    # 3 m east, 4 m north, 10 m up  ->  4 m north, 3 m east, 10 m UP = -10 m "down"
    assert enu_to_ned(3.0, 4.0, 10.0) == (4.0, 3.0, -10.0)


def test_offset_latlon_moves_the_right_distance_in_the_right_direction():
    lat, lon = launch.HOME[:2]
    n_lat, n_lon = offset_latlon(lat, lon, north_m=100.0, east_m=0.0)
    assert n_lon == lon and n_lat > lat
    assert math.radians(n_lat - lat) * EARTH_RADIUS_M == pytest.approx(100.0, abs=1e-6)
    e_lat, e_lon = offset_latlon(lat, lon, north_m=0.0, east_m=100.0)
    assert e_lat == lat and e_lon > lon  # east = longitude grows (even in the southern hemisphere)
    assert math.radians(e_lon - lon) * EARTH_RADIUS_M * math.cos(math.radians(lat)) == pytest.approx(100.0, abs=1e-6)


def test_square_mission_goes_north_first_and_ends_at_home():
    lat, lon = launch.HOME[:2]
    pts = square_mission(lat, lon, side_m=20.0, alt_m=10.0)
    assert len(pts) == 4 and all(alt == 10.0 for _, _, alt in pts)
    assert pts[0][0] > lat and pts[0][1] == pytest.approx(lon)  # first corner straight north
    assert pts[-1][:2] == pytest.approx((lat, lon))


@pytest.mark.skipif(os.environ.get("DRONELAB_SITL") != "1", reason="real SITL flight: set DRONELAB_SITL=1")
@pytest.mark.skipif(not launch.is_built(), reason="ArduCopter SITL not built (scripts/sitl_setup.sh)")
def test_real_arducopter_flies_the_demo_and_lands_home():
    from backend.sitl.fly import fly

    with launch.start(speedup=5) as sitl:
        report = fly(sitl.url, log=lambda *_: None)
    assert report.ok
    assert 9.5 <= report.max_alt_m <= 17.0  # 10 m flight, RTL climbs to 15 m
    assert report.fly_to_error_m < 1.0
    assert len(report.corner_errors_m) == 4 and max(report.corner_errors_m) < 3.0  # "reached" = within its radius
    assert report.landed_from_home_m < 2.0
