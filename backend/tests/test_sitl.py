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

    with launch.start(speedup=3) as sitl:
        report = fly(sitl.url, log=lambda *_: None)
    assert report.ok
    assert 9.5 <= report.max_alt_m <= 17.0  # 10 m flight, RTL climbs to 15 m
    assert report.fly_to_error_m < 1.0
    # "Reached" = within WP_RADIUS_M (2 m), measured from our latest position sample. Samples arrive ~10x per
    # real second, so at 3x speed the drone can move up to ~1.5 m between them: allow 2 + 1.5 m.
    assert len(report.corner_errors_m) == 4 and max(report.corner_errors_m) < 3.5
    assert report.landed_from_home_m < 2.0


# ---- Step 4: the 🛰 ArduPilot panel (bridge) --------------------------------------------------------

from backend.sitl.bridge import PAD_OFFSET_M, SitlBridge, dronelab_step, step_metrics  # noqa: E402
from backend.sitl.fly import attitude_ned_to_enu, latlon_to_ne  # noqa: E402


def test_attitude_ned_to_enu_heading_and_pitch():
    # Facing north (heading 0) = DroneLab yaw 90°; facing east (heading 90°) = yaw 0.
    assert attitude_ned_to_enu(0, 0, 0)[2] == pytest.approx(math.pi / 2)
    assert attitude_ned_to_enu(0, 0, math.pi / 2)[2] == pytest.approx(0.0, abs=1e-12)
    assert attitude_ned_to_enu(0, 0, math.radians(225))[2] == pytest.approx(math.radians(-135))  # wraps
    # Nose up is +pitch in NED (y right) but -pitch in DroneLab (y left); roll is the same.
    assert attitude_ned_to_enu(0.1, 0.2, 0)[:2] == (0.1, -0.2)


def test_latlon_to_ne_undoes_offset_latlon():
    lat, lon = launch.HOME[:2]
    n, e = latlon_to_ne(*offset_latlon(lat, lon, north_m=12.5, east_m=-7.0), lat, lon)
    assert (n, e) == (pytest.approx(12.5, abs=1e-6), pytest.approx(-7.0, abs=1e-6))


def test_step_metrics_on_a_known_curve():
    t = [0.0, 1.0, 2.0, 3.0, 4.0, 4.0, 5.0, 6.0]  # a repeated time is ignored
    north = [0.0, 1.0, 6.0, 10.5, 10.1, 10.1, 10.0, 10.0]
    m = step_metrics(t, north, 10.0)
    assert m["rise_s"] == pytest.approx(2.0)  # 1 m (10%) at 1 s -> 10.5 m (>= 90%) at 3 s
    assert m["overshoot_m"] == pytest.approx(0.5)
    assert m["settle_s"] == pytest.approx(4.0)  # within 0.2 m from t = 4 s on
    assert m["peak_speed_m_s"] > 0
    assert step_metrics([0, 1, 2], [0, 5, 7], 10.0)["settle_s"] is None  # never got there


def test_dronelab_side_of_the_comparison():
    d = dronelab_step()
    m = step_metrics(d["t_s"], d["north_m"], 10.0)
    assert d["north_m"][-1] == pytest.approx(10.0, abs=0.1)
    assert m["peak_speed_m_s"] <= 3.1  # DroneLab's horizontal speed limit is 3 m/s
    assert m["settle_s"] is not None and m["settle_s"] < 8


def test_panel_commands_explain_themselves_without_sitl(monkeypatch):
    b = SitlBridge()
    assert b.status() is None  # nothing in the state frames until the panel is used
    with pytest.raises(ValueError, match="Start ArduPilot first"):
        b.handle({"action": "takeoff"})
    with pytest.raises(ValueError, match="Unknown ArduPilot action"):
        b.handle({"action": "flip"})
    monkeypatch.setattr(launch, "is_built", lambda: False)
    assert "isn't built" in b.start()
    assert b.status()["state"] == "error"


def test_sitl_command_over_the_websocket_without_starting_it():
    from fastapi.testclient import TestClient

    from backend import api

    with TestClient(api.app).websocket_connect("/ws/sim") as ws:
        ws.receive_json()  # Connected.
        state = ws.receive_json()
        assert state["type"] == "state" and "sitl" in state
        ws.send_json({"cmd": "sitl", "action": "goto", "x_m": 1})  # not started (and y_m missing)
        msg = ws.receive_json()
        while msg.get("type") == "state":
            msg = ws.receive_json()
        assert msg["type"] == "error"


@pytest.mark.skipif(os.environ.get("DRONELAB_SITL") != "1", reason="real SITL flight: set DRONELAB_SITL=1")
@pytest.mark.skipif(not launch.is_built(), reason="ArduCopter SITL not built (scripts/sitl_setup.sh)")
def test_real_arducopter_in_the_panel_takes_off_flies_to_compares_and_lands():
    import time

    b = SitlBridge()

    def wait(cond, limit_s=150):
        t = time.monotonic()
        while not cond(b.status() or {}):
            assert time.monotonic() - t < limit_s, b.status()
            time.sleep(0.2)

    try:
        b.start(speedup=3)
        wait(lambda s: s.get("state") == "ready")
        assert b.status()["pos_m"][:2] == pytest.approx(list(PAD_OFFSET_M), abs=0.3)  # on its own pad
        b.handle({"action": "takeoff", "alt_m": 5})
        wait(lambda s: s.get("action") is None and s.get("armed"))
        b.handle({"action": "goto", "x_m": 6, "y_m": 8, "alt_m": 5})
        wait(lambda s: s.get("action") is None)
        assert b.status()["local_m"] == pytest.approx([6, 8, 5], abs=0.3)
        b.handle({"action": "compare"})
        wait(lambda s: s.get("compare") is not None, 200)
        ap = b.status()["compare"]["ardupilot"]
        assert ap["north_m"][-1] == pytest.approx(10.0, abs=0.3) and ap["settle_s"] is not None
        b.handle({"action": "rtl"})
        wait(lambda s: s.get("action") is None and not s.get("armed"), 200)
        assert b.status()["local_m"][:2] == pytest.approx([0, 0], abs=0.5)
    finally:
        b.stop()


# ---- Step 3: QGroundControl settings (no QGroundControl needed) -------------------------------------

def test_qgc_settings_turn_off_auto_connect_and_keep_one_local_tcp_link(tmp_path):
    from backend.sitl.qgc import LINK_NAME, configure

    ini = tmp_path / "QGroundControl.ini"
    ini.write_text(
        "[General]\nSettingsVersion=9\n\n"
        "[AutoConnectSettings]\nautoConnectUDP=false\n\n"  # a wrong section an earlier version wrote
        "[LinkConfigurations]\ncount=2\nLink0\\name=old\nLink1\\name=radio\n\n"
        "[LinkManager]\nautoConnectUDP=true\nsomethingElse=7\n"
    )
    configure(5762, ini)
    configure(5762, ini)  # running twice changes nothing
    text = ini.read_text()
    assert "[AutoConnectSettings]" not in text and "SettingsVersion=9" in text and "somethingElse=7" in text
    manager = text.split("[LinkManager]")[1].split("[")[0]
    for key in ("autoConnectUDP", "autoConnectZeroConf", "autoConnectPixhawk", "autoConnectSiKRadio"):
        assert f"{key}=false" in manager  # no 0.0.0.0 listeners, no serial/USB scanning
    links = text.split("[LinkConfigurations]")[1].split("[")[0]
    assert "count=1" in links and f"Link0\\name={LINK_NAME}" in links and "Link1" not in links
    assert "Link0\\host=127.0.0.1" in links and "Link0\\port=5762" in links and "Link0\\type=2" in links
    assert text.count("[LinkConfigurations]") == 1 and "EnableAutoUpload=false" in text


def test_wind_direction_conversion_to_ardupilot():
    from backend.sitl.plan_flight import wind_params

    assert wind_params(5.0, 90.0)["SIM_WIND_DIR"] == pytest.approx(180.0)  # blowing north = coming from the south
    assert wind_params(5.0, 0.0)["SIM_WIND_DIR"] == pytest.approx(270.0)  # blowing east = coming from the west


@pytest.mark.skipif(os.environ.get("DRONELAB_SITL") != "1", reason="real SITL flight: set DRONELAB_SITL=1")
@pytest.mark.skipif(not launch.is_built(), reason="ArduCopter SITL not built (scripts/sitl_setup.sh)")
def test_the_same_survey_on_real_ardupilot_covers_the_field():
    from backend.sitl.plan_flight import compare

    res = compare("orchard", alt_m=15.0, speedup=4.0, say=lambda _t: None)
    dl, ap = res["dronelab"], res["ardupilot"]
    assert dl["coverage_pct"] == 100.0
    assert ap["coverage_pct"] >= 95.0 and len(ap["photos_xy"]) >= 0.7 * dl["photos"]
    assert ap["landing_error_m"] < 2.0 and ap["flight_time_s"] < 3 * dl["flight_time_s"]


def test_offboard_limits_open_up_to_fit_the_plan():
    from backend.autonomy.agile import plan_track
    from backend.sitl.offboard import MASK_POS_ONLY, MASK_POS_VEL_ACC, ap_limits

    plan = plan_track("figure_eight", "geometric", 12.0, tilt_deg=30.0)
    lim = ap_limits(plan)
    assert lim["PSC_NE_JERK"] >= plan.trajectory.peaks()["jerk_m_s3"] * 1.4  # the shaper must allow the plan's jerk
    assert lim["ATC_ANGLE_MAX"] > 30.0 and lim["WP_ACC"] == pytest.approx(plan.limits.accel_m_s2, abs=0.01)
    # MAVLink type_mask bits are IGNORE flags: pos+vel+acc uses bits 0-8 (all zero), ignores yaw (10) + yaw rate (11)
    assert MASK_POS_VEL_ACC & 0b1_1111_1111 == 0 and MASK_POS_VEL_ACC & (1 << 9) == 0
    assert MASK_POS_ONLY & 0b111 == 0 and MASK_POS_ONLY & 0b1_1111_1000 == 0b1_1111_1000


@pytest.mark.skipif(os.environ.get("DRONELAB_SITL") != "1", reason="real SITL flight: set DRONELAB_SITL=1")
@pytest.mark.skipif(not launch.is_built(), reason="ArduCopter SITL not built (scripts/sitl_setup.sh)")
def test_real_arducopter_flies_our_trajectory_offboard():
    """Stream DroneLab's minimum-snap trajectory (pos + vel + accel) to ArduCopter in GUIDED: every ring, on the line."""
    from backend.sitl.offboard import compare

    res = compare("square_loop", tilt_deg=30.0, speedup=1.0, methods=("guided_pva",), say=lambda _t: None)
    ap = res["ardupilot"]["guided_pva"]
    assert ap["rings_passed"] == ap["rings_total"] == 4, ap
    assert ap["path_err_mean_m"] < 0.5 and ap["lap_time_s"] < 1.3 * res["dronelab"]["geometric"]["lap_time_s"]
