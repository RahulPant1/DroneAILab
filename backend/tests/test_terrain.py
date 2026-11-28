"""🏔 Terrain: ground heights in the physics, the live sim, and the offline parts of the downloader."""

import json
import struct
import zlib

import numpy as np
import pytest

from backend import terrain_fetch as tf
from backend.autonomy.live import MissionSim
from backend.sim import terrain
from backend.sim.config import get_preset
from backend.sim.physics import QuadcopterPhysics, State


def test_hills_flat_home_and_real_hills():
    t = terrain.hills()
    assert t.height(0, 0) == 0.0 and t.height(2.0, 1.0) == 0.0  # the take-off mat
    assert t.height(0, 75) > 15  # the ridge to the north
    assert t.height(45, -5) > 10  # the knoll to the east
    assert t.height(-50, -45) < 0  # the dip
    # bilinear lookup reproduces grid nodes
    r, c = 30, 70
    assert t.raw_height(t.x_min_m + c * t.cell_m, t.y_min_m + r * t.cell_m) == pytest.approx(float(t.heights_m[r, c]), abs=1e-4)


def test_blend_is_continuous():
    t = terrain.hills()
    prev = t.height(0, terrain.HOME_FLAT_M)
    for i in range(1, 60):
        d = terrain.HOME_FLAT_M + i * 0.05
        h = t.height(0, d)
        assert abs(h - prev) < 0.2
        prev = h


def test_to_dict_round_trip():
    import base64
    d = terrain.hills().to_dict()
    g = np.frombuffer(base64.b64decode(d["heights_b64"]), dtype="<f4").reshape(d["rows"], d["cols"])
    assert g[d["rows"] // 2, d["cols"] // 2] == 0.0  # home flattened
    assert d["max_m"] > 15


def test_physics_lands_on_a_hillside():
    """Default physics = flat ground (unchanged); with a height function the drone stops ON the hill."""
    ph = QuadcopterPhysics(get_preset("basic"))
    ph.ground_height_fn = lambda x, y: 5.0
    s = State.at(8.0)
    for _ in range(3000):
        ph.step(s, np.zeros(4), 0.002)
        if s.on_ground:
            break
    assert s.on_ground and s.pos_m[2] == pytest.approx(5.0)
    assert ph.last_impact_m_s > 3.0  # it fell 3 m


def test_unknown_terrain_and_bad_names():
    with pytest.raises(ValueError):
        terrain.load("nowhere")
    with pytest.raises(ValueError):
        terrain.load("../etc")


def _fly(sim: MissionSim, seconds: float):
    for _ in range(int(seconds / 0.02)):
        sim.advance(0.02)


def test_live_crash_into_the_ridge():
    """Hold 8 m above home and fly north: the ridge is higher than that, so the drone hits the hillside."""
    sim = MissionSim(get_preset("basic"))
    assert "🏔" in sim.command("terrain", id="hills")
    assert sim.terrain_info()["id"] == "hills"
    sim.command("takeoff", alt_m=8.0)
    _fly(sim, 6)
    assert sim.state.pos_m[2] > 6
    sim.command("target", x_m=0.0, y_m=80.0, alt_m=8.0)
    _fly(sim, 40)
    assert not sim.pilot.sp.armed or sim.state.on_ground
    assert any("Hit the ground" in n for n in sim.notices)
    assert sim.state.pos_m[1] < 75  # stopped on the slope, before the crest
    ground = sim.terrain.height(float(sim.state.pos_m[0]), float(sim.state.pos_m[1]))
    assert sim.state.pos_m[2] == pytest.approx(ground, abs=0.05)


def test_live_snapshot_and_flat_again():
    sim = MissionSim(get_preset("basic"))
    sim.command("terrain", id="hills")
    assert "terrain_agl_m" in sim.snapshot()
    sim.command("course", scenario=None)  # a course is flat: terrain off
    assert sim.terrain is None and sim.sim.physics.ground_height_fn is None
    assert "terrain_agl_m" not in sim.snapshot()


def _png(rgb: np.ndarray, filt: int) -> bytes:
    h, w, _ = rgb.shape
    raw = b""
    prev = np.zeros(w * 3, dtype=np.int32)
    for r in range(h):
        line = rgb[r].reshape(-1).astype(np.int32)
        if filt == 0:
            enc = line
        elif filt == 1:
            left = np.concatenate([np.zeros(3, dtype=np.int32), line[:-3]])
            enc = (line - left) & 0xFF
        elif filt == 2:
            enc = (line - prev) & 0xFF
        else:  # 4 = Paeth
            enc = np.zeros_like(line)
            for i in range(len(line)):
                a = line[i - 3] if i >= 3 else 0
                b = prev[i]
                c = prev[i - 3] if i >= 3 else 0
                pa, pb, pc = abs(b - c), abs(a - c), abs(a + b - 2 * c)
                p = a if pa <= pb and pa <= pc else (b if pb <= pc else c)
                enc[i] = (line[i] - p) & 0xFF
        raw += bytes([filt]) + bytes(enc.astype(np.uint8))
        prev = line

    def chunk(kind, body):
        return struct.pack(">I", len(body)) + kind + body + struct.pack(">I", zlib.crc32(kind + body))

    return (b"\x89PNG\r\n\x1a\n" + chunk(b"IHDR", struct.pack(">IIBBBBB", w, h, 8, 2, 0, 0, 0))
            + chunk(b"IDAT", zlib.compress(raw)) + chunk(b"IEND", b""))


@pytest.mark.parametrize("filt", [0, 1, 2, 4])
def test_png_decoder(filt):
    rng = np.random.default_rng(filt)
    img = rng.integers(0, 256, size=(5, 7, 3), dtype=np.uint8)
    assert np.array_equal(tf.decode_png(_png(img, filt)), img)


def test_terrarium_and_tile_maths():
    px = np.array([[[128, 0, 0], [128, 100, 128]]], dtype=np.uint8)
    assert tf.terrarium_heights(px).tolist() == [[0.0, 100.5]]
    lon, lat = 149.16523, -35.363261
    x, y = tf.lonlat_to_pixel(lon, lat, 15)
    assert tf.pixel_to_lonlat(x, y, 15) == pytest.approx((lon, lat), abs=1e-9)
    e, n = tf.lonlat_to_enu(*tf.enu_to_lonlat(120.0, -80.0, lon, lat), lon, lat)
    assert (e, n) == pytest.approx((120.0, -80.0), abs=1e-6)


def test_fetch_with_a_fake_server(tmp_path, monkeypatch):
    """The whole download with a fake tile server: heights relative to home, imagery tiles placed in ENU."""
    import httpx

    monkeypatch.setattr(tf, "TERRAIN_DIR", tmp_path)
    monkeypatch.setattr(terrain, "TERRAIN_DIR", tmp_path)

    def tile_png(z, x, y):
        # height rises 1 m per pixel to the east: R*256+G-32768 = 500 + column
        img = np.zeros((256, 256, 3), dtype=np.uint8)
        for c in range(256):
            v = 32768 + 500 + c + (x % 4) * 256
            img[:, c, 0], img[:, c, 1] = v // 256, v % 256
        return _png(img, 0)

    def handler(req: httpx.Request) -> httpx.Response:
        parts = req.url.path.split("/")
        if "terrarium" in req.url.path:
            z, x, y = int(parts[-3]), int(parts[-2]), int(parts[-1].split(".")[0])
            return httpx.Response(200, content=tile_png(z, x, y))
        return httpx.Response(200, content=b"\xff\xd8fakejpeg")

    client = httpx.Client(transport=httpx.MockTransport(handler))
    folder = tf.fetch(-35.363261, 149.16523, "demo", "Demo", size_m=200, cell_m=4, client=client, say=lambda *_: None)
    meta = json.loads((folder / "meta.json").read_text())
    t = terrain.load("demo")
    assert t.raw_height(0, 0) == pytest.approx(0.0, abs=0.05)
    m_per_px = meta["image"]["metres_per_pixel"]
    assert t.raw_height(40, 0) == pytest.approx(40 / m_per_px, rel=0.05)  # 1 m per pixel eastward
    assert meta["image"]["tiles"] and "CC BY 4.0" in meta["attribution"]
    assert any(p["id"] == "demo" for p in terrain.list_terrains())
    with pytest.raises(ValueError):
        tf.fetch(0, 0, "../x", "", client=client)


def test_own_image(tmp_path, monkeypatch):
    import httpx
    monkeypatch.setattr(tf, "TERRAIN_DIR", tmp_path)
    flat = np.zeros((256, 256, 3), dtype=np.uint8)
    flat[..., 0] = 128  # height 0 everywhere
    client = httpx.Client(transport=httpx.MockTransport(lambda r: httpx.Response(200, content=_png(flat, 0))))
    pic = tmp_path / "mine.png"
    pic.write_bytes(_png(np.zeros((50, 100, 3), dtype=np.uint8), 0))  # 100 x 50 px
    folder = tf.fetch(10.0, 20.0, "own", "", size_m=200, cell_m=10, client=client, say=lambda *_: None,
                      own_image=pic, own_image_width_m=300)
    meta = json.loads((folder / "meta.json").read_text())
    tile = meta["image"]["tiles"][0]
    assert tile["file"] == "0_0.png" and tile["x_max_m"] == 150 and tile["y_max_m"] == 75
    assert meta["image"]["metres_per_pixel"] == 3.0
    with pytest.raises(ValueError):
        tf.fetch(10.0, 20.0, "own2", "", client=client, own_image=pic)  # no width
