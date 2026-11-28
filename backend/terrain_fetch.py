"""🏔 Download a real place once, then fly over it offline.

    .venv/bin/python -m backend.terrain_fetch --lat -35.363261 --lon 149.165230 --name cmac --title "CMAC field"

Writes data/terrain/<name>/ (gitignored: a real place is often near someone's home):
  heights.npy   ground heights in metres relative to the home point (ENU grid, rows south -> north)
  meta.json     grid placement, the place, attribution
  image/*.jpg   optional satellite tiles (--imagery) the 3D view paints on the ground

Sources (open data, no account or key):
  * elevation: AWS Open Data "Terrain Tiles" (Mapzen terrarium PNG). Outside the USA it is mostly SRTM at
    ~30 m resolution, so small things (trees, hedges, houses) are NOT in it: only the shape of the land.
  * imagery: EOX "Sentinel-2 cloudless 2016", CC BY 4.0. 10 m pixels: fields and roads, not single trees.

The coordinates you give are sent to those two servers (as map-tile numbers). Nothing else is.
"""

from __future__ import annotations

import argparse
import json
import math
import struct
import sys
import zlib
from pathlib import Path

import httpx
import numpy as np

from backend.sim.terrain import TERRAIN_DIR

ELEVATION_URL = "https://s3.amazonaws.com/elevation-tiles-prod/terrarium/{z}/{x}/{y}.png"
IMAGERY_URL = "https://tiles.maps.eox.at/wmts/1.0.0/s2cloudless_3857/default/g/{z}/{y}/{x}.jpg"
ELEVATION_ATTR = "Elevation: Terrain Tiles (Mapzen / AWS Open Data; SRTM, USGS 3DEP and others)"
IMAGERY_ATTR = ("Imagery: Sentinel-2 cloudless 2016 by EOX IT Services GmbH (contains modified Copernicus Sentinel "
                "data 2016), CC BY 4.0")
EARTH_R_M = 6_378_137.0
TILE_PX = 256
HEADERS = {"User-Agent": "DroneLab-terrain-fetch/1.0 (offline simulator)"}


# ---- web-mercator tile maths ----

def lonlat_to_pixel(lon_deg: float, lat_deg: float, z: int) -> tuple[float, float]:
    """Global pixel coordinates at zoom z (x to the east, y to the SOUTH)."""
    n = TILE_PX * 2 ** z
    x = (lon_deg + 180.0) / 360.0 * n
    lat = math.radians(lat_deg)
    y = (1.0 - math.log(math.tan(lat) + 1.0 / math.cos(lat)) / math.pi) / 2.0 * n
    return x, y


def pixel_to_lonlat(px: float, py: float, z: int) -> tuple[float, float]:
    n = TILE_PX * 2 ** z
    lon = px / n * 360.0 - 180.0
    lat = math.degrees(math.atan(math.sinh(math.pi * (1 - 2 * py / n))))
    return lon, lat


def enu_to_lonlat(x_m: float, y_m: float, lon0: float, lat0: float) -> tuple[float, float]:
    """Flat-earth (equirectangular) offset around home: fine over a few km."""
    lat = lat0 + math.degrees(y_m / EARTH_R_M)
    lon = lon0 + math.degrees(x_m / (EARTH_R_M * math.cos(math.radians(lat0))))
    return lon, lat


def lonlat_to_enu(lon: float, lat: float, lon0: float, lat0: float) -> tuple[float, float]:
    x = math.radians(lon - lon0) * EARTH_R_M * math.cos(math.radians(lat0))
    y = math.radians(lat - lat0) * EARTH_R_M
    return x, y


# ---- a small PNG reader (8-bit RGB/RGBA, not interlaced: what terrarium tiles are). No Pillow needed. ----

def decode_png(data: bytes) -> np.ndarray:
    """PNG bytes -> uint8 array (h, w, channels)."""
    if data[:8] != b"\x89PNG\r\n\x1a\n":
        raise ValueError("not a PNG")
    pos, idat, w = 8, b"", 0
    while pos < len(data):
        length, kind = struct.unpack(">I4s", data[pos:pos + 8])
        body = data[pos + 8:pos + 8 + length]
        if kind == b"IHDR":
            w, h, depth, color, _, _, interlace = struct.unpack(">IIBBBBB", body)
            if depth != 8 or color not in (2, 6) or interlace:
                raise ValueError(f"unsupported PNG (depth {depth}, colour type {color}, interlace {interlace})")
            ch = 3 if color == 2 else 4
        elif kind == b"IDAT":
            idat += body
        elif kind == b"IEND":
            break
        pos += 12 + length
    raw = zlib.decompress(idat)
    stride = w * ch
    out = np.zeros((h, stride), dtype=np.uint8)
    prev = np.zeros(stride, dtype=np.int32)
    for r in range(h):
        f = raw[r * (stride + 1)]
        line = np.frombuffer(raw, dtype=np.uint8, count=stride, offset=r * (stride + 1) + 1).astype(np.int32)
        if f == 0:
            cur = line
        elif f == 2:
            cur = (line + prev) & 0xFF
        else:  # 1 = sub, 3 = average, 4 = Paeth depend on the pixel to the left: do them byte by byte
            cur = np.zeros(stride, dtype=np.int32)
            for i in range(stride):
                a = cur[i - ch] if i >= ch else 0
                b = prev[i]
                if f == 1:
                    p = a
                elif f == 3:
                    p = (a + b) >> 1
                else:
                    c = prev[i - ch] if i >= ch else 0
                    pa, pb, pc = abs(b - c), abs(a - c), abs(a + b - 2 * c)
                    p = a if pa <= pb and pa <= pc else (b if pb <= pc else c)
                cur[i] = (line[i] + p) & 0xFF
        out[r] = cur
        prev = cur
    return out.reshape(h, w, ch)


def terrarium_heights(rgb: np.ndarray) -> np.ndarray:
    """Terrarium encoding: height = R*256 + G + B/256 - 32768 (metres above sea level)."""
    r, g, b = (rgb[..., i].astype(np.float64) for i in range(3))
    return r * 256.0 + g + b / 256.0 - 32768.0


def _image_size(path: Path) -> tuple[int, int]:
    """(width, height) in pixels of a PNG or JPEG, read from its header (no Pillow)."""
    data = path.read_bytes()
    if data[:8] == b"\x89PNG\r\n\x1a\n":
        return struct.unpack(">II", data[16:24])
    i = 2
    while i < len(data) - 9:
        if data[i] != 0xFF:
            i += 1
            continue
        marker = data[i + 1]
        if marker in (0xC0, 0xC1, 0xC2):  # start of frame: height, width
            hgt, w = struct.unpack(">HH", data[i + 5:i + 9])
            return w, hgt
        i += 2 + struct.unpack(">H", data[i + 2:i + 4])[0]
    raise ValueError(f"{path}: couldn't read the picture's size.")


def _image_aspect(path: Path) -> float:
    w, h = _image_size(path)
    return h / w


# ---- the download ----

def tiles_for(lon0: float, lat0: float, half_m: float, z: int) -> tuple[range, range]:
    lon_w, lat_s = enu_to_lonlat(-half_m, -half_m, lon0, lat0)
    lon_e, lat_n = enu_to_lonlat(half_m, half_m, lon0, lat0)
    x0, y0 = lonlat_to_pixel(lon_w, lat_n, z)
    x1, y1 = lonlat_to_pixel(lon_e, lat_s, z)
    return range(int(x0 // TILE_PX), int(x1 // TILE_PX) + 1), range(int(y0 // TILE_PX), int(y1 // TILE_PX) + 1)


def fetch(lat: float, lon: float, name: str, title: str, size_m: float = 400.0, cell_m: float = 3.0,
          imagery: bool = True, z: int = 15, client: httpx.Client | None = None, say=print,
          own_image: Path | None = None, own_image_width_m: float | None = None) -> Path:
    """Download heights (and imagery) for a square of size_m around (lat, lon) into data/terrain/<name>/."""
    if not name.replace("-", "").replace("_", "").isalnum():
        raise ValueError("--name: letters, digits, - and _ only.")
    if not (-85 < lat < 85 and -180 <= lon <= 180):
        raise ValueError("--lat must be within ±85°, --lon within ±180°.")
    if not 100 <= size_m <= 2000:
        raise ValueError("--size must be 100-2000 m.")
    if own_image is not None:
        own_image = Path(own_image)
        head = own_image.read_bytes()[:8] if own_image.is_file() else b""
        if not (head.startswith(b"\xff\xd8") or head.startswith(b"\x89PNG")):
            raise ValueError(f"--image {own_image}: not a JPEG or PNG file.")
        if not own_image_width_m or own_image_width_m <= 0:
            raise ValueError("--image needs --image-width-m (how many metres the picture covers west to east).")
    own = client is None
    client = client or httpx.Client(timeout=30.0, headers=HEADERS, follow_redirects=True)
    half = size_m / 2.0
    xs_t, ys_t = tiles_for(lon, lat, half, z)
    say(f"Elevation: {len(xs_t) * len(ys_t)} tiles at zoom {z} ...")
    mosaic = np.zeros((len(ys_t) * TILE_PX, len(xs_t) * TILE_PX))
    for j, ty in enumerate(ys_t):
        for i, tx in enumerate(xs_t):
            r = client.get(ELEVATION_URL.format(z=z, x=tx, y=ty))
            r.raise_for_status()
            mosaic[j * TILE_PX:(j + 1) * TILE_PX, i * TILE_PX:(i + 1) * TILE_PX] = terrarium_heights(decode_png(r.content))
    ox, oy = xs_t.start * TILE_PX, ys_t.start * TILE_PX

    def sample(px: float, py: float) -> float:
        fx, fy = px - ox - 0.5, py - oy - 0.5  # pixel centres
        c, rr = int(math.floor(fx)), int(math.floor(fy))
        c = min(max(c, 0), mosaic.shape[1] - 2)
        rr = min(max(rr, 0), mosaic.shape[0] - 2)
        tx_, ty_ = min(max(fx - c, 0), 1), min(max(fy - rr, 0), 1)
        m = mosaic
        return float((m[rr, c] * (1 - tx_) + m[rr, c + 1] * tx_) * (1 - ty_) + (m[rr + 1, c] * (1 - tx_) + m[rr + 1, c + 1] * tx_) * ty_)

    n = int(round(size_m / cell_m)) + 1
    grid = np.zeros((n, n), dtype=np.float32)
    for row in range(n):
        y = -half + row * cell_m
        for col in range(n):
            lo, la = enu_to_lonlat(-half + col * cell_m, y, lon, lat)
            grid[row, col] = sample(*lonlat_to_pixel(lo, la, z))
    home_amsl = sample(*lonlat_to_pixel(lon, lat, z))
    grid -= home_amsl
    say(f"  ground from {grid.min():+.1f} m to {grid.max():+.1f} m around home (home is {home_amsl:.0f} m above sea level)")

    folder = TERRAIN_DIR / name
    (folder / "image").mkdir(parents=True, exist_ok=True)
    np.save(folder / "heights.npy", grid)
    image = None
    attribution = ELEVATION_ATTR
    if own_image is not None:
        ext = "png" if own_image.read_bytes()[:4] == b"\x89PNG" else "jpg"
        (folder / "image" / f"0_0.{ext}").write_bytes(own_image.read_bytes())
        w = float(own_image_width_m)
        hgt = w * _image_aspect(own_image)
        image = {"tiles": [{"file": f"0_0.{ext}", "x_min_m": -w / 2, "x_max_m": w / 2, "y_min_m": -hgt / 2, "y_max_m": hgt / 2}],
                 "metres_per_pixel": round(w / _image_size(own_image)[0], 3), "own": True}
        attribution += "; Imagery: your own picture"
    elif imagery:
        say(f"Imagery: {len(xs_t) * len(ys_t)} tiles ...")
        tiles = []
        for ty in ys_t:
            for tx in xs_t:
                r = client.get(IMAGERY_URL.format(z=z, x=tx, y=ty))
                if r.status_code != 200 or not r.content.startswith(b"\xff\xd8"):
                    say(f"  tile {tx},{ty}: not available ({r.status_code}), left blank")
                    continue
                fname = f"{tx}_{ty}.jpg"
                (folder / "image" / fname).write_bytes(r.content)
                lo_w, la_n = pixel_to_lonlat(tx * TILE_PX, ty * TILE_PX, z)
                lo_e, la_s = pixel_to_lonlat((tx + 1) * TILE_PX, (ty + 1) * TILE_PX, z)
                x0, y1 = lonlat_to_enu(lo_w, la_n, lon, lat)
                x1, y0 = lonlat_to_enu(lo_e, la_s, lon, lat)
                tiles.append({"file": fname, "x_min_m": round(x0, 2), "x_max_m": round(x1, 2),
                              "y_min_m": round(y0, 2), "y_max_m": round(y1, 2)})
        if tiles:
            image = {"tiles": tiles, "metres_per_pixel": round((tiles[0]["x_max_m"] - tiles[0]["x_min_m"]) / TILE_PX, 2)}
            attribution += "; " + IMAGERY_ATTR
    meta = {
        "title": title or name, "x_min_m": -half, "y_min_m": -half, "cell_m": cell_m,
        "image": image, "attribution": attribution,
        "info": {"home_amsl_m": round(home_amsl, 1), "zoom": z, "size_m": size_m,
                 "note": "Elevation is the shape of the land only (~30 m source detail outside the USA): no trees "
                         "or buildings. Check the real site before you fly there."},
        "lat": lat, "lon": lon,
    }
    (folder / "meta.json").write_text(json.dumps(meta, indent=1))
    if own:
        client.close()
    say(f"Saved {folder}. Fly it: Fly view -> 🏔 Terrain -> {title or name}.")
    return folder


def main(argv: list[str] | None = None) -> int:
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--lat", type=float, required=True, help="home latitude, degrees (north +)")
    ap.add_argument("--lon", type=float, required=True, help="home longitude, degrees (east +)")
    ap.add_argument("--name", required=True, help="folder name, e.g. myfield")
    ap.add_argument("--title", default="", help="name shown in the app")
    ap.add_argument("--size", type=float, default=400.0, help="square side in metres (100-2000, default 400)")
    ap.add_argument("--cell", type=float, default=3.0, help="grid spacing in metres (default 3)")
    ap.add_argument("--no-imagery", action="store_true", help="heights only, no satellite picture")
    ap.add_argument("--image", type=Path, help="your own north-up aerial picture (JPEG/PNG) centred on home, "
                    "instead of the 10 m satellite one (e.g. from your own drone's map, or a screenshot you may use)")
    ap.add_argument("--image-width-m", type=float, help="how many metres the picture covers west to east")
    a = ap.parse_args(argv)
    try:
        fetch(a.lat, a.lon, a.name, a.title, a.size, a.cell, not a.no_imagery,
              own_image=a.image, own_image_width_m=a.image_width_m)
    except (ValueError, OSError, httpx.HTTPError) as exc:
        print(f"Failed: {exc}", file=sys.stderr)
        return 1
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
