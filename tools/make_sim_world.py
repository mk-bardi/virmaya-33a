#!/usr/bin/env python3
"""
make_sim_world.py - build a Gazebo Harmonic world from real satellite imagery and real terrain.

The area around the takeoff point is cut into square tiles (default 1 km), each a textured 3D mesh
at high detail (default ~0.5 m per texture pixel, 10 m terrain grid). Around it a large low-detail
"horizon" ring (default 30 km) keeps the distant hills and mountains in view. The world has sky,
sun, shadows and a long camera view distance, and the origin (0,0,0) is the takeoff point on the ground,
so models spawn at the same height as before.

  # Mapbox satellite + Mapbox terrain (needs your Mapbox token, free tier is enough)
  export MAPBOX_TOKEN=pk....
  python3 make_sim_world.py --name adamawa_hd --lat 9.42689921517027 --lon 12.94404190151904

  # your own survey data (orthomosaic + DSM/DEM GeoTIFFs; needs: pip install rasterio)
  python3 make_sim_world.py --name site_a --lat .. --lon .. --imagery file:ortho.tif --elevation file:dsm.tif

  # no imagery: ground coloured from the terrain shape, flat takeoff spot found automatically,
  # trees/bushes/rocks/grass from Forest3D's converted models (visual only, no physics cost)
  python3 make_sim_world.py --name chappal --lat 7.03611 --lon 11.715 --elevation file:chappal_dem.tif \
      --imagery procedural --auto-site 1500 --vegetation ~/Forest3D/models

  Options worth knowing:
    --size 6000      side of the detailed area in metres (default 6000)
    --tile 1000      tile side in metres (default 1000)
    --zoom 17        imagery zoom (17 ~ 0.6 m/px with Mapbox @2x tiles, 18 ~ 0.3 m/px; more = sharper + more downloads)
    --tex 2048       texture pixels per tile side (4096 for very sharp; uses 4x the GPU memory)
    --grid 10        terrain mesh spacing in metres for the detailed area
    --horizon 30000  side of the low-detail outer area (0 = none)
    --pad 30         flatten a takeoff pad of this radius (m) at the origin (0 = keep real ground)
    --elevation terrarium   free open terrain tiles, no token needed

Output: ~/sim_worlds/<name>/  world.sdf, world.env (for simlaunch), preview.jpg, models/<name>_terrain/
Run:    virmaya <name>                 (or: GZ_SIM_RESOURCE_PATH=~/sim_worlds/<name>/models gz sim -r world.sdf)

Tiles are cached in ~/.cache/sim_worlds, so rebuilding with other settings doesn't download again.
Needs: numpy, Pillow (both in the ArduPilot venv). rasterio only for file: inputs.
"""
import argparse
import concurrent.futures as cf
import io
import math
import os
import sys
import threading
import time
import urllib.error
import urllib.request

import numpy as np
from PIL import Image

Image.MAX_IMAGE_PIXELS = None

# Same flat-earth constants ArduPilot uses, so the imagery lines up with SITL positions
M_PER_DEG_LAT = 111318.84502145034


def local_to_latlon(x, y, lat0, lon0):
    """x east, y north (m) from the origin -> lat, lon (deg)."""
    lat = lat0 + y / M_PER_DEG_LAT
    lon = lon0 + x / (M_PER_DEG_LAT * math.cos(math.radians(lat0)))
    return lat, lon


def latlon_to_px(lat, lon, z, tile_px):
    """Web-Mercator global pixel coordinates at zoom z for tiles of tile_px pixels."""
    n = (2 ** z) * tile_px
    lat = np.clip(lat, -85.05112878, 85.05112878)
    px = (np.asarray(lon) + 180.0) / 360.0 * n
    lr = np.radians(lat)
    py = (1.0 - np.log(np.tan(lr) + 1.0 / np.cos(lr)) / math.pi) / 2.0 * n
    return px, py


def m_per_px(lat, z, tile_px):
    return 40075016.686 * math.cos(math.radians(lat)) / (2 ** z * tile_px)


def zoom_for(lat, target_m_per_px, tile_px, max_zoom):
    """Lowest zoom whose pixels are at least as fine as target."""
    z = math.ceil(math.log2(40075016.686 * math.cos(math.radians(lat)) / (target_m_per_px * tile_px)))
    return int(max(1, min(max_zoom, z)))


# --------------------------------------------------------------------------------------------
# Tile sources
# --------------------------------------------------------------------------------------------
class TileSource:
    """XYZ web tiles, fetched in parallel, cached on disk and in memory, sampled bilinearly."""

    def __init__(self, name, url, tile_px, zoom, kind, cache_root, max_zoom=22):
        self.name, self.url, self.tile_px, self.kind = name, url, tile_px, kind
        self.zoom = min(zoom, max_zoom)
        self.cache = os.path.join(cache_root, name)
        self.mem = {}

    def _path(self, z, x, y):
        return os.path.join(self.cache, str(z), str(x), f"{y}.img")

    rate = 8.0                     # requests per second, all threads together (set by --rate)
    max_rate = 8.0
    _lock = threading.Lock()
    _next_t = 0.0

    @classmethod
    def _throttle(cls):
        with cls._lock:
            now = time.monotonic()
            wait = cls._next_t - now
            cls._next_t = max(now, cls._next_t) + 1.0 / cls.rate
        if wait > 0:
            time.sleep(wait)

    def _download(self, z, x, y, abort):
        p = self._path(z, x, y)
        if os.path.exists(p) or abort.is_set():
            return
        url = self.url.format(z=z, x=x, y=y)
        req = urllib.request.Request(url, headers={"User-Agent": "make_sim_world/1.0 (Gazebo simulation)"})
        last, attempt, limited = None, 0, 0
        while attempt < 6 and limited < 15:
            if abort.is_set():
                return
            self._throttle()
            try:
                with urllib.request.urlopen(req, timeout=30) as r:
                    data = r.read()
                if data[:1] in (b"{", b"<"):
                    raise ValueError(f"server sent text instead of an image: {data[:120]!r}")
                os.makedirs(os.path.dirname(p), exist_ok=True)
                with open(p + ".part", "wb") as fh:
                    fh.write(data)
                os.replace(p + ".part", p)
                with TileSource._lock:          # recover speed slowly after rate limiting
                    TileSource.rate = min(TileSource.max_rate, TileSource.rate * 1.03)
                return
            except urllib.error.HTTPError as e:
                if e.code in (401, 403):
                    abort.set()
                    raise RuntimeError(f"access refused (HTTP {e.code}). Check your token / URL:\n  {url.split('access_token')[0]}")
                if e.code == 404:
                    return              # no tile here (e.g. open sea): left empty
                last = f"HTTP {e.code} {e.reason}: {e.read()[:200]!r}"
                if e.code == 429:       # rate limited: slow everyone down and wait as told
                    with TileSource._lock:
                        TileSource.rate = max(1.0, TileSource.rate / 2)
                    ra = e.headers.get("Retry-After")
                    limited += 1
                    time.sleep(float(ra) if ra and ra.replace(".", "").isdigit() else min(60, 5 * limited))
                    continue
                attempt += 1
                time.sleep(1 + attempt * 2)
            except (urllib.error.URLError, TimeoutError, ConnectionError, OSError, ValueError) as e:
                last = f"{type(e).__name__}: {e}"
                attempt += 1
                time.sleep(1 + attempt * 2)
        abort.set()
        hint = ("\n  Mapbox is rate-limiting this token. Wait 10-15 minutes, then run again with --rate 3."
                if limited else "")
        raise RuntimeError(f"could not download {url.split('?')[0]}.\n  last error: {last}{hint}")

    def prefetch(self, lat_min, lat_max, lon_min, lon_max):
        z = self.zoom
        x0, y0 = latlon_to_px(lat_max, lon_min, z, 1)
        x1, y1 = latlon_to_px(lat_min, lon_max, z, 1)
        tiles = [(z, x, y) for x in range(int(x0), int(x1) + 1) for y in range(int(y0), int(y1) + 1)]
        todo = [t for t in tiles if not os.path.exists(self._path(*t))]
        print(f"  {self.name} z{z}: {len(tiles)} tiles ({len(todo)} to download)", flush=True)
        if not todo:
            return
        abort = threading.Event()
        ex = cf.ThreadPoolExecutor(4)
        futs = [ex.submit(self._download, *t, abort) for t in todo]
        done = 0
        try:
            for f in cf.as_completed(futs):
                f.result()
                done += 1
                if done % 25 == 0 or done == len(todo):
                    print(f"\r    downloaded {done}/{len(todo)}  ({TileSource.rate:.0f} req/s)", end="", flush=True)
        except (RuntimeError, KeyboardInterrupt) as e:
            abort.set()
            ex.shutdown(wait=False, cancel_futures=True)
            sys.exit(f"\n{self.name}: {e}\n  ({done} tiles saved; run again to continue, they are cached)")
        ex.shutdown()
        print()

    def _tile(self, x, y):
        key = (x, y)
        if key not in self.mem:
            p = self._path(self.zoom, x, y)
            if os.path.exists(p):
                im = Image.open(p)
                im = im.convert("RGB")
                a = np.asarray(im)
                if a.shape[0] != self.tile_px:
                    a = np.asarray(im.resize((self.tile_px, self.tile_px), Image.BILINEAR))
            else:
                a = np.zeros((self.tile_px, self.tile_px, 3), np.uint8)
            if self.kind == "terrain-rgb":
                a = a.astype(np.float64)
                a = (-10000.0 + (a[..., 0] * 65536 + a[..., 1] * 256 + a[..., 2]) * 0.1)[..., None]
            elif self.kind == "terrarium":
                a = a.astype(np.float64)
                a = (a[..., 0] * 256 + a[..., 1] + a[..., 2] / 256.0 - 32768.0)[..., None]
            self.mem[key] = a
            if len(self.mem) > 1500:
                self.mem.pop(next(iter(self.mem)))
        return self.mem[key]

    def sample(self, lat, lon):
        """Bilinear sample at arrays of lat/lon. Returns (..., C) float array."""
        px, py = latlon_to_px(lat, lon, self.zoom, self.tile_px)
        px, py = px - 0.5, py - 0.5
        tx0, ty0 = int(np.floor(px.min() / self.tile_px)), int(np.floor(py.min() / self.tile_px))
        tx1, ty1 = int(np.floor((px.max() + 1) / self.tile_px)), int(np.floor((py.max() + 1) / self.tile_px))
        rows = [np.concatenate([self._tile(tx, ty) for tx in range(tx0, tx1 + 1)], axis=1) for ty in range(ty0, ty1 + 1)]
        mosaic = np.concatenate(rows, axis=0)
        lx, ly = px - tx0 * self.tile_px, py - ty0 * self.tile_px
        return bilinear(mosaic, lx, ly)


class GeoTiffSource:
    """A local GeoTIFF (orthomosaic or DEM), any projection. Needs rasterio."""

    def __init__(self, path, kind):
        try:
            import rasterio
            from rasterio.warp import transform as rtransform
        except ImportError:
            sys.exit("file: inputs need rasterio:  pip install rasterio")
        self.ds = rasterio.open(os.path.expanduser(path))
        self.rtransform, self.kind, self.name = rtransform, kind, os.path.basename(path)
        bands = [1] if kind == "dem" else [1, 2, 3][: self.ds.count]
        self.arr = np.stack([self.ds.read(b).astype(np.float64) for b in bands], axis=-1)
        if kind == "dem" and self.ds.nodata is not None:
            nod = self.arr[..., 0] == self.ds.nodata
            if nod.any():
                self.arr[nod] = np.nanmedian(np.where(nod, np.nan, self.arr[..., 0]))
        if kind == "image" and self.arr.shape[-1] == 1:
            self.arr = np.repeat(self.arr, 3, axis=-1)
        print(f"  {self.name}: {self.ds.width}x{self.ds.height} px, {self.ds.crs}, "
              f"{abs(self.ds.transform.a):.3g} x {abs(self.ds.transform.e):.3g} units/px")

    def prefetch(self, *a):
        pass

    def sample(self, lat, lon):
        shp = np.shape(lat)
        if self.ds.crs is not None and self.ds.crs.is_geographic:
            xs, ys = np.ravel(lon), np.ravel(lat)          # already lat/lon: no reprojection needed
        else:
            xs, ys = self.rtransform("EPSG:4326", self.ds.crs, np.ravel(lon).tolist(), np.ravel(lat).tolist())
        inv = ~self.ds.transform
        col, row = inv * (np.asarray(xs, dtype=np.float64), np.asarray(ys, dtype=np.float64))
        out = bilinear(self.arr, col - 0.5, row - 0.5)
        return out.reshape(shp + (out.shape[-1],))


def bilinear(a, x, y):
    h, w = a.shape[:2]
    x = np.clip(x, 0, w - 1.000001)
    y = np.clip(y, 0, h - 1.000001)
    x0, y0 = np.floor(x).astype(int), np.floor(y).astype(int)
    fx, fy = (x - x0)[..., None], (y - y0)[..., None]
    a00, a01 = a[y0, x0], a[y0, x0 + 1]
    a10, a11 = a[y0 + 1, x0], a[y0 + 1, x0 + 1]
    return (a00 * (1 - fx) + a01 * fx) * (1 - fy) + (a10 * (1 - fx) + a11 * fx) * fy


# --------------------------------------------------------------------------------------------
# Procedural ground colour (no imagery needed): grassland, forest in the valleys, rock on steep faces
# --------------------------------------------------------------------------------------------
def _hash(ix, iy, seed):
    v = np.sin(ix * 127.1 + iy * 311.7 + seed * 74.7) * 43758.5453
    return v - np.floor(v)


def vnoise(x, y, scale, seed):
    """Smooth value noise in [0,1], a function of world position (so tiles join without seams)."""
    u, v = x / scale, y / scale
    iu, iv = np.floor(u), np.floor(v)
    fu, fv = u - iu, v - iv
    fu, fv = fu * fu * (3 - 2 * fu), fv * fv * (3 - 2 * fv)
    a, b = _hash(iu, iv, seed), _hash(iu + 1, iv, seed)
    c, d = _hash(iu, iv + 1, seed), _hash(iu + 1, iv + 1, seed)
    return (a * (1 - fu) + b * fu) * (1 - fv) + (c * (1 - fu) + d * fu) * fv


def fbm(x, y, scale, seed, octaves=3):
    t, amp, norm = 0.0, 1.0, 0.0
    for o in range(octaves):
        t = t + amp * vnoise(x, y, scale / 2 ** o, seed + 17 * o)
        norm += amp
        amp *= 0.5
    return t / norm


def smoothstep(e0, e1, v):
    t = np.clip((v - e0) / (e1 - e0), 0, 1)
    return t * t * (3 - 2 * t)


class TerrainFeatures:
    """Height, slope (deg) and concavity (m, + in valleys) from a DEM, at world x/y (m)."""

    def __init__(self, dem, lat0, lon0):
        self.dem, self.lat0, self.lon0 = dem, lat0, lon0

    def h(self, X, Y):
        lat, lon = local_to_latlon(X, Y, self.lat0, self.lon0)
        return self.dem.sample(lat, lon)[..., 0]

    def at(self, X, Y, d=15.0, ring=150.0):
        h = self.h(X, Y)
        gx = (self.h(X + d, Y) - self.h(X - d, Y)) / (2 * d)
        gy = (self.h(X, Y + d) - self.h(X, Y - d)) / (2 * d)
        slope = np.degrees(np.arctan(np.hypot(gx, gy)))
        acc = 0.0
        for k in range(8):
            ang = k * math.pi / 4
            acc = acc + self.h(X + ring * math.cos(ang), Y + ring * math.sin(ang))
        conc = acc / 8 - h
        return h, slope, conc


PALETTE = {   # sRGB, chosen for montane grassland with gallery forest (Mambilla / Gashaka-Gumti)
    "grass": (98, 126, 56), "grass_light": (118, 142, 66), "grass_dry": (146, 140, 86),
    "forest": (44, 72, 38), "forest_dark": (30, 54, 30), "rock": (124, 116, 102), "soil": (118, 96, 72),
}


class ProceduralSource:
    """Ground texture generated from the terrain shape. render() makes one texture square."""

    def __init__(self, feats, pad, seed=7):
        self.f, self.pad, self.seed = feats, pad, seed
        self.name = "procedural"

    def prefetch(self, *a):
        pass

    def render(self, x0, y0, side, npx):
        m = max(64, npx // 4)                              # terrain features at 1/4 res, colour noise at full res
        s = (np.arange(m) + 0.5) / m * side
        Xm, Ym = np.meshgrid(x0 + s, y0 + side - s)
        _, slope, conc = self.f.at(Xm, Ym)
        up = lambda a: np.asarray(Image.fromarray(a.astype(np.float32), "F").resize((npx, npx), Image.BILINEAR))
        slope, conc = up(slope), up(conc)
        s = (np.arange(npx) + 0.5) / npx * side
        X, Y = np.meshgrid(x0 + s, y0 + side - s)
        P = {k: np.array(v, np.float32) for k, v in PALETTE.items()}
        n_big = fbm(X, Y, 600, self.seed)                  # large patches
        n_mid = fbm(X, Y, 90, self.seed + 3)
        n_fine = vnoise(X, Y, 6, self.seed + 9) * 0.6 + vnoise(X, Y, 2, self.seed + 11) * 0.4
        g = smoothstep(0.3, 0.7, n_mid)[..., None]
        grass = P["grass"] * (1 - g) + P["grass_light"] * g
        dry = (smoothstep(0.55, 0.8, n_big) * 0.6)[..., None]
        grass = grass * (1 - dry) + P["grass_dry"] * dry
        forest_w = np.clip(smoothstep(4, 18, conc) + smoothstep(0.68, 0.82, n_big) * 0.35, 0, 1)
        forest_w = (forest_w * (1 - smoothstep(30, 42, slope)))[..., None]
        fcol = P["forest"] * (1 - n_fine[..., None] * 0.5) + P["forest_dark"] * (n_fine[..., None] * 0.5)
        col = grass * (1 - forest_w) + fcol * forest_w
        rock_w = np.clip(smoothstep(30, 45, slope) + smoothstep(0.85, 0.95, n_mid) * 0.3, 0, 1)[..., None]
        col = col * (1 - rock_w) + P["rock"] * rock_w
        if self.pad > 0:                                   # bare earth landing patch at the origin
            r = np.hypot(X, Y)
            soil_w = (1 - smoothstep(self.pad * 0.6, self.pad, r))[..., None]
            col = col * (1 - soil_w) + P["soil"] * soil_w
        col = col * (0.88 + 0.24 * n_fine[..., None])
        return np.clip(col, 0, 255).astype(np.uint8)


def find_flat_site(feats, radius, step=20.0):
    """Flattest spot (mean slope over ~100 m) within radius of the given centre. Returns x, y (m)."""
    n = int(radius / step)
    v = np.arange(-n, n + 1) * step
    X, Y = np.meshgrid(v, v)
    H = feats.h(X, Y)
    gy, gx = np.gradient(H, step)
    S = np.degrees(np.arctan(np.hypot(gx, gy)))
    k = 2                                                  # 5x5 window = 100 m
    Sp = np.pad(S, k, mode="edge")
    W = sum(Sp[i:i + S.shape[0], j:j + S.shape[1]] for i in range(2 * k + 1) for j in range(2 * k + 1)) / (2 * k + 1) ** 2
    score = W + 0.001 * np.hypot(X, Y)                     # small preference for staying near the centre
    score[np.hypot(X, Y) > radius] = np.inf
    i, j = np.unravel_index(np.argmin(score), score.shape)
    return float(X[i, j]), float(Y[i, j]), float(W[i, j])


VEG_CATS = ("tree", "bush", "rock", "grass")


def prepare_vegetation(src_root, dst_models):
    """Copy Forest3D-converted models (<root>/<cat>/<Name>/model.sdf) without their collision meshes."""
    import glob, re, shutil
    found = {}
    for cat in VEG_CATS:
        for sdf in sorted(glob.glob(os.path.join(os.path.expanduser(src_root), cat, "*", "model.sdf"))):
            src = os.path.dirname(sdf)
            name = os.path.basename(src)
            dst = os.path.join(dst_models, name)
            if os.path.exists(dst):
                shutil.rmtree(dst)
            shutil.copytree(src, dst, ignore=shutil.ignore_patterns("*_collision.*", "test.world"))
            txt = open(os.path.join(dst, "model.sdf")).read()
            txt = re.sub(r"\s*<collision\b.*?</collision>", "", txt, flags=re.S)   # visual only: no physics cost
            open(os.path.join(dst, "model.sdf"), "w").write(txt)
            found.setdefault(cat, []).append(name)
    return found


def mesh_z(feats, X, Y, grid, z_ref, pad):
    """Height of the terrain MESH (same grid, triangles and pad flattening as write_obj) at points X, Y."""
    gx, gy = np.floor(X / grid) * grid, np.floor(Y / grid) * grid
    u, v = (X - gx) / grid, (Y - gy) / grid

    def node(xn, yn):
        z = feats.h(xn, yn) - z_ref
        if pad > 0:
            t = np.clip((np.hypot(xn, yn) - pad) / pad, 0, 1)
            z = z * (t * t * (3 - 2 * t))
        return z
    za, zb = node(gx, gy), node(gx + grid, gy)
    zc, zd = node(gx + grid, gy + grid), node(gx, gy + grid)
    lower = u >= v                                         # triangle (a,b,c) vs (a,c,d), diagonal SW-NE
    return np.where(lower, za + u * (zb - za) + v * (zc - zb), za + v * (zd - za) + u * (zc - zd))


def place_vegetation(feats, models, counts, radius, grid, z_ref, pad, seed=11):
    """Scatter models by terrain: trees in clumps and valleys, rocks on steep ground, grass in the open."""
    rng = np.random.default_rng(seed)
    out, total = [], {}
    rules = {   # category: (acceptance probability function, radius factor, sink m, tilt rad)
        "tree": (lambda n, s, c: (0.08 + 0.92 * smoothstep(0.45, 0.65, n) + 0.5 * smoothstep(3, 15, c)) * (s < 38), 1.0, 0.3, 0.03),
        "bush": (lambda n, s, c: (0.25 + 0.75 * smoothstep(0.35, 0.5, n)) * (s < 42), 1.0, 0.15, 0.05),
        "rock": (lambda n, s, c: 0.08 + smoothstep(18, 38, s), 1.0, 0.25, 0.25),
        "grass": (lambda n, s, c: (0.7 - 0.6 * smoothstep(0.55, 0.7, n)) * (s < 35), 0.35, 0.05, 0.05),
    }
    for cat, want in counts.items():
        names = models.get(cat, [])
        if not names or want <= 0:
            continue
        accept, rf, sink, tilt = rules[cat]
        R = radius * rf
        got, tries = 0, 0
        while got < want and tries < 60:
            tries += 1
            n = 4000
            r = R * np.sqrt(rng.random(n))
            a = rng.random(n) * 2 * math.pi
            X, Y = r * np.cos(a), r * np.sin(a)
            keep = np.hypot(X, Y) > pad * 1.6               # keep the takeoff pad clear
            X, Y = X[keep], Y[keep]
            _, s, c = feats.at(X, Y)
            nz = fbm(X, Y, 250, seed + 5)
            p = np.clip(accept(nz, s, c), 0, 1)
            ok = rng.random(len(X)) < p
            X, Y = X[ok][: want - got], Y[ok][: want - got]
            if len(X) == 0:
                continue
            Z = mesh_z(feats, X, Y, grid, z_ref, pad) - sink
            for x, y, z in zip(X, Y, Z):
                name = names[rng.integers(len(names))]
                out.append(f"""    <include>
      <uri>model://{name}</uri>
      <name>{cat}_{got}</name>
      <pose>{x:.2f} {y:.2f} {z:.2f} {rng.normal(0, tilt):.3f} {rng.normal(0, tilt):.3f} {rng.random()*2*math.pi:.3f}</pose>
    </include>
""")
                got += 1
        total[cat] = got
    return "".join(out), total


def make_source(spec, role, zoom, token, cache):
    if spec.startswith("file:"):
        return GeoTiffSource(spec[5:], "dem" if role == "elevation" else "image")
    if spec == "mapbox":
        if not token:
            sys.exit("Mapbox needs a token: export MAPBOX_TOKEN=pk.... (or pass --token)")
        if role == "imagery":
            return TileSource("mapbox_satellite", "https://api.mapbox.com/v4/mapbox.satellite/{z}/{x}/{y}@2x.jpg90"
                              f"?access_token={token}", 512, zoom, "rgb", cache, max_zoom=19)
        return TileSource("mapbox_terrain", "https://api.mapbox.com/v4/mapbox.terrain-rgb/{z}/{x}/{y}.pngraw"
                          f"?access_token={token}", 256, zoom, "terrain-rgb", cache, max_zoom=15)
    if spec == "terrarium" and role == "elevation":
        return TileSource("terrarium", "https://s3.amazonaws.com/elevation-tiles-prod/terrarium/{z}/{x}/{y}.png",
                          256, zoom, "terrarium", cache, max_zoom=15)
    if spec.startswith(("http://", "https://")):         # custom XYZ template, e.g. a self-hosted tile server
        kind = "rgb" if role == "imagery" else ("terrain-rgb" if "rgb" in spec else "terrarium")
        safe = "".join(c if c.isalnum() else "_" for c in spec.split("?")[0])[-60:]
        return TileSource(f"{role}_{safe}", spec, 256, zoom, kind, cache)
    sys.exit(f"unknown {role} source '{spec}'")


# --------------------------------------------------------------------------------------------
# Mesh building
# --------------------------------------------------------------------------------------------
def grid_heights(src, xs, ys, lat0, lon0, z_ref, drop=0.0, pad=0.0):
    X, Y = np.meshgrid(xs, ys)                    # rows = y (south -> north), cols = x
    lat, lon = local_to_latlon(X, Y, lat0, lon0)
    Z = src.sample(lat, lon)[..., 0] - z_ref - drop
    if pad > 0:
        r = np.hypot(X, Y)
        t = np.clip((r - pad) / pad, 0, 1)
        Z = Z * (t * t * (3 - 2 * t))             # smoothstep: flat (z=0) inside pad, real ground beyond 2*pad
    return X, Y, Z


def write_obj(path, X, Y, Z, uv_box, keep_cell=None):
    """Grid -> OBJ with UVs and normals. uv_box = (x0, y0, w, h) the texture covers."""
    ny, nx = Z.shape
    x0, y0, w, h = uv_box
    dzdy, dzdx = np.gradient(Z, Y[:, 0], X[0, :])
    N = np.dstack([-dzdx, -dzdy, np.ones_like(Z)])
    N /= np.linalg.norm(N, axis=2, keepdims=True)
    U, V = (X - x0) / w, (Y - y0) / h
    idx = np.arange(ny * nx).reshape(ny, nx) + 1
    a, b = idx[:-1, :-1], idx[:-1, 1:]
    c, d = idx[1:, 1:], idx[1:, :-1]
    if keep_cell is not None:
        a, b, c, d = a[keep_cell], b[keep_cell], c[keep_cell], d[keep_cell]
    a, b, c, d = (v.ravel() for v in (a, b, c, d))
    with open(path, "w") as fh:
        fh.write("# generated by make_sim_world.py\n")
        np.savetxt(fh, np.column_stack([X.ravel(), Y.ravel(), Z.ravel()]), fmt="v %.3f %.3f %.3f")
        np.savetxt(fh, np.column_stack([U.ravel(), V.ravel()]), fmt="vt %.6f %.6f")
        np.savetxt(fh, N.reshape(-1, 3), fmt="vn %.4f %.4f %.4f")
        tri = np.column_stack([a, b, c, a, c, d]).reshape(-1, 3)
        np.savetxt(fh, np.repeat(tri, 3, axis=1), fmt="f %d/%d/%d %d/%d/%d %d/%d/%d")
    return len(tri)


def texture(src, x0, y0, side, npx, lat0, lon0, path, quality=90):
    """Sample imagery for the square [x0, x0+side] x [y0, y0+side]; row 0 = north edge."""
    s = (np.arange(npx) + 0.5) / npx * side
    X, Y = np.meshgrid(x0 + s, y0 + side - s)
    if hasattr(src, "render"):
        rgb = src.render(x0, y0, side, npx)
    else:
        lat, lon = local_to_latlon(X, Y, lat0, lon0)
        rgb = np.clip(src.sample(lat, lon), 0, 255).astype(np.uint8)
    Image.fromarray(rgb).save(path, quality=quality, subsampling=0)
    return rgb


# --------------------------------------------------------------------------------------------
# SDF
# --------------------------------------------------------------------------------------------
def visual_xml(name, mesh_uri, tex_uri, shadows=False):
    return f"""      <visual name="{name}">
        <cast_shadows>{str(shadows).lower()}</cast_shadows>
        <geometry><mesh><uri>{mesh_uri}</uri></mesh></geometry>
        <material>
          <ambient>1 1 1 1</ambient><diffuse>1 1 1 1</diffuse><specular>0 0 0 1</specular>
          <pbr><metal>
            <albedo_map>{tex_uri}</albedo_map><metalness>0</metalness><roughness>1</roughness>
          </metal></pbr>
        </material>
      </visual>
"""


def collision_xml(name, mesh_uri):
    return f"""      <collision name="{name}">
        <geometry><mesh><uri>{mesh_uri}</uri></mesh></geometry>
        <surface><friction><ode><mu>1</mu><mu2>1</mu2></ode></friction></surface>
      </collision>
"""


GUI = """    <gui fullscreen="0">
      <plugin filename="MinimalScene" name="3D View">
        <gz-gui><title>3D View</title><property type="bool" key="showTitleBar">false</property>
          <property type="string" key="state">docked</property></gz-gui>
        <engine>ogre2</engine>
        <scene>scene</scene>
        <ambient_light>0.55 0.55 0.55</ambient_light>
        <background_color>0.62 0.74 0.88</background_color>
        <camera_pose>-12 -12 8 0 0.35 0.785</camera_pose>
        <camera_clip><near>0.25</near><far>{far}</far></camera_clip>
        <sky></sky>
      </plugin>
      <plugin filename="EntityContextMenuPlugin" name="Entity context menu">
        <gz-gui><property key="state" type="string">floating</property><property key="width" type="double">5</property>
          <property key="height" type="double">5</property><property key="showTitleBar" type="bool">false</property></gz-gui>
      </plugin>
      <plugin filename="GzSceneManager" name="Scene Manager">
        <gz-gui><property key="resizable" type="bool">false</property><property key="width" type="double">5</property>
          <property key="height" type="double">5</property><property key="state" type="string">floating</property>
          <property key="showTitleBar" type="bool">false</property></gz-gui>
      </plugin>
      <plugin filename="InteractiveViewControl" name="Interactive view control">
        <gz-gui><property key="resizable" type="bool">false</property><property key="width" type="double">5</property>
          <property key="height" type="double">5</property><property key="state" type="string">floating</property>
          <property key="showTitleBar" type="bool">false</property></gz-gui>
      </plugin>
      <plugin filename="CameraTracking" name="Camera Tracking">
        <gz-gui><property key="resizable" type="bool">false</property><property key="width" type="double">5</property>
          <property key="height" type="double">5</property><property key="state" type="string">floating</property>
          <property key="showTitleBar" type="bool">false</property></gz-gui>
      </plugin>
      <plugin filename="MarkerManager" name="Marker manager">
        <gz-gui><property key="resizable" type="bool">false</property><property key="width" type="double">5</property>
          <property key="height" type="double">5</property><property key="state" type="string">floating</property>
          <property key="showTitleBar" type="bool">false</property></gz-gui>
      </plugin>
      <plugin filename="SelectEntities" name="Select Entities">
        <gz-gui><anchors target="Select entities"><line own="right" target="right"/><line own="top" target="top"/></anchors>
          <property key="resizable" type="bool">false</property><property key="width" type="double">5</property>
          <property key="height" type="double">5</property><property key="state" type="string">floating</property>
          <property key="showTitleBar" type="bool">false</property></gz-gui>
      </plugin>
      <plugin filename="Spawn" name="Spawn Entities">
        <gz-gui><property key="resizable" type="bool">false</property><property key="width" type="double">5</property>
          <property key="height" type="double">5</property><property key="state" type="string">floating</property>
          <property key="showTitleBar" type="bool">false</property></gz-gui>
      </plugin>
      <plugin filename="VisualizationCapabilities" name="Visualization Capabilities">
        <gz-gui><property key="resizable" type="bool">false</property><property key="width" type="double">5</property>
          <property key="height" type="double">5</property><property key="state" type="string">floating</property>
          <property key="showTitleBar" type="bool">false</property></gz-gui>
      </plugin>
      <plugin filename="WorldControl" name="World control">
        <gz-gui><title>World control</title><property type="bool" key="showTitleBar">false</property>
          <property type="bool" key="resizable">false</property><property type="double" key="height">72</property>
          <property type="double" key="width">121</property><property type="double" key="z">1</property>
          <property type="string" key="state">floating</property>
          <anchors target="3D View"><line own="left" target="left"/><line own="bottom" target="bottom"/></anchors></gz-gui>
        <play_pause>true</play_pause><step>true</step><start_paused>true</start_paused><use_event>true</use_event>
      </plugin>
      <plugin filename="WorldStats" name="World stats">
        <gz-gui><title>World stats</title><property type="bool" key="showTitleBar">false</property>
          <property type="bool" key="resizable">false</property><property type="double" key="height">110</property>
          <property type="double" key="width">290</property><property type="double" key="z">1</property>
          <property type="string" key="state">floating</property>
          <anchors target="3D View"><line own="right" target="right"/><line own="bottom" target="bottom"/></anchors></gz-gui>
        <sim_time>true</sim_time><real_time>true</real_time><real_time_factor>true</real_time_factor><iterations>true</iterations>
      </plugin>
      <plugin filename="EntityTree" name="Entity tree">
        <gz-gui><property type="string" key="state">docked_collapsed</property></gz-gui>
      </plugin>
    </gui>
"""


def sun_direction(lat_deg, hour_local=10.0, day_of_year=320):
    """Unit vector the sunlight travels along (ENU), for a mid-morning sun at this latitude."""
    decl = math.radians(23.44) * math.sin(2 * math.pi * (284 + day_of_year) / 365)
    ha = math.radians(15 * (hour_local - 12))
    lat = math.radians(lat_deg)
    alt = math.asin(math.sin(lat) * math.sin(decl) + math.cos(lat) * math.cos(decl) * math.cos(ha))
    az = math.atan2(-math.sin(ha), math.tan(decl) * math.cos(lat) - math.sin(lat) * math.cos(ha))  # from north, east +
    to_sun = (math.cos(alt) * math.sin(az), math.cos(alt) * math.cos(az), math.sin(alt))
    return tuple(-c for c in to_sun), math.degrees(alt)


def world_sdf(name, lat0, lon0, elev0, far, terrain_model, extra=""):
    d, alt = sun_direction(lat0)
    return f"""<?xml version="1.0"?>
<!-- {name}: real imagery + terrain, generated by make_sim_world.py.
     Origin = takeoff point on the ground ({lat0:.7f}, {lon0:.7f}, {elev0:.1f} m AMSL), x east, y north. -->
<sdf version="1.9">
  <world name="{name}">
    <physics name="1ms" type="ignore">
      <max_step_size>0.001</max_step_size>
      <real_time_factor>1.0</real_time_factor>
    </physics>
    <plugin filename="gz-sim-physics-system" name="gz::sim::systems::Physics"/>
    <plugin filename="gz-sim-user-commands-system" name="gz::sim::systems::UserCommands"/>
    <plugin filename="gz-sim-scene-broadcaster-system" name="gz::sim::systems::SceneBroadcaster"/>
    <plugin filename="gz-sim-sensors-system" name="gz::sim::systems::Sensors">
      <render_engine>ogre2</render_engine>
    </plugin>
    <plugin filename="gz-sim-imu-system" name="gz::sim::systems::Imu"/>
    <plugin filename="gz-sim-navsat-system" name="gz::sim::systems::NavSat"/>

{GUI.format(far=int(far))}
    <gravity>0 0 -9.8066</gravity>
    <spherical_coordinates>
      <surface_model>EARTH_WGS84</surface_model>
      <world_frame_orientation>ENU</world_frame_orientation>
      <latitude_deg>{lat0:.9f}</latitude_deg>
      <longitude_deg>{lon0:.9f}</longitude_deg>
      <elevation>{elev0:.1f}</elevation>
      <heading_deg>0</heading_deg>
    </spherical_coordinates>

    <scene>
      <ambient>0.55 0.55 0.55 1</ambient>
      <background>0.62 0.74 0.88 1</background>
      <grid>false</grid>
      <shadows>true</shadows>
      <sky></sky>
    </scene>

    <!-- mid-morning sun for this latitude (elevation {alt:.0f} deg) -->
    <light name="sun" type="directional">
      <cast_shadows>true</cast_shadows>
      <pose>0 0 100 0 0 0</pose>
      <diffuse>1.0 0.97 0.9 1</diffuse>
      <specular>0.1 0.1 0.1 1</specular>
      <intensity>1.2</intensity>
      <direction>{d[0]:.4f} {d[1]:.4f} {d[2]:.4f}</direction>
    </light>

    <include>
      <uri>model://{terrain_model}</uri>
      <pose>0 0 0 0 0 0</pose>
    </include>
{extra}  </world>
</sdf>
"""


# --------------------------------------------------------------------------------------------
def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--name", required=True, help="world name (letters, digits, _ only)")
    ap.add_argument("--lat", type=float, required=True, help="takeoff point latitude")
    ap.add_argument("--lon", type=float, required=True, help="takeoff point longitude")
    ap.add_argument("--size", type=float, default=6000)
    ap.add_argument("--tile", type=float, default=1000)
    ap.add_argument("--zoom", type=int, default=17)
    ap.add_argument("--tex", type=int, default=2048)
    ap.add_argument("--grid", type=float, default=10)
    ap.add_argument("--collision-grid", type=float, default=20)
    ap.add_argument("--horizon", type=float, default=30000)
    ap.add_argument("--horizon-grid", type=float, default=150)
    ap.add_argument("--horizon-tex", type=int, default=4096)
    ap.add_argument("--pad", type=float, default=30)
    ap.add_argument("--imagery", default="mapbox", help="mapbox | procedural | file:ortho.tif | https://.../{z}/{x}/{y}.jpg")
    ap.add_argument("--elevation", default=None, help="mapbox | terrarium | file:dem.tif | https://... (default: mapbox if token, else terrarium)")
    ap.add_argument("--dem-zoom", type=int, default=14)
    ap.add_argument("--token", default=os.environ.get("MAPBOX_TOKEN") or os.environ.get("MAPBOX_ACCESS_TOKEN"))
    ap.add_argument("--out", default=os.path.expanduser("~/sim_worlds"))
    ap.add_argument("--cache", default=os.path.expanduser("~/.cache/sim_worlds"))
    ap.add_argument("--rate", type=float, default=8, help="max tile requests per second (lower it if you get HTTP 429)")
    ap.add_argument("--auto-site", type=float, default=0, metavar="R",
                    help="move the origin to the flattest spot within R metres of --lat/--lon (0 = use them as given)")
    ap.add_argument("--vegetation", default=None, metavar="DIR",
                    help="models folder from Forest3D (contains tree/ bush/ rock/ grass/), placed as visual-only models")
    ap.add_argument("--veg-radius", type=float, default=1200, help="vegetation within this distance of the origin (m)")
    ap.add_argument("--trees", type=int, default=1200)
    ap.add_argument("--bushes", type=int, default=500)
    ap.add_argument("--rocks", type=int, default=150)
    ap.add_argument("--grass", type=int, default=1000)
    ap.add_argument("--seed", type=int, default=7, help="change for a different random layout")
    a = ap.parse_args()
    TileSource.rate = TileSource.max_rate = a.rate

    if not a.name.replace("_", "").isalnum():
        sys.exit("--name: use letters, digits and _ only (Gazebo service names break on spaces)")
    n_t = int(round(a.size / a.tile))
    if n_t < 1 or abs(n_t * a.tile - a.size) > 1e-6 or n_t % 2:
        sys.exit("--size must be an EVEN number of --tile (e.g. 6000 / 1000) so the origin sits on a tile corner")
    for g in (a.grid, a.collision_grid):
        if abs(a.tile / g - round(a.tile / g)) > 1e-6:
            sys.exit("--grid and --collision-grid must divide --tile exactly")
    if a.horizon and a.horizon <= a.size:
        sys.exit("--horizon must be larger than --size (or 0 for none)")
    elev_spec = a.elevation or ("mapbox" if a.token else "terrarium")

    lat0, lon0 = a.lat, a.lon
    root = os.path.join(a.out, a.name)
    tmodel = f"{a.name}_terrain"
    mdir = os.path.join(root, "models", tmodel)
    os.makedirs(os.path.join(mdir, "meshes"), exist_ok=True)
    os.makedirs(os.path.join(mdir, "textures"), exist_ok=True)
    os.makedirs(a.cache, exist_ok=True)

    print(f"World '{a.name}' around {lat0:.6f}, {lon0:.6f}")
    print(f"  detailed area {a.size/1000:g} km x {a.size/1000:g} km in {n_t*n_t} tiles of {a.tile:g} m, "
          f"texture {a.tile/a.tex:.2f} m/px, terrain grid {a.grid:g} m")
    dem = make_source(elev_spec, "elevation", a.dem_zoom, a.token, a.cache)
    if a.auto_site:
        if isinstance(dem, TileSource):
            r = a.auto_site + a.size / 2
            b0, c0 = local_to_latlon(-r, -r, lat0, lon0); b1, c1 = local_to_latlon(r, r, lat0, lon0)
            dem.prefetch(b0, b1, c0, c1)
        sx, sy, sl = find_flat_site(TerrainFeatures(dem, lat0, lon0), a.auto_site)
        lat0, lon0 = local_to_latlon(sx, sy, lat0, lon0)
        print(f"  takeoff site: flattest spot within {a.auto_site:g} m is {sx:+.0f} m E, {sy:+.0f} m N "
              f"(mean slope {sl:.1f} deg) -> {lat0:.6f}, {lon0:.6f}")
    feats = TerrainFeatures(dem, lat0, lon0)
    if a.imagery == "procedural":
        img = ProceduralSource(feats, a.pad, a.seed)
        print("  ground colour: procedural (grassland, valley forest, rock on steep faces)")
    else:
        img = make_source(a.imagery, "imagery", a.zoom, a.token, a.cache)
    if isinstance(img, TileSource):
        print(f"  imagery source resolution {m_per_px(lat0, img.zoom, img.tile_px):.2f} m/px")

    half = a.size / 2
    outer = a.horizon / 2 if a.horizon else half
    la0, lo0 = local_to_latlon(-half, -half, lat0, lon0)
    la1, lo1 = local_to_latlon(half, half, lat0, lon0)
    img.prefetch(la0, la1, lo0, lo1)
    Lb0, Lo0 = local_to_latlon(-outer, -outer, lat0, lon0)
    Lb1, Lo1 = local_to_latlon(outer, outer, lat0, lon0)
    dem.prefetch(la0, la1, lo0, lo1)

    # ground height at the takeoff point = world z 0
    elev0 = float(dem.sample(np.array([[lat0]]), np.array([[lon0]]))[0, 0, 0])
    print(f"  ground at takeoff point: {elev0:.1f} m AMSL")

    visuals, collisions, total_tris = [], [], 0
    uri = lambda sub, f: f"model://{tmodel}/{sub}/{f}"
    t0 = time.time()
    for i in range(n_t):              # south -> north
        for j in range(n_t):          # west -> east
            x0, y0 = -half + j * a.tile, -half + i * a.tile
            tag = f"tile_{i}_{j}"
            xs = np.arange(x0, x0 + a.tile + a.grid / 2, a.grid)
            ys = np.arange(y0, y0 + a.tile + a.grid / 2, a.grid)
            X, Y, Z = grid_heights(dem, xs, ys, lat0, lon0, elev0, pad=a.pad)
            total_tris += write_obj(os.path.join(mdir, "meshes", tag + ".obj"), X, Y, Z, (x0, y0, a.tile, a.tile))
            xs = np.arange(x0, x0 + a.tile + a.collision_grid / 2, a.collision_grid)
            ys = np.arange(y0, y0 + a.tile + a.collision_grid / 2, a.collision_grid)
            Xc, Yc, Zc = grid_heights(dem, xs, ys, lat0, lon0, elev0, pad=a.pad)
            write_obj(os.path.join(mdir, "meshes", tag + "_col.obj"), Xc, Yc, Zc, (x0, y0, a.tile, a.tile))
            texture(img, x0, y0, a.tile, a.tex, lat0, lon0, os.path.join(mdir, "textures", tag + ".jpg"))
            visuals.append(visual_xml(tag, uri("meshes", tag + ".obj"), uri("textures", tag + ".jpg")))
            collisions.append(collision_xml(tag + "_col", uri("meshes", tag + "_col.obj")))
            k = i * n_t + j + 1
            print(f"\r  tiles {k}/{n_t*n_t}  ({time.time()-t0:.0f} s)", end="", flush=True)
    print()

    preview_src = None
    if a.horizon:
        L = a.horizon
        if isinstance(img, TileSource):
            hz = zoom_for(lat0, L / a.horizon_tex, img.tile_px, img.zoom)
            himg = TileSource(img.name, img.url, img.tile_px, hz, img.kind, a.cache)
            himg.prefetch(Lb0, Lb1, Lo0, Lo1)
        else:
            himg, hz = img, "file"
        if isinstance(dem, TileSource):
            dz = zoom_for(lat0, a.horizon_grid / 2, dem.tile_px, dem.zoom)
            hdem = TileSource(dem.name, dem.url, dem.tile_px, dz, dem.kind, a.cache)
            hdem.prefetch(Lb0, Lb1, Lo0, Lo1)
        else:
            hdem = dem
        # grid that has the detailed area's edges on grid lines; cells inside it (minus one ring) are left out
        k_in = max(2, int(round(a.size / a.horizon_grid)))
        g = a.size / k_in
        n_half = int(math.ceil((L / 2) / g))
        xs = np.arange(-n_half, n_half + 1) * g
        X, Y, Z = grid_heights(hdem, xs, xs, lat0, lon0, elev0, drop=1.0)
        cx0, cx1 = xs[:-1], xs[1:]
        inside = (np.maximum(np.abs(cx0), np.abs(cx1)) <= half - g + 1e-6)
        keep = ~(inside[None, :] & inside[:, None])
        Ltot = xs[-1] - xs[0]
        total_tris += write_obj(os.path.join(mdir, "meshes", "horizon.obj"), X, Y, Z, (xs[0], xs[0], Ltot, Ltot), keep)
        print(f"  horizon {Ltot/1000:.0f} km: imagery z{hz} ({Ltot/a.horizon_tex:.1f} m/px), terrain grid {g:.0f} m")
        preview_src = texture(himg, xs[0], xs[0], Ltot, a.horizon_tex, lat0, lon0,
                              os.path.join(mdir, "textures", "horizon.jpg"))
        visuals.append(visual_xml("horizon", uri("meshes", "horizon.obj"), uri("textures", "horizon.jpg")))
        collisions.append(collision_xml("horizon_col", uri("meshes", "horizon.obj")))

    pad_box = ""
    if a.pad > 0:   # flat primitive under the pad: steadier landing contacts than a mesh
        s = min(a.pad * 1.2, 40)
        pad_box = f"""      <collision name="takeoff_pad">
        <pose>0 0 -0.5 0 0 0</pose>
        <geometry><box><size>{s:.0f} {s:.0f} 1</size></box></geometry>
        <surface><friction><ode><mu>1</mu><mu2>1</mu2></ode></friction></surface>
      </collision>
"""
    with open(os.path.join(mdir, "model.sdf"), "w") as fh:
        fh.write(f"""<?xml version="1.0"?>
<sdf version="1.9">
  <model name="{tmodel}">
    <static>true</static>
    <link name="ground">
{''.join(visuals)}{''.join(collisions)}{pad_box}    </link>
  </model>
</sdf>
""")
    with open(os.path.join(mdir, "model.config"), "w") as fh:
        fh.write(f"""<?xml version="1.0"?>
<model><name>{tmodel}</name><version>1.0</version><sdf version="1.9">model.sdf</sdf>
<description>Imagery + terrain around {lat0:.6f}, {lon0:.6f}. Generated by make_sim_world.py.</description></model>
""")
    extra, veg_note = "", ""
    if a.vegetation:
        models = prepare_vegetation(a.vegetation, os.path.join(root, "models"))
        if not models:
            print(f"  (no models found under {a.vegetation}/tree|bush|rock|grass/*/model.sdf, vegetation skipped)")
        else:
            R = min(a.veg_radius, half - 2 * a.grid)
            extra, placed = place_vegetation(feats, models, {"tree": a.trees, "bush": a.bushes, "rock": a.rocks,
                                                             "grass": a.grass}, R, a.grid, elev0, a.pad, a.seed)
            veg_note = ", ".join(f"{v} {k}" for k, v in placed.items())
            print(f"  vegetation within {R:.0f} m: {veg_note} (visual only, from {sum(len(v) for v in models.values())} models)")
    far = max(a.horizon, a.size) * 1.2
    with open(os.path.join(root, "world.sdf"), "w") as fh:
        fh.write(world_sdf(a.name, lat0, lon0, elev0, far, tmodel, extra))
    loc = f"{lat0:.9f},{lon0:.9f},{elev0:.1f},0"
    with open(os.path.join(root, "world.env"), "w") as fh:
        fh.write(f'WORLD_NAME="{a.name}"\nLOC="{loc}"\n'
                 f'# imagery: {a.imagery.split("?")[0]}  elevation: {elev_spec.split("?")[0]}\n')

    # preview: horizon texture (or the tile mosaic) with the detailed area and takeoff point marked
    try:
        from PIL import ImageDraw
        if preview_src is None:
            preview_src = texture(img, -half, -half, a.size, 2048, lat0, lon0, os.path.join(root, "_tmp.jpg"))
            os.remove(os.path.join(root, "_tmp.jpg"))
            span = a.size
        else:
            span = Ltot
        pv = Image.fromarray(preview_src).resize((1600, 1600))
        dr = ImageDraw.Draw(pv)
        s = 1600 / span
        c = 800
        dr.rectangle([c - half * s, c - half * s, c + half * s, c + half * s], outline=(255, 255, 255), width=2)
        dr.ellipse([c - 7, c - 7, c + 7, c + 7], outline=(255, 255, 255), width=3)
        pv.save(os.path.join(root, "preview.jpg"), quality=88)
    except Exception as e:      # preview is a convenience only
        print("  (preview skipped:", e, ")")

    mb = sum(os.path.getsize(os.path.join(dp, f)) for dp, _, fs in os.walk(mdir) for f in fs) / 1e6
    print(f"\nDone: {root}")
    print(f"  {total_tris/1e6:.2f} M triangles, {mb:.0f} MB on disk")
    print(f"  SITL home: --custom-location={loc}")
    print(f"  Run:  virmaya {a.name}")


if __name__ == "__main__":
    main()
