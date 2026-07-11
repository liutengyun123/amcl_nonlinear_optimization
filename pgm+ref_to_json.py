#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
PGM+YAML(+ReflectorJSON) -> unified JSON:
- occupyPoints
- reflectors
- knownMask
- freeBand (compressed bitset)

freeBand (scheme-1):
- seed: "free_seed" cells (occ_prob < free_seed_thr)
- reachable: geodesic flood within non-occupied (~occupied), 4/8-neighborhood
- freeBand: cells within K steps from reachable, still within ~occupied
- encode as bitset:
  - layout: row-major (x-fast, y-slow)
  - yAxis: bottom-up (row0 is bottom)
  - bitOrder: lsb0 (LSB-first in each byte)
"""

# -------------------- Config --------------------
name = "single_shot"
path = ""
yaml_name = "abtr_ref.yaml"
refmap_name = "abtr_ref.json"                   # optional: reflector json, e.g. "reflectors.json"
out_name = ""                      # default: <pgm_base>.json

version = 2.0
map_type = "2D"

point_anchor = "center"            # "center" or "corner"
float_round_ndigits = 3            # rounding for occupyPoints x/y
reflector_round_ndigits = 3        # rounding for reflectors x/y

size_mode = "full"                 # strongly recommended: full

include_known_mask = True          # keep knownMask (occupied OR strict-free)
include_free_band = False          # write freeBand bitset

# thresholds
occ_thr_override = None            # None => YAML occupied_thresh
free_thr_override = None           # None => YAML free_thresh (strict free for knownMask)
free_seed_thr_override = 0.45      # used to seed reachable free (broader than free_thr)

# scheme-1 params
free_band_radius_cells = 3         # K: band thickness in cells (e.g., 2~4)
connectivity = 4                   # 4-neighborhood recommended

json_indent = None
# ------------------------------------------------

import base64
import json
import os
from typing import Dict, Any, Tuple, Optional, List

import numpy as np

try:
    import yaml  # pip install pyyaml
except Exception:
    yaml = None

# optional accel
_HAS_SCIPY = False
try:
    from scipy import ndimage as ndi  # type: ignore
    _HAS_SCIPY = True
except Exception:
    _HAS_SCIPY = False


def _read_tokens_skip_comments(f) -> str:
    token = b""
    while True:
        ch = f.read(1)
        if not ch:
            break
        if ch in b" \t\r\n":
            if token:
                break
            continue
        if ch == b"#":
            f.readline()
            continue
        token += ch
    return token.decode("ascii")


def read_pgm(pgm_path: str) -> Tuple[np.ndarray, int]:
    with open(pgm_path, "rb") as f:
        magic = _read_tokens_skip_comments(f)
        if magic not in ("P5", "P2"):
            raise ValueError(f"Unsupported PGM magic: {magic} (only P5/P2)")

        w = int(_read_tokens_skip_comments(f))
        h = int(_read_tokens_skip_comments(f))
        maxval = int(_read_tokens_skip_comments(f))

        if magic == "P5":
            if maxval <= 255:
                data = f.read(w * h)
                if len(data) != w * h:
                    raise ValueError("PGM data truncated.")
                img = np.frombuffer(data, dtype=np.uint8).reshape((h, w))
            else:
                data = f.read(w * h * 2)
                if len(data) != w * h * 2:
                    raise ValueError("PGM data truncated.")
                img = np.frombuffer(data, dtype=">u2").reshape((h, w))
        else:
            vals = []
            while len(vals) < w * h:
                tok = _read_tokens_skip_comments(f)
                if not tok:
                    break
                vals.append(int(tok))
            if len(vals) != w * h:
                raise ValueError("ASCII PGM data truncated.")
            img = np.array(vals, dtype=np.uint16 if maxval > 255 else np.uint8).reshape((h, w))

        return img, maxval


def load_yaml_map(yaml_path: str) -> Dict[str, Any]:
    if yaml is None:
        raise RuntimeError("PyYAML not found. Please install: pip install pyyaml")
    with open(yaml_path, "r", encoding="utf-8") as f:
        data = yaml.safe_load(f)
    if not isinstance(data, dict):
        raise ValueError("Invalid YAML format.")
    return data


def load_json_file(json_path: str) -> Dict[str, Any]:
    with open(json_path, "r", encoding="utf-8") as f:
        data = json.load(f)
    if not isinstance(data, dict):
        raise ValueError(f"Invalid JSON format: {json_path}")
    return data


def resolve_path(base_dir: str, p: str) -> str:
    if os.path.isabs(p):
        return os.path.normpath(p)
    return os.path.normpath(os.path.join(base_dir, p))


def resolve_image_path(yaml_path: str, image_field: str) -> str:
    if os.path.isabs(image_field):
        return image_field
    base = os.path.dirname(os.path.abspath(yaml_path))
    return os.path.normpath(os.path.join(base, image_field))


def script_dir() -> str:
    return os.path.dirname(os.path.abspath(__file__))


def idx_to_world(ix: np.ndarray, iy: np.ndarray, ox: float, oy: float, res: float, anchor: str):
    if anchor == "center":
        wx = ox + (ix.astype(np.float64) + 0.5) * res
        wy = oy + (iy.astype(np.float64) + 0.5) * res
    elif anchor == "corner":
        wx = ox + ix.astype(np.float64) * res
        wy = oy + iy.astype(np.float64) * res
    else:
        raise ValueError("point_anchor must be 'center' or 'corner'")
    return wx, wy


def round_arr(a: np.ndarray, nd: Optional[int]):
    if nd is None:
        return a
    return np.round(a, nd)


def points_as_objects(wx: np.ndarray, wy: np.ndarray):
    return [{"x": float(x), "y": float(y)} for x, y in zip(wx.tolist(), wy.tolist())]


def load_reflectors_xy(json_path: str, ndigits: Optional[int] = None) -> List[Dict[str, float]]:
    """
    Input reflector json example:
    {
      "reflectors": [
        {"x": 1.23, "y": 4.56, "mean_intensity": 255, ...},
        ...
      ]
    }

    Output:
    [
      {"x": 1.23, "y": 4.56},
      ...
    ]
    """
    data = load_json_file(json_path)

    arr = data.get("reflectors", [])
    if not isinstance(arr, list):
        raise ValueError(f"'reflectors' must be a list in: {json_path}")

    out: List[Dict[str, float]] = []
    for i, item in enumerate(arr):
        if not isinstance(item, dict):
            raise ValueError(f"reflectors[{i}] must be an object")

        if "x" not in item or "y" not in item:
            raise KeyError(f"reflectors[{i}] missing x/y")

        x = float(item["x"])
        y = float(item["y"])

        if ndigits is not None:
            x = round(x, ndigits)
            y = round(y, ndigits)

        out.append({"x": x, "y": y})

    return out


def pack_bitset_to_b64(mask_bottom_up: np.ndarray) -> str:
    """
    mask_bottom_up: shape (h, w), bool/uint8
      - row0 is y=0 bottom row
      - flatten row-major: i=y*w+x
      - bit packing: lsb0 (little)
    """
    flat = mask_bottom_up.astype(np.uint8).reshape(-1)
    try:
        packed = np.packbits(flat, bitorder="little")
        raw = packed.tobytes()
    except TypeError:
        n = flat.size
        out = bytearray((n + 7) // 8)
        for i in range(n):
            if flat[i]:
                out[i >> 3] |= (1 << (i & 7))
        raw = bytes(out)
    return base64.b64encode(raw).decode("ascii")


def _struct4():
    s = np.zeros((3, 3), dtype=bool)
    s[1, 1] = True
    s[0, 1] = True
    s[2, 1] = True
    s[1, 0] = True
    s[1, 2] = True
    return s


def _struct8():
    return np.ones((3, 3), dtype=bool)


def dilate_numpy(src: np.ndarray, conn: int) -> np.ndarray:
    """binary dilation by 1 step using numpy slicing (no scipy)."""
    out = np.zeros_like(src, dtype=bool)
    out |= src
    out[:, 1:] |= src[:, :-1]
    out[:, :-1] |= src[:, 1:]
    out[1:, :] |= src[:-1, :]
    out[:-1, :] |= src[1:, :]
    if conn == 8:
        out[1:, 1:] |= src[:-1, :-1]
        out[1:, :-1] |= src[:-1, 1:]
        out[:-1, 1:] |= src[1:, :-1]
        out[:-1, :-1] |= src[1:, 1:]
    return out


def geodesic_reachable(seed: np.ndarray, mask: np.ndarray, conn: int) -> np.ndarray:
    """
    reachable = flood fill within mask, starting from seed, using 4/8-neighborhood.
    Prefer scipy.ndimage.binary_propagation if available.
    """
    if _HAS_SCIPY:
        struct = _struct4() if conn == 4 else _struct8()
        return ndi.binary_propagation(seed.astype(bool), mask=mask.astype(bool), structure=struct)

    reachable = seed.astype(bool).copy()
    while True:
        nbr = dilate_numpy(reachable, conn)
        new = nbr & mask & (~reachable)
        if not new.any():
            break
        reachable |= new
    return reachable


def band_from_reachable(reachable: np.ndarray, non_occ: np.ndarray, k: int, conn: int) -> np.ndarray:
    """
    band = cells within <=k steps from reachable, constrained in non_occ.
    Includes reachable itself.
    """
    if k <= 0:
        return reachable.astype(bool)

    if _HAS_SCIPY:
        struct = _struct4() if conn == 4 else _struct8()
        band = ndi.binary_dilation(
            reachable.astype(bool),
            structure=struct,
            iterations=k,
            mask=non_occ.astype(bool),
        )
        return band.astype(bool)

    band = reachable.astype(bool).copy()
    frontier = reachable.astype(bool).copy()
    for _ in range(k):
        nbr = dilate_numpy(frontier, conn)
        nbr &= non_occ
        new = nbr & (~band)
        if not new.any():
            break
        band |= new
        frontier = new
    return band


def main():
    global name, path, yaml_name, refmap_name, out_name, size_mode

    if not isinstance(yaml_name, str) or yaml_name.strip() == "":
        raise ValueError("yaml_name 不能为空，请配置 yaml_name='xxx.yaml'")

    base_dir = (path or "").strip() or script_dir()
    base_dir = os.path.abspath(base_dir)

    yaml_path = resolve_path(base_dir, yaml_name)
    if not os.path.isfile(yaml_path):
        raise FileNotFoundError(f"YAML not found: {yaml_path}")

    reflectors_xy: List[Dict[str, float]] = []
    refmap_path = None
    if isinstance(refmap_name, str) and refmap_name.strip() != "":
        refmap_path = resolve_path(base_dir, refmap_name)
        if not os.path.isfile(refmap_path):
            raise FileNotFoundError(f"Reflector JSON not found: {refmap_path}")
        reflectors_xy = load_reflectors_xy(refmap_path, reflector_round_ndigits)

    y = load_yaml_map(yaml_path)

    if "image" not in y:
        raise KeyError("YAML missing required key: image")
    image_path = resolve_image_path(yaml_path, str(y["image"]))
    if not os.path.isfile(image_path):
        raise FileNotFoundError(f"PGM not found: {image_path}")

    resolution = float(y.get("resolution", 0.0))
    origin = y.get("origin", [0.0, 0.0, 0.0])
    origin_x = float(origin[0])
    origin_y = float(origin[1])
    origin_yaw_raw = float(origin[2]) if len(origin) >= 3 else 0.0
    if abs(origin_yaw_raw) > 1e-9:
        raise ValueError(f"origin yaw must be 0.0 for this pipeline, got {origin_yaw_raw}")
    origin_yaw = 0.0

    negate = int(y.get("negate", 0))
    occ_thr = float(y.get("occupied_thresh", 0.65))
    free_thr = float(y.get("free_thresh", 0.196))

    if occ_thr_override is not None:
        occ_thr = float(occ_thr_override)
    if free_thr_override is not None:
        free_thr = float(free_thr_override)

    free_seed_thr = float(free_seed_thr_override) if free_seed_thr_override is not None else free_thr

    img_raw, maxval = read_pgm(image_path)
    h, w = img_raw.shape

    img = img_raw.astype(np.float32)
    if maxval != 255:
        img = img * (255.0 / float(maxval))

    # occupancy probability (map_server style)
    if negate == 0:
        occ_prob = (255.0 - img) / 255.0
    else:
        occ_prob = img / 255.0

    occupied = occ_prob > occ_thr
    non_occ = ~occupied

    # knownMask (old): known = occupied OR strict-free
    strict_free = occ_prob < free_thr
    known = occupied | strict_free

    # scheme-1: reachable free inside non-occ, seeded by free_seed
    free_seed = occ_prob < free_seed_thr
    free_seed &= non_occ  # seed must be non-occupied

    if include_free_band:
        if connectivity not in (4, 8):
            raise ValueError("connectivity must be 4 or 8")
        reachable = geodesic_reachable(free_seed, non_occ, connectivity)
        free_band = band_from_reachable(reachable, non_occ, int(free_band_radius_cells), connectivity)
    else:
        reachable = None
        free_band = None

    # stats
    ncell = int(w * h)
    occ_cnt = int(np.sum(occupied))
    free_cnt = int(np.sum(strict_free))
    known_cnt = int(np.sum(known))
    seed_cnt = int(np.sum(free_seed))

    print(f"[STAT] cells w*h = {ncell}")
    print(f"[STAT] occupied = {occ_cnt}")
    print(f"[STAT] strict_free(<free_thr) = {free_cnt}  free_ratio={free_cnt/ncell:.6f}")
    print(f"[STAT] known(occ|strict_free) = {known_cnt} known_ratio={known_cnt/ncell:.6f}")
    print(f"[STAT] thresholds: occ_thr={occ_thr:.6f} free_thr={free_thr:.6f} free_seed_thr={free_seed_thr:.6f}")
    print(f"[STAT] scipy_accel = {_HAS_SCIPY}")
    print(f"[STAT] reflectors loaded = {len(reflectors_xy)}")

    if include_free_band:
        reach_cnt = int(np.sum(reachable))
        band_cnt = int(np.sum(free_band))
        print(f"[STAT] free_seed cells = {seed_cnt} seed_ratio={seed_cnt/ncell:.6f}")
        print(f"[STAT] free_reachable cells = {reach_cnt} reach_ratio={reach_cnt/ncell:.6f}")
        print(f"[STAT] free_band cells (<=K) = {band_cnt} band_ratio={band_cnt/ncell:.6f}  K={free_band_radius_cells}")

        # proxy: boundary cells where occupied has at least one neighbor in free_band
        occ_bu = np.flipud(occupied)
        band_bu = np.flipud(free_band)

        neigh = np.zeros_like(occ_bu, dtype=bool)
        neigh[:, 1:] |= band_bu[:, :-1]
        neigh[:, :-1] |= band_bu[:, 1:]
        neigh[1:, :] |= band_bu[:-1, :]
        neigh[:-1, :] |= band_bu[1:, :]
        if connectivity == 8:
            neigh[1:, 1:] |= band_bu[:-1, :-1]
            neigh[1:, :-1] |= band_bu[:-1, 1:]
            neigh[:-1, 1:] |= band_bu[1:, :-1]
            neigh[:-1, :-1] |= band_bu[1:, 1:]
        boundary_proxy = int(np.sum(occ_bu & neigh))
        print(f"[STAT] boundary_proxy_occ_cells(using freeBand) = {boundary_proxy} ratio_vs_occ={boundary_proxy/max(1,occ_cnt):.6f}")

    # occupyPoints: from occupied cells (topdown coords -> bottom-up index)
    occ_rows, occ_cols = np.where(occupied)
    ix = occ_cols.astype(np.int32)
    iy = (h - 1 - occ_rows).astype(np.int32)
    wx, wy = idx_to_world(ix, iy, origin_x, origin_y, resolution, point_anchor)
    wx = round_arr(wx, float_round_ndigits)
    wy = round_arr(wy, float_round_ndigits)
    occupy_points = points_as_objects(wx, wy)

    header_name = (name or "").strip() or os.path.splitext(os.path.basename(yaml_name))[0]

    if include_free_band and size_mode != "full":
        print("[WARN] include_free_band=True strongly suggests size_mode='full'. Force to 'full'.")
        size_mode = "full"

    # size in meters
    if size_mode == "full":
        size_w = float(w) * resolution
        size_h = float(h) * resolution
        out_origin_x, out_origin_y = origin_x, origin_y
    else:
        raise ValueError("This script expects size_mode='full' for freeBand alignment.")

    outn = (out_name or "").strip()
    if outn == "":
        pgm_base = os.path.splitext(os.path.basename(image_path))[0]
        outn = pgm_base + "_map.json"
    elif not outn.lower().endswith(".json"):
        outn += ".json"
    out_path = os.path.join(base_dir, outn)

    out_obj: Dict[str, Any] = {
        "header": {
            "origin": {"x": out_origin_x, "y": out_origin_y, "yaw": 0.0},
            "name": header_name,
            "version": float(version),
            "mapType": map_type,
            "resolution": float(resolution),
            "size": {"width": float(size_w), "height": float(size_h)},
        },
        "reflectors": reflectors_xy,
        "occupyPoints": occupy_points,
    }

    if include_known_mask:
        known_bu = np.flipud(known)
        out_obj["knownMask"] = {
            "encoding": "base64",
            "codec": "bitset",
            "bitOrder": "lsb0",
            "layout": "row-major",
            "yAxis": "bottom-up",
            "width": int(w),
            "height": int(h),
            "data": pack_bitset_to_b64(known_bu),
            "meta": {
                "occThr": float(occ_thr),
                "freeThr": float(free_thr),
                "definition": "known = occupied OR (occ_prob < freeThr)",
            },
        }

    if include_free_band:
        band_bu = np.flipud(free_band)
        out_obj["freeBand"] = {
            "encoding": "base64",
            "codec": "bitset",
            "bitOrder": "lsb0",
            "layout": "row-major",
            "yAxis": "bottom-up",
            "width": int(w),
            "height": int(h),
            "data": pack_bitset_to_b64(band_bu),
            "meta": {
                "radiusCells": int(free_band_radius_cells),
                "connectivity": int(connectivity),
                "occThr": float(occ_thr),
                "freeSeedThr": float(free_seed_thr),
                "definition": "freeBand = nonOcc cells within <=K steps from free_reachable(seed=occ_prob<freeSeedThr), constrained in nonOcc",
                "scipyAccel": bool(_HAS_SCIPY),
            },
        }

    with open(out_path, "w", encoding="utf-8") as f:
        json.dump(
            out_obj,
            f,
            ensure_ascii=False,
            indent=json_indent,
            separators=(",", ":") if json_indent is None else None,
        )

    print(f"[OK] base_dir : {base_dir}")
    print(f"[OK] YAML     : {yaml_path}")
    print(f"[OK] PGM      : {image_path}")
    if refmap_path is not None:
        print(f"[OK] REFMAP   : {refmap_path}")
    else:
        print(f"[OK] REFMAP   : <none>")
    print(f"[OK] resolution={resolution}, origin=({origin_x},{origin_y},{origin_yaw}), negate={negate}")
    print(f"[OK] image cells: {w} x {h}  => size(m): {w*resolution:.3f} x {h*resolution:.3f}")
    print(f"[OK] reflectors: {len(reflectors_xy)}")
    print(f"[OK] occupyPoints: {len(occupy_points)}")
    if include_known_mask:
        nbits = w * h
        nbytes = (nbits + 7) // 8
        print(f"[OK] knownMask bitset: bits={nbits} bytes={nbytes}")
    if include_free_band:
        nbits = w * h
        nbytes = (nbits + 7) // 8
        print(f"[OK] freeBand bitset: bits={nbits} bytes={nbytes}  K={free_band_radius_cells} conn={connectivity}")
    print(f"[OK] output  : {out_path}")


if __name__ == "__main__":
    main()
