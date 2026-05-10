#!/usr/bin/env python3
from __future__ import annotations

import argparse
import json
import math
import sys


def extract_scene_bounds(scene_path: str, mi_variant: str) -> dict:
    import mitsuba as mi
    import numpy as np

    mi.set_variant(mi_variant)
    scene = mi.load_file(scene_path)
    bbox = scene.bbox()
    candidates = []
    try:
        candidates.append((np.asarray(bbox.min), np.asarray(bbox.max)))
    except Exception:
        pass
    try:
        candidates.append((np.asarray(bbox.min()), np.asarray(bbox.max())))
    except Exception:
        pass
    for bmin, bmax in candidates:
        bmin = np.asarray(bmin, dtype=float).reshape(-1)
        bmax = np.asarray(bmax, dtype=float).reshape(-1)
        if bmin.size >= 3 and bmax.size >= 3:
            values = [float(bmin[0]), float(bmax[0]), float(bmin[1]), float(bmax[1]), float(bmin[2]), float(bmax[2])]
            if all(math.isfinite(v) for v in values):
                return {
                    "min_x": values[0],
                    "max_x": values[1],
                    "min_y": values[2],
                    "max_y": values[3],
                    "min_z": values[4],
                    "max_z": values[5],
                }
    raise RuntimeError(f"failed to read scene bounds from XML: {scene_path}")


def main() -> int:
    parser = argparse.ArgumentParser(description="Read Mitsuba scene bounds from XML.")
    parser.add_argument("--scene-path", required=True)
    parser.add_argument("--mi-variant", default="cuda_ad_mono_polarized")
    args = parser.parse_args()

    try:
        result = extract_scene_bounds(args.scene_path, args.mi_variant)
    except Exception as exc:
        print(str(exc), file=sys.stderr)
        return 1

    print(json.dumps(result, ensure_ascii=False))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
