#!/usr/bin/env python3
import argparse
import os
import sys


def add_vendor_path() -> str:
    script_dir = os.path.dirname(os.path.realpath(__file__))
    project_root = os.path.realpath(os.path.join(script_dir, ".."))
    vendor_src = os.path.join(project_root, "third_party", "sionna_rt_gui", "src")
    if vendor_src not in sys.path:
        sys.path.insert(0, vendor_src)
    return vendor_src


add_vendor_path()

from sionna_rt_gui import DEFAULT_CONFIG_PATH  # noqa: E402
from sionna_rt_gui.config import load_config  # noqa: E402
from sionna_rt_gui.reload import AppHolder  # noqa: E402


def main() -> int:
    parser = argparse.ArgumentParser(description="Embedded launcher for the original sionna-rt-gui")
    parser.add_argument("--scene", required=True, help="Path to the scene XML to load")
    parser.add_argument("--title", required=True, help="Unique window title used for embedding")
    parser.add_argument("--envmap", default="", help="Optional environment map override")
    args = parser.parse_args()

    cfg = load_config(DEFAULT_CONFIG_PATH, scene_filename=args.scene)
    cfg.title = args.title
    cfg.use_live_reload = False
    if args.envmap:
        cfg.rendering.envmap = args.envmap

    app = AppHolder(cfg, scene_filename=args.scene, overrides={})
    app.show()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
