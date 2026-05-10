#!/usr/bin/env python3
from __future__ import annotations

import argparse
import json
import math
import os
import sys
import time
import tempfile
from dataclasses import asdict
from typing import Any, Dict, Iterable, List, Tuple

import numpy as np

try:
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    from matplotlib import colors as mcolors
    from matplotlib import cm as mcm
    from matplotlib import ticker
except Exception:  # pragma: no cover
    matplotlib = None
    plt = None
    mcolors = None
    mcm = None
    ticker = None

try:
    from PIL import Image
except Exception:  # pragma: no cover
    Image = None

try:
    from scipy import ndimage as ndi
except Exception:  # pragma: no cover
    ndi = None

from sionna_sim_only_topic2 import OfflineSionnaSimulator, SimulationConfig, load_bs_list_from_json

SUPPORTED_METRICS: List[str] = [
    "power_db",
    "path_loss_db",
    "tau_std_ns",
    "los_binary",
    "best_bs_index",
    "serving_bs_index",
    "sys_sinr_eff_db",
    "sys_spectral_efficiency_bpshz",
    "sys_mcs_index",
    "sys_tb_ok",
    "best_bs_rate_bpshz",
    "beam_oracle_index",
    "beam_oracle_gain_db",
    "beam_deepsense_index",
    "beam_deepsense_gain_db",
]

SYS_METRICS = {
    "serving_bs_index",
    "sys_sinr_eff_db",
    "sys_spectral_efficiency_bpshz",
    "sys_mcs_index",
    "sys_tb_ok",
    "best_bs_rate_bpshz",
}

BEAM_METRICS = {
    "beam_oracle_index",
    "beam_oracle_gain_db",
    "beam_deepsense_index",
    "beam_deepsense_gain_db",
}

PREDICTED_BEAM_METRICS = {
    "beam_deepsense_index",
    "beam_deepsense_gain_db",
}

METRIC_DISPLAY_NAMES: Dict[str, str] = {
    "power_db": "Power (dB)",
    "path_loss_db": "Path loss (dB)",
    "tau_std_ns": "RMS delay spread (ns)",
    "los_binary": "LoS binary",
    "best_bs_index": "Best BS index",
    "serving_bs_index": "Serving BS index",
    "sys_sinr_eff_db": "SYS effective SINR (dB)",
    "sys_spectral_efficiency_bpshz": "SYS spectral efficiency (bps/Hz)",
    "sys_mcs_index": "SYS MCS index",
    "sys_tb_ok": "SYS TB OK",
    "best_bs_rate_bpshz": "Best BS rate (bps/Hz)",
    "beam_oracle_index": "Oracle beam index",
    "beam_oracle_gain_db": "Oracle beam gain (dB)",
    "beam_deepsense_index": "Predicted beam index",
    "beam_deepsense_gain_db": "Predicted beam gain (dB)",
}

METRIC_COLORBAR_LABELS: Dict[str, str] = {
    "power_db": "Received power (dB)",
    "path_loss_db": "Path loss (dB)",
    "tau_std_ns": "RMS delay spread (ns)",
    "los_binary": "LoS indicator",
    "best_bs_index": "Best BS index",
    "serving_bs_index": "Serving BS index",
    "sys_sinr_eff_db": "Effective SINR (dB)",
    "sys_spectral_efficiency_bpshz": "Spectral efficiency (bps/Hz)",
    "sys_mcs_index": "MCS index",
    "sys_tb_ok": "Transport block OK",
    "best_bs_rate_bpshz": "Best BS rate (bps/Hz)",
    "beam_oracle_index": "Oracle beam index",
    "beam_oracle_gain_db": "Oracle beam gain (dB)",
    "beam_deepsense_index": "Predicted beam index",
    "beam_deepsense_gain_db": "Predicted beam gain (dB)",
}

METRIC_RENDER_LABELS: Dict[str, str] = {
    "power_db": "Received power",
    "path_loss_db": "Path loss",
    "tau_std_ns": "RMS delay spread",
    "los_binary": "LoS indicator",
    "best_bs_index": "Best BS index",
    "serving_bs_index": "Serving BS index",
    "sys_sinr_eff_db": "Effective SINR",
    "sys_spectral_efficiency_bpshz": "Spectral efficiency",
    "sys_mcs_index": "MCS index",
    "sys_tb_ok": "Transport block OK",
    "best_bs_rate_bpshz": "Best BS rate",
    "beam_oracle_index": "Oracle beam index",
    "beam_oracle_gain_db": "Oracle beam gain",
    "beam_deepsense_index": "Predicted beam index",
    "beam_deepsense_gain_db": "Predicted beam gain",
}

METRIC_UNITS: Dict[str, str | None] = {
    "power_db": "dB",
    "path_loss_db": "dB",
    "tau_std_ns": "ns",
    "los_binary": None,
    "best_bs_index": "index",
    "serving_bs_index": "index",
    "sys_sinr_eff_db": "dB",
    "sys_spectral_efficiency_bpshz": "bps/Hz",
    "sys_mcs_index": "index",
    "sys_tb_ok": None,
    "best_bs_rate_bpshz": "bps/Hz",
    "beam_oracle_index": "index",
    "beam_oracle_gain_db": "dB",
    "beam_deepsense_index": "index",
    "beam_deepsense_gain_db": "dB",
}

def metric_ui_name(metric: str) -> str:
    return str(metric).replace("beam_deepsense", "beam_predicted")


def metric_ui_list(metrics: Iterable[str]) -> str:
    return ", ".join(metric_ui_name(metric) for metric in metrics)

FAST_SCENE_RENDER_RESOLUTION: Tuple[int, int] = (1400, 1400)
FAST_SCENE_RENDER_SPP = 48
ORTHO_SCENE_RENDER_MARGIN = 1.04
PICK_MAP_MAX_CODE = 65534
PICK_MAP_INVALID_CODE = 65535


SCENE_IMAGE_TRANSFORM_MODES = [
    "identity",
    "rot90_cw",
    "rot90_ccw",
    "rot180",
    "flip_x",
    "flip_y",
    "transpose",
    "transpose_flipx",
    "transpose_flipy",
]


def apply_discrete_image_transform(image: np.ndarray, mode: str) -> np.ndarray:
    arr = np.asarray(image)
    name = str(mode or "identity").strip().lower()
    if name == "identity":
        return np.array(arr, copy=True)
    if name == "rot90_cw":
        return np.rot90(arr, k=3).copy()
    if name == "rot90_ccw":
        return np.rot90(arr, k=1).copy()
    if name == "rot180":
        return np.rot90(arr, k=2).copy()
    if name == "flip_x":
        return np.fliplr(arr).copy()
    if name == "flip_y":
        return np.flipud(arr).copy()
    if name == "transpose":
        return np.transpose(arr, (1, 0) + tuple(range(2, arr.ndim))).copy()
    if name == "transpose_flipx":
        return np.fliplr(np.transpose(arr, (1, 0) + tuple(range(2, arr.ndim)))).copy()
    if name == "transpose_flipy":
        return np.flipud(np.transpose(arr, (1, 0) + tuple(range(2, arr.ndim)))).copy()
    return np.array(arr, copy=True)


def transform_image_point(point_xy: Tuple[float, float], image_size: Tuple[int, int], mode: str) -> Tuple[float, float]:
    x = float(point_xy[0])
    y = float(point_xy[1])
    width = int(image_size[0])
    height = int(image_size[1])
    if width <= 0 or height <= 0:
        return (x, y)
    name = str(mode or "identity").strip().lower()
    if name == "identity":
        return (x, y)
    if name == "rot90_cw":
        return (height - 1.0 - y, x)
    if name == "rot90_ccw":
        return (y, width - 1.0 - x)
    if name == "rot180":
        return (width - 1.0 - x, height - 1.0 - y)
    if name == "flip_x":
        return (width - 1.0 - x, y)
    if name == "flip_y":
        return (x, height - 1.0 - y)
    if name == "transpose":
        return (y, x)
    if name == "transpose_flipx":
        return (height - 1.0 - y, x)
    if name == "transpose_flipy":
        return (y, width - 1.0 - x)
    return (x, y)


def transformed_image_size(image_size: Tuple[int, int], mode: str) -> Tuple[int, int]:
    width = int(image_size[0])
    height = int(image_size[1])
    if str(mode or "identity").strip().lower() in {"rot90_cw", "rot90_ccw", "transpose", "transpose_flipx", "transpose_flipy"}:
        return (height, width)
    return (width, height)


def transform_quad_image_xy(quad_xy: List[List[float]] | np.ndarray, image_size: Tuple[int, int], mode: str) -> List[List[float]]:
    quad = np.asarray(quad_xy, dtype=np.float64).reshape(-1, 2)
    transformed = [transform_image_point((float(pt[0]), float(pt[1])), image_size, mode) for pt in quad]
    return [[float(x), float(y)] for x, y in transformed]


def color_overlay_mask_from_rgb(rgb: np.ndarray) -> np.ndarray:
    arr = np.asarray(rgb, dtype=np.float32)
    if arr.ndim == 2:
        arr = np.repeat(arr[..., np.newaxis], 3, axis=-1)
    if arr.shape[-1] > 3:
        arr = arr[..., :3]
    if arr.max() > 1.5:
        arr = arr / 255.0
    arr = np.clip(arr, 0.0, 1.0)
    mx = np.max(arr, axis=-1)
    mn = np.min(arr, axis=-1)
    sat = np.where(mx > 1e-6, (mx - mn) / mx, 0.0)
    return (sat > 0.18) & (mx > 0.18)


def filtered_scene_overlay_mask(rgb: np.ndarray) -> np.ndarray:
    base_mask = color_overlay_mask_from_rgb(rgb)
    if not np.any(base_mask):
        return base_mask
    if ndi is None:
        return base_mask
    labels, num = ndi.label(base_mask)
    if int(num) <= 1:
        return base_mask
    counts = np.bincount(labels.ravel())
    if counts.size <= 1:
        return base_mask
    counts[0] = 0
    largest = int(np.max(counts))
    min_keep = max(64, int(round(largest * 0.01)))
    keep = counts >= min_keep
    keep[0] = False
    filtered = keep[labels]
    return filtered if np.any(filtered) else base_mask


def crop_mask_to_bbox(mask: np.ndarray) -> np.ndarray:
    ys, xs = np.nonzero(mask)
    if ys.size == 0 or xs.size == 0:
        return np.zeros((1, 1), dtype=bool)
    return mask[int(np.min(ys)):int(np.max(ys)) + 1, int(np.min(xs)):int(np.max(xs)) + 1]


def resize_binary_mask(mask: np.ndarray, out_size_hw: Tuple[int, int]) -> np.ndarray:
    if Image is None:
        raise RuntimeError("Pillow is required to resize CKM masks")
    src = Image.fromarray(np.asarray(mask, dtype=np.uint8) * 255, mode="L")
    if hasattr(Image, 'Resampling'):
        resized = src.resize((int(out_size_hw[1]), int(out_size_hw[0])), resample=Image.Resampling.NEAREST)
    else:
        resized = src.resize((int(out_size_hw[1]), int(out_size_hw[0])), resample=Image.NEAREST)
    return np.asarray(resized, dtype=np.uint8) >= 128


def detect_scene_image_transform_from_masks(reference_mask: np.ndarray, scene_rgb: np.ndarray) -> Tuple[str, float]:
    ref_crop = crop_mask_to_bbox(np.asarray(reference_mask, dtype=bool))
    scene_mask = filtered_scene_overlay_mask(scene_rgb)
    scene_crop = crop_mask_to_bbox(scene_mask)
    if not np.any(ref_crop) or not np.any(scene_crop):
        return ("identity", 0.0)
    best_mode = "identity"
    best_score = -1.0
    identity_score = -1.0
    target_shape = ref_crop.shape
    for mode in SCENE_IMAGE_TRANSFORM_MODES:
        transformed = apply_discrete_image_transform(scene_crop.astype(np.uint8), mode) > 0
        resized = resize_binary_mask(transformed, target_shape)
        union = np.logical_or(ref_crop, resized)
        union_count = int(np.count_nonzero(union))
        score = 0.0 if union_count <= 0 else float(np.count_nonzero(np.logical_and(ref_crop, resized))) / float(union_count)
        if mode == "identity":
            identity_score = score
        if score > best_score:
            best_score = score
            best_mode = mode
    if best_score < max(0.18, identity_score + 0.08):
        return ("identity", identity_score)
    return (best_mode, best_score)


def rewrite_png_with_transform(png_path: str, mode: str) -> None:
    if Image is None or not png_path or not os.path.isfile(png_path):
        return
    if str(mode or "identity").strip().lower() == "identity":
        return
    img = np.asarray(Image.open(png_path))
    transformed = apply_discrete_image_transform(img, mode)
    Image.fromarray(transformed).save(png_path, format="PNG")


def update_pickmap_and_click_mapping_for_image_transform(scene_pick_map: Dict[str, Any], scene_click_mapping: Dict[str, Any], mode: str) -> Tuple[Dict[str, Any], Dict[str, Any]]:
    name = str(mode or "identity").strip().lower()
    updated_pick = dict(scene_pick_map or {})
    updated_click = dict(scene_click_mapping or {})
    if name == "identity":
        return updated_pick, updated_click
    if updated_pick:
        width = int(updated_pick.get("image_width", 0))
        height = int(updated_pick.get("image_height", 0))
        quad = updated_pick.get("quad_image_xy")
        if width > 0 and height > 0 and quad:
            updated_pick["quad_image_xy"] = transform_quad_image_xy(quad, (width, height), name)
            new_w, new_h = transformed_image_size((width, height), name)
            updated_pick["image_width"] = int(new_w)
            updated_pick["image_height"] = int(new_h)
    if updated_click:
        width = int(updated_click.get("image_width", updated_pick.get("image_width", 0)))
        height = int(updated_click.get("image_height", updated_pick.get("image_height", 0)))
        quad = updated_click.get("quad_image_xy")
        if width > 0 and height > 0 and quad:
            updated_click["quad_image_xy"] = transform_quad_image_xy(quad, (width, height), name)
            new_w, new_h = transformed_image_size((width, height), name)
            updated_click["image_width"] = int(new_w)
            updated_click["image_height"] = int(new_h)
        updated_click.pop("camera", None)
    return updated_pick, updated_click


def reference_overlay_mask_from_metric_png(png_path: str) -> np.ndarray:
    if Image is None:
        raise RuntimeError("Pillow is required to inspect CKM PNG masks")
    arr = np.asarray(Image.open(png_path).convert("RGB"))
    return color_overlay_mask_from_rgb(arr)


class FixedFormatter(ticker.Formatter if ticker is not None else object):
    def __init__(self, tick_map: Dict[float, str]):
        self.tick_map = tick_map

    def __call__(self, x, pos=None):  # pragma: no cover - matplotlib callback
        for tick, label in self.tick_map.items():
            if abs(float(x) - float(tick)) < 1e-6:
                return label
        if abs(x - round(x)) < 1e-6:
            return str(int(round(x)))
        return f"{x:.2f}"


def parse_bool(text: str) -> bool:
    value = str(text).strip().lower()
    if value in {"1", "true", "yes", "on"}:
        return True
    if value in {"0", "false", "no", "off"}:
        return False
    raise argparse.ArgumentTypeError(f"invalid boolean value: {text}")


def create_unique_output_root(base_output_dir: str, explicit_output_root_dir: str = "") -> tuple[str, str]:
    explicit_dir = str(explicit_output_root_dir or "").strip()
    if explicit_dir:
        output_root_dir = os.path.abspath(explicit_dir)
        os.makedirs(output_root_dir, exist_ok=True)
        run_id = os.path.basename(os.path.normpath(output_root_dir)) or f"ckm_{os.getpid()}"
        return run_id, output_root_dir

    base_dir = os.path.abspath(base_output_dir)
    os.makedirs(base_dir, exist_ok=True)
    timestamp = time.strftime("%Y%m%d_%H%M%S", time.localtime())
    millis = int((time.time() % 1.0) * 1000.0)
    run_id = f"{timestamp}_{millis:03d}_{os.getpid()}"
    output_root_dir = os.path.join(base_dir, run_id)
    suffix = 0
    while os.path.exists(output_root_dir):
        suffix += 1
        output_root_dir = os.path.join(base_dir, f"{run_id}_{suffix}")
    os.makedirs(output_root_dir, exist_ok=True)
    return run_id, output_root_dir


def build_arg_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description="Generate dense CKM maps from Sionna RT/SYS.")
    parser.add_argument("--scene-path", required=True)
    parser.add_argument("--render-scene-path", default="")
    parser.add_argument("--bs-list-json", required=True)
    parser.add_argument("--metric", action="append", default=[])
    parser.add_argument("--default-metric", default="")
    parser.add_argument("--x-min", type=float, required=True)
    parser.add_argument("--x-max", type=float, required=True)
    parser.add_argument("--y-min", type=float, required=True)
    parser.add_argument("--y-max", type=float, required=True)
    parser.add_argument("--z-fixed", type=float, required=True)
    parser.add_argument("--resolution-m", type=float, required=True)
    parser.add_argument("--output-dir", required=True)
    parser.add_argument("--output-root-dir", default="")
    parser.add_argument("--range-source", default="manual_range")
    parser.add_argument("--render-scene-overlay", type=parse_bool, default=True)
    parser.add_argument("--render-xy-transform", default="auto")
    parser.add_argument("--fc-hz", type=float, default=3.5e9)
    parser.add_argument("--mi-variant", default="cuda_ad_mono_polarized")

    parser.add_argument("--tx-array-num-rows", type=int, default=1)
    parser.add_argument("--tx-array-num-cols", type=int, default=1)
    parser.add_argument("--tx-array-vertical-spacing", type=float, default=0.5)
    parser.add_argument("--tx-array-horizontal-spacing", type=float, default=0.5)
    parser.add_argument("--tx-array-pattern", default="iso")
    parser.add_argument("--tx-array-polarization", default="V")

    parser.add_argument("--rx-array-num-rows", type=int, default=1)
    parser.add_argument("--rx-array-num-cols", type=int, default=1)
    parser.add_argument("--rx-array-vertical-spacing", type=float, default=0.5)
    parser.add_argument("--rx-array-horizontal-spacing", type=float, default=0.5)
    parser.add_argument("--rx-array-pattern", default="iso")
    parser.add_argument("--rx-array-polarization", default="V")

    parser.add_argument("--max-depth", type=int, default=1)
    parser.add_argument("--samples-per-src", type=int, default=256)
    parser.add_argument("--max-num-paths-per-src", type=int, default=64)
    parser.add_argument("--synthetic-array", type=parse_bool, default=True)
    parser.add_argument("--merge-shapes", type=parse_bool, default=False)

    parser.add_argument("--enable-sys-integration", type=parse_bool, default=False)
    parser.add_argument("--enable-beamforming", type=parse_bool, default=False)
    parser.add_argument("--beam-selection-mode", default="exhaustive_sweep")
    parser.add_argument("--beam-codebook-type", default="auto")
    parser.add_argument("--beam-codebook-num-beams", type=int, default=8)
    parser.add_argument("--beam-oversampling-v", type=int, default=1)
    parser.add_argument("--beam-oversampling-h", type=int, default=1)
    parser.add_argument("--beam-manual-index", type=int, default=0)
    parser.add_argument("--beam-normalize-power", type=parse_bool, default=True)
    parser.add_argument("--beam-codebook-file", default="")
    parser.add_argument("--beam-model-checkpoint-path", default="")
    parser.add_argument("--beam-feature-mode", default="geom_vel_path13")
    parser.add_argument("--beam-top-k", type=int, default=3)
    parser.add_argument("--sys-num-subcarriers", type=int, default=128)
    parser.add_argument("--sys-subcarrier-spacing-hz", type=float, default=30e3)
    parser.add_argument("--sys-num-ofdm-symbols", type=int, default=12)
    parser.add_argument("--sys-temperature-k", type=float, default=294.0)
    parser.add_argument("--sys-bler-target", type=float, default=0.1)
    parser.add_argument("--sys-mcs-table-index", type=int, default=1)
    parser.add_argument("--sys-bs-tx-power-dbm", type=float, default=10.0)

    parser.add_argument("--los", type=parse_bool, default=True)
    parser.add_argument("--specular-reflection", type=parse_bool, default=True)
    parser.add_argument("--diffuse-reflection", type=parse_bool, default=False)
    parser.add_argument("--refraction", type=parse_bool, default=False)
    parser.add_argument("--diffraction", type=parse_bool, default=False)
    parser.add_argument("--edge-diffraction", type=parse_bool, default=False)
    parser.add_argument("--diffraction-lit-region", type=parse_bool, default=False)
    return parser


def build_grid(min_value: float, max_value: float, resolution_m: float) -> np.ndarray:
    count = max(2, int(np.floor((max_value - min_value) / max(resolution_m, 1e-9))) + 1)
    return np.linspace(min_value, max_value, count, dtype=float)


def to_numpy(value: Any) -> np.ndarray:
    if isinstance(value, np.ndarray):
        return value
    if hasattr(value, "numpy"):
        try:
            return np.asarray(value.numpy())
        except Exception:
            pass
    return np.asarray(value)


def safe_float(value: Any) -> float:
    try:
        number = float(value)
    except Exception:
        return float("nan")
    if math.isnan(number) or math.isinf(number):
        return float("nan")
    return number


def normalize_metrics(raw_metrics: List[str], fallback_metric: str) -> List[str]:
    metrics: List[str] = []
    seen = set()
    for raw_metric in raw_metrics:
        metric = str(raw_metric).strip()
        if not metric or metric not in SUPPORTED_METRICS or metric in seen:
            continue
        metrics.append(metric)
        seen.add(metric)
    if not metrics:
        fallback = fallback_metric.strip() if fallback_metric.strip() in SUPPORTED_METRICS else "power_db"
        metrics.append(fallback)
    return metrics


def metric_values_from_summary(metric_names: List[str], summary: Dict[str, Any]) -> Dict[str, float]:
    best_power_db = summary["best_power_db_raw"]
    best_tau_std_ns = summary["best_tau_std_ns_raw"]
    best_bs_index = summary["best_bs_index"]
    best_rate = summary["best_bs_rate_bpshz_raw"]
    has_any_path = summary["has_any_path"]
    has_any_los = summary["has_any_los"]
    serving_idx = summary["serving_bs_index"]
    candidate_sinr = summary["candidate_sinr_eff_db_raw"]
    candidate_se = summary["candidate_spectral_efficiency_raw"]
    candidate_mcs = summary["candidate_mcs_index_raw"]
    candidate_tb_ok = summary["candidate_tb_ok_raw"]
    beam_oracle_index = summary["beam_oracle_index_raw"]
    beam_oracle_gain_db = summary["beam_oracle_gain_db_raw"]
    beam_deepsense_index = summary["beam_deepsense_index_raw"]
    beam_deepsense_gain_db = summary["beam_deepsense_gain_db_raw"]

    values: Dict[str, float] = {}
    for metric in metric_names:
        value = float("nan")
        if metric == "power_db":
            value = best_power_db
        elif metric == "path_loss_db":
            value = -best_power_db if np.isfinite(best_power_db) else float("nan")
        elif metric == "tau_std_ns":
            value = best_tau_std_ns
        elif metric == "los_binary":
            value = 1.0 if has_any_los else (0.0 if has_any_path else float("nan"))
        elif metric == "best_bs_index":
            value = float(best_bs_index) if best_bs_index >= 0 else float("nan")
        elif metric == "serving_bs_index":
            value = float(serving_idx) if serving_idx >= 0 else float("nan")
        elif metric == "sys_sinr_eff_db":
            value = float(candidate_sinr[serving_idx]) if 0 <= serving_idx < candidate_sinr.size and np.isfinite(candidate_sinr[serving_idx]) else float("nan")
        elif metric == "sys_spectral_efficiency_bpshz":
            value = float(candidate_se[serving_idx]) if 0 <= serving_idx < candidate_se.size and np.isfinite(candidate_se[serving_idx]) else float("nan")
        elif metric == "sys_mcs_index":
            value = float(candidate_mcs[serving_idx]) if 0 <= serving_idx < candidate_mcs.size and np.isfinite(candidate_mcs[serving_idx]) else float("nan")
        elif metric == "sys_tb_ok":
            value = float(candidate_tb_ok[serving_idx]) if 0 <= serving_idx < candidate_tb_ok.size and np.isfinite(candidate_tb_ok[serving_idx]) else float("nan")
        elif metric == "best_bs_rate_bpshz":
            value = best_rate
        elif metric == "beam_oracle_index":
            value = float(beam_oracle_index) if beam_oracle_index >= 0 else float("nan")
        elif metric == "beam_oracle_gain_db":
            value = beam_oracle_gain_db
        elif metric == "beam_deepsense_index":
            value = float(beam_deepsense_index) if beam_deepsense_index >= 0 else float("nan")
        elif metric == "beam_deepsense_gain_db":
            value = beam_deepsense_gain_db
        else:
            raise ValueError(f"unsupported CKM metric: {metric}")
        values[metric] = value
    return values


def extract_point_metrics(
    sim: OfflineSionnaSimulator,
    pos_xyz: np.ndarray,
    metric_names: List[str],
) -> Tuple[Dict[str, float], Dict[str, Any]]:
    vel_xyz = np.zeros(3, dtype=float)
    paths = sim.simulate_one_snapshot(pos_xyz, vel_xyz)
    sys_summary = sim._run_sys_over_rt(paths)

    best_power_db = float("nan")
    best_tau_std_ns = float("nan")
    best_bs_index = -1
    best_rate = float("nan")
    has_any_path = False
    has_any_los = False
    per_bs_power: List[float] = []

    for tx_idx, bs in enumerate(sim.cfg.bs_list):
        tx_position = np.asarray(bs["position"], dtype=float)
        all_paths, best = sim.extract_all_paths_for_link(paths, rx_idx=0, tx_idx=tx_idx, tx_position=tx_position, rx_position=pos_xyz)
        if all_paths:
            has_any_path = True
        if any(str(path.get("path_type", "")).upper() == "LOS" for path in all_paths):
            has_any_los = True

        power_db = safe_float(best.get("power_db") if best else float("nan"))
        tau_std_s = safe_float(best.get("tau_std_s") if best else float("nan"))
        tau_std_ns = tau_std_s * 1e9 if np.isfinite(tau_std_s) else float("nan")
        per_bs_power.append(power_db)

        if np.isfinite(power_db) and (not np.isfinite(best_power_db) or power_db > best_power_db):
            best_power_db = power_db
            best_tau_std_ns = tau_std_ns
            best_bs_index = tx_idx

    if sys_summary.get("enabled"):
        candidate_rates = np.asarray(sys_summary.get("candidate_rate_bpshz", []), dtype=float).reshape(-1)
        finite_rates = np.where(np.isfinite(candidate_rates), candidate_rates, np.nan)
        if np.isfinite(finite_rates).any():
            best_rate = float(np.nanmax(finite_rates))

    serving_idx = int(sys_summary.get("serving_idx", -1)) if sys_summary.get("enabled") else -1
    candidate_sinr = np.asarray(sys_summary.get("candidate_sinr_eff_db", []), dtype=float).reshape(-1)
    candidate_se = np.asarray(sys_summary.get("candidate_spectral_efficiency", []), dtype=float).reshape(-1)
    candidate_mcs = np.asarray(sys_summary.get("candidate_mcs_index", []), dtype=float).reshape(-1)
    candidate_tb_ok = np.asarray(sys_summary.get("candidate_tb_ok", []), dtype=float).reshape(-1)

    beam_summary: Dict[str, Any] = {"enabled": False}
    if any(metric in BEAM_METRICS for metric in metric_names) or sim.cfg.enable_beamforming:
        try:
            beam_summary = sim._run_beamforming_sweep(paths, preferred_serving_idx=serving_idx, rx_position=pos_xyz)
        except Exception as beam_error:
            beam_summary = {
                "enabled": False,
                "predictor_status": "runtime_error",
                "predictor_error": str(beam_error),
            }

    beam_oracle_index = int(beam_summary.get("oracle_beam_index", -1)) if beam_summary.get("enabled") else -1
    beam_oracle_gain_db = safe_float(beam_summary.get("oracle_beam_gain_db", np.nan)) if beam_summary.get("enabled") else float("nan")
    beam_deepsense_index = -1
    beam_deepsense_gain_db = float("nan")
    if beam_summary.get("enabled") and str(beam_summary.get("selection_mode", "")).strip().lower() == "deepsense_predictor":
        beam_deepsense_index = int(beam_summary.get("selected_beam_index", -1))
        beam_deepsense_gain_db = safe_float(beam_summary.get("selected_beam_gain_db", np.nan))

    summary = {
        "best_power_db": None if not np.isfinite(best_power_db) else float(best_power_db),
        "best_power_db_raw": best_power_db,
        "best_bs_index": best_bs_index,
        "serving_bs_index": serving_idx,
        "has_any_path": has_any_path,
        "has_any_los": has_any_los,
        "best_bs_rate_bpshz": None if not np.isfinite(best_rate) else float(best_rate),
        "best_bs_rate_bpshz_raw": best_rate,
        "per_bs_power_db": [None if not np.isfinite(v) else float(v) for v in per_bs_power],
        "best_tau_std_ns": None if not np.isfinite(best_tau_std_ns) else float(best_tau_std_ns),
        "best_tau_std_ns_raw": best_tau_std_ns,
        "candidate_sinr_eff_db_raw": candidate_sinr,
        "candidate_spectral_efficiency_raw": candidate_se,
        "candidate_mcs_index_raw": candidate_mcs,
        "candidate_tb_ok_raw": candidate_tb_ok,
        "beam_enabled": bool(beam_summary.get("enabled", False)),
        "beam_selection_mode": beam_summary.get("selection_mode", ""),
        "beam_reference_tx_idx": beam_summary.get("reference_tx_idx"),
        "beam_reference_bs_name": beam_summary.get("reference_bs_name"),
        "beam_reference_anchor_id": beam_summary.get("reference_anchor_id"),
        "beam_oracle_index": None if beam_oracle_index < 0 else int(beam_oracle_index),
        "beam_oracle_index_raw": beam_oracle_index,
        "beam_oracle_gain_db": None if not np.isfinite(beam_oracle_gain_db) else float(beam_oracle_gain_db),
        "beam_oracle_gain_db_raw": beam_oracle_gain_db,
        "beam_deepsense_index": None if beam_deepsense_index < 0 else int(beam_deepsense_index),
        "beam_deepsense_index_raw": beam_deepsense_index,
        "beam_deepsense_gain_db": None if not np.isfinite(beam_deepsense_gain_db) else float(beam_deepsense_gain_db),
        "beam_deepsense_gain_db_raw": beam_deepsense_gain_db,
        "beam_predictor_status": beam_summary.get("predictor_status", ""),
        "beam_predictor_error": beam_summary.get("predictor_error", ""),
        "beam_predictor_available": beam_summary.get("predictor_available", False),
        "beam_feature_mode": beam_summary.get("feature_mode", ""),
        "beam_top_k": beam_summary.get("top_k"),
        "beam_selection_source": beam_summary.get("selection_source", ""),
        "beam_topk_indices": beam_summary.get("topk_indices", []),
        "beam_topk_probabilities": beam_summary.get("topk_probabilities", []),
    }
    values = metric_values_from_summary(metric_names, summary)
    for key in [
        "best_power_db_raw",
        "best_tau_std_ns_raw",
        "best_bs_rate_bpshz_raw",
        "candidate_sinr_eff_db_raw",
        "candidate_spectral_efficiency_raw",
        "candidate_mcs_index_raw",
        "candidate_tb_ok_raw",
        "beam_oracle_index_raw",
        "beam_oracle_gain_db_raw",
        "beam_deepsense_index_raw",
        "beam_deepsense_gain_db_raw",
    ]:
        summary.pop(key, None)
    del paths
    return values, summary


def metric_style(metric: str, grid: np.ndarray, station_count: int) -> Dict[str, Any]:
    if plt is None or mcolors is None:
        raise RuntimeError("matplotlib is required to save CKM heatmaps and scene renders")

    finite = grid[np.isfinite(grid)]
    if metric in {"best_bs_index", "serving_bs_index"}:
        max_index = max(0, station_count - 1)
        ticks = list(range(max_index + 1)) if max_index > 0 else [0]
        cmap = plt.get_cmap("tab20", max(1, max_index + 1))
        norm = mcolors.BoundaryNorm(np.arange(-0.5, max_index + 1.5, 1.0), cmap.N)
        return {
            "cmap": cmap,
            "scene_cmap": cmap,
            "norm": norm,
            "vmin": -0.5,
            "vmax": max_index + 0.5,
            "ticks": ticks,
            "formatter": ticker.FormatStrFormatter("%.0f") if ticker is not None else None,
            "discrete": True,
            "db_scale": False,
        }
    if metric in {"beam_oracle_index", "beam_deepsense_index"}:
        max_index = int(np.nanmax(finite)) if finite.size else 0
        max_index = max(0, max_index)
        ticks = list(range(max_index + 1)) if max_index <= 16 else list(range(0, max_index + 1, max(1, int(np.ceil((max_index + 1) / 12.0)))))
        cmap = plt.get_cmap("turbo", max(2, max_index + 1))
        norm = mcolors.BoundaryNorm(np.arange(-0.5, max_index + 1.5, 1.0), cmap.N)
        return {
            "cmap": cmap,
            "scene_cmap": cmap,
            "norm": norm,
            "vmin": -0.5,
            "vmax": max_index + 0.5,
            "ticks": ticks,
            "formatter": ticker.FormatStrFormatter("%.0f") if ticker is not None else None,
            "discrete": True,
            "db_scale": False,
        }
    if metric in {"los_binary", "sys_tb_ok"}:
        cmap = plt.get_cmap("RdYlGn", 2)
        norm = mcolors.BoundaryNorm([-0.5, 0.5, 1.5], cmap.N)
        labels = {0.0: "0", 1.0: "1"}
        return {
            "cmap": cmap,
            "scene_cmap": cmap,
            "norm": norm,
            "vmin": -0.5,
            "vmax": 1.5,
            "ticks": [0, 1],
            "formatter": FixedFormatter(labels) if ticker is not None else None,
            "discrete": True,
            "db_scale": False,
        }
    if metric == "sys_mcs_index":
        max_index = int(np.nanmax(finite)) if finite.size else 0
        max_index = max(0, max_index)
        ticks = list(range(0, max_index + 1)) if max_index <= 12 else list(range(0, max_index + 1, 2))
        cmap = plt.get_cmap("turbo", max(2, max_index + 1))
        norm = mcolors.BoundaryNorm(np.arange(-0.5, max_index + 1.5, 1.0), cmap.N)
        return {
            "cmap": cmap,
            "scene_cmap": cmap,
            "norm": norm,
            "vmin": -0.5,
            "vmax": max_index + 0.5,
            "ticks": ticks,
            "formatter": ticker.FormatStrFormatter("%.0f") if ticker is not None else None,
            "discrete": True,
            "db_scale": False,
        }

    if finite.size:
        vmin = float(np.nanpercentile(finite, 2.0))
        vmax = float(np.nanpercentile(finite, 98.0))
        if not np.isfinite(vmin):
            vmin = float(np.nanmin(finite))
        if not np.isfinite(vmax):
            vmax = float(np.nanmax(finite))
    else:
        vmin, vmax = 0.0, 1.0
    if not np.isfinite(vmin) or not np.isfinite(vmax) or abs(vmax - vmin) < 1e-9:
        center = vmin if np.isfinite(vmin) else 0.0
        vmin = center - 1.0
        vmax = center + 1.0

    cmap_name = "viridis"
    if metric in {"power_db", "sys_sinr_eff_db", "best_bs_rate_bpshz", "sys_spectral_efficiency_bpshz", "beam_oracle_gain_db", "beam_deepsense_gain_db"}:
        cmap_name = "turbo"
    elif metric in {"path_loss_db", "tau_std_ns"}:
        cmap_name = "magma"
    cmap = plt.get_cmap(cmap_name)
    norm = mcolors.Normalize(vmin=vmin, vmax=vmax)
    fmt = ticker.FormatStrFormatter("%.0f") if ticker is not None and metric.endswith("_db") else ticker.FormatStrFormatter("%.2f") if ticker is not None else None
    return {
        "cmap": cmap,
        "scene_cmap": cmap,
        "norm": norm,
        "vmin": vmin,
        "vmax": vmax,
        "ticks": None,
        "formatter": fmt,
        "discrete": False,
        "db_scale": False,
    }


def save_heatmap(
    png_path: str,
    grid: np.ndarray,
    metric: str,
    extent: Tuple[float, float, float, float],
    style: Dict[str, Any],
) -> None:
    if plt is None:
        raise RuntimeError("matplotlib is required to save CKM heatmaps")

    fig, ax = plt.subplots(figsize=(10, 8))
    masked = np.ma.masked_invalid(grid)
    image = ax.imshow(
        masked,
        origin="lower",
        aspect="auto",
        extent=[extent[0], extent[1], extent[2], extent[3]],
        cmap=style["cmap"],
        norm=style["norm"],
        interpolation="nearest",
    )
    colorbar_kwargs: Dict[str, Any] = {"shrink": 0.9, "pad": 0.03}
    if style.get("ticks") is not None:
        colorbar_kwargs["ticks"] = style["ticks"]
    cbar = fig.colorbar(image, **colorbar_kwargs)
    if style.get("formatter") is not None:
        cbar.ax.yaxis.set_major_formatter(style["formatter"])
        cbar.update_ticks()
    cbar.set_label(METRIC_COLORBAR_LABELS.get(metric, metric))
    ax.set_title(f"Dense CKM: {METRIC_DISPLAY_NAMES.get(metric, metric)}")
    ax.set_xlabel("x (m)")
    ax.set_ylabel("y (m)")
    fig.tight_layout()
    fig.savefig(png_path, dpi=180, bbox_inches="tight")
    plt.close(fig)


def create_aligned_sampling_radio_map(
    scene: Any,
    x_min: float,
    x_max: float,
    y_min: float,
    y_max: float,
    z_fixed: float,
    resolution_m: float,
) -> Tuple[Any, np.ndarray, np.ndarray, np.ndarray, Tuple[float, float, float, float]]:
    import mitsuba as mi
    from sionna.rt import PlanarRadioMap

    width = max(float(x_max) - float(x_min), float(resolution_m))
    height = max(float(y_max) - float(y_min), float(resolution_m))
    cell_size_x = max(float(resolution_m), 1e-6)
    cell_size_y = max(float(resolution_m), 1e-6)

    radio_map = PlanarRadioMap(
        scene=scene,
        cell_size=mi.Point2f(cell_size_x, cell_size_y),
        center=mi.Point3f(float((x_min + x_max) * 0.5), float((y_min + y_max) * 0.5), float(z_fixed)),
        orientation=mi.Point3f(0.0, 0.0, 0.0),
        size=mi.Point2f(width, height),
    )

    centers = to_numpy(radio_map.cell_centers).astype(float)
    if centers.ndim != 3 or centers.shape[2] < 3:
        raise RuntimeError(f"Unexpected PlanarRadioMap.cell_centers shape: {centers.shape}")

    xs = centers[0, :, 0].astype(float)
    ys = centers[:, 0, 1].astype(float)
    extent = (
        float(xs[0] - 0.5 * cell_size_x),
        float(xs[-1] + 0.5 * cell_size_x),
        float(ys[0] - 0.5 * cell_size_y),
        float(ys[-1] + 0.5 * cell_size_y),
    )
    return radio_map, centers, xs, ys, extent


def custom_radio_map_from_grid(base_radio_map: Any, metric: str, grid: np.ndarray) -> Any:
    radio_map = base_radio_map
    if not hasattr(radio_map, "register_metric"):
        raise RuntimeError(
            "The installed sionna-rt package does not expose RadioMap.register_metric(). "
            "Install the custom_metrics-enabled sionna-rt build before generating official-style CKM renders."
        )

    expected_shape = tuple(int(v) for v in to_numpy(radio_map.path_gain).shape)
    tensor = np.repeat(np.asarray(grid, dtype=np.float32)[np.newaxis, :, :], int(radio_map.num_tx), axis=0)
    if tuple(int(v) for v in tensor.shape) != expected_shape:
        raise RuntimeError(
            f"Aligned PlanarRadioMap shape mismatch for metric '{metric}': expected {expected_shape}, got {tuple(int(v) for v in tensor.shape)}"
        )
    # Keep NaN as the no-coverage sentinel. This is sufficient for the dense CKM
    # grids produced by this script and avoids shape/broadcasting pitfalls when
    # a custom sionna-rt build reduces the metric across transmitters for display.
    radio_map.register_metric(
        metric,
        values=tensor,
        label=METRIC_RENDER_LABELS.get(metric, metric),
        unit=METRIC_UNITS.get(metric),
        db_scale=False,
        no_coverage_value=float("nan"),
    )
    return radio_map


def create_ckm_render_camera_params(
    xs: np.ndarray,
    ys: np.ndarray,
    z_fixed: float,
    range_source: str = "manual_range",
    bs_list: List[Dict[str, Any]] | None = None,
) -> Dict[str, Any]:
    center_x = float((xs[0] + xs[-1]) * 0.5)
    center_y = float((ys[0] + ys[-1]) * 0.5)
    span_x = max(float(xs[-1] - xs[0]), 1.0)
    span_y = max(float(ys[-1] - ys[0]), 1.0)
    span = max(span_x, span_y, 50.0)

    margin = 1.14 if str(range_source).strip() == "loaded_scene_bounds" else 1.08
    fov = 36.0
    half_span = 0.5 * span * margin

    target_z = float(z_fixed)
    if bs_list:
        try:
            target_z = max(target_z, max(float(bs["position"][2]) for bs in bs_list) * 0.2)
        except Exception:
            pass

    height = half_span / max(math.tan(math.radians(fov) * 0.5), 1e-6)
    height += max(20.0, 0.18 * span)

    origin = (center_x, center_y, target_z + height)
    target = (center_x, center_y, target_z)
    up = (0.0, 1.0, 0.0)
    return {
        "origin": origin,
        "target": target,
        "up": up,
        "fov": fov,
    }



RENDER_XY_TRANSFORM_MATRICES: Dict[str, np.ndarray] = {
    "identity": np.asarray([[1.0, 0.0], [0.0, 1.0]], dtype=np.float64),
    "flip_x": np.asarray([[-1.0, 0.0], [0.0, 1.0]], dtype=np.float64),
    "flip_y": np.asarray([[1.0, 0.0], [0.0, -1.0]], dtype=np.float64),
    "rot180": np.asarray([[-1.0, 0.0], [0.0, -1.0]], dtype=np.float64),
    "swap_xy": np.asarray([[0.0, 1.0], [1.0, 0.0]], dtype=np.float64),
    "rot90_cw": np.asarray([[0.0, 1.0], [-1.0, 0.0]], dtype=np.float64),
    "rot90_ccw": np.asarray([[0.0, -1.0], [1.0, 0.0]], dtype=np.float64),
    "swap_xy_rot180": np.asarray([[0.0, -1.0], [-1.0, 0.0]], dtype=np.float64),
}


def extent_center_size(extent: Tuple[float, float, float, float]) -> Tuple[np.ndarray, np.ndarray]:
    x_min, x_max, y_min, y_max = [float(v) for v in extent]
    center = np.asarray([(x_min + x_max) * 0.5, (y_min + y_max) * 0.5], dtype=np.float64)
    size = np.asarray([max(x_max - x_min, 1e-6), max(y_max - y_min, 1e-6)], dtype=np.float64)
    return center, size


def normalize_render_xy_transform_mode(mode: str) -> str:
    value = str(mode or "auto").strip().lower()
    if not value:
        return "auto"
    aliases = {
        "rot90": "rot90_cw",
        "swap": "swap_xy",
        "swapxy": "swap_xy",
        "transpose": "swap_xy",
    }
    value = aliases.get(value, value)
    if value == "auto" or value in RENDER_XY_TRANSFORM_MATRICES:
        return value
    return "auto"


def estimate_render_xy_transform(
    sim_extent: Tuple[float, float, float, float],
    render_extent: Tuple[float, float, float, float],
    mode: str = "auto",
) -> Dict[str, Any]:
    sim_center, _ = extent_center_size(sim_extent)
    render_center, render_size = extent_center_size(render_extent)
    sim_corners = np.asarray(
        [
            [sim_extent[0], sim_extent[2]],
            [sim_extent[1], sim_extent[2]],
            [sim_extent[1], sim_extent[3]],
            [sim_extent[0], sim_extent[3]],
        ],
        dtype=np.float64,
    )
    sim_centered = sim_corners - sim_center.reshape(1, 2)
    mode = normalize_render_xy_transform_mode(mode)
    candidate_names = [mode] if mode != "auto" else list(RENDER_XY_TRANSFORM_MATRICES.keys())

    best_name = "identity"
    best_matrix = RENDER_XY_TRANSFORM_MATRICES["identity"]
    best_score = float("inf")
    best_oriented = sim_centered.copy()
    for name in candidate_names:
        matrix = RENDER_XY_TRANSFORM_MATRICES.get(name)
        if matrix is None:
            continue
        oriented = sim_centered @ matrix.T
        cand_min = np.min(oriented, axis=0)
        cand_max = np.max(oriented, axis=0)
        cand_size = np.maximum(cand_max - cand_min, 1e-6)
        score = float(np.sum(((cand_size - render_size) / np.maximum(render_size, 1e-6)) ** 2))
        if score < best_score:
            best_score = score
            best_name = name
            best_matrix = matrix
            best_oriented = oriented

    oriented_min = np.min(best_oriented, axis=0)
    oriented_max = np.max(best_oriented, axis=0)
    oriented_size = np.maximum(oriented_max - oriented_min, 1e-6)
    scale = render_size / oriented_size
    return {
        "mode": best_name,
        "matrix": best_matrix.tolist(),
        "matrix_inv": best_matrix.T.tolist(),
        "sim_center": sim_center.tolist(),
        "render_center": render_center.tolist(),
        "scale": scale.tolist(),
        "sim_extent": [float(v) for v in sim_extent],
        "render_extent": [float(v) for v in render_extent],
    }


def apply_render_xy_transform(points_xy: np.ndarray, transform: Dict[str, Any]) -> np.ndarray:
    pts = np.asarray(points_xy, dtype=np.float64).reshape(-1, 2)
    sim_center = np.asarray(transform["sim_center"], dtype=np.float64).reshape(1, 2)
    render_center = np.asarray(transform["render_center"], dtype=np.float64).reshape(1, 2)
    matrix = np.asarray(transform["matrix"], dtype=np.float64).reshape(2, 2)
    scale = np.asarray(transform["scale"], dtype=np.float64).reshape(1, 2)
    oriented = (pts - sim_center) @ matrix.T
    return oriented * scale + render_center


def invert_render_xy_transform(points_xy: np.ndarray, transform: Dict[str, Any]) -> np.ndarray:
    pts = np.asarray(points_xy, dtype=np.float64).reshape(-1, 2)
    sim_center = np.asarray(transform["sim_center"], dtype=np.float64).reshape(1, 2)
    render_center = np.asarray(transform["render_center"], dtype=np.float64).reshape(1, 2)
    matrix_inv = np.asarray(transform.get("matrix_inv", np.asarray(transform["matrix"], dtype=np.float64).T), dtype=np.float64).reshape(2, 2)
    scale = np.asarray(transform["scale"], dtype=np.float64).reshape(1, 2)
    centered = (pts - render_center) / np.maximum(scale, 1e-12)
    return centered @ matrix_inv.T + sim_center


def sim_extent_corners(extent: Tuple[float, float, float, float]) -> np.ndarray:
    x_min, x_max, y_min, y_max = [float(v) for v in extent]
    return np.asarray([[x_min, y_max], [x_max, y_max], [x_max, y_min], [x_min, y_min]], dtype=np.float64)


def transformed_sim_quad_render_world(
    sim_extent: Tuple[float, float, float, float],
    render_xy_transform: Dict[str, Any],
) -> np.ndarray:
    return apply_render_xy_transform(sim_extent_corners(sim_extent), render_xy_transform)


def expand_extent(extent: Tuple[float, float, float, float], margin: float) -> Tuple[float, float, float, float]:
    center, size = extent_center_size(extent)
    half = 0.5 * size * max(float(margin), 1.0)
    return (
        float(center[0] - half[0]),
        float(center[0] + half[0]),
        float(center[1] - half[1]),
        float(center[1] + half[1]),
    )


def render_world_to_image(
    points_xy: np.ndarray,
    display_extent: Tuple[float, float, float, float],
    resolution: Tuple[int, int],
) -> np.ndarray:
    pts = np.asarray(points_xy, dtype=np.float64).reshape(-1, 2)
    x_min, x_max, y_min, y_max = [float(v) for v in display_extent]
    width_px = float(resolution[0])
    height_px = float(resolution[1])
    u = (pts[:, 0] - x_min) / max(x_max - x_min, 1e-12) * (width_px - 1.0)
    v = (y_max - pts[:, 1]) / max(y_max - y_min, 1e-12) * (height_px - 1.0)
    return np.stack([u, v], axis=-1)


def build_scene_click_mapping_from_quad(
    resolution: Tuple[int, int],
    sim_extent: Tuple[float, float, float, float],
    dst_quad_xy: np.ndarray,
    z_fixed: float,
) -> Dict[str, Any]:
    width_px = int(resolution[0])
    height_px = int(resolution[1])
    return {
        "method": "image_homography",
        "image_width": width_px,
        "image_height": height_px,
        "plane_z": float(z_fixed),
        "projection_plane_z": float(z_fixed),
        "x_min": float(sim_extent[0]),
        "x_max": float(sim_extent[1]),
        "y_min": float(sim_extent[2]),
        "y_max": float(sim_extent[3]),
        "world_corners_xy": [[float(v) for v in row] for row in sim_extent_corners(sim_extent).tolist()],
        "quad_image_xy": [[float(v) for v in row] for row in np.asarray(dst_quad_xy, dtype=np.float64).tolist()],
    }


def build_scene_pick_map_images_from_quad(
    resolution: Tuple[int, int],
    sim_extent: Tuple[float, float, float, float],
    dst_quad_xy: np.ndarray,
) -> Tuple[np.ndarray, np.ndarray]:
    width_px = int(resolution[0])
    height_px = int(resolution[1])
    src_quad = sim_extent_corners(sim_extent)
    quad = np.asarray(dst_quad_xy, dtype=np.float64).reshape(4, 2)
    homography = solve_homography(src_quad, quad)
    inv_h = np.linalg.inv(homography)

    x_codes = np.full((height_px, width_px), PICK_MAP_INVALID_CODE, dtype=np.uint16)
    y_codes = np.full((height_px, width_px), PICK_MAP_INVALID_CODE, dtype=np.uint16)

    min_x = max(0, int(math.floor(float(np.min(quad[:, 0])))))
    max_x = min(width_px - 1, int(math.ceil(float(np.max(quad[:, 0])))))
    min_y = max(0, int(math.floor(float(np.min(quad[:, 1])))))
    max_y = min(height_px - 1, int(math.ceil(float(np.max(quad[:, 1])))))
    if min_x >= max_x or min_y >= max_y:
        raise RuntimeError(f"scene pick-map quad lies outside the frame: quad={quad.tolist()}")

    xs = np.arange(min_x, max_x + 1, dtype=np.float64)
    ys = np.arange(min_y, max_y + 1, dtype=np.float64)
    grid_x, grid_y = np.meshgrid(xs, ys)
    homogeneous = np.stack([grid_x + 0.5, grid_y + 0.5, np.ones_like(grid_x)], axis=-1)
    world_proj = homogeneous @ inv_h.T
    denom = world_proj[..., 2]
    valid = np.abs(denom) > 1e-9
    world_x = np.where(valid, world_proj[..., 0] / denom, np.nan)
    world_y = np.where(valid, world_proj[..., 1] / denom, np.nan)

    x_min, x_max, y_min, y_max = [float(v) for v in sim_extent]
    valid &= points_in_convex_quad_mask(grid_x + 0.5, grid_y + 0.5, quad)
    valid &= world_x >= x_min - 1e-6
    valid &= world_x <= x_max + 1e-6
    valid &= world_y >= y_min - 1e-6
    valid &= world_y <= y_max + 1e-6

    if np.any(valid):
        x_codes_roi = encode_pick_map_codes(world_x, x_min, x_max)
        y_codes_roi = encode_pick_map_codes(world_y, y_min, y_max)
        roi_x = x_codes[min_y:max_y + 1, min_x:max_x + 1]
        roi_y = y_codes[min_y:max_y + 1, min_x:max_x + 1]
        roi_x[valid] = x_codes_roi[valid]
        roi_y[valid] = y_codes_roi[valid]

    return pack_pick_codes_to_rgb(x_codes), pack_pick_codes_to_rgb(y_codes)


def build_scene_pick_map_images_from_scene_overlay(
    scene_rgb: np.ndarray,
    grid: np.ndarray,
    xs: np.ndarray,
    ys: np.ndarray,
) -> Tuple[np.ndarray, np.ndarray, np.ndarray]:
    rgb = np.asarray(scene_rgb, dtype=np.uint8)
    if rgb.ndim != 3 or rgb.shape[2] < 3:
        raise RuntimeError(f"scene overlay image must be HxWx3/4, got {rgb.shape}")

    overlay_mask = filtered_scene_overlay_mask(rgb[..., :3])
    rows, cols = np.nonzero(overlay_mask)
    if rows.size <= 0 or cols.size <= 0:
        raise RuntimeError("scene overlay does not contain a detectable color region for pick-map generation")

    min_y = int(np.min(rows))
    max_y = int(np.max(rows))
    min_x = int(np.min(cols))
    max_x = int(np.max(cols))
    bbox_w = max_x - min_x + 1
    bbox_h = max_y - min_y + 1
    if bbox_w <= 1 or bbox_h <= 1:
        raise RuntimeError(f"scene overlay bbox is too small: {(min_x, min_y, max_x, max_y)}")

    grid_arr = np.asarray(grid, dtype=np.float64)
    xs_arr = np.asarray(xs, dtype=np.float64).reshape(-1)
    ys_arr = np.asarray(ys, dtype=np.float64).reshape(-1)
    if grid_arr.ndim != 2:
        raise RuntimeError(f"CKM grid must be 2D, got {grid_arr.shape}")
    if grid_arr.shape != (ys_arr.size, xs_arr.size):
        raise RuntimeError(
            f"CKM grid shape {grid_arr.shape} does not match xs/ys lengths {(xs_arr.size, ys_arr.size)}"
        )

    # Match the visual orientation of power_db.png / imshow(origin='lower').
    valid_img = np.isfinite(np.flipud(grid_arr))
    world_x_img = np.tile(xs_arr.reshape(1, -1), (ys_arr.size, 1))
    world_y_img = np.tile(ys_arr[::-1].reshape(-1, 1), (1, xs_arr.size))

    x_codes = np.full((rgb.shape[0], rgb.shape[1]), PICK_MAP_INVALID_CODE, dtype=np.uint16)
    y_codes = np.full((rgb.shape[0], rgb.shape[1]), PICK_MAP_INVALID_CODE, dtype=np.uint16)

    roi_mask = overlay_mask[min_y:max_y + 1, min_x:max_x + 1]
    px = np.arange(min_x, max_x + 1, dtype=np.float64)
    py = np.arange(min_y, max_y + 1, dtype=np.float64)
    grid_x, grid_y = np.meshgrid(px, py)

    u = (grid_x - float(min_x) + 0.5) / float(bbox_w)
    v = (grid_y - float(min_y) + 0.5) / float(bbox_h)
    ix = np.floor(u * float(xs_arr.size)).astype(np.int32)
    iy = np.floor(v * float(ys_arr.size)).astype(np.int32)
    ix = np.clip(ix, 0, xs_arr.size - 1)
    iy = np.clip(iy, 0, ys_arr.size - 1)

    valid = roi_mask & valid_img[iy, ix]
    if not np.any(valid):
        raise RuntimeError("scene overlay pick-map generation found no valid pixels after raster lookup")

    roi_x = x_codes[min_y:max_y + 1, min_x:max_x + 1]
    roi_y = y_codes[min_y:max_y + 1, min_x:max_x + 1]
    roi_x[valid] = encode_pick_map_codes(world_x_img[iy, ix], float(xs_arr[0]), float(xs_arr[-1]))[valid]
    roi_y[valid] = encode_pick_map_codes(world_y_img[iy, ix], float(ys_arr[0]), float(ys_arr[-1]))[valid]

    quad = np.asarray(
        [
            [float(min_x), float(min_y)],
            [float(max_x), float(min_y)],
            [float(max_x), float(max_y)],
            [float(min_x), float(max_y)],
        ],
        dtype=np.float64,
    )
    return pack_pick_codes_to_rgb(x_codes), pack_pick_codes_to_rgb(y_codes), quad


def extract_scene_bounds_xy(render_scene_path: str, mi_variant: str) -> Tuple[float, float, float, float, float, float]:
    import mitsuba as mi

    mi.set_variant(mi_variant)
    scene = mi.load_file(render_scene_path)
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
        bmin = np.asarray(bmin, dtype=np.float64).reshape(-1)
        bmax = np.asarray(bmax, dtype=np.float64).reshape(-1)
        if bmin.size >= 3 and bmax.size >= 3 and np.all(np.isfinite(bmin[:3])) and np.all(np.isfinite(bmax[:3])):
            return (float(bmin[0]), float(bmax[0]), float(bmin[1]), float(bmax[1]), float(bmin[2]), float(bmax[2]))
    raise RuntimeError(f"failed to read scene bounds from render XML: {render_scene_path}")


def write_fast_render_scene_xml_orthographic(
    base_scene_path: str,
    output_xml_path: str,
    display_extent: Tuple[float, float, float, float],
    origin_z: float,
    target_z: float,
    resolution: Tuple[int, int] = FAST_SCENE_RENDER_RESOLUTION,
    spp: int = FAST_SCENE_RENDER_SPP,
) -> None:
    with open(base_scene_path, "r", encoding="utf-8") as f:
        xml_text = f.read()

    insert_at = xml_text.rfind("</scene>")
    if insert_at < 0:
        raise RuntimeError(f"scene XML does not contain </scene>: {base_scene_path}")

    x_min, x_max, y_min, y_max = [float(v) for v in display_extent]
    center_x = (x_min + x_max) * 0.5
    center_y = (y_min + y_max) * 0.5
    half_w = max((x_max - x_min) * 0.5, 1e-6)
    half_h = max((y_max - y_min) * 0.5, 1e-6)
    width_px = int(resolution[0])
    height_px = int(resolution[1])

    overlay_xml = f'''
    <!-- Dense CKM fast orthographic render camera inserted automatically -->
    <sensor type="orthographic" id="dense_ckm_sensor">
        <float name="near_clip" value="1e-2"/>
        <float name="far_clip" value="100000"/>
        <transform name="to_world">
            <scale x="{half_w:.9f}" y="{half_h:.9f}"/>
            <lookat origin="{center_x:.9f}, {center_y:.9f}, {origin_z:.9f}" target="{center_x:.9f}, {center_y:.9f}, {target_z:.9f}" up="0.000000000, 1.000000000, 0.000000000"/>
        </transform>
        <sampler type="independent">
            <integer name="sample_count" value="{int(spp)}"/>
        </sampler>
        <film type="hdrfilm">
            <integer name="width" value="{width_px}"/>
            <integer name="height" value="{height_px}"/>
            <rfilter type="tent"/>
        </film>
    </sensor>
'''
    xml_with_overlay = xml_text[:insert_at] + overlay_xml + xml_text[insert_at:]
    with open(output_xml_path, "w", encoding="utf-8") as f:
        f.write(xml_with_overlay)


def render_scene_background_fast_topdown(
    render_scene_path: str,
    display_extent: Tuple[float, float, float, float],
    scene_z_bounds: Tuple[float, float],
    mi_variant: str,
    resolution: Tuple[int, int] = FAST_SCENE_RENDER_RESOLUTION,
    spp: int = FAST_SCENE_RENDER_SPP,
) -> np.ndarray:
    import mitsuba as mi

    mi.set_variant(mi_variant)
    z_min, z_max = [float(v) for v in scene_z_bounds]
    target_z = z_min
    origin_z = z_max + max(25.0, (z_max - z_min) * 0.8 + 10.0)
    temp_scene_path = create_temp_overlay_scene_path(render_scene_path, "background_ortho")
    try:
        write_fast_render_scene_xml_orthographic(
            render_scene_path,
            temp_scene_path,
            display_extent=display_extent,
            origin_z=origin_z,
            target_z=target_z,
            resolution=resolution,
            spp=spp,
        )
        scene = mi.load_file(temp_scene_path)
        sensor_count = 0
        try:
            sensor_count = len(scene.sensors())
        except Exception:
            sensor_count = 0
        sensor_arg = max(0, sensor_count - 1)
        image = mi.render(scene, sensor=sensor_arg, spp=int(spp))
        rgb = mitsuba_image_to_rgb_numpy(image)
        rgb = np.clip(rgb * 1.10 + 0.03, 0.0, 1.0)
        return rgb
    finally:
        try:
            os.remove(temp_scene_path)
        except Exception:
            pass

def build_scene_click_mapping(
    camera_params: Dict[str, Any],
    resolution: Tuple[int, int],
    extent: Tuple[float, float, float, float],
    z_fixed: float,
    projection_plane_z: float | None = None,
) -> Dict[str, Any]:
    width_px = int(resolution[0])
    height_px = int(resolution[1])
    x_min, x_max, y_min, y_max = [float(v) for v in extent]
    plane_z = float(z_fixed)
    projection_plane = float(projection_plane_z) if projection_plane_z is not None else plane_z
    world_corners_xy = [
        [x_min, y_max],
        [x_max, y_max],
        [x_max, y_min],
        [x_min, y_min],
    ]
    quad_image_xy = project_overlay_plane_quad(
        camera_params,
        (width_px, height_px),
        extent,
        projection_plane,
    )
    return {
        "method": "camera_ray_plane",
        "image_width": width_px,
        "image_height": height_px,
        "plane_z": plane_z,
        "projection_plane_z": projection_plane,
        "x_min": x_min,
        "x_max": x_max,
        "y_min": y_min,
        "y_max": y_max,
        "camera": {
            "origin": [float(v) for v in camera_params["origin"]],
            "target": [float(v) for v in camera_params["target"]],
            "up": [float(v) for v in camera_params.get("up", (0.0, 1.0, 0.0))],
            "fov": float(camera_params.get("fov", 36.0)),
        },
        "world_corners_xy": world_corners_xy,
        "quad_image_xy": [[float(v) for v in row] for row in quad_image_xy.tolist()],
    }


def project_overlay_plane_quad(
    camera_params: Dict[str, Any],
    resolution: Tuple[int, int],
    extent: Tuple[float, float, float, float],
    plane_z: float,
) -> np.ndarray:
    width_px = int(resolution[0])
    height_px = int(resolution[1])
    x_min, x_max, y_min, y_max = [float(v) for v in extent]
    overlay_corners_xyz = np.asarray(
        [
            [x_min, y_max, float(plane_z)],
            [x_max, y_max, float(plane_z)],
            [x_max, y_min, float(plane_z)],
            [x_min, y_min, float(plane_z)],
        ],
        dtype=np.float64,
    )
    return project_world_points_to_image(overlay_corners_xyz, camera_params, (width_px, height_px))


def encode_pick_map_codes(values: np.ndarray, v_min: float, v_max: float) -> np.ndarray:
    vals = np.asarray(values, dtype=np.float64)
    if not np.isfinite(v_min) or not np.isfinite(v_max) or abs(v_max - v_min) < 1e-12:
        return np.full(vals.shape, PICK_MAP_INVALID_CODE, dtype=np.uint16)
    norm = (vals - float(v_min)) / float(v_max - v_min)
    norm = np.clip(norm, 0.0, 1.0)
    return np.rint(norm * float(PICK_MAP_MAX_CODE)).astype(np.uint16)


def pack_pick_codes_to_rgb(code_map: np.ndarray) -> np.ndarray:
    codes = np.asarray(code_map, dtype=np.uint16)
    packed = np.zeros(codes.shape + (3,), dtype=np.uint8)
    packed[..., 0] = ((codes >> 8) & 0xFF).astype(np.uint8)
    packed[..., 1] = (codes & 0xFF).astype(np.uint8)
    packed[..., 2] = 0
    return packed


def save_pick_map_png(output_path: str, image_rgb: np.ndarray) -> None:
    if Image is None:
        raise RuntimeError("Pillow is required to save CKM scene pick maps")
    rgb = np.asarray(image_rgb, dtype=np.uint8)
    if rgb.ndim != 3 or rgb.shape[-1] != 3:
        raise RuntimeError(f"pick map image must be HxWx3 uint8, got {rgb.shape}")
    Image.fromarray(rgb, mode="RGB").save(output_path, format="PNG")


def build_scene_pick_map_images(
    resolution: Tuple[int, int],
    extent: Tuple[float, float, float, float],
    camera_params: Dict[str, Any],
    plane_z: float,
) -> Tuple[np.ndarray, np.ndarray, np.ndarray]:
    width_px = int(resolution[0])
    height_px = int(resolution[1])
    x_min, x_max, y_min, y_max = [float(v) for v in extent]
    world_quad_xy = np.asarray(
        [
            [x_min, y_max],
            [x_max, y_max],
            [x_max, y_min],
            [x_min, y_min],
        ],
        dtype=np.float64,
    )
    dst_quad_xy = project_overlay_plane_quad(camera_params, (width_px, height_px), extent, plane_z)
    homography = solve_homography(world_quad_xy, dst_quad_xy)
    inv_h = np.linalg.inv(homography)

    x_codes = np.full((height_px, width_px), PICK_MAP_INVALID_CODE, dtype=np.uint16)
    y_codes = np.full((height_px, width_px), PICK_MAP_INVALID_CODE, dtype=np.uint16)

    min_x = max(0, int(math.floor(float(np.min(dst_quad_xy[:, 0])))))
    max_x = min(width_px - 1, int(math.ceil(float(np.max(dst_quad_xy[:, 0])))))
    min_y = max(0, int(math.floor(float(np.min(dst_quad_xy[:, 1])))))
    max_y = min(height_px - 1, int(math.ceil(float(np.max(dst_quad_xy[:, 1])))))
    if min_x >= max_x or min_y >= max_y:
        raise RuntimeError(f"scene pick-map projection lies outside the frame: quad={dst_quad_xy.tolist()}")

    xs = np.arange(min_x, max_x + 1, dtype=np.float64)
    ys = np.arange(min_y, max_y + 1, dtype=np.float64)
    grid_x, grid_y = np.meshgrid(xs, ys)
    homogeneous = np.stack([grid_x + 0.5, grid_y + 0.5, np.ones_like(grid_x)], axis=-1)
    world_proj = homogeneous @ inv_h.T
    denom = world_proj[..., 2]
    valid = np.abs(denom) > 1e-9
    world_x = np.where(valid, world_proj[..., 0] / denom, np.nan)
    world_y = np.where(valid, world_proj[..., 1] / denom, np.nan)

    valid &= points_in_convex_quad_mask(grid_x + 0.5, grid_y + 0.5, dst_quad_xy)
    eps_x = max(1.0, (x_max - x_min) * 0.01)
    eps_y = max(1.0, (y_max - y_min) * 0.01)
    valid &= world_x >= x_min - eps_x
    valid &= world_x <= x_max + eps_x
    valid &= world_y >= y_min - eps_y
    valid &= world_y <= y_max + eps_y

    if np.any(valid):
        x_codes_roi = encode_pick_map_codes(world_x, x_min, x_max)
        y_codes_roi = encode_pick_map_codes(world_y, y_min, y_max)
        roi_x = x_codes[min_y:max_y + 1, min_x:max_x + 1]
        roi_y = y_codes[min_y:max_y + 1, min_x:max_x + 1]
        roi_x[valid] = x_codes_roi[valid]
        roi_y[valid] = y_codes_roi[valid]

    return pack_pick_codes_to_rgb(x_codes), pack_pick_codes_to_rgb(y_codes), dst_quad_xy


def build_metric_rgba_image(grid: np.ndarray, style: Dict[str, Any]) -> np.ndarray:
    """
    Convert a CKM grid into an RGBA image that can be perspective-warped onto
    the rendered background scene.

    The returned image is vertically flipped so that its image-space top row
    corresponds to the largest y value, matching the usual camera raster
    coordinate system used by the perspective projection below.
    """
    masked = np.ma.masked_invalid(grid)
    safe_values = masked.filled(float(style.get("vmin", 0.0)))
    rgba = np.asarray(style["cmap"](style["norm"](safe_values)), dtype=np.float32)
    rgba[..., :3] = rgba[..., :3] * 0.84 + 0.16
    mask = np.ma.getmaskarray(masked)
    rgba[mask, 0:3] = 1.0
    rgba[..., 3] = np.where(mask, 0.0, 0.94).astype(np.float32)
    return np.flipud(rgba)


def xml_escape(text: str) -> str:
    return (str(text).replace("&", "&amp;").replace('\"', '&quot;').replace("<", "&lt;").replace(">", "&gt;"))


def write_fast_render_scene_xml(
    base_scene_path: str,
    output_xml_path: str,
    camera_params: Dict[str, Any],
    resolution: Tuple[int, int] = FAST_SCENE_RENDER_RESOLUTION,
    spp: int = FAST_SCENE_RENDER_SPP,
) -> None:
    with open(base_scene_path, "r", encoding="utf-8") as f:
        xml_text = f.read()

    insert_at = xml_text.rfind("</scene>")
    if insert_at < 0:
        raise RuntimeError(f"scene XML does not contain </scene>: {base_scene_path}")

    ox, oy, oz = camera_params["origin"]
    tx, ty, tz = camera_params["target"]
    ux, uy, uz = camera_params["up"]
    fov = float(camera_params.get("fov", 38.0))
    width_px = int(resolution[0])
    height_px = int(resolution[1])

    overlay_xml = f'''
    <!-- Dense CKM fast render camera inserted automatically -->
    <sensor type="perspective" id="dense_ckm_sensor">
        <float name="fov" value="{fov:.6f}"/>
        <string name="fov_axis" value="x"/>
        <transform name="to_world">
            <lookat origin="{ox:.9f}, {oy:.9f}, {oz:.9f}" target="{tx:.9f}, {ty:.9f}, {tz:.9f}" up="{ux:.9f}, {uy:.9f}, {uz:.9f}"/>
        </transform>
        <sampler type="independent">
            <integer name="sample_count" value="{int(spp)}"/>
        </sampler>
        <film type="hdrfilm">
            <integer name="width" value="{width_px}"/>
            <integer name="height" value="{height_px}"/>
            <rfilter type="tent"/>
        </film>
    </sensor>
'''
    xml_with_overlay = xml_text[:insert_at] + overlay_xml + xml_text[insert_at:]
    with open(output_xml_path, "w", encoding="utf-8") as f:
        f.write(xml_with_overlay)


def create_temp_overlay_scene_path(base_scene_path: str, metric_base: str) -> str:
    base_dir = os.path.dirname(os.path.abspath(base_scene_path))
    base_name = os.path.splitext(os.path.basename(base_scene_path))[0]
    fd, temp_path = tempfile.mkstemp(
        prefix=f".{base_name}_{metric_base}_ckm_overlay_",
        suffix=".xml",
        dir=base_dir,
    )
    os.close(fd)
    return temp_path


def convert_render_array_to_display_rgb(image_array: np.ndarray) -> np.ndarray:
    arr = np.asarray(image_array)
    if arr.ndim == 4 and arr.shape[0] == 1:
        arr = arr[0]
    if arr.ndim == 3 and arr.shape[0] in {1, 3, 4} and arr.shape[-1] not in {1, 3, 4}:
        arr = np.moveaxis(arr, 0, -1)
    if arr.ndim == 2:
        arr = np.repeat(arr[..., np.newaxis], 3, axis=-1)
    if arr.ndim != 3:
        raise RuntimeError(f"unexpected Mitsuba render tensor shape: {arr.shape}")
    if arr.shape[-1] == 1:
        arr = np.repeat(arr, 3, axis=-1)
    if arr.shape[-1] > 3:
        arr = arr[..., :3]

    if np.issubdtype(arr.dtype, np.integer):
        info = np.iinfo(arr.dtype)
        scale = float(max(info.max, 1))
        arr = arr.astype(np.float32) / scale
    else:
        arr = arr.astype(np.float32, copy=False)

    arr = np.where(np.isfinite(arr), arr, 0.0)
    arr = np.clip(arr, 0.0, None)
    finite = arr[np.isfinite(arr)]
    if finite.size:
        p995 = float(np.nanpercentile(finite, 99.5))
        if np.isfinite(p995) and p995 > 1.0:
            arr = arr / p995
    arr = np.clip(arr, 0.0, 1.0)
    arr = np.power(arr, 1.0 / 2.2)
    return arr.astype(np.float32, copy=False)


def mitsuba_image_to_rgb_numpy(image: Any) -> np.ndarray:
    import mitsuba as mi

    direct_candidates: List[Any] = []
    try:
        direct_candidates.append(np.asarray(image))
    except Exception:
        pass
    try:
        direct_candidates.append(np.asarray(np.array(image)))
    except Exception:
        pass

    convert_to_bitmap = getattr(getattr(mi, "util", None), "convert_to_bitmap", None)
    if callable(convert_to_bitmap):
        try:
            direct_candidates.append(np.asarray(convert_to_bitmap(image)))
        except Exception:
            pass

    try:
        bitmap = mi.Bitmap(image)
        direct_candidates.append(np.asarray(bitmap))
        bitmap_pixel_format = getattr(mi.Bitmap, "PixelFormat", None)
        struct_type = getattr(getattr(mi, "Struct", None), "Type", None)
        if bitmap_pixel_format is not None and struct_type is not None:
            try:
                direct_candidates.append(
                    np.asarray(bitmap.convert(mi.Bitmap.PixelFormat.RGB, mi.Struct.Type.UInt8, True))
                )
            except Exception:
                pass
            try:
                direct_candidates.append(
                    np.asarray(bitmap.convert(mi.Bitmap.PixelFormat.RGB, mi.Struct.Type.Float32, False))
                )
            except Exception:
                pass
    except Exception:
        pass

    last_error: Exception | None = None
    for candidate in direct_candidates:
        try:
            return convert_render_array_to_display_rgb(candidate)
        except Exception as exc:
            last_error = exc
            continue

    if last_error is not None:
        raise RuntimeError(f"failed to convert Mitsuba render output to RGB numpy array: {last_error}")
    raise RuntimeError("failed to convert Mitsuba render output to RGB numpy array")


def render_scene_background_fast(
    render_scene_path: str,
    camera_params: Dict[str, Any],
    mi_variant: str,
    resolution: Tuple[int, int] = FAST_SCENE_RENDER_RESOLUTION,
    spp: int = FAST_SCENE_RENDER_SPP,
) -> np.ndarray:
    import mitsuba as mi

    mi.set_variant(mi_variant)
    temp_scene_path = create_temp_overlay_scene_path(render_scene_path, "background")
    try:
        write_fast_render_scene_xml(
            render_scene_path,
            temp_scene_path,
            camera_params,
            resolution=resolution,
            spp=spp,
        )
        scene = mi.load_file(temp_scene_path)
        sensor_count = 0
        try:
            sensor_count = len(scene.sensors())
        except Exception:
            sensor_count = 0
        sensor_arg = max(0, sensor_count - 1)
        image = mi.render(scene, sensor=sensor_arg, spp=int(spp))
        rgb = mitsuba_image_to_rgb_numpy(image)
        rgb = np.clip(rgb * 1.10 + 0.02, 0.0, 1.0)
        return rgb
    finally:
        try:
            os.remove(temp_scene_path)
        except Exception:
            pass


def scene_render_title(metric: str) -> str:
    return f"Dense CKM: {METRIC_DISPLAY_NAMES.get(metric, metric)}"


def attach_display_content_rect(metadata: Dict[str, Any], content_rect: Dict[str, int] | None) -> Dict[str, Any]:
    if not metadata:
        return {}
    updated = dict(metadata)
    if content_rect:
        updated["display_content_rect"] = {
            "x": int(content_rect.get("x", 0)),
            "y": int(content_rect.get("y", 0)),
            "width": int(content_rect.get("width", 0)),
            "height": int(content_rect.get("height", 0)),
        }
    return updated


def save_scene_overlay_render(
    output_png_path: str,
    scene_rgb: np.ndarray,
    metric: str,
    style: Dict[str, Any],
) -> Dict[str, Any]:
    if matplotlib is None or plt is None or mcm is None:
        raise RuntimeError("matplotlib is required to save CKM scene renders")

    rgb = np.clip(np.asarray(scene_rgb, dtype=np.float32), 0.0, 1.0)
    if rgb.ndim != 3 or rgb.shape[2] < 3:
        raise RuntimeError(f"scene render image must be HxWx3/4, got {rgb.shape}")
    rgb = rgb[..., :3]

    raw_h, raw_w = rgb.shape[:2]
    if raw_w <= 0 or raw_h <= 0:
        raise RuntimeError(f"scene render image has invalid size: {rgb.shape}")

    # Keep the mapped content rectangle logic unchanged, but reserve enough
    # canvas around the scene so the title, ticks, and colorbar label never get
    # clipped at the figure edges.
    scene_scale = 1.12
    scene_w_px = max(1, int(round(raw_w * scene_scale)))
    scene_h_px = max(1, int(round(raw_h * scene_scale)))

    left_pad_px = max(22, int(round(raw_w * 0.025)))
    right_pad_px = max(90, int(round(raw_w * 0.070)))
    bottom_pad_px = max(22, int(round(raw_h * 0.025)))
    top_pad_px = max(68, int(round(raw_h * 0.085)))
    title_band_px = max(44, int(round(raw_h * 0.065)))

    cbar_gap_px = max(18, int(round(raw_w * 0.018)))
    cbar_w_px = max(28, int(round(raw_w * 0.026)))
    cbar_tick_margin_px = max(84, int(round(raw_w * 0.080)))
    cbar_label_margin_px = max(90, int(round(raw_w * 0.090)))

    out_w = (
        left_pad_px
        + scene_w_px
        + cbar_gap_px
        + cbar_w_px
        + cbar_tick_margin_px
        + cbar_label_margin_px
        + right_pad_px
    )
    out_h = top_pad_px + title_band_px + scene_h_px + bottom_pad_px
    dpi = 180

    fig = plt.figure(figsize=(out_w / dpi, out_h / dpi), dpi=dpi, facecolor="white")
    try:
        scene_left = left_pad_px / out_w
        scene_bottom = bottom_pad_px / out_h
        scene_width = scene_w_px / out_w
        scene_height = scene_h_px / out_h

        cbar_left = (left_pad_px + scene_w_px + cbar_gap_px) / out_w
        cbar_bottom = (bottom_pad_px + scene_h_px * 0.14) / out_h
        cbar_width = cbar_w_px / out_w
        cbar_height = (scene_h_px * 0.72) / out_h

        ax = fig.add_axes([scene_left, scene_bottom, scene_width, scene_height])
        ax.imshow(rgb, interpolation="nearest")
        ax.set_axis_off()

        scalar_mappable = mcm.ScalarMappable(norm=style["norm"], cmap=style.get("scene_cmap") or style.get("cmap"))
        scalar_mappable.set_array([])

        cax = fig.add_axes([cbar_left, cbar_bottom, cbar_width, cbar_height])
        colorbar_kwargs: Dict[str, Any] = {}
        if style.get("ticks") is not None:
            colorbar_kwargs["ticks"] = style["ticks"]
        cbar = fig.colorbar(scalar_mappable, cax=cax, **colorbar_kwargs)
        if style.get("formatter") is not None:
            cbar.ax.yaxis.set_major_formatter(style["formatter"])
            cbar.update_ticks()
        cbar.ax.tick_params(labelsize=11, pad=5)
        cbar.set_label(
            METRIC_COLORBAR_LABELS.get(metric, METRIC_DISPLAY_NAMES.get(metric, metric)),
            fontsize=12,
            labelpad=16,
        )

        title_y = 1.0 - ((top_pad_px + title_band_px * 0.50) / out_h)
        fig.text(0.5, title_y, scene_render_title(metric), ha="center", va="center", fontsize=18)
        fig.savefig(output_png_path, dpi=dpi, bbox_inches=None, pad_inches=0)
    finally:
        plt.close(fig)

    return {
        "x": int(left_pad_px),
        "y": int(top_pad_px + title_band_px),
        "width": int(scene_w_px),
        "height": int(scene_h_px),
    }


def normalize_vector(vec: np.ndarray) -> np.ndarray:
    norm = float(np.linalg.norm(vec))
    if norm < 1e-12:
        raise RuntimeError(f"cannot normalize near-zero vector: {vec}")
    return vec / norm


def project_world_points_to_image(
    points_xyz: np.ndarray,
    camera_params: Dict[str, Any],
    resolution: Tuple[int, int],
) -> np.ndarray:
    points = np.asarray(points_xyz, dtype=np.float64).reshape(-1, 3)
    origin = np.asarray(camera_params["origin"], dtype=np.float64)
    target = np.asarray(camera_params["target"], dtype=np.float64)
    up_hint = np.asarray(camera_params.get("up", (0.0, 0.0, 1.0)), dtype=np.float64)
    forward = normalize_vector(target - origin)
    right = normalize_vector(np.cross(forward, up_hint))
    up = normalize_vector(np.cross(right, forward))

    rel = points - origin.reshape(1, 3)
    x_cam = rel @ right
    y_cam = rel @ up
    z_cam = rel @ forward
    if np.any(z_cam <= 1e-6):
        raise RuntimeError("camera projection failed because at least one overlay corner is behind the camera")

    width_px = float(resolution[0])
    height_px = float(resolution[1])
    fov_rad = math.radians(float(camera_params.get("fov", 38.0)))
    focal_px = width_px / (2.0 * math.tan(max(fov_rad, 1e-6) * 0.5))
    principal_x = (width_px - 1.0) * 0.5
    principal_y = (height_px - 1.0) * 0.5

    u = principal_x + focal_px * (x_cam / z_cam)
    v = principal_y - focal_px * (y_cam / z_cam)
    return np.stack([u, v], axis=-1)


def solve_homography(src_xy: np.ndarray, dst_xy: np.ndarray) -> np.ndarray:
    src = np.asarray(src_xy, dtype=np.float64).reshape(-1, 2)
    dst = np.asarray(dst_xy, dtype=np.float64).reshape(-1, 2)
    if src.shape != (4, 2) or dst.shape != (4, 2):
        raise RuntimeError(f"homography expects four source and four destination points, got {src.shape} and {dst.shape}")

    a_rows: List[List[float]] = []
    b_rows: List[float] = []
    for (sx, sy), (dx, dy) in zip(src, dst):
        a_rows.append([sx, sy, 1.0, 0.0, 0.0, 0.0, -dx * sx, -dx * sy])
        b_rows.append(dx)
        a_rows.append([0.0, 0.0, 0.0, sx, sy, 1.0, -dy * sx, -dy * sy])
        b_rows.append(dy)
    a = np.asarray(a_rows, dtype=np.float64)
    b = np.asarray(b_rows, dtype=np.float64)
    try:
        params = np.linalg.solve(a, b)
    except np.linalg.LinAlgError:
        params, *_ = np.linalg.lstsq(a, b, rcond=None)
    h = np.asarray([params[0], params[1], params[2], params[3], params[4], params[5], params[6], params[7], 1.0], dtype=np.float64).reshape(3, 3)
    return h


def points_in_convex_quad_mask(grid_x: np.ndarray, grid_y: np.ndarray, quad_xy: np.ndarray) -> np.ndarray:
    quad = np.asarray(quad_xy, dtype=np.float64).reshape(4, 2)
    pts_x = np.asarray(grid_x, dtype=np.float64)
    pts_y = np.asarray(grid_y, dtype=np.float64)
    cross_terms = []
    for idx in range(4):
        x1, y1 = quad[idx]
        x2, y2 = quad[(idx + 1) % 4]
        cross = (x2 - x1) * (pts_y - y1) - (y2 - y1) * (pts_x - x1)
        cross_terms.append(cross)
    cross_stack = np.stack(cross_terms, axis=-1)
    return np.logical_or(np.all(cross_stack >= -1e-6, axis=-1), np.all(cross_stack <= 1e-6, axis=-1))


def bilinear_sample_rgba(image_rgba: np.ndarray, sample_x: np.ndarray, sample_y: np.ndarray) -> np.ndarray:
    h, w = image_rgba.shape[:2]
    x0 = np.floor(sample_x).astype(np.int32)
    y0 = np.floor(sample_y).astype(np.int32)
    x1 = np.clip(x0 + 1, 0, w - 1)
    y1 = np.clip(y0 + 1, 0, h - 1)
    x0 = np.clip(x0, 0, w - 1)
    y0 = np.clip(y0, 0, h - 1)

    wx = (sample_x - x0).astype(np.float32)[..., np.newaxis]
    wy = (sample_y - y0).astype(np.float32)[..., np.newaxis]

    top_left = image_rgba[y0, x0]
    top_right = image_rgba[y0, x1]
    bottom_left = image_rgba[y1, x0]
    bottom_right = image_rgba[y1, x1]

    top = top_left * (1.0 - wx) + top_right * wx
    bottom = bottom_left * (1.0 - wx) + bottom_right * wx
    return top * (1.0 - wy) + bottom * wy


def alpha_composite_rgb(base_rgb: np.ndarray, overlay_rgba: np.ndarray) -> np.ndarray:
    base = np.asarray(base_rgb, dtype=np.float32)
    overlay = np.asarray(overlay_rgba, dtype=np.float32)
    if base.shape[:2] != overlay.shape[:2]:
        raise RuntimeError(f"alpha composite shape mismatch: {base.shape} vs {overlay.shape}")
    alpha = np.clip(overlay[..., 3:4], 0.0, 1.0)
    out = base * (1.0 - alpha) + overlay[..., :3] * alpha
    return np.clip(out, 0.0, 1.0)


def compose_projective_overlay_on_background(
    background_rgb: np.ndarray,
    overlay_rgba: np.ndarray,
    dst_quad_xy: np.ndarray,
) -> np.ndarray:
    background = np.asarray(background_rgb, dtype=np.float32)
    src = np.asarray(overlay_rgba, dtype=np.float32)
    out_h, out_w = background.shape[:2]
    if out_h <= 0 or out_w <= 0:
        raise RuntimeError(f"invalid background render size: {background.shape}")

    quad = np.asarray(dst_quad_xy, dtype=np.float64).reshape(4, 2)
    min_x = max(0, int(math.floor(float(np.min(quad[:, 0])))))
    max_x = min(out_w - 1, int(math.ceil(float(np.max(quad[:, 0])))))
    min_y = max(0, int(math.floor(float(np.min(quad[:, 1])))))
    max_y = min(out_h - 1, int(math.ceil(float(np.max(quad[:, 1])))))
    if min_x >= max_x or min_y >= max_y:
        raise RuntimeError(f"projected CKM overlay lies outside the render frame: quad={quad}")

    src_h, src_w = src.shape[:2]
    src_quad = np.asarray(
        [[0.0, 0.0], [float(src_w - 1), 0.0], [float(src_w - 1), float(src_h - 1)], [0.0, float(src_h - 1)]],
        dtype=np.float64,
    )
    homography = solve_homography(src_quad, quad)
    inv_h = np.linalg.inv(homography)

    xs = np.arange(min_x, max_x + 1, dtype=np.float64)
    ys = np.arange(min_y, max_y + 1, dtype=np.float64)
    grid_x, grid_y = np.meshgrid(xs, ys)
    homogeneous = np.stack([grid_x, grid_y, np.ones_like(grid_x)], axis=-1)
    src_proj = homogeneous @ inv_h.T
    denom = src_proj[..., 2]
    valid = np.abs(denom) > 1e-9
    src_x = np.where(valid, src_proj[..., 0] / denom, -1.0)
    src_y = np.where(valid, src_proj[..., 1] / denom, -1.0)
    valid &= points_in_convex_quad_mask(grid_x + 0.5, grid_y + 0.5, quad)
    valid &= src_x >= 0.0
    valid &= src_x <= float(src_w - 1)
    valid &= src_y >= 0.0
    valid &= src_y <= float(src_h - 1)

    sampled = bilinear_sample_rgba(src, src_x, src_y)
    sampled[~valid] = 0.0

    overlay_full = np.zeros((out_h, out_w, 4), dtype=np.float32)
    overlay_full[min_y:max_y + 1, min_x:max_x + 1, :] = sampled
    return alpha_composite_rgb(background, overlay_full)


def render_projective_scene_overlay(
    background_rgb: np.ndarray,
    grid: np.ndarray,
    style: Dict[str, Any],
    camera_params: Dict[str, Any],
    extent: Tuple[float, float, float, float],
    z_fixed: float,
) -> np.ndarray:
    overlay_rgba = build_metric_rgba_image(grid, style)
    plane_z = float(z_fixed) + 0.05
    dst_quad_xy = project_overlay_plane_quad(
        camera_params,
        (int(background_rgb.shape[1]), int(background_rgb.shape[0])),
        extent,
        plane_z,
    )
    return compose_projective_overlay_on_background(background_rgb, overlay_rgba, dst_quad_xy)


def render_official_scene_rgb(
    scene: Any,
    bs_list: List[Dict[str, Any]],
    base_radio_map: Any,
    metric: str,
    grid: np.ndarray,
    xs: np.ndarray,
    ys: np.ndarray,
    z_fixed: float,
    style: Dict[str, Any],
    range_source: str = "manual_range",
) -> np.ndarray:
    if plt is None:
        raise RuntimeError("matplotlib is required to render official-style CKM scene images")

    from sionna.rt import Camera

    radio_map = custom_radio_map_from_grid(base_radio_map, metric, grid)
    camera_params = create_ckm_render_camera_params(xs, ys, z_fixed, range_source=range_source, bs_list=bs_list)
    camera = Camera(position=list(camera_params["origin"]))
    camera.look_at(camera_params["target"])

    figure = scene.render(
        camera=camera,
        radio_map=radio_map,
        rm_metric=metric,
        rm_db_scale=style.get("db_scale", False),
        rm_cmap=style.get("scene_cmap"),
        rm_vmin=style.get("vmin"),
        rm_vmax=style.get("vmax"),
        rm_show_color_bar=False,
        show_devices=True,
        show_orientations=False,
        num_samples=96,
        resolution=FAST_SCENE_RENDER_RESOLUTION,
    )
    if not hasattr(figure, "savefig"):
        raise RuntimeError("scene.render() did not return a Matplotlib figure")

    dpi = 200
    figure.set_size_inches(FAST_SCENE_RENDER_RESOLUTION[0] / dpi, FAST_SCENE_RENDER_RESOLUTION[1] / dpi, forward=True)
    try:
        figure.canvas.draw()
        rgba = np.asarray(figure.canvas.buffer_rgba(), dtype=np.uint8)
        if rgba.ndim != 3 or rgba.shape[2] < 4:
            raise RuntimeError(f"unexpected official scene canvas shape: {rgba.shape}")
        return rgba[..., :3].astype(np.float32) / 255.0
    finally:
        plt.close(figure)


def main() -> int:

    parser = build_arg_parser()
    args = parser.parse_args()

    metrics = normalize_metrics(args.metric, args.default_metric or "power_db")
    sys_metrics_requested = [metric for metric in metrics if metric in SYS_METRICS]
    if sys_metrics_requested and not args.enable_sys_integration:
        parser.error(f"metrics {metric_ui_list(sys_metrics_requested)} require --enable-sys-integration true")
    beam_metrics_requested = [metric for metric in metrics if metric in BEAM_METRICS]
    predicted_beam_metrics_requested = [metric for metric in metrics if metric in PREDICTED_BEAM_METRICS]
    if beam_metrics_requested and not args.enable_beamforming:
        parser.error(f"metrics {metric_ui_list(beam_metrics_requested)} require --enable-beamforming true")
    if predicted_beam_metrics_requested and not str(args.beam_model_checkpoint_path).strip():
        parser.error(f"metrics {metric_ui_list(predicted_beam_metrics_requested)} require --beam-model-checkpoint-path")

    run_id, output_root_dir = create_unique_output_root(args.output_dir, args.output_root_dir)
    print(f"OUTPUT_ROOT={os.path.abspath(output_root_dir)}", flush=True)
    print(f"[ckm] versioned output root: {output_root_dir}", flush=True)
    bs_list = load_bs_list_from_json(args.bs_list_json)
    if not bs_list:
        parser.error("base station list is empty")

    cfg = SimulationConfig(
        scene_path=args.scene_path,
        fc_hz=args.fc_hz,
        mi_variant=args.mi_variant,
        tx_array_num_rows=args.tx_array_num_rows,
        tx_array_num_cols=args.tx_array_num_cols,
        tx_array_vertical_spacing=args.tx_array_vertical_spacing,
        tx_array_horizontal_spacing=args.tx_array_horizontal_spacing,
        tx_array_pattern=args.tx_array_pattern,
        tx_array_polarization=args.tx_array_polarization,
        rx_array_num_rows=args.rx_array_num_rows,
        rx_array_num_cols=args.rx_array_num_cols,
        rx_array_vertical_spacing=args.rx_array_vertical_spacing,
        rx_array_horizontal_spacing=args.rx_array_horizontal_spacing,
        rx_array_pattern=args.rx_array_pattern,
        rx_array_polarization=args.rx_array_polarization,
        max_depth=args.max_depth,
        samples_per_src=args.samples_per_src,
        max_num_paths_per_src=args.max_num_paths_per_src,
        synthetic_array=args.synthetic_array,
        merge_shapes=args.merge_shapes,
        enable_sys_integration=args.enable_sys_integration,
        enable_beamforming=args.enable_beamforming,
        beam_selection_mode=("deepsense_predictor" if predicted_beam_metrics_requested else args.beam_selection_mode),
        beam_codebook_type=args.beam_codebook_type,
        beam_codebook_num_beams=args.beam_codebook_num_beams,
        beam_oversampling_v=args.beam_oversampling_v,
        beam_oversampling_h=args.beam_oversampling_h,
        beam_manual_index=args.beam_manual_index,
        beam_normalize_power=args.beam_normalize_power,
        beam_codebook_file=args.beam_codebook_file,
        beam_model_checkpoint_path=args.beam_model_checkpoint_path,
        beam_feature_mode=args.beam_feature_mode,
        beam_top_k=args.beam_top_k,
        sys_num_subcarriers=args.sys_num_subcarriers,
        sys_subcarrier_spacing_hz=args.sys_subcarrier_spacing_hz,
        sys_num_ofdm_symbols=args.sys_num_ofdm_symbols,
        sys_temperature_k=args.sys_temperature_k,
        sys_bler_target=args.sys_bler_target,
        sys_mcs_table_index=args.sys_mcs_table_index,
        sys_bs_tx_power_dbm=args.sys_bs_tx_power_dbm,
        los=args.los,
        specular_reflection=args.specular_reflection,
        diffuse_reflection=args.diffuse_reflection,
        refraction=args.refraction,
        diffraction=args.diffraction,
        edge_diffraction=args.edge_diffraction,
        diffraction_lit_region=args.diffraction_lit_region,
        enable_preview_render=False,
        bs_list=bs_list,
    )

    render_scene_path = args.render_scene_path.strip() or args.scene_path
    same_render_scene = os.path.abspath(render_scene_path) == os.path.abspath(args.scene_path)
    use_fast_dual_xml_overlay = bool(args.render_scene_overlay) and not same_render_scene

    print("[ckm] initializing Sionna RT/SYS backend", flush=True)
    print(f"[ckm] simulation scene XML: {os.path.abspath(args.scene_path)}", flush=True)
    print(f"[ckm] render scene XML: {os.path.abspath(render_scene_path)}", flush=True)
    if use_fast_dual_xml_overlay:
        print("[ckm] high-detail scene will use the original render XML as the Mitsuba base scene", flush=True)
    sim = OfflineSionnaSimulator(cfg)

    # Keep the measurement receiver at the requested CKM height. This helps the
    # official-style render place the measurement plane exactly where the user
    # expects it, while the actual RT/SYS sampling still uses explicit points.
    try:
        sim.scene.get("uav").position = [0.0, 0.0, float(args.z_fixed)]
    except Exception:
        pass

    sampling_scene = sim.scene
    sampling_radio_map, cell_centers, xs, ys, heatmap_extent = create_aligned_sampling_radio_map(
        sampling_scene,
        float(args.x_min),
        float(args.x_max),
        float(args.y_min),
        float(args.y_max),
        float(args.z_fixed),
        float(args.resolution_m),
    )
    grids: Dict[str, np.ndarray] = {metric: np.full((ys.size, xs.size), np.nan, dtype=float) for metric in metrics}
    per_metric_summaries: Dict[str, List[Dict[str, Any]]] = {metric: [] for metric in metrics}

    total = int(xs.size * ys.size)
    counter = 0
    started_at = time.time()
    for yi in range(int(cell_centers.shape[0])):
        for xi in range(int(cell_centers.shape[1])):
            pos = np.asarray(cell_centers[yi, xi, :3], dtype=float)
            x = float(pos[0])
            y = float(pos[1])
            values, summary = extract_point_metrics(sim, pos, metrics)
            for metric, value in values.items():
                grids[metric][yi, xi] = value
                metric_summary = dict(summary)
                metric_summary["x_m"] = float(x)
                metric_summary["y_m"] = float(y)
                metric_summary["metric"] = metric
                metric_summary["metric_value"] = None if not np.isfinite(value) else float(value)
                per_metric_summaries[metric].append(metric_summary)
            counter += 1
        elapsed = time.time() - started_at
        print(f"[ckm] progress {counter}/{total} elapsed={elapsed:.2f}s", flush=True)

    camera_params_for_scene_overlay: Dict[str, Any] | None = None
    prepared_background_rgb: np.ndarray | None = None
    prepared_background_mode = ""
    fallback_background_rgb: np.ndarray | None = None
    fallback_background_mode = ""
    common_scene_pick_map: Dict[str, Any] = {}
    common_scene_click_mapping: Dict[str, Any] = {}
    fast_overlay_dst_quad: np.ndarray | None = None
    render_xy_transform: Dict[str, Any] = {}
    render_display_extent: Tuple[float, float, float, float] | None = None
    render_scene_extent_xy: Tuple[float, float, float, float] | None = None
    if args.render_scene_overlay:
        camera_params_for_scene_overlay = create_ckm_render_camera_params(
            xs,
            ys,
            float(args.z_fixed),
            range_source=str(args.range_source).strip() or "manual_range",
            bs_list=bs_list,
        )
        if use_fast_dual_xml_overlay:
            try:
                bounds = extract_scene_bounds_xy(render_scene_path, cfg.mi_variant)
                render_scene_extent_xy = (bounds[0], bounds[1], bounds[2], bounds[3])
                render_xy_transform = estimate_render_xy_transform(
                    heatmap_extent,
                    render_scene_extent_xy,
                    mode=str(args.render_xy_transform).strip() or "auto",
                )
                render_display_extent = expand_extent(render_scene_extent_xy, ORTHO_SCENE_RENDER_MARGIN)
                prepared_background_rgb = render_scene_background_fast_topdown(
                    render_scene_path,
                    render_display_extent,
                    scene_z_bounds=(bounds[4], bounds[5]),
                    mi_variant=cfg.mi_variant,
                    resolution=FAST_SCENE_RENDER_RESOLUTION,
                    spp=FAST_SCENE_RENDER_SPP,
                )
                transformed_quad = transformed_sim_quad_render_world(heatmap_extent, render_xy_transform)
                fast_overlay_dst_quad = render_world_to_image(
                    transformed_quad,
                    render_display_extent,
                    FAST_SCENE_RENDER_RESOLUTION,
                )
                prepared_background_mode = "fast_topdown_orthographic_overlay_on_original_xml"
                print(
                    f"[ckm] prepared top-down orthographic background from original XML: {os.path.abspath(render_scene_path)}",
                    flush=True,
                )
                print(
                    f"[ckm] render XY transform: {render_xy_transform.get('mode', 'identity')} render_extent={render_scene_extent_xy}",
                    flush=True,
                )
                scene_pick_x_rgb, scene_pick_y_rgb = build_scene_pick_map_images_from_quad(
                    FAST_SCENE_RENDER_RESOLUTION,
                    heatmap_extent,
                    fast_overlay_dst_quad,
                )
                scene_pick_x_path = os.path.join(output_root_dir, "scene_pick_x.png")
                scene_pick_y_path = os.path.join(output_root_dir, "scene_pick_y.png")
                save_pick_map_png(scene_pick_x_path, scene_pick_x_rgb)
                save_pick_map_png(scene_pick_y_path, scene_pick_y_rgb)
                common_scene_pick_map = {
                    "encoding": "png_rgb_u16_hi_lo_split",
                    "image_width": int(FAST_SCENE_RENDER_RESOLUTION[0]),
                    "image_height": int(FAST_SCENE_RENDER_RESOLUTION[1]),
                    "x_min": float(heatmap_extent[0]),
                    "x_max": float(heatmap_extent[1]),
                    "y_min": float(heatmap_extent[2]),
                    "y_max": float(heatmap_extent[3]),
                    "x_image_path": os.path.abspath(scene_pick_x_path),
                    "y_image_path": os.path.abspath(scene_pick_y_path),
                    "quad_image_xy": [[float(v) for v in row] for row in fast_overlay_dst_quad.tolist()],
                    "render_xy_transform": render_xy_transform,
                }
                common_scene_click_mapping = build_scene_click_mapping_from_quad(
                    FAST_SCENE_RENDER_RESOLUTION,
                    heatmap_extent,
                    fast_overlay_dst_quad,
                    float(args.z_fixed),
                )
                print(f"[ckm] wrote scene pick maps: {scene_pick_x_path} , {scene_pick_y_path}", flush=True)
            except Exception as background_error:
                print(
                    "[ckm] fast original-XML top-down render failed; "
                    f"falling back to official/custom-metric render or simulation-scene fallback: {background_error}",
                    flush=True,
                )
        if not common_scene_pick_map:
            try:
                pick_plane_z = float(args.z_fixed) + 0.05
                scene_pick_x_rgb, scene_pick_y_rgb, scene_pick_quad = build_scene_pick_map_images(
                    FAST_SCENE_RENDER_RESOLUTION,
                    heatmap_extent,
                    camera_params_for_scene_overlay,
                    pick_plane_z,
                )
                scene_pick_x_path = os.path.join(output_root_dir, "scene_pick_x.png")
                scene_pick_y_path = os.path.join(output_root_dir, "scene_pick_y.png")
                save_pick_map_png(scene_pick_x_path, scene_pick_x_rgb)
                save_pick_map_png(scene_pick_y_path, scene_pick_y_rgb)
                common_scene_pick_map = {
                    "encoding": "png_rgb_u16_hi_lo_split",
                    "image_width": int(FAST_SCENE_RENDER_RESOLUTION[0]),
                    "image_height": int(FAST_SCENE_RENDER_RESOLUTION[1]),
                    "x_min": float(heatmap_extent[0]),
                    "x_max": float(heatmap_extent[1]),
                    "y_min": float(heatmap_extent[2]),
                    "y_max": float(heatmap_extent[3]),
                    "x_image_path": os.path.abspath(scene_pick_x_path),
                    "y_image_path": os.path.abspath(scene_pick_y_path),
                    "quad_image_xy": [[float(v) for v in row] for row in scene_pick_quad.tolist()],
                }
                common_scene_click_mapping = build_scene_click_mapping_from_quad(
                    FAST_SCENE_RENDER_RESOLUTION,
                    heatmap_extent,
                    scene_pick_quad,
                    float(args.z_fixed),
                )
                print(f"[ckm] wrote legacy scene pick maps: {scene_pick_x_path} , {scene_pick_y_path}", flush=True)
            except Exception as pick_map_error:
                print(f"[ckm] scene pick-map generation failed: {pick_map_error}", flush=True)

    default_metric = args.default_metric.strip() if args.default_metric.strip() in metrics else metrics[0]
    metric_entries: List[Dict[str, Any]] = []
    default_entry: Dict[str, Any] | None = None
    official_scene_image_transform_mode = "identity"
    official_scene_image_transform_detected = False

    for metric in metrics:
        metric_base = metric.strip() or "ckm_metric"
        metric_dir = os.path.join(output_root_dir, metric_base)
        os.makedirs(metric_dir, exist_ok=True)
        npz_path = os.path.join(metric_dir, f"{metric_base}.npz")
        png_path = os.path.join(metric_dir, f"{metric_base}.png")
        summary_json_path = os.path.join(metric_dir, f"{metric_base}_samples.json")
        scene_png_path = os.path.join(metric_dir, f"{metric_base}_scene.png")

        np.savez_compressed(npz_path, metric_grid=grids[metric], xs=xs, ys=ys)
        style = metric_style(metric, grids[metric], len(bs_list))
        save_heatmap(png_path, grids[metric], metric, heatmap_extent, style)
        with open(summary_json_path, "w", encoding="utf-8") as f:
            json.dump(per_metric_summaries[metric], f, indent=2, ensure_ascii=False)

        scene_image_path = ""
        scene_render_mode = ""
        scene_click_mapping: Dict[str, Any] = {}
        scene_display_content_rect: Dict[str, int] | None = None
        official_scene_rgb_for_pickmap: np.ndarray | None = None
        if args.render_scene_overlay:
            try:
                rendered_scene = False
                if prepared_background_rgb is not None:
                    if fast_overlay_dst_quad is not None:
                        composite_rgb = compose_projective_overlay_on_background(
                            prepared_background_rgb,
                            build_metric_rgba_image(grids[metric], style),
                            fast_overlay_dst_quad,
                        )
                        scene_display_content_rect = save_scene_overlay_render(scene_png_path, composite_rgb, metric, style)
                        scene_render_mode = prepared_background_mode
                        scene_click_mapping = dict(common_scene_click_mapping)
                        rendered_scene = True
                        print(f"[ckm] wrote fast orthographic scene overlay render: {scene_png_path}", flush=True)
                    elif camera_params_for_scene_overlay is not None:
                        composite_rgb = render_projective_scene_overlay(
                            prepared_background_rgb,
                            grids[metric],
                            style,
                            camera_params_for_scene_overlay,
                            heatmap_extent,
                            float(args.z_fixed),
                        )
                        scene_display_content_rect = save_scene_overlay_render(scene_png_path, composite_rgb, metric, style)
                        scene_render_mode = prepared_background_mode
                        scene_click_mapping = build_scene_click_mapping(
                            camera_params_for_scene_overlay,
                            (int(composite_rgb.shape[1]), int(composite_rgb.shape[0])),
                            heatmap_extent,
                            float(args.z_fixed),
                            projection_plane_z=float(args.z_fixed) + 0.05,
                        )
                        rendered_scene = True
                        print(f"[ckm] wrote fast scene overlay render: {scene_png_path}", flush=True)

                if not rendered_scene:
                    try:
                        official_scene_rgb = render_official_scene_rgb(
                            sim.scene,
                            bs_list,
                            sampling_radio_map,
                            metric,
                            grids[metric],
                            xs,
                            ys,
                            float(args.z_fixed),
                            style,
                            range_source=str(args.range_source).strip() or "manual_range",
                        )
                        scene_render_mode = "official_scene_render_with_custom_metrics"
                        if common_scene_click_mapping:
                            scene_click_mapping = dict(common_scene_click_mapping)
                        else:
                            scene_click_mapping = build_scene_click_mapping(
                                camera_params_for_scene_overlay,
                                FAST_SCENE_RENDER_RESOLUTION,
                                heatmap_extent,
                                float(args.z_fixed),
                                projection_plane_z=float(args.z_fixed) + 0.05,
                            ) if camera_params_for_scene_overlay is not None else {}

                        if not official_scene_image_transform_detected:
                            try:
                                reference_mask = reference_overlay_mask_from_metric_png(png_path)
                                scene_rgb_for_detection = np.asarray(official_scene_rgb * 255.0, dtype=np.uint8)
                                detected_mode, detected_score = detect_scene_image_transform_from_masks(reference_mask, scene_rgb_for_detection)
                                official_scene_image_transform_mode = detected_mode
                                official_scene_image_transform_detected = True
                                print(
                                    f"[ckm] official scene image transform detected: mode={detected_mode} score={detected_score:.3f}",
                                    flush=True,
                                )
                                if detected_mode != "identity":
                                    if common_scene_pick_map:
                                        rewrite_png_with_transform(common_scene_pick_map.get("x_image_path", ""), detected_mode)
                                        rewrite_png_with_transform(common_scene_pick_map.get("y_image_path", ""), detected_mode)
                                        common_scene_pick_map, common_scene_click_mapping = update_pickmap_and_click_mapping_for_image_transform(
                                            common_scene_pick_map,
                                            common_scene_click_mapping,
                                            detected_mode,
                                        )
                                    official_scene_rgb = apply_discrete_image_transform(official_scene_rgb, detected_mode)
                                    if common_scene_click_mapping:
                                        scene_click_mapping = dict(common_scene_click_mapping)
                            except Exception as official_scene_transform_error:
                                official_scene_image_transform_detected = True
                                official_scene_image_transform_mode = "identity"
                                print(
                                    f"[ckm] official scene image transform detection failed: {official_scene_transform_error}",
                                    flush=True,
                                )
                        elif official_scene_image_transform_mode != "identity":
                            official_scene_rgb = apply_discrete_image_transform(official_scene_rgb, official_scene_image_transform_mode)
                            if common_scene_click_mapping:
                                scene_click_mapping = dict(common_scene_click_mapping)

                        official_scene_rgb_for_pickmap = np.array(official_scene_rgb, copy=True)
                        scene_display_content_rect = save_scene_overlay_render(scene_png_path, official_scene_rgb, metric, style)
                        rendered_scene = True
                        print(f"[ckm] wrote official scene render: {scene_png_path}", flush=True)
                    except Exception as official_render_error:
                        print(f"[ckm] official scene.render() failed for {metric}: {official_render_error}", flush=True)
                        if camera_params_for_scene_overlay is None:
                            raise
                        if fallback_background_rgb is None:
                            fallback_background_rgb = render_scene_background_fast(
                                args.scene_path,
                                camera_params_for_scene_overlay,
                                cfg.mi_variant,
                                resolution=FAST_SCENE_RENDER_RESOLUTION,
                                spp=FAST_SCENE_RENDER_SPP,
                            )
                            fallback_background_mode = "fast_projective_overlay_render_on_simulation_xml"
                            print(
                                f"[ckm] prepared fallback background from simulation XML: {os.path.abspath(args.scene_path)}",
                                flush=True,
                            )
                        composite_rgb = render_projective_scene_overlay(
                            fallback_background_rgb,
                            grids[metric],
                            style,
                            camera_params_for_scene_overlay,
                            heatmap_extent,
                            float(args.z_fixed),
                        )
                        scene_display_content_rect = save_scene_overlay_render(scene_png_path, composite_rgb, metric, style)
                        scene_render_mode = fallback_background_mode
                        scene_click_mapping = build_scene_click_mapping(
                            camera_params_for_scene_overlay,
                            (int(composite_rgb.shape[1]), int(composite_rgb.shape[0])),
                            heatmap_extent,
                            float(args.z_fixed),
                            projection_plane_z=float(args.z_fixed) + 0.05,
                        )
                        rendered_scene = True
                        print(f"[ckm] wrote fallback projective scene overlay render: {scene_png_path}", flush=True)

                if rendered_scene:
                    scene_image_path = os.path.abspath(scene_png_path)
            except Exception as render_error:
                print(f"[ckm] scene render failed for {metric}: {render_error}", flush=True)

        metric_scene_pick_map = attach_display_content_rect(dict(common_scene_pick_map), scene_display_content_rect) if (scene_image_path and common_scene_pick_map) else {}
        metric_scene_click_mapping = attach_display_content_rect(dict(scene_click_mapping), scene_display_content_rect) if scene_image_path else {}
        if scene_image_path and scene_render_mode == "official_scene_render_with_custom_metrics" and official_scene_rgb_for_pickmap is not None:
            try:
                scene_rgb_final = np.asarray(np.clip(official_scene_rgb_for_pickmap, 0.0, 1.0) * 255.0, dtype=np.uint8)
                metric_pick_x_rgb, metric_pick_y_rgb, metric_pick_quad = build_scene_pick_map_images_from_scene_overlay(
                    scene_rgb_final,
                    grids[metric],
                    xs,
                    ys,
                )
                metric_pick_x_path = os.path.join(metric_dir, f"{metric}_scene_pick_x.png")
                metric_pick_y_path = os.path.join(metric_dir, f"{metric}_scene_pick_y.png")
                save_pick_map_png(metric_pick_x_path, metric_pick_x_rgb)
                save_pick_map_png(metric_pick_y_path, metric_pick_y_rgb)
                metric_scene_pick_map = attach_display_content_rect({
                    "encoding": "png_rgb_u16_hi_lo_split",
                    "image_width": int(scene_rgb_final.shape[1]),
                    "image_height": int(scene_rgb_final.shape[0]),
                    "x_min": float(heatmap_extent[0]),
                    "x_max": float(heatmap_extent[1]),
                    "y_min": float(heatmap_extent[2]),
                    "y_max": float(heatmap_extent[3]),
                    "x_image_path": os.path.abspath(metric_pick_x_path),
                    "y_image_path": os.path.abspath(metric_pick_y_path),
                    "quad_image_xy": [[float(v) for v in row] for row in metric_pick_quad.tolist()],
                    "method": "scene_overlay_mask_bbox_grid_lookup",
                }, scene_display_content_rect)
                metric_scene_click_mapping = attach_display_content_rect(build_scene_click_mapping_from_quad(
                    (int(scene_rgb_final.shape[1]), int(scene_rgb_final.shape[0])),
                    heatmap_extent,
                    metric_pick_quad,
                    float(args.z_fixed),
                ), scene_display_content_rect)
                print(
                    f"[ckm] wrote official scene pick maps from final overlay mask: {metric_pick_x_path} , {metric_pick_y_path}",
                    flush=True,
                )
            except Exception as official_pick_error:
                print(f"[ckm] official scene pick-map generation failed for {metric}: {official_pick_error}", flush=True)

        entry = {
            "metric": metric,
            "display_name": METRIC_DISPLAY_NAMES.get(metric, metric),
            "metric_dir": os.path.abspath(metric_dir),
            "image_path": os.path.abspath(png_path),
            "scene_image_path": scene_image_path,
            "scene_render_mode": scene_render_mode if scene_image_path else "",
            "scene_click_mapping": metric_scene_click_mapping if scene_image_path else {},
            "scene_pick_map": metric_scene_pick_map if scene_image_path else {},
            "render_xy_transform": render_xy_transform if scene_image_path and render_xy_transform else {},
            "npz_path": os.path.abspath(npz_path),
            "samples_json_path": os.path.abspath(summary_json_path),
            "x_min": float(heatmap_extent[0]),
            "x_max": float(heatmap_extent[1]),
            "y_min": float(heatmap_extent[2]),
            "y_max": float(heatmap_extent[3]),
            "z_fixed": float(args.z_fixed),
            "grid_width": int(xs.size),
            "grid_height": int(ys.size),
            "resolution_m": float(args.resolution_m),
        }
        metric_entries.append(entry)
        if metric == default_metric:
            default_entry = entry

        print(f"[ckm] wrote npz: {npz_path}", flush=True)
        print(f"[ckm] wrote png: {png_path}", flush=True)
        print(f"[ckm] wrote samples: {summary_json_path}", flush=True)

    if default_entry is None:
        default_entry = metric_entries[0]

    meta_path = os.path.join(output_root_dir, "meta.json")
    meta = {
        "metric": default_metric,
        "default_metric": default_metric,
        "display_name": METRIC_DISPLAY_NAMES.get(default_metric, default_metric),
        "run_id": run_id,
        "selected_metrics": metrics,
        "metrics": metric_entries,
        "output_base_dir": os.path.abspath(args.output_dir),
        "output_root_dir": os.path.abspath(output_root_dir),
        "scene_path": os.path.abspath(args.scene_path),
        "render_scene_path": os.path.abspath(render_scene_path),
        "image_path": default_entry["image_path"],
        "scene_image_path": default_entry.get("scene_image_path", ""),
        "scene_render_mode": default_entry.get("scene_render_mode", ""),
        "scene_click_mapping": default_entry.get("scene_click_mapping", {}),
        "scene_pick_map": default_entry.get("scene_pick_map", {}),
        "render_xy_transform": default_entry.get("render_xy_transform", {}),
        "npz_path": default_entry["npz_path"],
        "samples_json_path": default_entry["samples_json_path"],
        "x_min": float(heatmap_extent[0]),
        "x_max": float(heatmap_extent[1]),
        "y_min": float(heatmap_extent[2]),
        "y_max": float(heatmap_extent[3]),
        "requested_x_min": float(args.x_min),
        "requested_x_max": float(args.x_max),
        "requested_y_min": float(args.y_min),
        "requested_y_max": float(args.y_max),
        "z_fixed": float(args.z_fixed),
        "resolution_m": float(args.resolution_m),
        "grid_width": int(xs.size),
        "grid_height": int(ys.size),
        "range_source": str(args.range_source).strip() or "manual_range",
        "station_names": [str(bs.get("name", f"bs_{idx}")) for idx, bs in enumerate(bs_list)],
        "station_ids": [int(bs.get("id", idx)) if str(bs.get("id", idx)).isdigit() else idx for idx, bs in enumerate(bs_list)],
        "sys_enabled": bool(args.enable_sys_integration),
        "render_scene_overlay": bool(args.render_scene_overlay),
        "render_scene_overlay_mode": default_entry.get("scene_render_mode", "") if args.render_scene_overlay else "",
        "simulation_config": asdict(cfg),
        "generated_at_unix_s": float(time.time()),
    }
    with open(meta_path, "w", encoding="utf-8") as f:
        json.dump(meta, f, indent=2, ensure_ascii=False)

    print(f"META_JSON={os.path.abspath(meta_path)}", flush=True)
    print(f"[ckm] wrote meta: {meta_path}", flush=True)
    return 0


if __name__ == "__main__":
    sys.exit(main())
