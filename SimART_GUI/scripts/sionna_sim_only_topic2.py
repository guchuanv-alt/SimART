#!/usr/bin/env python3
# -*- coding: utf-8 -*-
# Online version: receive the UAV's true position and velocity from odometry,
# run only Sionna RT simulation, and do not perform position estimation.
# Final version:
# 1) Publish rf_msgs/RfObservationArray
# 2) Publish tau / doppler / AoA
# 3) Compute and publish tau_std_s / doppler_std_hz / aoa_az_std_rad / aoa_el_std_rad
# 4) Wait indefinitely for odometry until manually stopped
# 5) Disable CSV output by passing an empty measurement_csv string

from __future__ import annotations

import json
import math
import os
import threading
import time
import traceback
from dataclasses import dataclass, field
from typing import Any, Dict, List, Optional, Tuple

import numpy as np
import pandas as pd

from deepsense_beam_selector import (
    DEFAULT_FEATURE_MODE,
    DEEPSENSE_POS_BBOX4_MODE,
    DeepSenseBeamPredictor,
)
from sionna_beam_topic_utils import (
    build_beam_codebook_msg,
    build_beam_observation_array_msg,
    build_codebook_payload,
    effective_dft_oversampling_for_num_beams,
    load_codebook_payload_from_file,
    normalize_codebook_num_beams,
    select_codebook_num_beams,
)

C = 299792458.0

# path_type encoding convention. Update this if your project already has a fixed convention.
PATH_TYPE_UNKNOWN = 0
PATH_TYPE_LOS = 1
PATH_TYPE_NLOS = 2
PATH_TYPE_NO_PATH = 3


# =========================================================
# 0) Common utilities
# =========================================================
def to_numpy(x):
    if isinstance(x, np.ndarray):
        return x
    if hasattr(x, "numpy"):
        return np.array(x.numpy())
    return np.array(x)


def ensure_dir_for_file(path: str) -> None:
    folder = os.path.dirname(os.path.abspath(path))
    if folder:
        os.makedirs(folder, exist_ok=True)


def append_rows_csv(df: pd.DataFrame, csv_path: str, label: str = "csv") -> None:
    if df is None or len(df) == 0 or not csv_path:
        return
    ensure_dir_for_file(csv_path)
    file_exists = os.path.exists(csv_path) and os.path.getsize(csv_path) > 0
    if not file_exists:
        df.to_csv(csv_path, index=False)
        print(f"[output] {label} appended: {csv_path} (+rows={len(df)})")
        return
    try:
        existing_columns = pd.read_csv(csv_path, nrows=0).columns.tolist()
    except Exception:
        existing_columns = []
    new_columns = df.columns.tolist()
    if existing_columns == new_columns:
        df.to_csv(csv_path, mode="a", header=False, index=False)
        print(f"[output] {label} appended: {csv_path} (+rows={len(df)})")
        return
    existing_df = pd.read_csv(csv_path)
    ordered_columns = list(existing_df.columns.tolist())
    for column in new_columns:
        if column not in ordered_columns:
            ordered_columns.append(column)
    existing_df = existing_df.reindex(columns=ordered_columns)
    df = df.reindex(columns=ordered_columns)
    combined_df = pd.concat([existing_df, df], ignore_index=True)
    combined_df.to_csv(csv_path, index=False)
    print(f"[output] {label} schema-aligned rewrite: {csv_path} (+rows={len(df)})")


def reset_output_file(path: str) -> None:
    if not path:
        return
    ensure_dir_for_file(path)
    if os.path.exists(path):
        os.remove(path)


def load_bs_list_from_json(json_path: str):
    with open(json_path, "r", encoding="utf-8") as f:
        raw = json.load(f)
    bs_list = raw.get("base_stations", raw) if isinstance(raw, dict) else raw
    if not isinstance(bs_list, list):
        raise ValueError(f"base-station JSON must contain a list or 'base_stations' array: {json_path}")
    normalized: List[Dict[str, Any]] = []
    for idx, item in enumerate(bs_list):
        if not isinstance(item, dict):
            continue
        station = dict(item)
        position = np.asarray(station.get("position", [0.0, 0.0, 0.0]), dtype=float).reshape(-1)
        pos_xyz = np.zeros(3, dtype=float)
        count = min(int(position.size), 3)
        if count > 0:
            pos_xyz[:count] = position[:count]
        station["position"] = [float(v) for v in pos_xyz.tolist()]
        station["name"] = str(station.get("name", station.get("id", f"bs_{idx}")))
        station["id"] = str(station.get("id", f"bs_{idx}"))
        try:
            station["preview_offset_z"] = float(station.get("preview_offset_z", 0.0) or 0.0)
        except Exception:
            station["preview_offset_z"] = 0.0
        station["preview_camera_name"] = str(station.get("preview_camera_name", "") or "").strip()
        normalized.append(station)
    return normalized


def candidate_airsim_settings_paths() -> List[str]:
    candidates: List[str] = []
    env_path = os.environ.get("AIRSIM_SETTINGS_FILE", "").strip()
    if env_path:
        candidates.append(os.path.abspath(os.path.expanduser(env_path)))
    docs_candidate = os.path.abspath(os.path.expanduser("~/Documents/AirSim/settings.json"))
    if docs_candidate not in candidates:
        candidates.append(docs_candidate)
    return candidates


def resolve_airsim_settings_path() -> str:
    for candidate in candidate_airsim_settings_paths():
        if candidate and os.path.exists(candidate):
            return candidate
    return ""


def normalize_vector(vec_xyz: Any, fallback: Optional[np.ndarray] = None) -> np.ndarray:
    vec = np.asarray(vec_xyz, dtype=float).reshape(-1)
    out = np.zeros(3, dtype=float)
    count = min(int(vec.size), 3)
    if count > 0:
        out[:count] = vec[:count]
    norm = float(np.linalg.norm(out))
    if not np.isfinite(norm) or norm < 1e-12:
        if fallback is None:
            return np.array([1.0, 0.0, 0.0], dtype=float)
        return normalize_vector(fallback, fallback=None)
    return out / norm


def _parse_capture_settings(camera_obj: Dict[str, Any]) -> Dict[str, Any]:
    width = 1920
    height = 1080
    fov_deg = 90.0
    capture_items = camera_obj.get("CaptureSettings", [])
    if isinstance(capture_items, list):
        preferred = None
        for item in capture_items:
            if isinstance(item, dict) and int(item.get("ImageType", 0)) == 0:
                preferred = item
                break
        if preferred is None:
            for item in capture_items:
                if isinstance(item, dict):
                    preferred = item
                    break
        if isinstance(preferred, dict):
            width = int(preferred.get("Width", width) or width)
            height = int(preferred.get("Height", height) or height)
            fov_deg = float(preferred.get("FOV_Degrees", preferred.get("FOV", fov_deg)) or fov_deg)
    return {
        "width": max(int(width), 2),
        "height": max(int(height), 2),
        "fov_deg": float(fov_deg) if np.isfinite(float(fov_deg)) else 90.0,
    }


def load_airsim_camera_intrinsics(settings_path: str) -> Dict[str, Dict[str, Any]]:
    if not settings_path or not os.path.exists(settings_path):
        return {}
    try:
        with open(settings_path, "r", encoding="utf-8") as f:
            root = json.load(f)
    except Exception:
        return {}
    out: Dict[str, Dict[str, Any]] = {}
    if isinstance(root, dict):
        external = root.get("ExternalCameras", {})
        if isinstance(external, dict):
            for name, value in external.items():
                if isinstance(value, dict):
                    out[str(name)] = _parse_capture_settings(value)
        vehicles = root.get("Vehicles", {})
        if isinstance(vehicles, dict):
            for vehicle_obj in vehicles.values():
                if not isinstance(vehicle_obj, dict):
                    continue
                cameras = vehicle_obj.get("Cameras", {})
                if not isinstance(cameras, dict):
                    continue
                for name, value in cameras.items():
                    if str(name) not in out and isinstance(value, dict):
                        out[str(name)] = _parse_capture_settings(value)
    return out


def scene_center_from_bs_list(bs_list: List[Dict[str, Any]]) -> np.ndarray:
    if not bs_list:
        return np.array([0.0, 0.0, GROUND_HEIGHT + 2.0], dtype=float)
    points = []
    for bs in bs_list:
        try:
            points.append(np.asarray(bs.get("position", [0.0, 0.0, 0.0]), dtype=float).reshape(3))
        except Exception:
            continue
    if not points:
        return np.array([0.0, 0.0, GROUND_HEIGHT + 2.0], dtype=float)
    center = np.mean(np.stack(points, axis=0), axis=0)
    center[2] = max(float(GROUND_HEIGHT + 2.0), min(float(center[2]), float(GROUND_HEIGHT + 5.0)))
    return center.astype(float)


def scene_xy_distance_scale_from_bs_list(bs_list: List[Dict[str, Any]]) -> float:
    points = []
    for bs in bs_list:
        try:
            points.append(np.asarray(bs.get("position", [0.0, 0.0, 0.0]), dtype=float).reshape(3)[:2])
        except Exception:
            continue
    if len(points) < 2:
        return 100.0
    xy = np.stack(points, axis=0)
    span_x = float(np.max(xy[:, 0]) - np.min(xy[:, 0]))
    span_y = float(np.max(xy[:, 1]) - np.min(xy[:, 1]))
    return max(math.hypot(span_x, span_y), 100.0)


def build_station_proxy_camera(bs_cfg: Dict[str, Any],
                               scene_center: np.ndarray,
                               intrinsics_by_name: Dict[str, Dict[str, Any]]) -> Dict[str, Any]:
    origin = np.asarray(bs_cfg.get("position", [0.0, 0.0, 0.0]), dtype=float).reshape(3).copy()
    origin[2] += float(bs_cfg.get("preview_offset_z", 0.0) or 0.0)
    explicit = bs_cfg.get("deepsense_camera", {})
    if isinstance(explicit, dict) and explicit:
        target = np.asarray(explicit.get("target", scene_center.tolist()), dtype=float).reshape(3)
        up_hint = np.asarray(explicit.get("up", [0.0, 0.0, 1.0]), dtype=float).reshape(3)
        fov_deg = float(explicit.get("fov_deg", explicit.get("fov", 90.0)) or 90.0)
        width = max(int(explicit.get("width", 1920) or 1920), 2)
        height = max(int(explicit.get("height", 1080) or 1080), 2)
        if "origin" in explicit:
            origin = np.asarray(explicit.get("origin"), dtype=float).reshape(3)
        camera_source = "bs_json_deepsense_camera"
    else:
        preview_name = str(bs_cfg.get("preview_camera_name", "") or "").strip()
        intrinsic = intrinsics_by_name.get(preview_name, {})
        up_hint = np.array([0.0, 0.0, 1.0], dtype=float)
        fov_deg = float(intrinsic.get("fov_deg", 90.0) or 90.0)
        width = max(int(intrinsic.get("width", 1920) or 1920), 2)
        height = max(int(intrinsic.get("height", 1080) or 1080), 2)
        preview_target_raw = bs_cfg.get("preview_camera_target")
        target = None
        if isinstance(preview_target_raw, (list, tuple)) and len(preview_target_raw) >= 3:
            try:
                target = np.asarray(preview_target_raw[:3], dtype=float).reshape(3)
            except Exception:
                target = None
        if target is not None:
            camera_source = "bs_json_preview_camera_target"
            if intrinsic:
                camera_source += "+airsim_settings_intrinsics"
        else:
            target = np.asarray(scene_center, dtype=float).reshape(3).copy()
            target[2] = max(float(GROUND_HEIGHT + 2.0), min(float(target[2]), float(origin[2] - 1.0)))
            camera_source = "airsim_settings_intrinsics" if intrinsic else "synthetic_station_camera"
        if float(np.linalg.norm(target - origin)) < 1e-6:
            target = origin + np.array([1.0, 0.0, -0.1], dtype=float)
    return {
        "origin": origin.astype(float),
        "target": target.astype(float),
        "up": normalize_vector(up_hint, fallback=np.array([0.0, 0.0, 1.0], dtype=float)),
        "width": max(int(width), 2),
        "height": max(int(height), 2),
        "fov_deg": float(fov_deg) if np.isfinite(float(fov_deg)) else 90.0,
        "camera_name": str(bs_cfg.get("preview_camera_name", "") or "").strip(),
        "camera_source": camera_source,
    }


def project_scene_point_to_normalized_image(point_xyz: np.ndarray, camera_params: Dict[str, Any]) -> Dict[str, Any]:
    point = np.asarray(point_xyz, dtype=float).reshape(3)
    origin = np.asarray(camera_params.get("origin", [0.0, 0.0, 0.0]), dtype=float).reshape(3)
    target = np.asarray(camera_params.get("target", [1.0, 0.0, 0.0]), dtype=float).reshape(3)
    up_hint = np.asarray(camera_params.get("up", [0.0, 0.0, 1.0]), dtype=float).reshape(3)
    forward = normalize_vector(target - origin, fallback=np.array([1.0, 0.0, 0.0], dtype=float))
    right = normalize_vector(np.cross(forward, up_hint), fallback=np.array([0.0, 1.0, 0.0], dtype=float))
    up = normalize_vector(np.cross(right, forward), fallback=np.array([0.0, 0.0, 1.0], dtype=float))
    rel = point - origin
    x_cam = float(np.dot(rel, right))
    y_cam = float(np.dot(rel, up))
    z_cam = float(np.dot(rel, forward))
    width = max(int(camera_params.get("width", 1920) or 1920), 2)
    height = max(int(camera_params.get("height", 1080) or 1080), 2)
    aspect = float(width) / float(max(height, 1))
    fov_deg = float(camera_params.get("fov_deg", 90.0) or 90.0)
    tan_half_h = math.tan(math.radians(max(min(fov_deg, 170.0), 20.0)) * 0.5)
    tan_half_v = tan_half_h / max(aspect, 1e-6)
    u = float("nan")
    v = float("nan")
    valid = False
    if abs(tan_half_h) > 1e-9 and abs(tan_half_v) > 1e-9 and z_cam > 1e-6:
        x_ndc = x_cam / (z_cam * tan_half_h)
        y_ndc = y_cam / (z_cam * tan_half_v)
        u = 0.5 * (x_ndc + 1.0)
        v = 0.5 * (1.0 - y_ndc)
        valid = np.isfinite(u) and np.isfinite(v)
    visible = bool(valid and 0.0 <= u <= 1.0 and 0.0 <= v <= 1.0)
    return {
        "valid": bool(valid),
        "visible": visible,
        "u_norm": float(u) if np.isfinite(u) else float("nan"),
        "v_norm": float(v) if np.isfinite(v) else float("nan"),
        "x_cam": float(x_cam),
        "y_cam": float(y_cam),
        "z_cam": float(z_cam),
    }


def normalized_image_distance_angle_from_projection(proj: Dict[str, Any]) -> Tuple[float, float]:
    u = float(proj.get("u_norm", float("nan")))
    v = float(proj.get("v_norm", float("nan")))
    if np.isfinite(u) and np.isfinite(v):
        du = u - 0.5
        dv = 0.5 - v
        distance = min(math.hypot(du, dv) / math.hypot(0.5, 0.5), 1.0)
        angle = (math.atan2(dv, du) + 2.0 * math.pi) / (2.0 * math.pi)
        return float(distance), float(angle)
    x_cam = float(proj.get("x_cam", 0.0))
    y_cam = float(proj.get("y_cam", 0.0))
    angle = (math.atan2(y_cam, x_cam) + 2.0 * math.pi) / (2.0 * math.pi)
    return 1.0, float(angle)


GROUND_HEIGHT = 0.0
TB_STATUS_UNKNOWN = 0
TB_STATUS_ACK = 1
TB_STATUS_NACK = 2
SCHED_UNKNOWN = 0
SCHED_BEST_RATE_SINGLE_UAV = 1


def load_transform_matrix(matrix_file: str) -> np.ndarray:
    if not matrix_file:
        return np.eye(4, dtype=float)
    with open(matrix_file, "r", encoding="utf-8") as f:
        data = json.load(f)
    mat = np.array(data, dtype=float)
    if mat.shape != (4, 4):
        raise ValueError(f"transform matrix must be 4x4, got {mat.shape}")
    return mat


def transform_position_by_matrix(T: np.ndarray, pos_xyz: np.ndarray) -> np.ndarray:
    pos_h = np.array([pos_xyz[0], pos_xyz[1], pos_xyz[2], 1.0], dtype=float)
    out = np.dot(T, pos_h)[:3]
    out[2] = max(float(out[2]), GROUND_HEIGHT)
    return out


def transform_vector_by_matrix(R: np.ndarray, vec_xyz: np.ndarray) -> np.ndarray:
    vec = np.asarray(vec_xyz, dtype=float).reshape(3)
    return np.dot(R, vec)


def quaternion_xyzw_to_rotation_matrix(quat_xyzw: Any) -> np.ndarray:
    quat = np.asarray(quat_xyzw if quat_xyzw is not None else [0.0, 0.0, 0.0, 1.0], dtype=float).reshape(-1)
    if quat.size < 4 or not np.all(np.isfinite(quat[:4])):
        quat = np.array([0.0, 0.0, 0.0, 1.0], dtype=float)
    else:
        quat = quat[:4].astype(float)
    norm = float(np.linalg.norm(quat))
    if not np.isfinite(norm) or norm < 1e-12:
        quat = np.array([0.0, 0.0, 0.0, 1.0], dtype=float)
    else:
        quat = quat / norm
    x, y, z, w = [float(v) for v in quat]
    return np.array([
        [1.0 - 2.0 * (y * y + z * z), 2.0 * (x * y - z * w), 2.0 * (x * z + y * w)],
        [2.0 * (x * y + z * w), 1.0 - 2.0 * (x * x + z * z), 2.0 * (y * z - x * w)],
        [2.0 * (x * z - y * w), 2.0 * (y * z + x * w), 1.0 - 2.0 * (x * x + y * y)],
    ], dtype=float)


def rotation_matrix_to_sionna_orientation(rotation_matrix: Any) -> Tuple[float, float, float]:
    r_mat = np.asarray(rotation_matrix, dtype=float).reshape(3, 3)
    x_ang = math.atan2(float(r_mat[2, 1]), float(r_mat[2, 2]))
    y_ang = math.atan2(
        float(-r_mat[2, 0]),
        float(math.sqrt(float(r_mat[2, 1] ** 2 + r_mat[2, 2] ** 2))),
    )
    z_ang = math.atan2(float(r_mat[1, 0]), float(r_mat[0, 0]))
    return (z_ang, y_ang, x_ang)


def quaternion_xyzw_to_sionna_orientation(quat_xyzw: Any) -> Tuple[float, float, float]:
    return rotation_matrix_to_sionna_orientation(quaternion_xyzw_to_rotation_matrix(quat_xyzw))


def _safe_number(v: Any):
    try:
        f = float(v)
        if math.isnan(f) or math.isinf(f):
            return None
        return f
    except Exception:
        return v


def path_type_str_to_uint8(path_type: Any) -> int:
    s = str(path_type).strip().upper() if path_type is not None else ""
    if s == "LOS":
        return PATH_TYPE_LOS
    if s == "NLOS":
        return PATH_TYPE_NLOS
    if s == "NO_PATH":
        return PATH_TYPE_NO_PATH
    return PATH_TYPE_UNKNOWN


def ros_param_bool(rospy, name: str, default: bool) -> bool:
    value = rospy.get_param(name, default)
    if isinstance(value, bool):
        return value
    if isinstance(value, (int, float)):
        return bool(value)
    text = str(value).strip().lower()
    if text in {"1", "true", "yes", "on"}:
        return True
    if text in {"0", "false", "no", "off", ""}:
        return False
    return bool(value)


def infer_anchor_id(bs: Dict[str, Any], tx_idx: int) -> int:
    """
    Prefer anchor_id from the base-station config, then id, and finally tx_idx.
    """
    for key in ("anchor_id", "id"):
        if key in bs:
            try:
                return int(bs[key])
            except Exception:
                pass
    return int(tx_idx)


# =========================================================
# 1) Sionna RT simulation configuration
# =========================================================
@dataclass
class SimulationConfig:
    scene_path: str
    fc_hz: float = 3.5e9
    mi_variant: str = "cuda_ad_mono_polarized"

    tx_array_num_rows: int = 1
    tx_array_num_cols: int = 1
    tx_array_vertical_spacing: float = 0.5
    tx_array_horizontal_spacing: float = 0.5
    tx_array_pattern: str = "iso"
    tx_array_polarization: str = "V"

    rx_array_num_rows: int = 1
    rx_array_num_cols: int = 1
    rx_array_vertical_spacing: float = 0.5
    rx_array_horizontal_spacing: float = 0.5
    rx_array_pattern: str = "iso"
    rx_array_polarization: str = "V"

    max_depth: int = 1
    samples_per_src: int = 2000
    max_num_paths_per_src: int = 64
    synthetic_array: bool = True
    merge_shapes: bool = False

    enable_sys_integration: bool = False
    enable_beamforming: bool = False
    beam_selection_mode: str = "exhaustive_sweep"
    beam_codebook_type: str = "auto"
    beam_codebook_num_beams: int = 8
    beam_oversampling_v: int = 1
    beam_oversampling_h: int = 1
    beam_manual_index: int = 0
    beam_normalize_power: bool = True
    beam_codebook_file: str = ""
    beam_model_checkpoint_path: str = ""
    beam_feature_mode: str = DEFAULT_FEATURE_MODE
    beam_top_k: int = 3
    beam_export_training_dataset: bool = False
    beam_training_dataset_path: str = "beam_training_samples.csv"
    beam_training_reset_dataset_on_start: bool = False
    beam_training_output_checkpoint_path: str = "beam_trained_model.pt"
    beam_training_epochs: int = 30
    beam_training_batch_size: int = 256
    beam_training_learning_rate: float = 1e-3
    beam_training_validation_split: float = 0.2
    beam_training_hidden_dim: int = 512
    sys_num_subcarriers: int = 128
    sys_subcarrier_spacing_hz: float = 30e3
    sys_num_ofdm_symbols: int = 12
    sys_temperature_k: float = 294.0
    sys_bler_target: float = 0.1
    sys_mcs_table_index: int = 1
    sys_bs_tx_power_dbm: float = 10.0

    los: bool = True
    specular_reflection: bool = True
    diffuse_reflection: bool = False
    refraction: bool = False
    diffraction: bool = False
    edge_diffraction: bool = False
    diffraction_lit_region: bool = False
    seed: int = 42

    enable_preview_render: bool = False
    preview_output_dir: str = "preview_frames"
    preview_width: int = 960
    preview_height: int = 720
    preview_show_devices: bool = True
    preview_show_orientations: bool = False
    preview_camera_follow_y_offset_m: float = 80.0
    preview_camera_height_offset_m: float = 120.0

    bs_list: List[Dict[str, Any]] = field(default_factory=list)


class OfflineSionnaSimulator:
    def __init__(self, cfg: SimulationConfig):
        self.cfg = cfg
        self.scene = None
        self.solver = None
        self.top_cam = None
        self.sys_runtime: Optional[Dict[str, Any]] = None
        self.beam_runtime: Optional[Dict[str, Any]] = None
        self.beam_dataset_exporter: Optional[Dict[str, Any]] = None
        self.beam_proxy_context: Dict[str, Any] = self._build_beam_proxy_context()
        self._build_scene_and_solver()
        self._build_sys_runtime()
        self._build_beam_runtime()
        self._build_beam_dataset_exporter()

    def _build_sys_runtime(self) -> None:
        if not self.cfg.enable_sys_integration:
            self.sys_runtime = None
            return

        try:
            import torch
            import sionna.phy  # noqa: F401
            import sionna.sys  # noqa: F401
            from sionna.phy import config as phy_config
            from sionna.phy.constants import BOLTZMANN_CONSTANT
            from sionna.phy.nr.utils import decode_mcs_index
            from sionna.phy.ofdm import ResourceGrid
            from sionna.phy.utils import dbm_to_watt, lin_to_db
            from sionna.rt import subcarrier_frequencies
            from sionna.sys import OuterLoopLinkAdaptation, PHYAbstraction
        except Exception as e:
            raise ImportError(
                "SYS over RT requires the current Python environment to import sionna.sys, sionna.phy, and sionna.rt."
            ) from e

        phy_config.precision = "single"
        phy_config.seed = int(self.cfg.seed)

        num_candidates = max(int(len(self.cfg.bs_list)), 1)
        resource_grid = ResourceGrid(
            num_ofdm_symbols=int(self.cfg.sys_num_ofdm_symbols),
            fft_size=int(self.cfg.sys_num_subcarriers),
            subcarrier_spacing=float(self.cfg.sys_subcarrier_spacing_hz),
            num_tx=num_candidates,
            num_streams_per_tx=1,
        )
        frequencies = subcarrier_frequencies(
            num_subcarriers=int(self.cfg.sys_num_subcarriers),
            subcarrier_spacing=float(self.cfg.sys_subcarrier_spacing_hz),
        )

        device = "cuda" if torch.cuda.is_available() else "cpu"
        phy_abs = PHYAbstraction()
        olla = OuterLoopLinkAdaptation(
            phy_abs,
            num_ut=num_candidates,
            bler_target=float(self.cfg.sys_bler_target),
            batch_size=[1],
        ).to(device)

        self.sys_runtime = {
            "torch": torch,
            "device": device,
            "resource_grid": resource_grid,
            "frequencies": frequencies,
            "phy_abs": phy_abs,
            "olla": olla,
            "dbm_to_watt": dbm_to_watt,
            "lin_to_db": lin_to_db,
            "decode_mcs_index": decode_mcs_index,
            "noise_watt": float(BOLTZMANN_CONSTANT) * float(self.cfg.sys_temperature_k) * float(self.cfg.sys_subcarrier_spacing_hz),
            "num_candidates": num_candidates,
            "harq_feedback": -torch.ones([1, num_candidates], dtype=torch.int32, device=device),
            "sinr_eff_feedback": torch.zeros([1, num_candidates], dtype=torch.float32, device=device),
            "num_decoded_bits": torch.zeros([1, num_candidates], dtype=torch.int32, device=device),
        }

    def _build_beam_runtime(self) -> None:
        mode = str(self.cfg.beam_selection_mode).strip().lower()
        if (not self.cfg.enable_beamforming) or mode in {"", "off", "disabled", "none"}:
            self.beam_runtime = None
            return

        try:
            import torch
            from sionna.phy import config as phy_config
            from sionna.phy.mimo import (
                grid_of_beams_dft,
                grid_of_beams_dft_ula,
                normalize_precoding_power,
            )
            from sionna.phy.ofdm import ResourceGrid
            from sionna.rt import subcarrier_frequencies
        except Exception as e:
            raise ImportError(
                "Beamforming V2 requires the current Python environment to import sionna.phy.mimo, sionna.phy.ofdm, and sionna.rt."
            ) from e

        phy_config.precision = "single"
        phy_config.seed = int(self.cfg.seed)
        configured_feature_mode = DeepSenseBeamPredictor.normalize_feature_mode(str(self.cfg.beam_feature_mode or DEFAULT_FEATURE_MODE))

        tx_rows = max(int(self.cfg.tx_array_num_rows), 1)
        tx_cols = max(int(self.cfg.tx_array_num_cols), 1)
        tx_ant = tx_rows * tx_cols
        device = "cuda" if torch.cuda.is_available() else "cpu"
        custom_codebook_path = str(self.cfg.beam_codebook_file or "").strip()
        codebook_payload = None
        if custom_codebook_path:
            codebook_payload, codebook_np = load_codebook_payload_from_file(custom_codebook_path)
            if int(codebook_np.shape[1]) != int(tx_ant):
                raise ValueError(
                    f"custom codebook antenna dimension mismatch: file has tx_ant={int(codebook_np.shape[1])}, but current TX array requires {int(tx_ant)}"
                )
            codebook_type = str(codebook_payload.get("codebook_type", "custom_file") or "custom_file")
            codebook = torch.as_tensor(codebook_np, dtype=torch.complex64, device=device)
        else:
            requested_type = str(self.cfg.beam_codebook_type).strip().lower() or "auto"
            if requested_type == "auto":
                codebook_type = "dft_ura" if (tx_rows > 1 and tx_cols > 1) else "dft_ula"
            else:
                codebook_type = requested_type
            if codebook_type == "dft_ura" and (tx_rows <= 1 or tx_cols <= 1):
                codebook_type = "dft_ula"
            target_num_beams = normalize_codebook_num_beams(self.cfg.beam_codebook_num_beams)
            oversampling_v, oversampling_h = effective_dft_oversampling_for_num_beams(
                codebook_type=codebook_type,
                tx_rows=tx_rows,
                tx_cols=tx_cols,
                target_num_beams=target_num_beams,
                min_oversampling_v=max(int(self.cfg.beam_oversampling_v), 1),
                min_oversampling_h=max(int(self.cfg.beam_oversampling_h), 1),
            )

            if codebook_type == "dft_ura":
                codebook = grid_of_beams_dft(
                    num_ant_v=tx_rows,
                    num_ant_h=tx_cols,
                    oversmpl_v=oversampling_v,
                    oversmpl_h=oversampling_h,
                )
                if hasattr(codebook, "reshape"):
                    codebook = codebook.reshape(-1, tx_ant)
            else:
                oversmpl = max(int(oversampling_h if tx_cols > 1 else oversampling_v), 1)
                codebook = grid_of_beams_dft_ula(num_ant=tx_ant, oversmpl=oversmpl)
            codebook, selected_source_indices = select_codebook_num_beams(codebook, target_num_beams)

            if self.cfg.beam_normalize_power:
                codebook = normalize_precoding_power(codebook)

            if hasattr(codebook, "to"):
                codebook = codebook.to(device)
            codebook = codebook.to(torch.complex64) if hasattr(codebook, "to") else codebook
        if custom_codebook_path:
            oversampling_v = max(int(self.cfg.beam_oversampling_v), 1)
            oversampling_h = max(int(self.cfg.beam_oversampling_h), 1)
            target_num_beams = int(codebook.shape[0]) if hasattr(codebook, "shape") else 0
            selected_source_indices = list(range(target_num_beams))

        resource_grid = ResourceGrid(
            num_ofdm_symbols=int(self.cfg.sys_num_ofdm_symbols),
            fft_size=int(self.cfg.sys_num_subcarriers),
            subcarrier_spacing=float(self.cfg.sys_subcarrier_spacing_hz),
            num_tx=max(int(len(self.cfg.bs_list)), 1),
            num_streams_per_tx=1,
        )
        frequencies = subcarrier_frequencies(
            num_subcarriers=int(self.cfg.sys_num_subcarriers),
            subcarrier_spacing=float(self.cfg.sys_subcarrier_spacing_hz),
        )

        num_beams = int(codebook.shape[0]) if hasattr(codebook, "shape") and len(codebook.shape) >= 1 else 0
        predictor = None
        predictor_available = False
        predictor_status = "inactive"
        predictor_error = ""
        if mode == "deepsense_predictor":
            predictor_status = "checkpoint_missing"
            ckpt = str(self.cfg.beam_model_checkpoint_path or "").strip()
            if not ckpt:
                predictor_error = "Beam selection checkpoint path is empty"
            else:
                try:
                    predictor = DeepSenseBeamPredictor(
                        checkpoint_path=ckpt,
                        device=device,
                        default_feature_mode=configured_feature_mode,
                    )
                    if int(predictor.metadata.output_size) != int(num_beams):
                        predictor = None
                        predictor_status = "output_mismatch"
                        predictor_error = (
                            f"checkpoint outputs {int(predictor.metadata.output_size)} beams but current Sionna codebook has {int(num_beams)} beams"
                        )
                    else:
                        predictor_available = True
                        predictor_status = "ready"
                except Exception as e:
                    predictor = None
                    predictor_status = "load_failed"
                    predictor_error = str(e)
        active_feature_mode = configured_feature_mode
        predictor_feature_names = getattr(predictor.metadata, "feature_names", None) if predictor is not None else None
        if predictor is not None and predictor_feature_names and getattr(predictor.metadata, "feature_mode", ""):
            active_feature_mode = DeepSenseBeamPredictor.normalize_feature_mode(str(predictor.metadata.feature_mode))

        self.beam_runtime = {
            "torch": torch,
            "device": device,
            "resource_grid": resource_grid,
            "frequencies": frequencies,
            "selection_mode": mode,
            "codebook_type": codebook_type,
            "codebook": codebook,
            "codebook_payload": codebook_payload,
            "custom_codebook_path": custom_codebook_path,
            "num_beams": num_beams,
            "beam_codebook_num_beams": int(target_num_beams),
            "beam_oversampling_v": int(oversampling_v),
            "beam_oversampling_h": int(oversampling_h),
            "codebook_source_indices": [int(v) for v in selected_source_indices],
            "tx_ant": tx_ant,
            "manual_beam_index": max(int(self.cfg.beam_manual_index), 0),
            "feature_mode": active_feature_mode,
            "top_k": max(int(self.cfg.beam_top_k), 1),
            "checkpoint_path": str(self.cfg.beam_model_checkpoint_path or ""),
            "predictor": predictor,
            "predictor_available": predictor_available,
            "predictor_status": predictor_status,
            "predictor_error": predictor_error,
        }

    def build_beam_codebook_payload(self) -> Optional[Dict[str, Any]]:
        if self.beam_runtime:
            payload = self.beam_runtime.get("codebook_payload")
            if isinstance(payload, dict) and payload:
                return payload
        return build_codebook_payload(self.beam_runtime, self.cfg)

    def _build_beam_proxy_context(self) -> Dict[str, Any]:
        settings_path = resolve_airsim_settings_path()
        intrinsics_by_name = load_airsim_camera_intrinsics(settings_path)
        scene_center = scene_center_from_bs_list(self.cfg.bs_list)
        geom_distance_scale_m = scene_xy_distance_scale_from_bs_list(self.cfg.bs_list)
        station_cameras = [
            build_station_proxy_camera(bs_cfg, scene_center=scene_center, intrinsics_by_name=intrinsics_by_name)
            for bs_cfg in self.cfg.bs_list
        ]
        return {
            "airsim_settings_path": settings_path,
            "geom_distance_scale_m": float(geom_distance_scale_m),
            "scene_center": scene_center.astype(float),
            "station_cameras": station_cameras,
        }

    def _build_deepsense_proxy_summary(self, tx_idx: int, rx_position: Optional[np.ndarray]) -> Dict[str, Any]:
        summary = {
            "geom_distance_scale_m": float(self.beam_proxy_context.get("geom_distance_scale_m", 100.0)),
            "bbox_distance": 1.0,
            "bbox_angle": 0.0,
            "bbox_visible": 0,
            "bbox_u_norm": float("nan"),
            "bbox_v_norm": float("nan"),
            "camera_source": "",
            "camera_name": "",
        }
        if rx_position is None:
            return summary
        station_cameras = self.beam_proxy_context.get("station_cameras", [])
        if not (0 <= int(tx_idx) < len(station_cameras)):
            return summary
        camera = station_cameras[int(tx_idx)]
        proj = project_scene_point_to_normalized_image(np.asarray(rx_position, dtype=float).reshape(3), camera)
        bbox_distance, bbox_angle = normalized_image_distance_angle_from_projection(proj)
        summary.update({
            "bbox_distance": float(bbox_distance),
            "bbox_angle": float(bbox_angle),
            "bbox_visible": int(bool(proj.get("visible", False))),
            "bbox_u_norm": float(proj.get("u_norm", float("nan"))),
            "bbox_v_norm": float(proj.get("v_norm", float("nan"))),
            "camera_source": str(camera.get("camera_source", "")),
            "camera_name": str(camera.get("camera_name", "")),
        })
        return summary

    def _build_beam_dataset_exporter(self) -> None:
        if (not self.cfg.enable_beamforming) or (not bool(self.cfg.beam_export_training_dataset)):
            self.beam_dataset_exporter = None
            return

        dataset_path = str(self.cfg.beam_training_dataset_path or "").strip()
        if not dataset_path:
            self.beam_dataset_exporter = {
                "enabled": False,
                "path": "",
                "sample_count": 0,
                "status": "path_empty",
                "error": "beam training dataset path is empty",
            }
            print("[beam-dataset] export requested but dataset path is empty", flush=True)
            return

        ensure_dir_for_file(dataset_path)
        if bool(self.cfg.beam_training_reset_dataset_on_start):
            reset_output_file(dataset_path)

        exporter = {
            "enabled": True,
            "path": dataset_path,
            "sample_count": 0,
            "status": "ready",
            "error": "",
        }
        self.beam_dataset_exporter = exporter

        manifest_path = dataset_path + ".manifest.json"
        feature_mode = DeepSenseBeamPredictor.normalize_feature_mode(str(self.cfg.beam_feature_mode or DEFAULT_FEATURE_MODE))
        manifest = {
            "scene_path": str(self.cfg.scene_path),
            "beam_codebook_type": str(self.beam_runtime.get("codebook_type", self.cfg.beam_codebook_type) if self.beam_runtime else self.cfg.beam_codebook_type),
            "beam_codebook_file": str(self.cfg.beam_codebook_file),
            "beam_selection_mode": str(self.cfg.beam_selection_mode),
            "beam_feature_mode": feature_mode,
            "beam_feature_names": list(DeepSenseBeamPredictor.feature_schema(feature_mode)),
            "beam_feature_count": int(len(DeepSenseBeamPredictor.feature_schema(feature_mode))),
            "beam_num_beams": int(self.beam_runtime.get("num_beams", 0)) if self.beam_runtime else 0,
            "tx_array_num_rows": int(self.cfg.tx_array_num_rows),
            "tx_array_num_cols": int(self.cfg.tx_array_num_cols),
            "beam_codebook_num_beams": int(self.beam_runtime.get("beam_codebook_num_beams", self.cfg.beam_codebook_num_beams)) if self.beam_runtime else int(self.cfg.beam_codebook_num_beams),
            "beam_oversampling_v": int(self.beam_runtime.get("beam_oversampling_v", self.cfg.beam_oversampling_v)) if self.beam_runtime else int(self.cfg.beam_oversampling_v),
            "beam_oversampling_h": int(self.beam_runtime.get("beam_oversampling_h", self.cfg.beam_oversampling_h)) if self.beam_runtime else int(self.cfg.beam_oversampling_h),
            "fc_hz": float(self.cfg.fc_hz),
            "created_utc": time.strftime("%Y-%m-%dT%H:%M:%SZ", time.gmtime()),
        }
        try:
            ensure_dir_for_file(manifest_path)
            with open(manifest_path, "w", encoding="utf-8") as f:
                json.dump(manifest, f, ensure_ascii=False, indent=2)
        except Exception as e:
            print(f"[beam-dataset] failed to write manifest {manifest_path}: {e}", flush=True)
        print(f"[beam-dataset] ready: {dataset_path}", flush=True)

    def _export_beam_training_samples(
        self,
        beam_summary: Dict[str, Any],
        rx_position: Optional[np.ndarray],
        rx_velocity: Optional[np.ndarray],
        sim_idx: int,
        odom_stamp_s: float,
    ) -> None:
        exporter = self.beam_dataset_exporter
        if not exporter or not bool(exporter.get("enabled", False)):
            return
        if rx_position is None:
            return
        rows: List[Dict[str, Any]] = []
        ref_tx_idx = int(beam_summary.get("reference_tx_idx", -1))
        for item in beam_summary.get("candidates", []):
            tx_idx = int(item.get("tx_idx", -1))
            if tx_idx < 0 or tx_idx >= len(self.cfg.bs_list):
                continue
            oracle_idx = int(item.get("oracle_beam_index", -1))
            if oracle_idx < 0:
                continue
            feature_vector = item.get("feature_vector", [])
            if len(feature_vector) <= 0:
                continue
            bs_cfg = self.cfg.bs_list[tx_idx]
            tx_position = np.asarray(bs_cfg.get("position", [0.0, 0.0, 0.0]), dtype=float).reshape(3)
            link_num_paths = int(item.get("link_num_paths", 0) or 0)
            link_power_db = float(item.get("link_power_db", np.nan))
            link_tau_std_s = float(item.get("link_tau_std_s", np.nan))
            link_doppler_hz = float(item.get("link_doppler_hz", np.nan))
            row = {
                "sim_idx": int(sim_idx),
                "odom_stamp_s": float(odom_stamp_s),
                "timestamp_utc": time.strftime("%Y-%m-%dT%H:%M:%SZ", time.gmtime()),
                "scene_path": str(self.cfg.scene_path),
                "bs_name": str(bs_cfg.get("name", "")),
                "anchor_id": infer_anchor_id(bs_cfg, tx_idx),
                "tx_idx": int(tx_idx),
                "is_reference_candidate": int(tx_idx == ref_tx_idx),
                "selection_mode": str(beam_summary.get("selection_mode", "")),
                "selection_source": str(item.get("selection_source", "")),
                "codebook_type": str(beam_summary.get("codebook_type", "")),
                "num_beams": int(beam_summary.get("num_beams", 0)),
                "feature_mode": str(beam_summary.get("feature_mode", self.cfg.beam_feature_mode)),
                "feature_count": int(len(feature_vector)),
                "oracle_beam_index": int(oracle_idx),
                "oracle_beam_gain_db": float(item.get("oracle_beam_gain_db", np.nan)),
                "selected_beam_index": int(item.get("selected_beam_index", -1)),
                "selected_beam_gain_db": float(item.get("selected_beam_gain_db", np.nan)),
                "rx_x": float(rx_position[0]),
                "rx_y": float(rx_position[1]),
                "rx_z": float(rx_position[2]),
                "rx_vx_mps": float(rx_velocity[0]) if rx_velocity is not None and len(rx_velocity) >= 1 else 0.0,
                "rx_vy_mps": float(rx_velocity[1]) if rx_velocity is not None and len(rx_velocity) >= 2 else 0.0,
                "rx_vz_mps": float(rx_velocity[2]) if rx_velocity is not None and len(rx_velocity) >= 3 else 0.0,
                "tx_x": float(tx_position[0]),
                "tx_y": float(tx_position[1]),
                "tx_z": float(tx_position[2]),
                "tx_array_num_rows": int(self.cfg.tx_array_num_rows),
                "tx_array_num_cols": int(self.cfg.tx_array_num_cols),
                "beam_codebook_num_beams": int(self.beam_runtime.get("beam_codebook_num_beams", self.cfg.beam_codebook_num_beams)) if self.beam_runtime else int(self.cfg.beam_codebook_num_beams),
                "beam_oversampling_v": int(self.beam_runtime.get("beam_oversampling_v", self.cfg.beam_oversampling_v)) if self.beam_runtime else int(self.cfg.beam_oversampling_v),
                "beam_oversampling_h": int(self.beam_runtime.get("beam_oversampling_h", self.cfg.beam_oversampling_h)) if self.beam_runtime else int(self.cfg.beam_oversampling_h),
                "link_num_paths": link_num_paths,
                "link_power_db": link_power_db,
                "link_tau_std_s": link_tau_std_s,
                "link_doppler_hz": link_doppler_hz,
                "proxy_camera_source": str(item.get("proxy_camera_source", "")),
                "proxy_camera_name": str(item.get("proxy_camera_name", "")),
                "proxy_bbox_visible": int(item.get("proxy_bbox_visible", 0) or 0),
                "proxy_bbox_u_norm": float(item.get("proxy_bbox_u_norm", np.nan)),
                "proxy_bbox_v_norm": float(item.get("proxy_bbox_v_norm", np.nan)),
                "proxy_bbox_distance": float(item.get("proxy_bbox_distance", np.nan)),
                "proxy_bbox_angle": float(item.get("proxy_bbox_angle", np.nan)),
                "proxy_geom_distance_scale_m": float(item.get("proxy_geom_distance_scale_m", np.nan)),
            }
            for i, v in enumerate(feature_vector):
                row[f"feature_{i}"] = float(v)
            rows.append(row)
        if not rows:
            return
        try:
            csv_path = str(exporter["path"])
            df = pd.DataFrame(rows)
            append_rows_csv(df, csv_path, label="beam-dataset")
            exporter["sample_count"] = int(exporter.get("sample_count", 0)) + len(rows)
            if exporter["sample_count"] <= len(rows) or exporter["sample_count"] % 100 == 0:
                print(f"[beam-dataset] appended {len(rows)} rows -> total={exporter['sample_count']} path={csv_path}", flush=True)
        except Exception as e:
            exporter["status"] = "write_failed"
            exporter["error"] = str(e)
            print(f"[beam-dataset] write failed: {e}", flush=True)


    def _run_beamforming_sweep(
        self,
        paths,
        preferred_serving_idx: int = -1,
        rx_position: Optional[np.ndarray] = None,
        rx_velocity: Optional[np.ndarray] = None,
    ) -> Dict[str, Any]:
        exporter = self.beam_dataset_exporter or {}
        configured_feature_mode = DeepSenseBeamPredictor.normalize_feature_mode(str(self.cfg.beam_feature_mode or DEFAULT_FEATURE_MODE))
        configured_feature_names = list(DeepSenseBeamPredictor.feature_schema(configured_feature_mode))
        if self.beam_runtime is None:
            return {
                "enabled": False,
                "selection_mode": str(self.cfg.beam_selection_mode),
                "codebook_type": str(self.cfg.beam_codebook_type),
                "num_beams": 0,
                "manual_beam_index": int(self.cfg.beam_manual_index),
                "predictor_available": False,
                "predictor_status": "disabled",
                "predictor_error": "",
                "feature_mode": configured_feature_mode,
                "feature_names": configured_feature_names,
                "top_k": int(self.cfg.beam_top_k),
                "dataset_export_enabled": bool(exporter.get("enabled", False)),
                "dataset_export_path": str(exporter.get("path", "")),
                "dataset_export_sample_count": int(exporter.get("sample_count", 0)),
                "dataset_export_status": str(exporter.get("status", "disabled")),
                "dataset_export_error": str(exporter.get("error", "")),
                "candidates": [],
            }

        rt = self.beam_runtime
        feature_mode = DeepSenseBeamPredictor.normalize_feature_mode(str(rt.get("feature_mode", configured_feature_mode)))
        feature_names = list(DeepSenseBeamPredictor.feature_schema(feature_mode))
        torch = rt["torch"]
        device = rt["device"]
        codebook = rt["codebook"]
        num_beams = int(rt["num_beams"])
        if num_beams <= 0:
            return {
                "enabled": True,
                "selection_mode": rt["selection_mode"],
                "codebook_type": rt["codebook_type"],
                "num_beams": 0,
                "manual_beam_index": int(rt["manual_beam_index"]),
                "predictor_available": bool(rt.get("predictor_available", False)),
                "predictor_status": str(rt.get("predictor_status", "inactive")),
                "predictor_error": str(rt.get("predictor_error", "")),
                "feature_mode": feature_mode,
                "feature_names": feature_names,
                "top_k": int(rt.get("top_k", 1)),
                "dataset_export_enabled": bool(exporter.get("enabled", False)),
                "dataset_export_path": str(exporter.get("path", "")),
                "dataset_export_sample_count": int(exporter.get("sample_count", 0)),
                "dataset_export_status": str(exporter.get("status", "disabled")),
                "dataset_export_error": str(exporter.get("error", "")),
                "candidates": [],
            }

        h_freq = paths.cfr(
            frequencies=rt["frequencies"],
            sampling_frequency=1.0 / float(rt["resource_grid"].ofdm_symbol_duration),
            num_time_steps=int(self.cfg.sys_num_ofdm_symbols),
            out_type="torch",
        )
        if hasattr(h_freq, "to"):
            h_freq = h_freq.to(device)

        if len(h_freq.shape) != 6:
            raise ValueError(f"Unexpected CFR tensor shape for beamforming: {tuple(h_freq.shape)}")

        candidates: List[Dict[str, Any]] = []
        predictor = rt.get("predictor")
        predictor_available = bool(rt.get("predictor_available", False))
        predictor_status_global = str(rt.get("predictor_status", "inactive"))
        predictor_error_global = str(rt.get("predictor_error", ""))
        for tx_idx in range(min(int(h_freq.shape[2]), len(self.cfg.bs_list))):
            H = h_freq[0, :, tx_idx, :, :, :]
            if int(H.shape[1]) != int(rt["tx_ant"]):
                candidates.append({
                    "tx_idx": tx_idx,
                    "selected_beam_index": -1,
                    "selected_beam_gain_db": np.nan,
                    "oracle_beam_index": -1,
                    "oracle_beam_gain_db": np.nan,
                    "predicted_beam_index": -1,
                    "predicted_beam_confidence": np.nan,
                    "oracle_in_topk": False,
                    "beam_hit": False,
                    "topk_indices": [],
                    "topk_probabilities": [],
                    "feature_names": feature_names,
                    "feature_vector": [],
                    "link_num_paths": 0,
                    "link_power_db": np.nan,
                    "link_tau_std_s": np.nan,
                    "link_doppler_hz": np.nan,
                    "selection_source": "invalid_tx_ant",
                    "predictor_status": "invalid_tx_ant",
                    "predictor_error": f'tx_ant mismatch: expected {int(rt["tx_ant"])} got {int(H.shape[1])}',
                })
                continue

            h_eff = torch.einsum("rats,ka->krts", H.to(torch.complex64), codebook)
            beam_power = torch.mean(torch.sum(torch.abs(h_eff).pow(2).float(), dim=1), dim=(1, 2))
            beam_power_cpu = beam_power.detach().cpu().numpy().reshape(-1)
            predictor_status = predictor_status_global
            predictor_error = predictor_error_global
            predicted_idx = -1
            predicted_confidence = np.nan
            topk_indices: List[int] = []
            topk_probabilities: List[float] = []
            feature_vector: List[float] = []
            link_num_paths = 0
            link_power_db = np.nan
            link_tau_std_s = np.nan
            link_doppler_hz = np.nan
            proxy_summary: Optional[Dict[str, Any]] = None
            tx_position = None
            if rx_position is not None:
                try:
                    tx_position = np.asarray(self.cfg.bs_list[tx_idx].get("position", [0.0, 0.0, 0.0]), dtype=float).reshape(3)
                    link_summary = None
                    if feature_mode == DEFAULT_FEATURE_MODE:
                        all_paths, best_path = self.extract_all_paths_for_link(
                            paths,
                            rx_idx=0,
                            tx_idx=tx_idx,
                            tx_position=tx_position,
                            rx_position=rx_position,
                        )
                        link_num_paths = int(len(all_paths))
                        if best_path is not None:
                            link_power_db = float(best_path.get("power_db", np.nan))
                            link_tau_std_s = float(best_path.get("tau_std_s", np.nan))
                            link_doppler_hz = float(best_path.get("doppler_hz", np.nan))
                        link_summary = {
                            "num_paths": link_num_paths,
                            "power_db": link_power_db,
                            "tau_std_s": link_tau_std_s,
                            "doppler_hz": link_doppler_hz,
                        }
                    if feature_mode == DEEPSENSE_POS_BBOX4_MODE:
                        proxy_summary = self._build_deepsense_proxy_summary(tx_idx=tx_idx, rx_position=rx_position)
                    feature_vector = [float(v) for v in DeepSenseBeamPredictor.build_feature_vector(
                        tx_position=tx_position,
                        rx_position=rx_position,
                        feature_mode=feature_mode,
                        rx_velocity=rx_velocity,
                        link_summary=link_summary,
                        proxy_context=proxy_summary,
                    ).reshape(-1).tolist()]
                except Exception:
                    feature_vector = []
            oracle_in_topk = False
            beam_hit = False
            selection_source = "oracle"
            if beam_power_cpu.size <= 0:
                oracle_idx = -1
                oracle_gain_db = np.nan
                selected_idx = -1
                selected_gain_db = np.nan
            else:
                oracle_idx = int(np.argmax(beam_power_cpu))
                oracle_gain_lin = float(beam_power_cpu[oracle_idx])
                oracle_gain_db = float(10.0 * np.log10(max(oracle_gain_lin, 1e-30)))
                if rt["selection_mode"] == "manual_index":
                    selected_idx = min(max(int(rt["manual_beam_index"]), 0), beam_power_cpu.size - 1)
                    selection_source = "manual_index"
                elif rt["selection_mode"] == "deepsense_predictor":
                    if predictor_available and predictor is not None and rx_position is not None:
                        try:
                            if tx_position is None:
                                tx_position = np.asarray(self.cfg.bs_list[tx_idx].get("position", [0.0, 0.0, 0.0]), dtype=float).reshape(3)
                            features_np = np.asarray(feature_vector, dtype=np.float32).reshape(-1) if feature_vector else DeepSenseBeamPredictor.build_feature_vector(
                                tx_position=tx_position,
                                rx_position=rx_position,
                                feature_mode=feature_mode,
                                rx_velocity=rx_velocity,
                                link_summary={
                                    "num_paths": link_num_paths,
                                    "power_db": link_power_db,
                                    "tau_std_s": link_tau_std_s,
                                    "doppler_hz": link_doppler_hz,
                                },
                                proxy_context=proxy_summary,
                            )
                            pred = predictor.predict(features_np, top_k=int(rt.get("top_k", 1)))
                            predicted_idx = int(pred.get("pred_index", -1))
                            predicted_confidence = float(pred.get("pred_confidence", np.nan))
                            topk_indices = [int(v) for v in pred.get("topk_indices", [])]
                            topk_probabilities = [float(v) for v in pred.get("topk_probabilities", [])]
                            feature_vector = [float(v) for v in pred.get("feature_vector", [])]
                            predictor_status = "ready"
                            predictor_error = ""
                            if 0 <= predicted_idx < beam_power_cpu.size:
                                selected_idx = predicted_idx
                                selection_source = "deepsense_predictor"
                            else:
                                selected_idx = oracle_idx
                                selection_source = "oracle_fallback_invalid_prediction"
                                predictor_status = "invalid_prediction"
                                predictor_error = f'predicted beam {predicted_idx} outside [0,{beam_power_cpu.size - 1}]'
                        except Exception as e:
                            selected_idx = oracle_idx
                            selection_source = "oracle_fallback_predictor_error"
                            predictor_status = "runtime_error"
                            predictor_error = str(e)
                    else:
                        selected_idx = oracle_idx
                        selection_source = "oracle_fallback_no_predictor"
                        if rx_position is None and predictor_status == "ready":
                            predictor_status = "rx_position_missing"
                            predictor_error = "receiver position is unavailable for feature extraction"
                else:
                    selected_idx = oracle_idx
                    selection_source = "oracle"
                beam_hit = bool(selected_idx >= 0 and oracle_idx >= 0 and selected_idx == oracle_idx)
                oracle_in_topk = bool(oracle_idx >= 0 and len(topk_indices) > 0 and oracle_idx in topk_indices)
                selected_gain_lin = float(beam_power_cpu[selected_idx])
                selected_gain_db = float(10.0 * np.log10(max(selected_gain_lin, 1e-30)))

            candidates.append({
                "tx_idx": tx_idx,
                "selected_beam_index": selected_idx,
                "selected_beam_gain_db": selected_gain_db,
                "oracle_beam_index": oracle_idx,
                "oracle_beam_gain_db": oracle_gain_db,
                "predicted_beam_index": predicted_idx,
                "predicted_beam_confidence": predicted_confidence,
                "oracle_in_topk": bool(oracle_in_topk),
                "beam_hit": bool(beam_hit),
                "topk_indices": topk_indices,
                "topk_probabilities": topk_probabilities,
                "feature_names": feature_names,
                "feature_vector": feature_vector,
                "link_num_paths": int(link_num_paths),
                "link_power_db": float(link_power_db),
                "link_tau_std_s": float(link_tau_std_s),
                "link_doppler_hz": float(link_doppler_hz),
                "proxy_camera_source": str(proxy_summary.get("camera_source", "")) if proxy_summary else "",
                "proxy_camera_name": str(proxy_summary.get("camera_name", "")) if proxy_summary else "",
                "proxy_bbox_visible": int(proxy_summary.get("bbox_visible", 0)) if proxy_summary else 0,
                "proxy_bbox_u_norm": float(proxy_summary.get("bbox_u_norm", np.nan)) if proxy_summary else np.nan,
                "proxy_bbox_v_norm": float(proxy_summary.get("bbox_v_norm", np.nan)) if proxy_summary else np.nan,
                "proxy_bbox_distance": float(proxy_summary.get("bbox_distance", np.nan)) if proxy_summary else np.nan,
                "proxy_bbox_angle": float(proxy_summary.get("bbox_angle", np.nan)) if proxy_summary else np.nan,
                "proxy_geom_distance_scale_m": float(proxy_summary.get("geom_distance_scale_m", np.nan)) if proxy_summary else np.nan,
                "selection_source": selection_source,
                "predictor_status": predictor_status,
                "predictor_error": predictor_error,
            })

        reference_tx_idx = -1
        serving_source = "beam_oracle_anchor"
        if 0 <= int(preferred_serving_idx) < len(candidates):
            reference_tx_idx = int(preferred_serving_idx)
            serving_source = "sys_serving_bs"
        elif candidates:
            best_idx = -1
            best_gain = -np.inf
            for item in candidates:
                gain = item.get("oracle_beam_gain_db", np.nan)
                if np.isfinite(gain) and gain > best_gain:
                    best_gain = float(gain)
                    best_idx = int(item.get("tx_idx", -1))
            reference_tx_idx = best_idx

        reference_bs_name = ""
        reference_anchor_id = None
        selected_beam_index = -1
        selected_beam_gain_db = np.nan
        oracle_beam_index = -1
        oracle_beam_gain_db = np.nan
        predicted_beam_index = -1
        predicted_beam_confidence = np.nan
        oracle_in_topk = False
        beam_hit = False
        topk_indices: List[int] = []
        topk_probabilities: List[float] = []
        feature_vector: List[float] = []
        selection_source = "oracle"
        predictor_status_ref = str(rt.get("predictor_status", "inactive"))
        predictor_error_ref = str(rt.get("predictor_error", ""))
        feature_names_ref = list(feature_names)
        if 0 <= reference_tx_idx < len(self.cfg.bs_list):
            reference_bs_name = str(self.cfg.bs_list[reference_tx_idx].get("name", ""))
            reference_anchor_id = infer_anchor_id(self.cfg.bs_list[reference_tx_idx], reference_tx_idx)
        for item in candidates:
            if int(item.get("tx_idx", -1)) == reference_tx_idx:
                selected_beam_index = int(item.get("selected_beam_index", -1))
                selected_beam_gain_db = float(item.get("selected_beam_gain_db", np.nan))
                oracle_beam_index = int(item.get("oracle_beam_index", -1))
                oracle_beam_gain_db = float(item.get("oracle_beam_gain_db", np.nan))
                predicted_beam_index = int(item.get("predicted_beam_index", -1))
                predicted_beam_confidence = float(item.get("predicted_beam_confidence", np.nan))
                oracle_in_topk = bool(item.get("oracle_in_topk", False))
                beam_hit = bool(item.get("beam_hit", False))
                topk_indices = [int(v) for v in item.get("topk_indices", [])]
                topk_probabilities = [float(v) for v in item.get("topk_probabilities", [])]
                feature_names_ref = [str(v) for v in item.get("feature_names", feature_names)]
                feature_vector = [float(v) for v in item.get("feature_vector", [])]
                selection_source = str(item.get("selection_source", selection_source))
                predictor_status_ref = str(item.get("predictor_status", predictor_status_ref))
                predictor_error_ref = str(item.get("predictor_error", predictor_error_ref))
                break

        return {
            "enabled": True,
            "selection_mode": rt["selection_mode"],
            "codebook_type": rt["codebook_type"],
            "num_beams": num_beams,
            "manual_beam_index": int(rt["manual_beam_index"]),
            "predictor_available": bool(rt.get("predictor_available", False)),
            "predictor_status": predictor_status_ref,
            "predictor_error": predictor_error_ref,
            "feature_mode": feature_mode,
            "feature_names": feature_names_ref,
            "top_k": int(rt.get("top_k", 1)),
            "checkpoint_path": str(rt.get("checkpoint_path", "")),
            "reference_tx_idx": reference_tx_idx,
            "reference_bs_name": reference_bs_name,
            "reference_anchor_id": reference_anchor_id,
            "selected_beam_index": selected_beam_index,
            "selected_beam_gain_db": selected_beam_gain_db,
            "oracle_beam_index": oracle_beam_index,
            "oracle_beam_gain_db": oracle_beam_gain_db,
            "predicted_beam_index": predicted_beam_index,
            "predicted_beam_confidence": predicted_beam_confidence,
            "oracle_in_topk": bool(oracle_in_topk),
            "beam_hit": bool(beam_hit),
            "topk_indices": topk_indices,
            "topk_probabilities": topk_probabilities,
            "feature_vector": feature_vector,
            "selection_source": selection_source,
            "serving_source": serving_source,
            "dataset_export_enabled": bool(exporter.get("enabled", False)),
            "dataset_export_path": str(exporter.get("path", "")),
            "dataset_export_sample_count": int(exporter.get("sample_count", 0)),
            "dataset_export_status": str(exporter.get("status", "disabled")),
            "dataset_export_error": str(exporter.get("error", "")),
            "candidates": candidates,
        }

    def _run_sys_over_rt(self, paths) -> Dict[str, Any]:
        if self.sys_runtime is None:
            return {"enabled": False}

        rt = self.sys_runtime
        torch = rt["torch"]
        device = rt["device"]

        h_freq = paths.cfr(
            frequencies=rt["frequencies"],
            sampling_frequency=1.0 / float(rt["resource_grid"].ofdm_symbol_duration),
            num_time_steps=int(self.cfg.sys_num_ofdm_symbols),
            out_type="torch",
        )
        if hasattr(h_freq, "to"):
            h_freq = h_freq.to(device)

        if len(h_freq.shape) != 6:
            raise ValueError(f"Unexpected CFR tensor shape for SYS over RT: {tuple(h_freq.shape)}")

        # [num_rx=1, num_rx_ant, num_tx, num_tx_ant, num_ofdm_symbols, num_subcarriers]
        channel_gain_re = torch.mean(torch.abs(h_freq).pow(2).float(), dim=(1, 3))
        channel_gain_re = channel_gain_re[0, ...]
        num_candidates = min(int(channel_gain_re.shape[0]), int(rt["num_candidates"]))
        if num_candidates <= 0:
            return {"enabled": True, "num_candidates": 0}

        channel_gain_re = channel_gain_re[:num_candidates, ...]

        tx_power_watt = float(rt["dbm_to_watt"](float(self.cfg.sys_bs_tx_power_dbm)))
        noise_watt = max(float(rt["noise_watt"]), 1e-30)
        sinr_re = torch.clamp(channel_gain_re * tx_power_watt / noise_watt, min=0.0)
        rate_per_candidate = torch.mean(torch.log2(1.0 + sinr_re), dim=(1, 2))
        serving_idx = int(torch.argmax(rate_per_candidate).item())

        sinr_tensor = torch.permute(sinr_re, (1, 2, 0)).unsqueeze(0).unsqueeze(-1)
        num_allocated_re = torch.zeros([1, rt["num_candidates"]], dtype=torch.int32, device=device)
        num_allocated_re[0, serving_idx] = int(self.cfg.sys_num_subcarriers) * int(self.cfg.sys_num_ofdm_symbols)

        mcs_index = rt["olla"](
            num_allocated_re=num_allocated_re,
            sinr_eff=rt["sinr_eff_feedback"],
            mcs_table_index=int(self.cfg.sys_mcs_table_index),
            mcs_category=1,
            harq_feedback=rt["harq_feedback"],
        )
        num_decoded_bits, harq_feedback_out, sinr_eff_true, *_ = rt["phy_abs"](
            mcs_index,
            sinr=sinr_tensor,
            mcs_table_index=int(self.cfg.sys_mcs_table_index),
            mcs_category=1,
        )

        scheduled_mask = num_allocated_re > 0
        neg_one = -torch.ones_like(harq_feedback_out)
        zero_float = torch.zeros_like(sinr_eff_true)
        zero_int = torch.zeros_like(num_decoded_bits)
        rt["harq_feedback"] = torch.where(scheduled_mask, harq_feedback_out, neg_one)
        rt["sinr_eff_feedback"] = torch.where(scheduled_mask, sinr_eff_true, zero_float)
        rt["num_decoded_bits"] = torch.where(scheduled_mask, num_decoded_bits, zero_int)

        mod_order, coderate = rt["decode_mcs_index"](
            mcs_index,
            table_index=int(self.cfg.sys_mcs_table_index),
            is_pusch=False,
        )
        spectral_efficiency = mod_order.to(coderate.dtype) * coderate
        sinr_eff_db = rt["lin_to_db"](sinr_eff_true)

        result = {
            "enabled": True,
            "serving_idx": serving_idx,
            "num_candidates": num_candidates,
            "candidate_rate_bpshz": rate_per_candidate.detach().cpu().numpy().reshape(-1)[:num_candidates],
            "candidate_sinr_eff_db": sinr_eff_db.detach().cpu().numpy().reshape(-1)[:num_candidates],
            "candidate_mcs_index": mcs_index.detach().cpu().numpy().reshape(-1)[:num_candidates],
            "candidate_num_decoded_bits": num_decoded_bits.detach().cpu().numpy().reshape(-1)[:num_candidates],
            "candidate_tb_ok": harq_feedback_out.detach().cpu().numpy().reshape(-1)[:num_candidates],
            "candidate_spectral_efficiency": spectral_efficiency.detach().cpu().numpy().reshape(-1)[:num_candidates],
        }
        return result

    def _build_scene_and_solver(self) -> None:
        try:
            import drjit as dr
            import mitsuba as mi
        except Exception as e:
            raise ImportError(
                "Could not import drjit/mitsuba. Make sure the current environment has dependencies that match your Sionna RT version."
            ) from e

        mi.set_variant(self.cfg.mi_variant)
        try:
            dr.set_flag(dr.JitFlag.Debug, False)
        except Exception:
            pass

        try:
            from sionna.rt import load_scene, Transmitter, Receiver, PlanarArray, PathSolver, Camera
        except Exception as e:
            raise ImportError("Could not import sionna.rt. Activate the correct Sionna RT environment first.") from e

        if not self.cfg.scene_path or not os.path.exists(self.cfg.scene_path):
            raise FileNotFoundError(
                f"scene_path does not exist: {self.cfg.scene_path}\n"
                "Specify the scene XML file through the launch file or ROS parameters."
            )

        scene = load_scene(self.cfg.scene_path, merge_shapes=self.cfg.merge_shapes)
        scene.frequency = self.cfg.fc_hz

        scene.tx_array = PlanarArray(
            num_rows=self.cfg.tx_array_num_rows,
            num_cols=self.cfg.tx_array_num_cols,
            vertical_spacing=self.cfg.tx_array_vertical_spacing,
            horizontal_spacing=self.cfg.tx_array_horizontal_spacing,
            pattern=self.cfg.tx_array_pattern,
            polarization=self.cfg.tx_array_polarization,
        )
        scene.rx_array = PlanarArray(
            num_rows=self.cfg.rx_array_num_rows,
            num_cols=self.cfg.rx_array_num_cols,
            vertical_spacing=self.cfg.rx_array_vertical_spacing,
            horizontal_spacing=self.cfg.rx_array_horizontal_spacing,
            pattern=self.cfg.rx_array_pattern,
            polarization=self.cfg.rx_array_polarization,
        )

        for bs in self.cfg.bs_list:
            scene.add(Transmitter(name=bs["name"], position=bs["position"]))

        scene.add(Receiver(name="uav", position=[0.0, 0.0, 10.0]))

        bs_positions = np.array([bs["position"] for bs in self.cfg.bs_list], dtype=float)
        uav_init = np.array([0.0, 0.0, 10.0], dtype=float)
        all_positions = np.vstack([bs_positions, uav_init.reshape(1, 3)])
        scene_center = np.mean(all_positions, axis=0)
        scene_span = np.max(all_positions, axis=0) - np.min(all_positions, axis=0)
        span_xy = max(float(scene_span[0]), float(scene_span[1]), 50.0)
        cam_position = (
            float(scene_center[0]),
            float(scene_center[1] - 1.2 * span_xy),
            float(scene_center[2] + 2.0 * span_xy),
        )
        self.top_cam = Camera(position=cam_position)
        self.top_cam.look_at((float(scene_center[0]), float(scene_center[1]), float(scene_center[2])))

        solver = PathSolver()
        try:
            solver.loop_mode = "symbolic"
        except Exception:
            pass

        self.scene = scene
        self.solver = solver

    @staticmethod
    def squeeze_1x1_link_tensor(x):
        x = np.asarray(x)
        if x.ndim == 5:
            return x[:, 0, :, 0, :]
        if x.ndim == 3:
            return x
        raise ValueError(f"Unexpected link tensor shape: {x.shape}")

    @staticmethod
    def squeeze_1x1_interactions(x):
        x = np.asarray(x)
        if x.ndim == 6:
            return x[:, :, 0, :, 0, :]
        if x.ndim == 4:
            return x
        raise ValueError(f"Unexpected interactions shape: {x.shape}")

    @staticmethod
    def _infer_path_type_from_order(path_type: Optional[str], path_order: int) -> str:
        s = str(path_type).strip().upper() if path_type is not None else "UNKNOWN"
        if s not in {"", "UNKNOWN", "NONE", "NAN"}:
            return s
        if path_order == 0:
            return "LOS"
        if path_order > 0:
            return "NLOS"
        return "NO_PATH"

    @staticmethod
    def _weighted_mean_and_std(values: np.ndarray, weights: np.ndarray):
        values = np.asarray(values, dtype=float).reshape(-1)
        weights = np.asarray(weights, dtype=float).reshape(-1)

        mask = np.isfinite(values) & np.isfinite(weights) & (weights > 0.0)
        if not np.any(mask):
            return np.nan, np.nan

        values = values[mask]
        weights = weights[mask]

        w_sum = np.sum(weights)
        if w_sum <= 0.0:
            return np.nan, np.nan

        mean = np.sum(weights * values) / w_sum
        var = np.sum(weights * (values - mean) ** 2) / w_sum
        var = max(float(var), 0.0)
        return float(mean), float(np.sqrt(var))

    @staticmethod
    def _weighted_angle_std(values: np.ndarray, weights: np.ndarray, ref_angle: float):
        """
        Compute angular standard deviation around ref_angle to avoid +/-pi wrap jumps.
        """
        values = np.asarray(values, dtype=float).reshape(-1)
        weights = np.asarray(weights, dtype=float).reshape(-1)

        mask = np.isfinite(values) & np.isfinite(weights) & (weights > 0.0)
        if not np.any(mask) or not np.isfinite(ref_angle):
            return np.nan

        values = values[mask]
        weights = weights[mask]

        delta = np.angle(np.exp(1j * (values - ref_angle)))
        w_sum = np.sum(weights)
        if w_sum <= 0.0:
            return np.nan

        var = np.sum(weights * (delta ** 2)) / w_sum
        var = max(float(var), 0.0)
        return float(np.sqrt(var))

    @staticmethod
    def extract_best_path_for_link(paths, rx_idx: int = 0, tx_idx: int = 0):
        try:
            if hasattr(paths, "a"):
                if isinstance(paths.a, tuple) and len(paths.a) == 2:
                    a_real, a_imag = paths.a
                    a = to_numpy(a_real) + 1j * to_numpy(a_imag)
                else:
                    a = to_numpy(paths.a)
            else:
                a = None

            tau = to_numpy(paths.tau)
            doppler = to_numpy(paths.doppler)
            valid = to_numpy(paths.valid).astype(bool) if hasattr(paths, "valid") else None
            phi_r = to_numpy(paths.phi_r) if hasattr(paths, "phi_r") else None
            theta_r = to_numpy(paths.theta_r) if hasattr(paths, "theta_r") else None
            interactions = to_numpy(paths.interactions) if hasattr(paths, "interactions") else None
        except Exception as e:
            print(f"[extract] failed to access path tensors: {e}")
            return None

        try:
            tau = OfflineSionnaSimulator.squeeze_1x1_link_tensor(tau)
            doppler = OfflineSionnaSimulator.squeeze_1x1_link_tensor(doppler)
            if a is not None:
                a = OfflineSionnaSimulator.squeeze_1x1_link_tensor(a)

            if valid is not None:
                if valid.ndim == 5:
                    valid = valid[:, 0, :, 0, :]
                elif valid.ndim != 3:
                    raise ValueError(f"Unexpected valid shape: {valid.shape}")

            if phi_r is not None:
                phi_r = OfflineSionnaSimulator.squeeze_1x1_link_tensor(phi_r)
            if theta_r is not None:
                theta_r = OfflineSionnaSimulator.squeeze_1x1_link_tensor(theta_r)
            if interactions is not None:
                interactions = OfflineSionnaSimulator.squeeze_1x1_interactions(interactions)
        except Exception as e:
            print(f"[extract] tensor squeeze failed: {e}")
            return None

        if rx_idx >= tau.shape[0] or tx_idx >= tau.shape[1]:
            return None

        tau_link = np.asarray(tau[rx_idx, tx_idx]).reshape(-1)
        doppler_link = np.asarray(doppler[rx_idx, tx_idx]).reshape(-1)

        if a is not None:
            a_link = np.asarray(a[rx_idx, tx_idx]).reshape(-1)
            amp_link = np.abs(a_link)
            power_link = amp_link ** 2
        else:
            amp_link = np.ones_like(tau_link, dtype=float)
            power_link = np.ones_like(tau_link, dtype=float)

        if valid is not None:
            valid_link = np.asarray(valid[rx_idx, tx_idx]).astype(bool).reshape(-1)
        else:
            valid_link = np.isfinite(tau_link) & np.isfinite(doppler_link)

        aoa_phi_link = np.asarray(phi_r[rx_idx, tx_idx]).reshape(-1) if phi_r is not None else None
        aoa_theta_link = np.asarray(theta_r[rx_idx, tx_idx]).reshape(-1) if theta_r is not None else None

        min_len = min(len(tau_link), len(doppler_link), len(amp_link), len(power_link), len(valid_link))
        if aoa_phi_link is not None:
            min_len = min(min_len, len(aoa_phi_link))
        if aoa_theta_link is not None:
            min_len = min(min_len, len(aoa_theta_link))

        if min_len == 0:
            return None

        tau_link = tau_link[:min_len]
        doppler_link = doppler_link[:min_len]
        amp_link = amp_link[:min_len]
        power_link = power_link[:min_len]
        valid_link = valid_link[:min_len]
        if aoa_phi_link is not None:
            aoa_phi_link = aoa_phi_link[:min_len]
        if aoa_theta_link is not None:
            aoa_theta_link = aoa_theta_link[:min_len]

        valid_idx = np.where(valid_link)[0]
        if valid_idx.size == 0:
            return None

        best_local = int(valid_idx[np.argmax(power_link[valid_idx])])

        path_order = -1
        if interactions is not None and interactions.ndim == 4:
            try:
                inter_link = interactions[:, rx_idx, tx_idx, best_local].reshape(-1)
                path_order = int(np.sum(inter_link != 0))
            except Exception:
                path_order = -1

        path_type = OfflineSionnaSimulator._infer_path_type_from_order(None, path_order)
        aoa_theta = float(aoa_theta_link[best_local]) if aoa_theta_link is not None else np.nan
        aoa_phi = float(aoa_phi_link[best_local]) if aoa_phi_link is not None else np.nan
        amplitude_abs = float(amp_link[best_local])
        path_gain_linear = float(power_link[best_local])

        tau_valid = tau_link[valid_idx]
        doppler_valid = doppler_link[valid_idx]
        power_valid = power_link[valid_idx]

        _, tau_std_s = OfflineSionnaSimulator._weighted_mean_and_std(tau_valid, power_valid)
        _, doppler_std_hz = OfflineSionnaSimulator._weighted_mean_and_std(doppler_valid, power_valid)

        if aoa_phi_link is not None:
            aoa_phi_valid = aoa_phi_link[valid_idx]
            aoa_phi_std = OfflineSionnaSimulator._weighted_angle_std(
                aoa_phi_valid, power_valid, ref_angle=aoa_phi
            )
        else:
            aoa_phi_std = np.nan

        if aoa_theta_link is not None:
            aoa_theta_valid = aoa_theta_link[valid_idx]
            _, aoa_theta_std = OfflineSionnaSimulator._weighted_mean_and_std(
                aoa_theta_valid, power_valid
            )
        else:
            aoa_theta_std = np.nan

        return {
            "path_index": int(best_local),
            "amplitude_abs": amplitude_abs,
            "path_gain_linear": path_gain_linear,
            "tau_s": float(tau_link[best_local]),
            "tau_std_s": float(tau_std_s) if np.isfinite(tau_std_s) else np.nan,
            "doppler_hz": float(doppler_link[best_local]),
            "doppler_std_hz": float(doppler_std_hz) if np.isfinite(doppler_std_hz) else np.nan,
            "aoa_theta_r_rad": aoa_theta,
            "aoa_theta_std_rad": float(aoa_theta_std) if np.isfinite(aoa_theta_std) else np.nan,
            "aoa_phi_r_rad": aoa_phi,
            "aoa_phi_std_rad": float(aoa_phi_std) if np.isfinite(aoa_phi_std) else np.nan,
            "path_type": path_type,
            "path_order": int(path_order),
        }

    def _render_snapshot_preview(self, sample_idx: int, pos_xyz: np.ndarray, paths: Any) -> Optional[str]:
        if not self.cfg.enable_preview_render:
            return None
        try:
            os.makedirs(self.cfg.preview_output_dir, exist_ok=True)
            cam_position = (
                float(pos_xyz[0]),
                float(pos_xyz[1] - self.cfg.preview_camera_follow_y_offset_m),
                float(pos_xyz[2] + self.cfg.preview_camera_height_offset_m),
            )
            self.top_cam.position = cam_position
            self.top_cam.look_at((float(pos_xyz[0]), float(pos_xyz[1]), float(pos_xyz[2])))
            out_path = os.path.join(self.cfg.preview_output_dir, f"preview_{sample_idx:06d}.png")
            self.scene.render_to_file(
                camera=self.top_cam,
                filename=out_path,
                paths=paths,
                resolution=(self.cfg.preview_width, self.cfg.preview_height),
                show_devices=self.cfg.preview_show_devices,
                show_orientations=self.cfg.preview_show_orientations,
            )
            return out_path
        except Exception as e:
            print(f"[preview] sample={sample_idx:06d} render failed: {e}")
            return None

    def simulate_one_snapshot(self, pos_xyz: np.ndarray, vel_xyz: np.ndarray, rx_quat_xyzw: Optional[np.ndarray] = None) -> Any:
        try:
            self.scene.get("uav").position = pos_xyz.tolist()
        except Exception:
            self.scene.get("uav").position = pos_xyz
        try:
            self.scene.get("uav").velocity = vel_xyz.tolist()
        except Exception:
            self.scene.get("uav").velocity = vel_xyz
        rx_orientation = quaternion_xyzw_to_sionna_orientation(rx_quat_xyzw)
        try:
            self.scene.get("uav").orientation = list(rx_orientation)
        except Exception:
            self.scene.get("uav").orientation = rx_orientation

        paths = self.solver(
            self.scene,
            max_depth=self.cfg.max_depth,
            max_num_paths_per_src=self.cfg.max_num_paths_per_src,
            samples_per_src=self.cfg.samples_per_src,
            synthetic_array=self.cfg.synthetic_array,
            los=self.cfg.los,
            specular_reflection=self.cfg.specular_reflection,
            diffuse_reflection=self.cfg.diffuse_reflection,
            refraction=self.cfg.refraction,
            diffraction=self.cfg.diffraction,
            edge_diffraction=self.cfg.edge_diffraction,
            diffraction_lit_region=self.cfg.diffraction_lit_region,
            seed=self.cfg.seed,
        )
        return paths

    @staticmethod
    def _safe_path_points_for_link(paths, rx_idx: int, tx_idx: int, path_idx: int) -> List[np.ndarray]:
        candidates = []
        for attr in ("vertices", "points", "path_vertices"):
            if hasattr(paths, attr):
                try:
                    candidates.append((attr, to_numpy(getattr(paths, attr))))
                except Exception:
                    pass

        for _, verts in candidates:
            arr = np.asarray(verts)
            if arr.size == 0:
                continue
            try:
                if arr.shape[-1] != 3:
                    continue
                pts = None
                if arr.ndim >= 7:
                    pts = arr[:, rx_idx, 0, tx_idx, 0, path_idx, :]
                elif arr.ndim == 5:
                    pts = arr[:, rx_idx, tx_idx, path_idx, :]
                elif arr.ndim == 3:
                    pts = arr[:, path_idx, :]
                if pts is None:
                    continue
                out = []
                for p in np.asarray(pts):
                    p = np.asarray(p, dtype=float).reshape(-1)
                    if p.size >= 3 and np.all(np.isfinite(p[:3])):
                        out.append(p[:3].copy())
                if out:
                    return out
            except Exception:
                continue
        return []

    def extract_all_paths_for_link(self, paths, rx_idx: int = 0, tx_idx: int = 0, tx_position: Optional[np.ndarray] = None, rx_position: Optional[np.ndarray] = None):
        try:
            if hasattr(paths, "a"):
                if isinstance(paths.a, tuple) and len(paths.a) == 2:
                    a_real, a_imag = paths.a
                    a = to_numpy(a_real) + 1j * to_numpy(a_imag)
                else:
                    a = to_numpy(paths.a)
            else:
                a = None

            tau = to_numpy(paths.tau)
            doppler = to_numpy(paths.doppler)
            valid = to_numpy(paths.valid).astype(bool) if hasattr(paths, "valid") else None
            phi_r = to_numpy(paths.phi_r) if hasattr(paths, "phi_r") else None
            theta_r = to_numpy(paths.theta_r) if hasattr(paths, "theta_r") else None
            interactions = to_numpy(paths.interactions) if hasattr(paths, "interactions") else None
        except Exception as e:
            print(f"[extract-all] failed to access path tensors: {e}")
            return [], None

        try:
            tau = OfflineSionnaSimulator.squeeze_1x1_link_tensor(tau)
            doppler = OfflineSionnaSimulator.squeeze_1x1_link_tensor(doppler)
            if a is not None:
                a = OfflineSionnaSimulator.squeeze_1x1_link_tensor(a)

            if valid is not None:
                if valid.ndim == 5:
                    valid = valid[:, 0, :, 0, :]
                elif valid.ndim != 3:
                    raise ValueError(f"Unexpected valid shape: {valid.shape}")

            if phi_r is not None:
                phi_r = OfflineSionnaSimulator.squeeze_1x1_link_tensor(phi_r)
            if theta_r is not None:
                theta_r = OfflineSionnaSimulator.squeeze_1x1_link_tensor(theta_r)
            if interactions is not None:
                interactions = OfflineSionnaSimulator.squeeze_1x1_interactions(interactions)
        except Exception as e:
            print(f"[extract-all] tensor squeeze failed: {e}")
            return [], None

        if rx_idx >= tau.shape[0] or tx_idx >= tau.shape[1]:
            return [], None

        tau_link = np.asarray(tau[rx_idx, tx_idx]).reshape(-1)
        doppler_link = np.asarray(doppler[rx_idx, tx_idx]).reshape(-1)
        if a is not None:
            a_link = np.asarray(a[rx_idx, tx_idx]).reshape(-1)
            amp_link = np.abs(a_link)
            power_link = amp_link ** 2
        else:
            amp_link = np.ones_like(tau_link, dtype=float)
            power_link = np.ones_like(tau_link, dtype=float)

        if valid is not None:
            valid_link = np.asarray(valid[rx_idx, tx_idx]).astype(bool).reshape(-1)
        else:
            valid_link = np.isfinite(tau_link) & np.isfinite(doppler_link)

        aoa_phi_link = np.asarray(phi_r[rx_idx, tx_idx]).reshape(-1) if phi_r is not None else None
        aoa_theta_link = np.asarray(theta_r[rx_idx, tx_idx]).reshape(-1) if theta_r is not None else None

        min_len = min(len(tau_link), len(doppler_link), len(amp_link), len(power_link), len(valid_link))
        if aoa_phi_link is not None:
            min_len = min(min_len, len(aoa_phi_link))
        if aoa_theta_link is not None:
            min_len = min(min_len, len(aoa_theta_link))

        if min_len == 0:
            return [], None

        tau_link = tau_link[:min_len]
        doppler_link = doppler_link[:min_len]
        amp_link = amp_link[:min_len]
        power_link = power_link[:min_len]
        valid_link = valid_link[:min_len]
        if aoa_phi_link is not None:
            aoa_phi_link = aoa_phi_link[:min_len]
        if aoa_theta_link is not None:
            aoa_theta_link = aoa_theta_link[:min_len]

        valid_idx = np.where(valid_link)[0]
        if valid_idx.size == 0:
            return [], None

        tau_valid = tau_link[valid_idx]
        doppler_valid = doppler_link[valid_idx]
        power_valid = power_link[valid_idx]
        _, tau_std_s = OfflineSionnaSimulator._weighted_mean_and_std(tau_valid, power_valid)
        _, doppler_std_hz = OfflineSionnaSimulator._weighted_mean_and_std(doppler_valid, power_valid)

        path_payloads = []
        best_payload = None
        best_power = -np.inf
        for local_idx in valid_idx.tolist():
            path_order = -1
            if interactions is not None and interactions.ndim == 4:
                try:
                    inter_link = interactions[:, rx_idx, tx_idx, local_idx].reshape(-1)
                    path_order = int(np.sum(inter_link != 0))
                except Exception:
                    path_order = -1

            aoa_phi = float(aoa_phi_link[local_idx]) if aoa_phi_link is not None else np.nan
            aoa_theta = float(aoa_theta_link[local_idx]) if aoa_theta_link is not None else np.nan

            if aoa_phi_link is not None:
                aoa_phi_std = OfflineSionnaSimulator._weighted_angle_std(aoa_phi_link[valid_idx], power_valid, ref_angle=aoa_phi)
            else:
                aoa_phi_std = np.nan
            if aoa_theta_link is not None:
                _, aoa_theta_std = OfflineSionnaSimulator._weighted_mean_and_std(aoa_theta_link[valid_idx], power_valid)
            else:
                aoa_theta_std = np.nan

            path_gain_linear = float(power_link[local_idx])
            power_db = 10.0 * math.log10(max(path_gain_linear, 1e-30)) if np.isfinite(path_gain_linear) else np.nan
            path_points = self._safe_path_points_for_link(paths, rx_idx=rx_idx, tx_idx=tx_idx, path_idx=local_idx)

            payload = {
                "path_index": int(local_idx),
                "path_id": f"tx{tx_idx}_rx{rx_idx}_path{int(local_idx)}",
                "is_valid": True,
                "amplitude_abs": float(amp_link[local_idx]),
                "path_gain_linear": path_gain_linear,
                "power_db": float(power_db) if np.isfinite(power_db) else np.nan,
                "tau_s": float(tau_link[local_idx]),
                "tau_std_s": float(tau_std_s) if np.isfinite(tau_std_s) else np.nan,
                "doppler_hz": float(doppler_link[local_idx]),
                "doppler_std_hz": float(doppler_std_hz) if np.isfinite(doppler_std_hz) else np.nan,
                "aoa_az_rad": aoa_phi,
                "aoa_el_rad": aoa_theta,
                "aoa_az_std_rad": float(aoa_phi_std) if np.isfinite(aoa_phi_std) else np.nan,
                "aoa_el_std_rad": float(aoa_theta_std) if np.isfinite(aoa_theta_std) else np.nan,
                "path_type": OfflineSionnaSimulator._infer_path_type_from_order(None, path_order),
                "path_order": int(path_order),
                "tx_position": None if tx_position is None else np.asarray(tx_position, dtype=float).copy(),
                "rx_position": None if rx_position is None else np.asarray(rx_position, dtype=float).copy(),
                "path_points": path_points,
            }
            path_payloads.append(payload)
            if path_gain_linear > best_power:
                best_power = path_gain_linear
                best_payload = dict(payload)

        path_payloads.sort(key=lambda item: item.get("path_gain_linear", -np.inf), reverse=True)
        return path_payloads, best_payload

    def simulate_from_odometry_snapshot(
        self,
        sim_idx: int,
        odom_stamp_s: float,
        pos_xyz: np.ndarray,
        vel_xyz: np.ndarray,
        rx_quat_xyzw: Optional[np.ndarray] = None,
    ) -> Tuple[pd.DataFrame, List[Dict[str, Any]]]:
        rows: List[Dict[str, Any]] = []
        anchor_payloads: List[Dict[str, Any]] = []
        paths = self.simulate_one_snapshot(pos_xyz, vel_xyz, rx_quat_xyzw=rx_quat_xyzw)
        sys_summary = self._run_sys_over_rt(paths)
        preferred_serving_idx = int(sys_summary.get("serving_idx", -1)) if sys_summary.get("enabled") else -1
        beam_summary = self._run_beamforming_sweep(
            paths,
            preferred_serving_idx=preferred_serving_idx,
            rx_position=pos_xyz,
            rx_velocity=vel_xyz,
        )
        self._export_beam_training_samples(
            beam_summary=beam_summary,
            rx_position=pos_xyz,
            rx_velocity=vel_xyz,
            sim_idx=sim_idx,
            odom_stamp_s=odom_stamp_s,
        )
        if self.beam_dataset_exporter is not None:
            beam_summary["dataset_export_enabled"] = bool(self.beam_dataset_exporter.get("enabled", False))
            beam_summary["dataset_export_path"] = str(self.beam_dataset_exporter.get("path", ""))
            beam_summary["dataset_export_sample_count"] = int(self.beam_dataset_exporter.get("sample_count", 0))
            beam_summary["dataset_export_status"] = str(self.beam_dataset_exporter.get("status", "disabled"))
            beam_summary["dataset_export_error"] = str(self.beam_dataset_exporter.get("error", ""))
        beam_candidate_map = {
            int(item.get("tx_idx", -1)): item
            for item in beam_summary.get("candidates", [])
            if int(item.get("tx_idx", -1)) >= 0
        }
        preview_path = self._render_snapshot_preview(sim_idx, pos_xyz, paths)
        if preview_path is not None:
            print(f"[preview] saved: {preview_path}")

        serving_idx = int(sys_summary.get("serving_idx", -1)) if sys_summary.get("enabled") else -1
        candidate_rates = np.asarray(sys_summary.get("candidate_rate_bpshz", []), dtype=float).reshape(-1)
        candidate_sinr_db = np.asarray(sys_summary.get("candidate_sinr_eff_db", []), dtype=float).reshape(-1)
        candidate_mcs = np.asarray(sys_summary.get("candidate_mcs_index", []), dtype=float).reshape(-1)
        candidate_bits = np.asarray(sys_summary.get("candidate_num_decoded_bits", []), dtype=float).reshape(-1)
        candidate_tb_ok = np.asarray(sys_summary.get("candidate_tb_ok", []), dtype=float).reshape(-1)
        candidate_se = np.asarray(sys_summary.get("candidate_spectral_efficiency", []), dtype=float).reshape(-1)

        serving_bs_name = ""
        serving_anchor_id: Optional[int] = None
        if 0 <= serving_idx < len(self.cfg.bs_list):
            serving_bs_name = str(self.cfg.bs_list[serving_idx].get("name", ""))
            serving_anchor_id = infer_anchor_id(self.cfg.bs_list[serving_idx], serving_idx)

        for tx_idx, bs in enumerate(self.cfg.bs_list):
            anchor_id = infer_anchor_id(bs, tx_idx)
            tx_position = np.asarray(bs["position"], dtype=float)
            all_paths, best = self.extract_all_paths_for_link(
                paths, rx_idx=0, tx_idx=tx_idx, tx_position=tx_position, rx_position=pos_xyz
            )

            anchor_payloads.append({
                "anchor_id": anchor_id,
                "anchor_name": bs["name"],
                "anchor_position": tx_position.copy(),
                "paths": all_paths,
            })

            row = {
                "sim_idx": int(sim_idx),
                "odom_stamp_s": float(odom_stamp_s),
                "sim_wall_time_s": float(time.time()),
                "uav_true_x_m": float(pos_xyz[0]),
                "uav_true_y_m": float(pos_xyz[1]),
                "uav_true_z_m": float(pos_xyz[2]),
                "uav_true_vx_mps": float(vel_xyz[0]),
                "uav_true_vy_mps": float(vel_xyz[1]),
                "uav_true_vz_mps": float(vel_xyz[2]),
                "bs_name": bs["name"],
                "anchor_id": anchor_id,
                "bs_x_m": float(tx_position[0]),
                "bs_y_m": float(tx_position[1]),
                "bs_z_m": float(tx_position[2]),
                "num_paths": int(len(all_paths)),
                "path_index": -1,
                "amplitude_abs": np.nan,
                "path_gain_linear": np.nan,
                "power_db": np.nan,
                "tau_s": np.nan,
                "tau_std_s": np.nan,
                "doppler_hz": np.nan,
                "doppler_std_hz": np.nan,
                "aoa_theta_r_rad": np.nan,
                "aoa_theta_std_rad": np.nan,
                "aoa_phi_r_rad": np.nan,
                "aoa_phi_std_rad": np.nan,
                "path_type": "NO_PATH",
                "path_order": -1,
                "sys_enabled": bool(sys_summary.get("enabled", False)),
                "sys_candidate_rate_bpshz": np.nan,
                "sys_candidate_sinr_eff_db": np.nan,
                "sys_mcs_index": np.nan,
                "sys_num_decoded_bits": np.nan,
                "sys_tb_ok": np.nan,
                "sys_spectral_efficiency_bpshz": np.nan,
                "sys_is_serving_bs": bool(tx_idx == serving_idx),
                "sys_serving_bs_name": serving_bs_name,
                "sys_serving_anchor_id": serving_anchor_id if serving_anchor_id is not None else np.nan,
                "sys_bler_target": float(self.cfg.sys_bler_target) if self.cfg.enable_sys_integration else np.nan,
                "beam_enabled": bool(beam_summary.get("enabled", False)),
                "beam_selection_mode": beam_summary.get("selection_mode", ""),
                "beam_codebook_type": beam_summary.get("codebook_type", ""),
                "beam_num_beams": int(beam_summary.get("num_beams", 0)) if beam_summary.get("num_beams", None) is not None else np.nan,
                "beam_selected_index": np.nan,
                "beam_selected_gain_db": np.nan,
                "beam_oracle_index": np.nan,
                "beam_oracle_gain_db": np.nan,
                "beam_selected_anchor": bool(tx_idx == int(beam_summary.get("reference_tx_idx", -1))),
                "beam_manual_index": int(beam_summary.get("manual_beam_index", -1)) if beam_summary.get("manual_beam_index", None) is not None else np.nan,
                "beam_serving_source": str(beam_summary.get("serving_source", "")),
                "beam_predictor_available": bool(beam_summary.get("predictor_available", False)),
                "beam_predictor_status": str(beam_summary.get("predictor_status", "")),
                "beam_predictor_error": str(beam_summary.get("predictor_error", "")),
                "beam_feature_mode": str(beam_summary.get("feature_mode", "")),
                "beam_top_k": int(beam_summary.get("top_k", 0)) if beam_summary.get("top_k", None) is not None else np.nan,
                "beam_predicted_index": np.nan,
                "beam_predicted_confidence": np.nan,
                "beam_oracle_in_topk": np.nan,
                "beam_hit": np.nan,
                "beam_selection_source": "",
                "beam_topk_indices": "",
            }
            if best is not None:
                row.update({
                    "path_index": best.get("path_index", -1),
                    "amplitude_abs": best.get("amplitude_abs", np.nan),
                    "path_gain_linear": best.get("path_gain_linear", np.nan),
                    "power_db": best.get("power_db", np.nan),
                    "tau_s": best.get("tau_s", np.nan),
                    "tau_std_s": best.get("tau_std_s", np.nan),
                    "doppler_hz": best.get("doppler_hz", np.nan),
                    "doppler_std_hz": best.get("doppler_std_hz", np.nan),
                    "aoa_theta_r_rad": best.get("aoa_el_rad", np.nan),
                    "aoa_theta_std_rad": best.get("aoa_el_std_rad", np.nan),
                    "aoa_phi_r_rad": best.get("aoa_az_rad", np.nan),
                    "aoa_phi_std_rad": best.get("aoa_az_std_rad", np.nan),
                    "path_type": best.get("path_type", "NO_PATH"),
                    "path_order": best.get("path_order", -1),
                })
            if sys_summary.get("enabled") and tx_idx < candidate_rates.size:
                tb_ok_raw = candidate_tb_ok[tx_idx] if tx_idx < candidate_tb_ok.size else np.nan
                row.update({
                    "sys_candidate_rate_bpshz": float(candidate_rates[tx_idx]) if np.isfinite(candidate_rates[tx_idx]) else np.nan,
                    "sys_candidate_sinr_eff_db": float(candidate_sinr_db[tx_idx]) if tx_idx < candidate_sinr_db.size and np.isfinite(candidate_sinr_db[tx_idx]) else np.nan,
                    "sys_mcs_index": int(candidate_mcs[tx_idx]) if tx_idx < candidate_mcs.size and np.isfinite(candidate_mcs[tx_idx]) else np.nan,
                    "sys_num_decoded_bits": int(candidate_bits[tx_idx]) if tx_idx < candidate_bits.size and np.isfinite(candidate_bits[tx_idx]) else np.nan,
                    "sys_tb_ok": int(tb_ok_raw) if np.isfinite(tb_ok_raw) else np.nan,
                    "sys_spectral_efficiency_bpshz": float(candidate_se[tx_idx]) if tx_idx < candidate_se.size and np.isfinite(candidate_se[tx_idx]) else np.nan,
                })
            beam_candidate = beam_candidate_map.get(int(tx_idx))
            if beam_candidate is not None:
                row.update({
                    "beam_selected_index": int(beam_candidate.get("selected_beam_index", -1)) if beam_candidate.get("selected_beam_index", -1) is not None else np.nan,
                    "beam_selected_gain_db": float(beam_candidate.get("selected_beam_gain_db", np.nan)),
                    "beam_oracle_index": int(beam_candidate.get("oracle_beam_index", -1)) if beam_candidate.get("oracle_beam_index", -1) is not None else np.nan,
                    "beam_oracle_gain_db": float(beam_candidate.get("oracle_beam_gain_db", np.nan)),
                    "beam_selected_anchor": bool(tx_idx == int(beam_summary.get("reference_tx_idx", -1))),
                    "beam_manual_index": int(beam_summary.get("manual_beam_index", -1)) if beam_summary.get("manual_beam_index", None) is not None else np.nan,
                    "beam_serving_source": str(beam_summary.get("serving_source", "")),
                    "beam_predictor_available": bool(beam_summary.get("predictor_available", False)),
                    "beam_predictor_status": str(beam_candidate.get("predictor_status", beam_summary.get("predictor_status", ""))),
                    "beam_predictor_error": str(beam_candidate.get("predictor_error", beam_summary.get("predictor_error", ""))),
                    "beam_feature_mode": str(beam_summary.get("feature_mode", "")),
                    "beam_top_k": int(beam_summary.get("top_k", 0)) if beam_summary.get("top_k", None) is not None else np.nan,
                    "beam_predicted_index": int(beam_candidate.get("predicted_beam_index", -1)) if beam_candidate.get("predicted_beam_index", -1) is not None else np.nan,
                    "beam_predicted_confidence": float(beam_candidate.get("predicted_beam_confidence", np.nan)),
                    "beam_oracle_in_topk": int(bool(beam_candidate.get("oracle_in_topk", False))),
                    "beam_hit": int(bool(beam_candidate.get("beam_hit", False))),
                    "beam_selection_source": str(beam_candidate.get("selection_source", "")),
                    "beam_topk_indices": ",".join(str(int(v)) for v in beam_candidate.get("topk_indices", [])),
                })
            rows.append(row)

        if sys_summary.get("enabled"):
            serving_mcs = int(candidate_mcs[serving_idx]) if 0 <= serving_idx < candidate_mcs.size and np.isfinite(candidate_mcs[serving_idx]) else None
            serving_sinr = float(candidate_sinr_db[serving_idx]) if 0 <= serving_idx < candidate_sinr_db.size and np.isfinite(candidate_sinr_db[serving_idx]) else None
            serving_se = float(candidate_se[serving_idx]) if 0 <= serving_idx < candidate_se.size and np.isfinite(candidate_se[serving_idx]) else None
            tb_ok_value = None
            if 0 <= serving_idx < candidate_tb_ok.size and np.isfinite(candidate_tb_ok[serving_idx]):
                tb_ok_value = bool(int(candidate_tb_ok[serving_idx]) == 1)
            print(
                "[sys-log] " + json.dumps({
                    "enabled": True,
                    "scheduler_policy": "best_rate_single_uav",
                    "serving_idx": serving_idx,
                    "serving_bs_name": serving_bs_name,
                    "serving_anchor_id": serving_anchor_id,
                    "serving_mcs_index": serving_mcs,
                    "serving_sinr_eff_db": serving_sinr,
                    "serving_spectral_efficiency_bpshz": serving_se,
                    "serving_tb_ok": tb_ok_value,
                }, ensure_ascii=False)
            )

        print(
            "[beam-log] " + json.dumps({
                "enabled": bool(beam_summary.get("enabled", False)),
                "selection_mode": beam_summary.get("selection_mode"),
                "codebook_type": beam_summary.get("codebook_type"),
                "num_beams": beam_summary.get("num_beams"),
                "reference_bs_name": beam_summary.get("reference_bs_name"),
                "reference_anchor_id": beam_summary.get("reference_anchor_id"),
                "reference_tx_idx": beam_summary.get("reference_tx_idx"),
                "selected_beam_index": beam_summary.get("selected_beam_index"),
                "selected_beam_gain_db": beam_summary.get("selected_beam_gain_db"),
                "oracle_beam_index": beam_summary.get("oracle_beam_index"),
                "oracle_beam_gain_db": beam_summary.get("oracle_beam_gain_db"),
                "manual_beam_index": beam_summary.get("manual_beam_index"),
                "predictor_available": beam_summary.get("predictor_available"),
                "predictor_status": beam_summary.get("predictor_status"),
                "predictor_error": beam_summary.get("predictor_error"),
                "feature_mode": beam_summary.get("feature_mode"),
                "feature_names": beam_summary.get("feature_names"),
                "top_k": beam_summary.get("top_k"),
                "predicted_beam_index": beam_summary.get("predicted_beam_index"),
                "predicted_beam_confidence": beam_summary.get("predicted_beam_confidence"),
                "oracle_in_topk": beam_summary.get("oracle_in_topk"),
                "beam_hit": beam_summary.get("beam_hit"),
                "selection_source": beam_summary.get("selection_source"),
                "topk_indices": beam_summary.get("topk_indices"),
                "topk_probabilities": beam_summary.get("topk_probabilities"),
                "feature_vector": beam_summary.get("feature_vector"),
                "serving_source": beam_summary.get("serving_source"),
                "dataset_export_enabled": beam_summary.get("dataset_export_enabled"),
                "dataset_export_path": beam_summary.get("dataset_export_path"),
                "dataset_export_sample_count": beam_summary.get("dataset_export_sample_count"),
                "dataset_export_status": beam_summary.get("dataset_export_status"),
                "dataset_export_error": beam_summary.get("dataset_export_error"),
            }, ensure_ascii=False)
        )

        del paths
        return pd.DataFrame(rows), anchor_payloads


# =========================================================
# 2) Odometry subscription cache
# =========================================================
@dataclass
class OdomSnapshot:
    stamp_s: float
    pos_xyz: np.ndarray
    vel_xyz: np.ndarray
    recv_wall_time_s: float
    seq: int


class LatestOdometryBuffer:
    def __init__(self):
        self._lock = threading.Lock()
        self._latest: Optional[OdomSnapshot] = None
        self._msg_count = 0

    def update(self, stamp_s: float, pos_xyz: np.ndarray, vel_xyz: np.ndarray) -> None:
        with self._lock:
            self._msg_count += 1
            self._latest = OdomSnapshot(
                stamp_s=float(stamp_s),
                pos_xyz=np.asarray(pos_xyz, dtype=float).copy(),
                vel_xyz=np.asarray(vel_xyz, dtype=float).copy(),
                recv_wall_time_s=float(time.time()),
                seq=int(self._msg_count),
            )

    def latest(self) -> Optional[OdomSnapshot]:
        with self._lock:
            if self._latest is None:
                return None
            s = self._latest
            return OdomSnapshot(
                stamp_s=s.stamp_s,
                pos_xyz=s.pos_xyz.copy(),
                vel_xyz=s.vel_xyz.copy(),
                recv_wall_time_s=s.recv_wall_time_s,
                seq=s.seq,
            )


# =========================================================
# 3) Log output
# =========================================================
def log_measurement_rows(df: pd.DataFrame) -> None:
    if df is None or len(df) == 0:
        print("[sim-log] empty measurement rows")
        return

    for _, row in df.iterrows():
        payload = {
            "sim_idx": int(row["sim_idx"]),
            "odom_stamp_s": _safe_number(row["odom_stamp_s"]),
            "uav_pos_m": [
                _safe_number(row["uav_true_x_m"]),
                _safe_number(row["uav_true_y_m"]),
                _safe_number(row["uav_true_z_m"]),
            ],
            "uav_vel_mps": [
                _safe_number(row["uav_true_vx_mps"]),
                _safe_number(row["uav_true_vy_mps"]),
                _safe_number(row["uav_true_vz_mps"]),
            ],
            "bs_name": row["bs_name"],
            "anchor_id": int(row["anchor_id"]) if "anchor_id" in row else None,
            "tau_s": _safe_number(row["tau_s"]),
            "tau_std_s": _safe_number(row["tau_std_s"]),
            "doppler_hz": _safe_number(row["doppler_hz"]),
            "doppler_std_hz": _safe_number(row["doppler_std_hz"]),
            "aoa_theta_r_rad": _safe_number(row["aoa_theta_r_rad"]),
            "aoa_theta_std_rad": _safe_number(row["aoa_theta_std_rad"]),
            "aoa_phi_r_rad": _safe_number(row["aoa_phi_r_rad"]),
            "aoa_phi_std_rad": _safe_number(row["aoa_phi_std_rad"]),
            "path_type": row["path_type"],
            "path_order": int(row["path_order"]),
            "path_gain_linear": _safe_number(row["path_gain_linear"]),
            "power_db": _safe_number(row["power_db"]) if "power_db" in row else None,
            "num_paths": int(row["num_paths"]) if "num_paths" in row and _safe_number(row["num_paths"]) is not None else None,
            "sys_enabled": bool(row["sys_enabled"]) if "sys_enabled" in row and not pd.isna(row["sys_enabled"]) else None,
            "sys_candidate_rate_bpshz": _safe_number(row["sys_candidate_rate_bpshz"]) if "sys_candidate_rate_bpshz" in row else None,
            "sys_candidate_sinr_eff_db": _safe_number(row["sys_candidate_sinr_eff_db"]) if "sys_candidate_sinr_eff_db" in row else None,
            "sys_mcs_index": int(row["sys_mcs_index"]) if "sys_mcs_index" in row and _safe_number(row["sys_mcs_index"]) is not None else None,
            "sys_num_decoded_bits": int(row["sys_num_decoded_bits"]) if "sys_num_decoded_bits" in row and _safe_number(row["sys_num_decoded_bits"]) is not None else None,
            "sys_tb_ok": int(row["sys_tb_ok"]) if "sys_tb_ok" in row and _safe_number(row["sys_tb_ok"]) is not None else None,
            "sys_spectral_efficiency_bpshz": _safe_number(row["sys_spectral_efficiency_bpshz"]) if "sys_spectral_efficiency_bpshz" in row else None,
            "sys_is_serving_bs": bool(row["sys_is_serving_bs"]) if "sys_is_serving_bs" in row and not pd.isna(row["sys_is_serving_bs"]) else None,
            "sys_serving_bs_name": str(row["sys_serving_bs_name"]) if "sys_serving_bs_name" in row and not pd.isna(row["sys_serving_bs_name"]) else None,
        }
        print("[sim-log] " + json.dumps(payload, ensure_ascii=False))


# =========================================================
# 4) ROS parameters
# =========================================================
def require_ros_param(rospy, name: str) -> Any:
    value = rospy.get_param(name, None)
    if value is None or value == "":
        raise ValueError(f"Missing required ROS parameter: {name}")
    return value


def get_ros_runtime_params(rospy) -> Dict[str, Any]:
    params: Dict[str, Any] = {}

    # Required values
    params["scene_path"] = require_ros_param(rospy, "~scene_path")
    params["bs_list_json"] = require_ros_param(rospy, "~bs_list_json")

    # Runtime parameters
    params["fc"] = float(rospy.get_param("~fc", 3.5e9))
    params["pose_topic"] = str(rospy.get_param("~pose_topic", rospy.get_param("~odom_topic", "/odometry")))
    params["ros_to_map_matrix_json"] = str(rospy.get_param("~ros_to_map_matrix_json", ""))
    params["sim_hz"] = float(rospy.get_param("~sim_hz", 10.0))
    params["max_odom_staleness_s"] = float(rospy.get_param("~max_odom_staleness_s", 1.0))

    params["measurement_csv"] = str(rospy.get_param("~measurement_csv", "sionna_measurements_from_odom.csv"))
    params["reset_measurement_csv"] = ros_param_bool(rospy, "~reset_measurement_csv", True)

    # RF observation publishing
    params["rf_observation_topic"] = str(rospy.get_param("~rf_observation_topic", "/rf_observations"))
    params["sys_observation_topic"] = str(rospy.get_param("~sys_observation_topic", "/sys_observations"))
    params["beam_observation_topic"] = str(rospy.get_param("~beam_observation_topic", "/beam_observations"))
    params["beam_codebook_topic"] = str(rospy.get_param("~beam_codebook_topic", "/beam_codebook"))
    params["rf_frame_id"] = str(rospy.get_param("~rf_frame_id", "map"))

    # Sionna RT configuration
    params["mi_variant"] = str(rospy.get_param("~mi_variant", "cuda_ad_mono_polarized"))
    params["tx_array_num_rows"] = int(rospy.get_param("~tx_array_num_rows", 1))
    params["tx_array_num_cols"] = int(rospy.get_param("~tx_array_num_cols", 1))
    params["tx_array_vertical_spacing"] = float(rospy.get_param("~tx_array_vertical_spacing", 0.5))
    params["tx_array_horizontal_spacing"] = float(rospy.get_param("~tx_array_horizontal_spacing", 0.5))
    params["tx_array_pattern"] = str(rospy.get_param("~tx_array_pattern", "iso"))
    params["tx_array_polarization"] = str(rospy.get_param("~tx_array_polarization", "V"))
    params["rx_array_num_rows"] = int(rospy.get_param("~rx_array_num_rows", 1))
    params["rx_array_num_cols"] = int(rospy.get_param("~rx_array_num_cols", 1))
    params["rx_array_vertical_spacing"] = float(rospy.get_param("~rx_array_vertical_spacing", 0.5))
    params["rx_array_horizontal_spacing"] = float(rospy.get_param("~rx_array_horizontal_spacing", 0.5))
    params["rx_array_pattern"] = str(rospy.get_param("~rx_array_pattern", "iso"))
    params["rx_array_polarization"] = str(rospy.get_param("~rx_array_polarization", "V"))
    params["max_depth"] = int(rospy.get_param("~max_depth", 1))
    params["samples_per_src"] = int(rospy.get_param("~samples_per_src", 2000))
    params["max_num_paths_per_src"] = int(rospy.get_param("~max_num_paths_per_src", 64))
    params["synthetic_array"] = ros_param_bool(rospy, "~synthetic_array", True)
    params["merge_shapes"] = ros_param_bool(rospy, "~merge_shapes", False)
    params["enable_sys_integration"] = ros_param_bool(rospy, "~enable_sys_integration", False)
    params["enable_beamforming"] = ros_param_bool(rospy, "~enable_beamforming", False)
    params["beam_selection_mode"] = str(rospy.get_param("~beam_selection_mode", "exhaustive_sweep"))
    params["beam_codebook_type"] = str(rospy.get_param("~beam_codebook_type", "auto"))
    params["beam_codebook_num_beams"] = int(rospy.get_param("~beam_codebook_num_beams", 8))
    params["beam_oversampling_v"] = int(rospy.get_param("~beam_oversampling_v", 1))
    params["beam_oversampling_h"] = int(rospy.get_param("~beam_oversampling_h", 1))
    params["beam_manual_index"] = int(rospy.get_param("~beam_manual_index", 0))
    params["beam_normalize_power"] = ros_param_bool(rospy, "~beam_normalize_power", True)
    params["beam_codebook_file"] = str(rospy.get_param("~beam_codebook_file", ""))
    params["beam_model_checkpoint_path"] = str(rospy.get_param("~beam_model_checkpoint_path", ""))
    params["beam_feature_mode"] = str(rospy.get_param("~beam_feature_mode", DEFAULT_FEATURE_MODE))
    params["beam_top_k"] = int(rospy.get_param("~beam_top_k", 3))
    params["beam_export_training_dataset"] = ros_param_bool(rospy, "~beam_export_training_dataset", False)
    params["beam_training_dataset_path"] = str(rospy.get_param("~beam_training_dataset_path", "beam_training_samples.csv"))
    params["beam_training_reset_dataset_on_start"] = ros_param_bool(rospy, "~beam_training_reset_dataset_on_start", False)
    params["beam_training_output_checkpoint_path"] = str(rospy.get_param("~beam_training_output_checkpoint_path", "beam_trained_model.pt"))
    params["beam_training_epochs"] = int(rospy.get_param("~beam_training_epochs", 30))
    params["beam_training_batch_size"] = int(rospy.get_param("~beam_training_batch_size", 256))
    params["beam_training_learning_rate"] = float(rospy.get_param("~beam_training_learning_rate", 1e-3))
    params["beam_training_validation_split"] = float(rospy.get_param("~beam_training_validation_split", 0.2))
    params["beam_training_hidden_dim"] = int(rospy.get_param("~beam_training_hidden_dim", 512))
    params["sys_num_subcarriers"] = int(rospy.get_param("~sys_num_subcarriers", 128))
    params["sys_subcarrier_spacing_hz"] = float(rospy.get_param("~sys_subcarrier_spacing_hz", 30e3))
    params["sys_num_ofdm_symbols"] = int(rospy.get_param("~sys_num_ofdm_symbols", 12))
    params["sys_temperature_k"] = float(rospy.get_param("~sys_temperature_k", 294.0))
    params["sys_bler_target"] = float(rospy.get_param("~sys_bler_target", 0.1))
    params["sys_mcs_table_index"] = int(rospy.get_param("~sys_mcs_table_index", 1))
    params["sys_bs_tx_power_dbm"] = float(rospy.get_param("~sys_bs_tx_power_dbm", 10.0))
    params["los"] = ros_param_bool(rospy, "~los", True)
    params["specular_reflection"] = ros_param_bool(rospy, "~specular_reflection", True)
    params["diffuse_reflection"] = ros_param_bool(rospy, "~diffuse_reflection", False)
    params["refraction"] = ros_param_bool(rospy, "~refraction", False)
    params["diffraction"] = ros_param_bool(rospy, "~diffraction", False)
    params["edge_diffraction"] = ros_param_bool(rospy, "~edge_diffraction", False)
    params["diffraction_lit_region"] = ros_param_bool(rospy, "~diffraction_lit_region", False)

    # Visualization preview
    params["enable_preview_render"] = ros_param_bool(rospy, "~enable_preview_render", False)
    params["preview_output_dir"] = str(rospy.get_param("~preview_output_dir", "preview_frames"))
    params["preview_width"] = int(rospy.get_param("~preview_width", 960))
    params["preview_height"] = int(rospy.get_param("~preview_height", 720))
    params["preview_show_devices"] = ros_param_bool(rospy, "~preview_show_devices", True)
    params["preview_show_orientations"] = ros_param_bool(rospy, "~preview_show_orientations", False)
    params["preview_camera_follow_y_offset_m"] = float(rospy.get_param("~preview_camera_follow_y_offset_m", 80.0))
    params["preview_camera_height_offset_m"] = float(rospy.get_param("~preview_camera_height_offset_m", 120.0))

    return params


# =========================================================
# 5) DataFrame -> RfObservationArray
# =========================================================
def _make_point(Point, xyz: Optional[np.ndarray]):
    p = Point()
    if xyz is None:
        p.x = 0.0
        p.y = 0.0
        p.z = 0.0
        return p
    arr = np.asarray(xyz, dtype=float).reshape(-1)
    p.x = float(arr[0]) if arr.size > 0 and np.isfinite(arr[0]) else 0.0
    p.y = float(arr[1]) if arr.size > 1 and np.isfinite(arr[1]) else 0.0
    p.z = float(arr[2]) if arr.size > 2 and np.isfinite(arr[2]) else 0.0
    return p


def _make_vector3(Vector3, xyz: Optional[np.ndarray]):
    v = Vector3()
    if xyz is None:
        v.x = 0.0
        v.y = 0.0
        v.z = 0.0
        return v
    arr = np.asarray(xyz, dtype=float).reshape(-1)
    v.x = float(arr[0]) if arr.size > 0 and np.isfinite(arr[0]) else 0.0
    v.y = float(arr[1]) if arr.size > 1 and np.isfinite(arr[1]) else 0.0
    v.z = float(arr[2]) if arr.size > 2 and np.isfinite(arr[2]) else 0.0
    return v


def _safe_float_msg(value: Any) -> Optional[float]:
    try:
        f = float(value)
        if not np.isfinite(f):
            return None
        return float(f)
    except Exception:
        return None



def _safe_int_msg(value: Any) -> Optional[int]:
    f = _safe_float_msg(value)
    if f is None:
        return None
    try:
        return int(round(f))
    except Exception:
        return None



def _safe_bool_msg(value: Any) -> Optional[bool]:
    if isinstance(value, bool):
        return value
    if value is None:
        return None
    if isinstance(value, (int, float)):
        try:
            if not np.isfinite(float(value)):
                return None
        except Exception:
            return None
        return bool(int(round(float(value))))
    text = str(value).strip().lower()
    if text in {"1", "true", "yes", "on"}:
        return True
    if text in {"0", "false", "no", "off"}:
        return False
    return None



def _tb_status_from_value(value: Any) -> int:
    parsed = _safe_bool_msg(value)
    if parsed is None:
        return TB_STATUS_UNKNOWN
    return TB_STATUS_ACK if parsed else TB_STATUS_NACK



def _scheduler_policy_code(name: str) -> int:
    return SCHED_BEST_RATE_SINGLE_UAV if str(name).strip() == "best_rate_single_uav" else SCHED_UNKNOWN



def build_sys_snapshot_from_rows(rows: List[Dict[str, Any]], params: Dict[str, Any]) -> Dict[str, Any]:
    snapshot: Dict[str, Any] = {
        "enabled": bool(params.get("enable_sys_integration", False)),
        "scheduler_policy": "best_rate_single_uav",
        "sim_idx": -1,
        "odom_stamp_s": None,
        "num_candidates": 0,
        "serving": {
            "valid": False,
            "serving_index": -1,
            "anchor_id": -1,
            "anchor_name": "",
            "rate_bpshz": None,
            "sinr_eff_db": None,
            "mcs_index": None,
            "num_decoded_bits": None,
            "tb_status": TB_STATUS_UNKNOWN,
            "spectral_efficiency_bpshz": None,
        },
        "candidates": [],
    }
    if rows:
        first = rows[0]
        snapshot["sim_idx"] = int(_safe_int_msg(first.get("sim_idx")) or -1)
        snapshot["odom_stamp_s"] = _safe_float_msg(first.get("odom_stamp_s"))
    for idx, row in enumerate(rows or []):
        candidate = {
            "sim_idx": int(_safe_int_msg(row.get("sim_idx")) or snapshot["sim_idx"]),
            "odom_stamp_s": _safe_float_msg(row.get("odom_stamp_s")),
            "anchor_id": int(_safe_int_msg(row.get("anchor_id")) or -1),
            "anchor_name": str(row.get("bs_name", "")),
            "anchor_position": [
                _safe_float_msg(row.get("bs_x_m")) or 0.0,
                _safe_float_msg(row.get("bs_y_m")) or 0.0,
                _safe_float_msg(row.get("bs_z_m")) or 0.0,
            ],
            "is_serving": bool(_safe_bool_msg(row.get("sys_is_serving_bs")) or False),
            "num_paths": int(_safe_int_msg(row.get("num_paths")) or 0),
            "strongest_path_power_db": _safe_float_msg(row.get("power_db")),
            "strongest_path_type": str(row.get("path_type", "")),
            "strongest_path_order": int(_safe_int_msg(row.get("path_order")) or -1),
            "candidate_rate_bpshz": _safe_float_msg(row.get("sys_candidate_rate_bpshz")),
            "candidate_sinr_eff_db": _safe_float_msg(row.get("sys_candidate_sinr_eff_db")),
            "mcs_index": int(_safe_int_msg(row.get("sys_mcs_index")) or -1),
            "num_decoded_bits": int(_safe_int_msg(row.get("sys_num_decoded_bits")) or 0),
            "tb_status": _tb_status_from_value(row.get("sys_tb_ok")),
            "spectral_efficiency_bpshz": _safe_float_msg(row.get("sys_spectral_efficiency_bpshz")),
            "bler_target": _safe_float_msg(row.get("sys_bler_target")),
        }
        snapshot["candidates"].append(candidate)
        if candidate["is_serving"]:
            snapshot["serving"] = {
                "valid": True,
                "serving_index": idx,
                "anchor_id": candidate["anchor_id"],
                "anchor_name": candidate["anchor_name"],
                "rate_bpshz": candidate["candidate_rate_bpshz"],
                "sinr_eff_db": candidate["candidate_sinr_eff_db"],
                "mcs_index": candidate["mcs_index"],
                "num_decoded_bits": candidate["num_decoded_bits"],
                "tb_status": candidate["tb_status"],
                "spectral_efficiency_bpshz": candidate["spectral_efficiency_bpshz"],
            }
    snapshot["num_candidates"] = len(snapshot["candidates"])
    return snapshot



def build_sys_observation_msg(
    rospy,
    Header,
    Point,
    Vector3,
    SysObservationArray,
    SysConfig,
    SysServingLink,
    SysCandidateObservation,
    rows: List[Dict[str, Any]],
    params: Dict[str, Any],
    stamp_s: float,
    frame_id: str,
    rx_position: np.ndarray,
    rx_velocity: np.ndarray,
):
    snapshot = build_sys_snapshot_from_rows(rows, params)
    msg = SysObservationArray()
    msg.header = Header()
    msg.header.stamp = rospy.Time.from_sec(float(stamp_s))
    msg.header.frame_id = frame_id
    msg.rx_position = _make_point(Point, rx_position)
    msg.rx_velocity = _make_vector3(Vector3, rx_velocity)
    msg.sim_idx = max(int(snapshot.get("sim_idx", -1)), 0)
    msg.odom_stamp_s = float(snapshot.get("odom_stamp_s") or stamp_s)
    msg.enabled = bool(snapshot.get("enabled", False))
    msg.scheduler_policy = _scheduler_policy_code(snapshot.get("scheduler_policy", ""))
    msg.scheduler_policy_name = str(snapshot.get("scheduler_policy", ""))
    msg.num_candidates = int(snapshot.get("num_candidates", 0))

    config_msg = SysConfig()
    config_msg.enabled = bool(params.get("enable_sys_integration", False))
    config_msg.num_subcarriers = int(params.get("sys_num_subcarriers", 0))
    config_msg.subcarrier_spacing_hz = float(params.get("sys_subcarrier_spacing_hz", 0.0))
    config_msg.num_ofdm_symbols = int(params.get("sys_num_ofdm_symbols", 0))
    config_msg.temperature_k = float(params.get("sys_temperature_k", 0.0))
    config_msg.bler_target = float(params.get("sys_bler_target", 0.0))
    config_msg.mcs_table_index = int(params.get("sys_mcs_table_index", 0))
    config_msg.bs_tx_power_dbm = float(params.get("sys_bs_tx_power_dbm", 0.0))
    config_msg.carrier_frequency_hz = float(params.get("fc", 0.0))
    msg.config = config_msg

    serving = snapshot.get("serving", {})
    serving_msg = SysServingLink()
    serving_msg.valid = bool(serving.get("valid", False))
    serving_msg.serving_index = int(serving.get("serving_index", -1))
    serving_msg.anchor_id = int(serving.get("anchor_id", -1))
    serving_msg.anchor_name = str(serving.get("anchor_name", ""))
    serving_msg.rate_bpshz = float(serving.get("rate_bpshz") or 0.0)
    serving_msg.sinr_eff_db = float(serving.get("sinr_eff_db") or 0.0)
    serving_msg.mcs_index = int(serving.get("mcs_index", -1))
    serving_msg.num_decoded_bits = int(serving.get("num_decoded_bits", 0))
    serving_msg.tb_status = int(serving.get("tb_status", TB_STATUS_UNKNOWN))
    serving_msg.spectral_efficiency_bpshz = float(serving.get("spectral_efficiency_bpshz") or 0.0)
    msg.serving = serving_msg

    for candidate in snapshot.get("candidates", []):
        candidate_msg = SysCandidateObservation()
        candidate_msg.sim_idx = int(candidate.get("sim_idx", -1))
        candidate_msg.odom_stamp_s = float(candidate.get("odom_stamp_s") or 0.0)
        candidate_msg.anchor_id = int(candidate.get("anchor_id", -1))
        candidate_msg.anchor_name = str(candidate.get("anchor_name", ""))
        candidate_msg.anchor_position = _make_point(Point, candidate.get("anchor_position"))
        candidate_msg.is_serving = bool(candidate.get("is_serving", False))
        candidate_msg.num_paths = int(candidate.get("num_paths", 0))
        candidate_msg.strongest_path_power_db = float(candidate.get("strongest_path_power_db") or 0.0)
        candidate_msg.strongest_path_type = str(candidate.get("strongest_path_type", ""))
        candidate_msg.strongest_path_order = int(candidate.get("strongest_path_order", -1))
        candidate_msg.candidate_rate_bpshz = float(candidate.get("candidate_rate_bpshz") or 0.0)
        candidate_msg.candidate_sinr_eff_db = float(candidate.get("candidate_sinr_eff_db") or 0.0)
        candidate_msg.mcs_index = int(candidate.get("mcs_index", -1))
        candidate_msg.num_decoded_bits = int(candidate.get("num_decoded_bits", 0))
        candidate_msg.tb_status = int(candidate.get("tb_status", TB_STATUS_UNKNOWN))
        candidate_msg.spectral_efficiency_bpshz = float(candidate.get("spectral_efficiency_bpshz") or 0.0)
        candidate_msg.bler_target = float(candidate.get("bler_target") or 0.0)
        msg.candidates.append(candidate_msg)

    snapshot["config"] = {
        "enabled": config_msg.enabled,
        "num_subcarriers": config_msg.num_subcarriers,
        "subcarrier_spacing_hz": config_msg.subcarrier_spacing_hz,
        "num_ofdm_symbols": config_msg.num_ofdm_symbols,
        "temperature_k": config_msg.temperature_k,
        "bler_target": config_msg.bler_target,
        "mcs_table_index": config_msg.mcs_table_index,
        "bs_tx_power_dbm": config_msg.bs_tx_power_dbm,
        "carrier_frequency_hz": config_msg.carrier_frequency_hz,
    }
    return msg


def build_rf_observation_array_msg(
    rospy,
    Header,
    Point,
    Vector3,
    RfObservationArray,
    RfAnchorObservation,
    RfPathObservation,
    anchor_payloads: List[Dict[str, Any]],
    stamp_s: float,
    frame_id: str,
    rx_position: np.ndarray,
    rx_velocity: np.ndarray,
):
    msg = RfObservationArray()
    msg.header = Header()
    msg.header.stamp = rospy.Time.from_sec(float(stamp_s))
    msg.header.frame_id = frame_id
    msg.rx_position = _make_point(Point, rx_position)
    msg.rx_velocity = _make_vector3(Vector3, rx_velocity)

    for anchor_payload in anchor_payloads or []:
        anchor_msg = RfAnchorObservation()
        anchor_msg.anchor_id = int(anchor_payload.get("anchor_id", 0))
        anchor_msg.anchor_name = str(anchor_payload.get("anchor_name", ""))
        anchor_msg.anchor_position = _make_point(Point, anchor_payload.get("anchor_position"))

        for path_payload in anchor_payload.get("paths", []):
            path_msg = RfPathObservation()
            path_msg.path_type = path_type_str_to_uint8(path_payload.get("path_type"))
            path_msg.path_order = int(path_payload.get("path_order", 0))
            path_msg.path_index = int(path_payload.get("path_index", 0))
            path_msg.path_id = str(path_payload.get("path_id", f"anchor_{anchor_msg.anchor_id}_path_{path_msg.path_index}"))
            path_msg.is_valid = bool(path_payload.get("is_valid", False))

            amplitude_abs = _safe_number(path_payload.get("amplitude_abs"))
            path_gain_linear = _safe_number(path_payload.get("path_gain_linear"))
            power_db = _safe_number(path_payload.get("power_db"))
            tau_s = _safe_number(path_payload.get("tau_s"))
            tau_std_s = _safe_number(path_payload.get("tau_std_s"))
            doppler_hz = _safe_number(path_payload.get("doppler_hz"))
            doppler_std_hz = _safe_number(path_payload.get("doppler_std_hz"))
            aoa_az = _safe_number(path_payload.get("aoa_az_rad"))
            aoa_el = _safe_number(path_payload.get("aoa_el_rad"))
            aoa_az_std = _safe_number(path_payload.get("aoa_az_std_rad"))
            aoa_el_std = _safe_number(path_payload.get("aoa_el_std_rad"))

            path_msg.has_amplitude = amplitude_abs is not None
            path_msg.amplitude_abs = float(amplitude_abs) if amplitude_abs is not None else 0.0

            path_msg.has_path_gain = path_gain_linear is not None
            path_msg.path_gain_linear = float(path_gain_linear) if path_gain_linear is not None else 0.0

            path_msg.has_power_db = power_db is not None
            path_msg.power_db = float(power_db) if power_db is not None else 0.0

            path_msg.has_tau = tau_s is not None
            path_msg.tau_s = float(tau_s) if tau_s is not None else 0.0
            path_msg.tau_std_s = float(tau_std_s) if tau_std_s is not None else 0.0

            path_msg.has_doppler = doppler_hz is not None
            path_msg.doppler_hz = float(doppler_hz) if doppler_hz is not None else 0.0
            path_msg.doppler_std_hz = float(doppler_std_hz) if doppler_std_hz is not None else 0.0

            path_msg.has_aoa = (aoa_az is not None) and (aoa_el is not None)
            path_msg.aoa_az_rad = float(aoa_az) if aoa_az is not None else 0.0
            path_msg.aoa_el_rad = float(aoa_el) if aoa_el is not None else 0.0
            path_msg.aoa_az_std_rad = float(aoa_az_std) if aoa_az_std is not None else 0.0
            path_msg.aoa_el_std_rad = float(aoa_el_std) if aoa_el_std is not None else 0.0

            path_msg.tx_position = _make_point(Point, path_payload.get("tx_position"))
            path_msg.rx_position = _make_point(Point, path_payload.get("rx_position"))
            for xyz in path_payload.get("path_points", []):
                path_msg.path_points.append(_make_point(Point, xyz))

            anchor_msg.paths.append(path_msg)

        msg.anchors.append(anchor_msg)

    return msg


# =========================================================
# 6) Main flow
# =========================================================
def main() -> None:
    try:
        import rospy
        from std_msgs.msg import Header
        from geometry_msgs.msg import Point, Vector3
        from nav_msgs.msg import Odometry
        from geometry_msgs.msg import PoseStamped
        from rf_msgs.msg import RfObservationArray, RfAnchorObservation, RfPathObservation
    except Exception as e:
        raise ImportError(
            "This script requires a ROS1 Python environment with rospy, geometry_msgs/nav_msgs, and rf_msgs."
        ) from e

    try:
        from sionna_sys_msgs.msg import (
            SysCandidateObservation,
            SysConfig,
            SysObservationArray,
            SysServingLink,
        )
        has_sionna_sys_msgs = True
    except Exception:
        SysCandidateObservation = None
        SysConfig = None
        SysObservationArray = None
        SysServingLink = None
        has_sionna_sys_msgs = False

    try:
        from sionna_beam_msgs.msg import (
            BeamCodebook,
            BeamEntry,
            BeamObservation,
            BeamObservationArray,
        )
        has_sionna_beam_msgs = True
    except Exception:
        BeamCodebook = None
        BeamEntry = None
        BeamObservation = None
        BeamObservationArray = None
        has_sionna_beam_msgs = False

    rospy.init_node("sionna_uav_odometry_sim_only", anonymous=False)
    params = get_ros_runtime_params(rospy)
    ros_to_map_T = load_transform_matrix(params["ros_to_map_matrix_json"])
    ros_to_map_R = ros_to_map_T[:3, :3]

    rospy.loginfo("Loaded ROS params in private namespace (~)")
    rospy.loginfo("scene_path=%s", params["scene_path"])
    rospy.loginfo("bs_list_json=%s", params["bs_list_json"])
    rospy.loginfo("pose_topic=%s", params["pose_topic"])
    rospy.loginfo("ros_to_map_matrix_json=%s", params["ros_to_map_matrix_json"] if params["ros_to_map_matrix_json"] else "<identity>")
    rospy.loginfo("sim_hz=%.3f", params["sim_hz"])
    rospy.loginfo("rf_observation_topic=%s", params["rf_observation_topic"])
    rospy.loginfo("sys_observation_topic=%s", params["sys_observation_topic"])
    rospy.loginfo("beam_observation_topic=%s", params["beam_observation_topic"])
    rospy.loginfo("beam_codebook_topic=%s", params["beam_codebook_topic"])
    rospy.loginfo("rf_frame_id=%s", params["rf_frame_id"])
    rospy.loginfo("merge_shapes=%s", str(params["merge_shapes"]).lower())
    rospy.loginfo("enable_sys_integration=%s", str(params["enable_sys_integration"]).lower())
    rospy.loginfo("enable_beamforming=%s", str(params["enable_beamforming"]).lower())

    bs_list = load_bs_list_from_json(params["bs_list_json"])

    sim_cfg = SimulationConfig(
        scene_path=params["scene_path"],
        fc_hz=params["fc"],
        mi_variant=params["mi_variant"],
        tx_array_num_rows=params["tx_array_num_rows"],
        tx_array_num_cols=params["tx_array_num_cols"],
        tx_array_vertical_spacing=params["tx_array_vertical_spacing"],
        tx_array_horizontal_spacing=params["tx_array_horizontal_spacing"],
        tx_array_pattern=params["tx_array_pattern"],
        tx_array_polarization=params["tx_array_polarization"],
        rx_array_num_rows=params["rx_array_num_rows"],
        rx_array_num_cols=params["rx_array_num_cols"],
        rx_array_vertical_spacing=params["rx_array_vertical_spacing"],
        rx_array_horizontal_spacing=params["rx_array_horizontal_spacing"],
        rx_array_pattern=params["rx_array_pattern"],
        rx_array_polarization=params["rx_array_polarization"],
        max_depth=params["max_depth"],
        samples_per_src=params["samples_per_src"],
        max_num_paths_per_src=params["max_num_paths_per_src"],
        synthetic_array=params["synthetic_array"],
        merge_shapes=params["merge_shapes"],
        enable_sys_integration=params["enable_sys_integration"],
        enable_beamforming=params["enable_beamforming"],
        beam_selection_mode=params["beam_selection_mode"],
        beam_codebook_type=params["beam_codebook_type"],
        beam_codebook_num_beams=params["beam_codebook_num_beams"],
        beam_oversampling_v=params["beam_oversampling_v"],
        beam_oversampling_h=params["beam_oversampling_h"],
        beam_manual_index=params["beam_manual_index"],
        beam_normalize_power=params["beam_normalize_power"],
        beam_codebook_file=params["beam_codebook_file"],
        beam_model_checkpoint_path=params["beam_model_checkpoint_path"],
        beam_feature_mode=params["beam_feature_mode"],
        beam_top_k=params["beam_top_k"],
        beam_export_training_dataset=params["beam_export_training_dataset"],
        beam_training_dataset_path=params["beam_training_dataset_path"],
        beam_training_reset_dataset_on_start=params["beam_training_reset_dataset_on_start"],
        beam_training_output_checkpoint_path=params["beam_training_output_checkpoint_path"],
        beam_training_epochs=params["beam_training_epochs"],
        beam_training_batch_size=params["beam_training_batch_size"],
        beam_training_learning_rate=params["beam_training_learning_rate"],
        beam_training_validation_split=params["beam_training_validation_split"],
        beam_training_hidden_dim=params["beam_training_hidden_dim"],
        sys_num_subcarriers=params["sys_num_subcarriers"],
        sys_subcarrier_spacing_hz=params["sys_subcarrier_spacing_hz"],
        sys_num_ofdm_symbols=params["sys_num_ofdm_symbols"],
        sys_temperature_k=params["sys_temperature_k"],
        sys_bler_target=params["sys_bler_target"],
        sys_mcs_table_index=params["sys_mcs_table_index"],
        sys_bs_tx_power_dbm=params["sys_bs_tx_power_dbm"],
        los=params["los"],
        specular_reflection=params["specular_reflection"],
        diffuse_reflection=params["diffuse_reflection"],
        refraction=params["refraction"],
        diffraction=params["diffraction"],
        edge_diffraction=params["edge_diffraction"],
        diffraction_lit_region=params["diffraction_lit_region"],
        enable_preview_render=params["enable_preview_render"],
        preview_output_dir=params["preview_output_dir"],
        preview_width=params["preview_width"],
        preview_height=params["preview_height"],
        preview_show_devices=params["preview_show_devices"],
        preview_show_orientations=params["preview_show_orientations"],
        preview_camera_follow_y_offset_m=params["preview_camera_follow_y_offset_m"],
        preview_camera_height_offset_m=params["preview_camera_height_offset_m"],
        bs_list=bs_list,
    )
    simulator = OfflineSionnaSimulator(sim_cfg)

    if params["measurement_csv"] and params["reset_measurement_csv"]:
        reset_output_file(params["measurement_csv"])

    rf_obs_pub = rospy.Publisher(
        params["rf_observation_topic"],
        RfObservationArray,
        queue_size=10,
    )
    sys_obs_pub = None
    if has_sionna_sys_msgs:
        sys_obs_pub = rospy.Publisher(
            params["sys_observation_topic"],
            SysObservationArray,
            queue_size=10,
        )
    else:
        rospy.logwarn("sionna_sys_msgs not available; SYS observation topic will not be published.")

    beam_obs_pub = None
    beam_codebook_pub = None
    beam_codebook_payload = simulator.build_beam_codebook_payload()
    if has_sionna_beam_msgs:
        beam_obs_pub = rospy.Publisher(
            params["beam_observation_topic"],
            BeamObservationArray,
            queue_size=10,
        )
        beam_codebook_pub = rospy.Publisher(
            params["beam_codebook_topic"],
            BeamCodebook,
            queue_size=1,
            latch=True,
        )
        if beam_codebook_pub is not None and beam_codebook_payload is not None:
            codebook_msg = build_beam_codebook_msg(
                rospy=rospy,
                Header=Header,
                BeamCodebook=BeamCodebook,
                BeamEntry=BeamEntry,
                payload=beam_codebook_payload,
                stamp_s=float(rospy.Time.now().to_sec()),
                frame_id=params["rf_frame_id"],
            )
            if codebook_msg is not None:
                beam_codebook_pub.publish(codebook_msg)
    else:
        rospy.logwarn("sionna_beam_msgs not available; beam topics will not be published.")

    odom_buffer = LatestOdometryBuffer()

    def pose_callback(msg) -> None:
        try:
            stamp_s = float(msg.header.stamp.to_sec())
        except Exception:
            stamp_s = float(rospy.Time.now().to_sec())

        if isinstance(msg, Odometry):
            pos_xyz = np.array([
                msg.pose.pose.position.x,
                msg.pose.pose.position.y,
                msg.pose.pose.position.z,
            ], dtype=float)
            vel_xyz = np.array([
                msg.twist.twist.linear.x,
                msg.twist.twist.linear.y,
                msg.twist.twist.linear.z,
            ], dtype=float)
        else:
            pos_xyz = np.array([
                msg.pose.position.x,
                msg.pose.position.y,
                msg.pose.position.z,
            ], dtype=float)
            vel_xyz = np.zeros(3, dtype=float)
        pos_xyz = transform_position_by_matrix(ros_to_map_T, pos_xyz)
        vel_xyz = transform_vector_by_matrix(ros_to_map_R, vel_xyz)
        odom_buffer.update(stamp_s=stamp_s, pos_xyz=pos_xyz, vel_xyz=vel_xyz)

    pose_topic_type = rospy.get_param("~pose_topic_type", "")
    if pose_topic_type == "geometry_msgs/PoseStamped":
        rospy.Subscriber(params["pose_topic"], PoseStamped, pose_callback, queue_size=1)
    else:
        rospy.Subscriber(params["pose_topic"], Odometry, pose_callback, queue_size=1)
    rospy.loginfo("Subscribed pose topic: %s (type=%s)", params["pose_topic"], pose_topic_type if pose_topic_type else "nav_msgs/Odometry")
    rospy.loginfo("Simulation hz: %.3f", params["sim_hz"])
    rospy.loginfo("Each simulation uses the latest odometry only; no backlog replay.")

    rospy.loginfo("Waiting for first pose message on topic: %s", params["pose_topic"])
    warn_last_time = 0.0
    while not rospy.is_shutdown():
        snap = odom_buffer.latest()
        if snap is not None:
            rospy.loginfo(
                "Received first odometry message: seq=%d stamp=%.6f",
                snap.seq,
                snap.stamp_s,
            )
            break

        now_t = time.time()
        if now_t - warn_last_time > 5.0:
            rospy.logwarn("Still waiting for pose on topic: %s", params["pose_topic"])
            warn_last_time = now_t

        rospy.sleep(0.05)

    sim_idx = 0
    rate = rospy.Rate(params["sim_hz"])

    while not rospy.is_shutdown():
        snap = odom_buffer.latest()
        if snap is None:
            rospy.logwarn_throttle(2.0, "No odometry received yet.")
            rate.sleep()
            continue

        odom_age_s = time.time() - snap.recv_wall_time_s
        if odom_age_s > params["max_odom_staleness_s"]:
            rospy.logwarn(
                "Skip simulation because odometry is stale: age=%.3f s > %.3f s",
                odom_age_s,
                params["max_odom_staleness_s"],
            )
            rate.sleep()
            continue

        sim_idx += 1
        rospy.loginfo(
            "[sim] start sim_idx=%d using latest odom seq=%d stamp=%.6f pos=[%.3f, %.3f, %.3f] vel=[%.3f, %.3f, %.3f]",
            sim_idx,
            snap.seq,
            snap.stamp_s,
            snap.pos_xyz[0], snap.pos_xyz[1], snap.pos_xyz[2],
            snap.vel_xyz[0], snap.vel_xyz[1], snap.vel_xyz[2],
        )

        sim_t0 = time.time()
        try:
            meas_df, anchor_payloads = simulator.simulate_from_odometry_snapshot(
                sim_idx=sim_idx,
                odom_stamp_s=snap.stamp_s,
                pos_xyz=snap.pos_xyz,
                vel_xyz=snap.vel_xyz,
            )

            log_measurement_rows(meas_df)
            append_rows_csv(meas_df, params["measurement_csv"], label="measurement csv")

            rf_msg = build_rf_observation_array_msg(
                rospy=rospy,
                Header=Header,
                Point=Point,
                Vector3=Vector3,
                RfObservationArray=RfObservationArray,
                RfAnchorObservation=RfAnchorObservation,
                RfPathObservation=RfPathObservation,
                anchor_payloads=anchor_payloads,
                stamp_s=snap.stamp_s,
                frame_id=params["rf_frame_id"],
                rx_position=snap.pos_xyz,
                rx_velocity=snap.vel_xyz,
            )
            rf_obs_pub.publish(rf_msg)

            published_sys_candidates = 0
            if sys_obs_pub is not None:
                sys_msg = build_sys_observation_msg(
                    rospy=rospy,
                    Header=Header,
                    Point=Point,
                    Vector3=Vector3,
                    SysObservationArray=SysObservationArray,
                    SysConfig=SysConfig,
                    SysServingLink=SysServingLink,
                    SysCandidateObservation=SysCandidateObservation,
                    rows=meas_df.to_dict(orient="records"),
                    params=params,
                    stamp_s=snap.stamp_s,
                    frame_id=params["rf_frame_id"],
                    rx_position=snap.pos_xyz,
                    rx_velocity=snap.vel_xyz,
                )
                sys_obs_pub.publish(sys_msg)
                published_sys_candidates = len(sys_msg.candidates)

            published_beam_candidates = 0
            if beam_obs_pub is not None:
                beam_msg = build_beam_observation_array_msg(
                    rospy=rospy,
                    Header=Header,
                    BeamObservationArray=BeamObservationArray,
                    BeamObservation=BeamObservation,
                    rows=meas_df.to_dict(orient="records"),
                    params=params,
                    stamp_s=snap.stamp_s,
                    frame_id=params["rf_frame_id"],
                    codebook_payload=beam_codebook_payload,
                )
                if beam_msg is not None:
                    beam_obs_pub.publish(beam_msg)
                    published_beam_candidates = len(beam_msg.observations)

            published_paths = sum(len(anchor.paths) for anchor in rf_msg.anchors)
            rospy.loginfo(
                "[sim] done sim_idx=%d elapsed=%.3f s rows=%d anchors=%d published_paths=%d rf_topic=%s sys_candidates=%d sys_topic=%s beam_candidates=%d beam_topic=%s",
                sim_idx,
                time.time() - sim_t0,
                len(meas_df),
                len(rf_msg.anchors),
                published_paths,
                params["rf_observation_topic"],
                published_sys_candidates,
                params["sys_observation_topic"],
                published_beam_candidates,
                params["beam_observation_topic"],
            )
        except Exception as e:
            rospy.logerr("[sim] failed sim_idx=%d: %s", sim_idx, str(e))
            print(traceback.format_exc())

        # If simulation is slower than odometry, do not backfill history; use the latest value only.
        rate.sleep()


if __name__ == "__main__":
    main()
