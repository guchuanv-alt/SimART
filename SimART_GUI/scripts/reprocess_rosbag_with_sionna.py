#!/usr/bin/env python3
from __future__ import annotations

import argparse
import json
import os
import signal
import socket
import struct
import subprocess
import sys
import threading
from dataclasses import dataclass
from typing import Any, Dict, List, Optional, Tuple

import numpy as np

from sionna_beam_topic_utils import (
    build_beam_codebook_msg,
    build_beam_observation_array_msg,
)
from sionna_tf_frame_utils import (
    FrameTransformHelper,
    clamp_scene_ground,
    normalize_output_frame_key,
    rx_array_center_frame_id,
    scene_frame_id,
    transform_anchor_payloads,
    transform_measurement_rows,
)

try:
    import rosbag
    import rospy
    from geometry_msgs.msg import Point, Vector3
    from nav_msgs.msg import Odometry
    from geometry_msgs.msg import PoseStamped
    from std_msgs.msg import Header
    from rf_msgs.msg import RfObservationArray, RfAnchorObservation, RfPathObservation
except Exception as exc:
    raise ImportError("This script requires a ROS1 Python environment with rosbag/rospy and rf_msgs installed.") from exc

try:
    from sionna_sys_msgs.msg import SysObservationArray, SysConfig, SysServingLink, SysCandidateObservation
    HAS_SIONNA_SYS_MSGS = True
except Exception:
    SysObservationArray = SysConfig = SysServingLink = SysCandidateObservation = None
    HAS_SIONNA_SYS_MSGS = False

try:
    from sionna_beam_msgs.msg import BeamCodebook, BeamEntry, BeamObservation, BeamObservationArray
    HAS_SIONNA_BEAM_MSGS = True
except Exception:
    BeamCodebook = BeamEntry = BeamObservation = BeamObservationArray = None
    HAS_SIONNA_BEAM_MSGS = False


_STOP_REQUESTED = False
_PACKET_HEADER = struct.Struct("!Q")

PATH_TYPE_UNKNOWN = 0
PATH_TYPE_LOS = 1
PATH_TYPE_NLOS = 2
PATH_TYPE_NO_PATH = 3


GROUND_HEIGHT = 0.0


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


def _safe_number(v: Any):
    try:
        f = float(v)
        if np.isnan(f) or np.isinf(f):
            return None
        return f
    except Exception:
        return v


def _json_compatible(value: Any):
    if isinstance(value, dict):
        return {str(k): _json_compatible(v) for k, v in value.items()}
    if isinstance(value, np.ndarray):
        return [_json_compatible(v) for v in value.tolist()]
    if isinstance(value, np.generic):
        return _json_compatible(value.item())
    if isinstance(value, (list, tuple)):
        return [_json_compatible(v) for v in value]
    if isinstance(value, float):
        return value if np.isfinite(value) else None
    if isinstance(value, (int, bool, str)) or value is None:
        return value
    if hasattr(value, 'tolist'):
        try:
            return _json_compatible(value.tolist())
        except Exception:
            pass
    try:
        numeric = float(value)
        return numeric if np.isfinite(numeric) else None
    except Exception:
        return str(value)


def _packet_to_bytes(payload: Any) -> bytes:
    return json.dumps(_json_compatible(payload), ensure_ascii=False, separators=(",", ":")).encode("utf-8")


def _packet_from_bytes(blob: bytes) -> Any:
    return json.loads(blob.decode("utf-8"))


def _xyz_components(xyz: Any) -> Tuple[float, float, float]:
    if xyz is None:
        return 0.0, 0.0, 0.0
    try:
        if isinstance(xyz, np.ndarray):
            values = xyz.reshape(-1).tolist()
        elif isinstance(xyz, (list, tuple)):
            values = list(xyz)
        else:
            values = [getattr(xyz, "x", 0.0), getattr(xyz, "y", 0.0), getattr(xyz, "z", 0.0)]
    except Exception:
        values = []
    out = []
    for idx in range(3):
        value = values[idx] if idx < len(values) else 0.0
        try:
            f = float(value)
            out.append(f if np.isfinite(f) else 0.0)
        except Exception:
            out.append(0.0)
    return float(out[0]), float(out[1]), float(out[2])



def path_type_str_to_uint8(path_type: Any) -> int:
    s = str(path_type).strip().upper() if path_type is not None else ""
    if s == "LOS":
        return PATH_TYPE_LOS
    if s == "NLOS":
        return PATH_TYPE_NLOS
    if s == "NO_PATH":
        return PATH_TYPE_NO_PATH
    return PATH_TYPE_UNKNOWN



def _make_point(Point, xyz: Optional[np.ndarray]):
    p = Point()
    x, y, z = _xyz_components(xyz)
    p.x = x
    p.y = y
    p.z = z
    return p



def _make_vector3(Vector3, xyz: Optional[np.ndarray]):
    v = Vector3()
    x, y, z = _xyz_components(xyz)
    v.x = x
    v.y = y
    v.z = z
    return v



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



def _handle_stop(signum, _frame):
    global _STOP_REQUESTED
    _STOP_REQUESTED = True
    print(f"[stop] received signal {signum}; will stop after the current step and keep partial output", flush=True)


signal.signal(signal.SIGINT, _handle_stop)
signal.signal(signal.SIGTERM, _handle_stop)


@dataclass
class PoseSnapshot:
    bag_time_s: float
    stamp_s: float
    uav_pos_xyz: np.ndarray
    uav_quat_xyzw: np.ndarray
    pos_xyz: np.ndarray
    rx_quat_xyzw: np.ndarray
    vel_xyz: np.ndarray
    source_frame_id: str


@dataclass
class SimulatedBagMessage:
    topic: str
    bag_time_s: float
    stamp_s: float
    msg: Any


class WorkerSession:
    def __init__(self, proc: subprocess.Popen, sock: socket.socket, log_thread: threading.Thread, hello: Optional[Dict[str, Any]] = None):
        self.proc = proc
        self.sock = sock
        self.log_thread = log_thread
        self.hello = hello or {}



def _bool_arg(text: str) -> bool:
    return str(text).strip().lower() in {"1", "true", "yes", "on"}



def _header_stamp_or_fallback(msg: Any, bag_time_s: float) -> float:
    try:
        stamp = float(msg.header.stamp.to_sec())
        if np.isfinite(stamp) and stamp > 0.0:
            return stamp
    except Exception:
        pass
    return float(bag_time_s)



def _extract_pose_snapshot(msg: Any, bag_time_s: float, frame_helper: FrameTransformHelper) -> PoseSnapshot:
    stamp_s = _header_stamp_or_fallback(msg, bag_time_s)
    msg_type = str(getattr(msg, "_type", "") or "")
    is_odom = msg_type == "nav_msgs/Odometry" or isinstance(msg, Odometry)
    is_pose_stamped = msg_type == "geometry_msgs/PoseStamped" or isinstance(msg, PoseStamped)

    if not (is_odom or is_pose_stamped):
        # rosbag may deserialize messages into temporary generated classes such as
        # tmpxxxx._nav_msgs__Odometry. Fall back to structural checks so bag replay
        # still works even when isinstance(...) fails.
        try:
            is_odom = all([
                hasattr(msg, "pose"),
                hasattr(msg.pose, "pose"),
                hasattr(msg.pose.pose, "position"),
                hasattr(msg, "twist"),
                hasattr(msg.twist, "twist"),
                hasattr(msg.twist.twist, "linear"),
            ])
        except Exception:
            is_odom = False
        if not is_odom:
            try:
                is_pose_stamped = all([
                    hasattr(msg, "pose"),
                    hasattr(msg.pose, "position"),
                ])
            except Exception:
                is_pose_stamped = False

    if is_odom:
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
        quat_xyzw = np.array([
            msg.pose.pose.orientation.x,
            msg.pose.pose.orientation.y,
            msg.pose.pose.orientation.z,
            msg.pose.pose.orientation.w,
        ], dtype=float)
    elif is_pose_stamped:
        pos_xyz = np.array([
            msg.pose.position.x,
            msg.pose.position.y,
            msg.pose.position.z,
        ], dtype=float)
        vel_xyz = np.zeros(3, dtype=float)
        quat_xyzw = np.array([
            msg.pose.orientation.x,
            msg.pose.orientation.y,
            msg.pose.orientation.z,
            msg.pose.orientation.w,
        ], dtype=float)
    else:
        raise TypeError(
            f"Unsupported pose message type: {type(msg)!r}, _type={msg_type!r}"
        )
    if not np.all(np.isfinite(quat_xyzw)) or float(np.linalg.norm(quat_xyzw)) < 1e-9:
        quat_xyzw = np.array([0.0, 0.0, 0.0, 1.0], dtype=float)
    else:
        quat_xyzw = quat_xyzw / float(np.linalg.norm(quat_xyzw))
    source_frame_id = str(getattr(getattr(msg, "header", None), "frame_id", "") or "").strip()
    if source_frame_id:
        frame_helper.set_ros_frame_id(source_frame_id)
    resolved_source_frame = source_frame_id or frame_helper.resolved_ros_frame_id()
    scene_uav_pos_xyz = clamp_scene_ground(
        frame_helper.transform_point(pos_xyz, resolved_source_frame, scene_frame_id()),
        ground_height=GROUND_HEIGHT,
    )
    scene_vel_xyz = frame_helper.transform_vector(vel_xyz, resolved_source_frame, scene_frame_id())
    scene_quat_xyzw = frame_helper.transform_quaternion(quat_xyzw, resolved_source_frame, scene_frame_id())
    frame_helper.set_runtime_uav_pose_scene(scene_uav_pos_xyz, scene_quat_xyzw)
    scene_rx_pos_xyz = clamp_scene_ground(
        frame_helper.transform_point([0.0, 0.0, 0.0], rx_array_center_frame_id(), scene_frame_id()),
        ground_height=GROUND_HEIGHT,
    )
    scene_rx_quat_xyzw = frame_helper.transform_quaternion([0.0, 0.0, 0.0, 1.0], rx_array_center_frame_id(), scene_frame_id())
    return PoseSnapshot(
        bag_time_s=float(bag_time_s),
        stamp_s=stamp_s,
        uav_pos_xyz=scene_uav_pos_xyz,
        uav_quat_xyzw=scene_quat_xyzw,
        pos_xyz=scene_rx_pos_xyz,
        rx_quat_xyzw=scene_rx_quat_xyzw,
        vel_xyz=scene_vel_xyz,
        source_frame_id=resolved_source_frame,
    )



def _progress(value: float,
              message: str,
              *,
              phase: str = "",
              current: Optional[int] = None,
              total: Optional[int] = None) -> None:
    clipped = max(0.0, min(100.0, float(value)))
    payload: Dict[str, Any] = {
        "phase": str(phase or ""),
        "percent": clipped,
        "message": str(message),
    }
    if current is not None:
        payload["current"] = int(current)
    if total is not None:
        payload["total"] = int(total)
    print(f"[progress-json] {json.dumps(payload, ensure_ascii=False, separators=(',', ':'))}", flush=True)



TB_STATUS_UNKNOWN = 0
TB_STATUS_ACK = 1
TB_STATUS_NACK = 2
SCHED_UNKNOWN = 0
SCHED_BEST_RATE_SINGLE_UAV = 1


def _safe_float_msg(value: Any) -> Optional[float]:
    try:
        f = float(value)
        return f if np.isfinite(f) else None
    except Exception:
        return None


def _safe_int_msg(value: Any) -> Optional[int]:
    try:
        return int(value)
    except Exception:
        try:
            f = float(value)
            if not np.isfinite(f):
                return None
            return int(round(f))
        except Exception:
            return None


def _safe_bool_msg(value: Any) -> Optional[bool]:
    if isinstance(value, bool):
        return value
    if value is None:
        return None
    if isinstance(value, (int, float, np.integer, np.floating)):
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


def build_sys_snapshot_from_rows(rows: List[Dict[str, Any]], args: argparse.Namespace) -> Dict[str, Any]:
    snapshot: Dict[str, Any] = {
        "enabled": bool(getattr(args, "enable_sys_integration", False)),
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
    rows: List[Dict[str, Any]],
    args: argparse.Namespace,
    stamp_s: float,
    frame_id: str,
    rx_position: np.ndarray,
    rx_velocity: np.ndarray,
):
    if not HAS_SIONNA_SYS_MSGS:
        raise RuntimeError("sionna_sys_msgs is required to write SYS observations")
    snapshot = build_sys_snapshot_from_rows(rows, args)
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
    config_msg.enabled = bool(getattr(args, "enable_sys_integration", False))
    config_msg.num_subcarriers = int(getattr(args, "sys_num_subcarriers", 0))
    config_msg.subcarrier_spacing_hz = float(getattr(args, "sys_subcarrier_spacing_hz", 0.0))
    config_msg.num_ofdm_symbols = int(getattr(args, "sys_num_ofdm_symbols", 0))
    config_msg.temperature_k = float(getattr(args, "sys_temperature_k", 0.0))
    config_msg.bler_target = float(getattr(args, "sys_bler_target", 0.0))
    config_msg.mcs_table_index = int(getattr(args, "sys_mcs_table_index", 0))
    config_msg.bs_tx_power_dbm = float(getattr(args, "sys_bs_tx_power_dbm", 0.0))
    config_msg.carrier_frequency_hz = float(getattr(args, "fc_hz", 0.0))
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
    return msg


def _send_packet(sock: socket.socket, payload: Any) -> None:
    blob = _packet_to_bytes(payload)
    sock.sendall(_PACKET_HEADER.pack(len(blob)))
    sock.sendall(blob)



def _recv_exact(sock: socket.socket, size: int) -> bytes:
    chunks: List[bytes] = []
    remaining = int(size)
    while remaining > 0:
        chunk = sock.recv(remaining)
        if not chunk:
            raise ConnectionError("Socket closed while receiving data")
        chunks.append(chunk)
        remaining -= len(chunk)
    return b"".join(chunks)



def _recv_packet(sock: socket.socket) -> Any:
    header = _recv_exact(sock, _PACKET_HEADER.size)
    (size,) = _PACKET_HEADER.unpack(header)
    blob = _recv_exact(sock, int(size))
    return _packet_from_bytes(blob)



def _forward_worker_logs(proc: subprocess.Popen) -> None:
    try:
        if proc.stdout is None:
            return
        for raw_line in proc.stdout:
            line = raw_line.rstrip("\n")
            if line:
                print(f"[worker] {line}", flush=True)
    except Exception as exc:
        print(f"[worker-log-error] {exc}", flush=True)



def _start_worker(args: argparse.Namespace) -> WorkerSession:
    server = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    server.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    server.bind(("127.0.0.1", 0))
    server.listen(1)
    server.settimeout(float(args.worker_timeout_s))
    host, port = server.getsockname()

    worker_cmd = [
        args.sionna_python,
        args.worker_script,
        "--host", str(host),
        "--port", str(port),
        "--scene-path", args.scene_path,
        "--bs-list-json", args.bs_list_json,
        "--fc-hz", str(args.fc_hz),
        "--mi-variant", args.mi_variant,
        "--tx-array-num-rows", str(args.tx_array_num_rows),
        "--tx-array-num-cols", str(args.tx_array_num_cols),
        "--tx-array-vertical-spacing", str(args.tx_array_vertical_spacing),
        "--tx-array-horizontal-spacing", str(args.tx_array_horizontal_spacing),
        "--tx-array-pattern", args.tx_array_pattern,
        "--tx-array-polarization", args.tx_array_polarization,
        "--rx-array-num-rows", str(args.rx_array_num_rows),
        "--rx-array-num-cols", str(args.rx_array_num_cols),
        "--rx-array-vertical-spacing", str(args.rx_array_vertical_spacing),
        "--rx-array-horizontal-spacing", str(args.rx_array_horizontal_spacing),
        "--rx-array-pattern", args.rx_array_pattern,
        "--rx-array-polarization", args.rx_array_polarization,
        "--max-depth", str(args.max_depth),
        "--samples-per-src", str(args.samples_per_src),
        "--max-num-paths-per-src", str(args.max_num_paths_per_src),
        "--synthetic-array", "true" if args.synthetic_array else "false",
        "--merge-shapes", "true" if args.merge_shapes else "false",
        "--enable-sys-integration", "true" if args.enable_sys_integration else "false",
        "--enable-beamforming", "true" if args.enable_beamforming else "false",
        "--beam-selection-mode", str(args.beam_selection_mode),
        "--beam-codebook-type", str(args.beam_codebook_type),
        "--beam-codebook-num-beams", str(args.beam_codebook_num_beams),
        "--beam-oversampling-v", str(args.beam_oversampling_v),
        "--beam-oversampling-h", str(args.beam_oversampling_h),
        "--beam-manual-index", str(args.beam_manual_index),
        "--beam-normalize-power", "true" if args.beam_normalize_power else "false",
        "--beam-codebook-file", str(args.beam_codebook_file),
        "--beam-model-checkpoint-path", str(args.beam_model_checkpoint_path),
        "--beam-feature-mode", str(args.beam_feature_mode),
        "--beam-top-k", str(args.beam_top_k),
        "--sys-num-subcarriers", str(args.sys_num_subcarriers),
        "--sys-subcarrier-spacing-hz", str(args.sys_subcarrier_spacing_hz),
        "--sys-num-ofdm-symbols", str(args.sys_num_ofdm_symbols),
        "--sys-temperature-k", str(args.sys_temperature_k),
        "--sys-bler-target", str(args.sys_bler_target),
        "--sys-mcs-table-index", str(args.sys_mcs_table_index),
        "--sys-bs-tx-power-dbm", str(args.sys_bs_tx_power_dbm),
        "--los", "true" if args.los else "false",
        "--specular-reflection", "true" if args.specular_reflection else "false",
        "--diffuse-reflection", "true" if args.diffuse_reflection else "false",
        "--refraction", "true" if args.refraction else "false",
        "--diffraction", "true" if args.diffraction else "false",
        "--edge-diffraction", "true" if args.edge_diffraction else "false",
        "--diffraction-lit-region", "true" if args.diffraction_lit_region else "false",
    ]
    print("[info] starting socket Sionna worker: " + " ".join(worker_cmd), flush=True)
    proc = subprocess.Popen(
        worker_cmd,
        stdout=subprocess.PIPE,
        stderr=subprocess.STDOUT,
        text=True,
        bufsize=1,
    )
    log_thread = threading.Thread(target=_forward_worker_logs, args=(proc,), daemon=True)
    log_thread.start()

    try:
        conn, _addr = server.accept()
    except Exception:
        proc.poll()
        if proc.returncode is not None:
            raise RuntimeError(f"Sionna worker exited before opening the socket (exitCode={proc.returncode})")
        raise
    finally:
        server.close()

    try:
        hello = _recv_packet(conn)
    except Exception:
        conn.close()
        proc.poll()
        if proc.returncode is not None:
            raise RuntimeError(f"Sionna worker exited before initialization completed (exitCode={proc.returncode})")
        raise

    if not isinstance(hello, dict) or hello.get("kind") != "hello":
        conn.close()
        raise RuntimeError(f"Unexpected worker handshake: {hello!r}")
    print(f"[info] socket worker ready: {hello.get('message', '')}".strip(), flush=True)
    return WorkerSession(proc=proc, sock=conn, log_thread=log_thread, hello=hello)



def _stop_worker(session: Optional[WorkerSession], wait_s: float = 5.0) -> None:
    if session is None:
        return
    try:
        try:
            _send_packet(session.sock, {"kind": "stop"})
        except Exception:
            pass
        try:
            session.sock.shutdown(socket.SHUT_RDWR)
        except Exception:
            pass
        try:
            session.sock.close()
        except Exception:
            pass
        if session.proc.poll() is None:
            try:
                session.proc.wait(timeout=wait_s)
            except subprocess.TimeoutExpired:
                session.proc.terminate()
                try:
                    session.proc.wait(timeout=2.0)
                except subprocess.TimeoutExpired:
                    session.proc.kill()
    finally:
        if session.log_thread.is_alive():
            session.log_thread.join(timeout=1.0)



def _collect_pose_samples(args: argparse.Namespace,
                          frame_helper: FrameTransformHelper) -> Tuple[List[PoseSnapshot], float, float]:
    with rosbag.Bag(args.input_bag, "r") as bag:
        start_time = float(bag.get_start_time())
        end_time = float(bag.get_end_time())
        topic_info = bag.get_type_and_topic_info().topics
        if args.pose_topic not in topic_info:
            raise RuntimeError(f"Pose topic not found in bag: {args.pose_topic}")
        pose_info = topic_info[args.pose_topic]
        if pose_info.msg_type not in {"nav_msgs/Odometry", "geometry_msgs/PoseStamped"}:
            raise RuntimeError(
                f"Pose topic {args.pose_topic} has unsupported type {pose_info.msg_type}; expected nav_msgs/Odometry or geometry_msgs/PoseStamped"
            )
        pose_count = max(int(pose_info.message_count), 1)
        samples: List[PoseSnapshot] = []
        next_sample_bag_time: Optional[float] = None
        seen = 0
        for topic, msg, t in bag.read_messages(topics=[args.pose_topic]):
            del topic
            if _STOP_REQUESTED:
                break
            seen += 1
            bag_time_s = float(t.to_sec())
            snap = _extract_pose_snapshot(msg, bag_time_s, frame_helper)
            if next_sample_bag_time is None or bag_time_s + 1e-9 >= next_sample_bag_time:
                samples.append(snap)
                next_sample_bag_time = bag_time_s + args.interval_s
            if seen == 1 or seen % 100 == 0 or seen == pose_count:
                _progress(
                    10.0 * seen / pose_count,
                    f"Scanning pose messages: {seen}/{pose_count}, selected {len(samples)} snapshots",
                    phase="scan_pose",
                    current=seen,
                    total=pose_count,
                )
        if not samples:
            raise RuntimeError(f"No usable pose samples found on topic {args.pose_topic}")
        return samples, start_time, end_time



def _simulate_wireless_messages(args: argparse.Namespace,
                                samples: List[PoseSnapshot],
                                frame_helper: FrameTransformHelper) -> List[SimulatedBagMessage]:
    session: Optional[WorkerSession] = None
    messages: List[SimulatedBagMessage] = []
    total = max(len(samples), 1)
    completed = 0
    try:
        _progress(10.0, "Launching socket-based Sionna worker", phase="launch_worker", current=0, total=1)
        session = _start_worker(args)
        beam_codebook_payload = session.hello.get("beam_codebook") if isinstance(session.hello, dict) else None
        if bool(getattr(args, "write_beam", False)) and beam_codebook_payload:
            first_sample = samples[0]
            output_frame_id = frame_helper.frame_id_for_output(
                args.output_frame_key,
                first_sample.source_frame_id or getattr(args, "rf_frame_id", "ROS"),
            )
            codebook_msg = build_beam_codebook_msg(
                rospy=rospy,
                Header=Header,
                BeamCodebook=BeamCodebook,
                BeamEntry=BeamEntry,
                payload=beam_codebook_payload,
                stamp_s=float(first_sample.stamp_s),
                frame_id=output_frame_id,
            )
            if codebook_msg is not None:
                messages.append(
                    SimulatedBagMessage(
                        topic=args.beam_codebook_topic,
                        bag_time_s=float(first_sample.bag_time_s),
                        stamp_s=float(first_sample.stamp_s),
                        msg=codebook_msg,
                    )
                )
        _progress(
            10.0,
            f"Simulating wireless snapshots: 0/{total}",
            phase="simulate_snapshots",
            current=0,
            total=total,
        )
        for idx, snap in enumerate(samples, start=1):
            if _STOP_REQUESTED:
                break
            request = {
                "kind": "simulate",
                "sim_idx": idx,
                "bag_time_s": snap.bag_time_s,
                "stamp_s": snap.stamp_s,
                "pos_xyz": snap.pos_xyz.tolist(),
                "rx_quat_xyzw": snap.rx_quat_xyzw.tolist(),
                "vel_xyz": snap.vel_xyz.tolist(),
            }
            _send_packet(session.sock, request)
            response = _recv_packet(session.sock)
            if not isinstance(response, dict):
                raise RuntimeError(f"Unexpected worker response: {response!r}")
            if response.get("kind") == "error":
                detail = str(response.get("message", "Unknown Sionna worker error"))
                tb = str(response.get("traceback", "")).strip()
                if tb:
                    print(tb, flush=True)
                    detail = f"{detail}\n{tb}"
                raise RuntimeError(detail)
            if response.get("kind") != "result":
                raise RuntimeError(f"Unexpected worker response kind: {response!r}")

            stamp_s = float(response.get("stamp_s", snap.stamp_s))
            bag_time_s = float(response.get("bag_time_s", snap.bag_time_s))
            scene_rx_position = np.asarray(response.get("rx_position", snap.pos_xyz), dtype=float)
            scene_rx_velocity = np.asarray(response.get("rx_velocity", snap.vel_xyz), dtype=float)
            anchor_payloads = response.get("anchor_payloads", [])
            measurement_rows = response.get("measurement_rows", []) or []
            output_frame_id = frame_helper.frame_id_for_output(
                args.output_frame_key,
                snap.source_frame_id or getattr(args, "rf_frame_id", "ROS"),
            )
            rx_position = frame_helper.transform_point(scene_rx_position, scene_frame_id(), output_frame_id)
            rx_velocity = frame_helper.transform_vector(scene_rx_velocity, scene_frame_id(), output_frame_id)
            transformed_anchor_payloads = transform_anchor_payloads(
                anchor_payloads,
                frame_helper,
                scene_frame_id(),
                output_frame_id,
            )
            transformed_measurement_rows = transform_measurement_rows(
                measurement_rows,
                frame_helper,
                scene_frame_id(),
                output_frame_id,
            )

            if bool(getattr(args, "write_rf", True)):
                rf_msg = build_rf_observation_array_msg(
                    rospy=rospy,
                    Header=Header,
                    Point=Point,
                    Vector3=Vector3,
                    RfObservationArray=RfObservationArray,
                    RfAnchorObservation=RfAnchorObservation,
                    RfPathObservation=RfPathObservation,
                    anchor_payloads=transformed_anchor_payloads,
                    stamp_s=stamp_s,
                    frame_id=output_frame_id,
                    rx_position=rx_position,
                    rx_velocity=rx_velocity,
                )
                messages.append(SimulatedBagMessage(topic=args.rf_topic, bag_time_s=bag_time_s, stamp_s=stamp_s, msg=rf_msg))

            if bool(getattr(args, "write_sys", False)):
                sys_msg = build_sys_observation_msg(
                    rows=transformed_measurement_rows,
                    args=args,
                    stamp_s=stamp_s,
                    frame_id=output_frame_id,
                    rx_position=rx_position,
                    rx_velocity=rx_velocity,
                )
                messages.append(SimulatedBagMessage(topic=args.sys_topic, bag_time_s=bag_time_s, stamp_s=stamp_s, msg=sys_msg))

            if bool(getattr(args, "write_beam", False)):
                beam_msg = build_beam_observation_array_msg(
                    rospy=rospy,
                    Header=Header,
                    BeamObservationArray=BeamObservationArray,
                    BeamObservation=BeamObservation,
                    rows=transformed_measurement_rows,
                    params=args,
                    stamp_s=stamp_s,
                    frame_id=output_frame_id,
                    codebook_payload=beam_codebook_payload,
                )
                if beam_msg is not None:
                    messages.append(SimulatedBagMessage(topic=args.beam_topic, bag_time_s=bag_time_s, stamp_s=stamp_s, msg=beam_msg))

            completed = idx
            _progress(
                10.0 + 80.0 * completed / total,
                f"Simulating wireless snapshot {completed}/{total}",
                phase="simulate_snapshots",
                current=completed,
                total=total,
            )

        messages.sort(key=lambda item: (item.bag_time_s, item.topic))
        if messages:
            _progress(
                90.0,
                f"Finished simulating {len(messages)} wireless messages",
                phase="simulate_snapshots",
                current=completed,
                total=total,
            )
        return messages
    finally:
        _stop_worker(session)


def _merge_bags(args: argparse.Namespace, generated_messages: List[SimulatedBagMessage]) -> None:
    if os.path.abspath(args.input_bag) == os.path.abspath(args.output_bag):
        raise RuntimeError("Input bag and output bag must be different files so the original bag is preserved")

    output_dir = os.path.dirname(os.path.abspath(args.output_bag))
    if output_dir:
        os.makedirs(output_dir, exist_ok=True)
    if os.path.exists(args.output_bag):
        os.remove(args.output_bag)

    replace_topics = set()
    if bool(getattr(args, "write_rf", True)):
        replace_topics.add(str(args.rf_topic))
    if bool(getattr(args, "write_sys", False)) and str(getattr(args, "sys_topic", "")).strip():
        replace_topics.add(str(args.sys_topic))
    if bool(getattr(args, "write_beam", False)):
        if str(getattr(args, "beam_topic", "")).strip():
            replace_topics.add(str(args.beam_topic))
        if str(getattr(args, "beam_codebook_topic", "")).strip():
            replace_topics.add(str(args.beam_codebook_topic))

    pending_index = 0
    total_pending = len(generated_messages)
    with rosbag.Bag(args.input_bag, "r") as in_bag, rosbag.Bag(args.output_bag, "w") as out_bag:
        total_messages = max(int(in_bag.get_message_count()), 1)
        written_messages = 0
        for topic, msg, t in in_bag.read_messages():
            if _STOP_REQUESTED:
                break
            current_bag_time_s = float(t.to_sec())
            while pending_index < total_pending and generated_messages[pending_index].bag_time_s <= current_bag_time_s + 1e-9:
                item = generated_messages[pending_index]
                out_bag.write(item.topic, item.msg, t=rospy.Time.from_sec(item.bag_time_s))
                pending_index += 1
            if topic not in replace_topics:
                out_bag.write(topic, msg, t=t)
            written_messages += 1
            if written_messages == 1 or written_messages % 500 == 0 or written_messages == total_messages:
                merge_ratio = written_messages / total_messages
                _progress(
                    90.0 + 10.0 * merge_ratio,
                    f"Writing output bag: {written_messages}/{total_messages} original messages copied, {pending_index}/{total_pending} generated messages inserted",
                    phase="merge_bag",
                    current=written_messages,
                    total=total_messages,
                )

        while pending_index < total_pending and not _STOP_REQUESTED:
            item = generated_messages[pending_index]
            out_bag.write(item.topic, item.msg, t=rospy.Time.from_sec(item.bag_time_s))
            pending_index += 1

    if _STOP_REQUESTED:
        print(f"[done] stopped early; partial output kept at {args.output_bag}", flush=True)
    else:
        _progress(100.0, f"Finished writing output bag: {args.output_bag}", phase="merge_bag", current=total_messages, total=total_messages)
        print(f"[done] output bag written to {args.output_bag}", flush=True)


def parse_args(argv: Optional[List[str]] = None) -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="Copy a rosbag and replace one RF topic with offline Sionna re-simulated data")
    parser.add_argument("--input-bag", required=True)
    parser.add_argument("--output-bag", required=True)
    parser.add_argument("--pose-topic", required=True)
    parser.add_argument("--rf-topic", required=True)
    parser.add_argument("--sys-topic", default="")
    parser.add_argument("--beam-topic", default="")
    parser.add_argument("--beam-codebook-topic", default="")
    parser.add_argument("--write-rf", type=_bool_arg, default=True)
    parser.add_argument("--write-sys", type=_bool_arg, default=False)
    parser.add_argument("--write-beam", type=_bool_arg, default=False)
    parser.add_argument("--ros-to-map-matrix-json", default="")
    parser.add_argument("--airsim-to-scene-matrix-json", default="")
    parser.add_argument("--rx-array-facing-direction", default="front")
    parser.add_argument("--uav-to-rx-array-tx", type=float, default=0.0)
    parser.add_argument("--uav-to-rx-array-ty", type=float, default=0.0)
    parser.add_argument("--uav-to-rx-array-tz", type=float, default=0.0)
    parser.add_argument("--uav-to-rx-array-center-matrix-json", default="")
    parser.add_argument("--rx-array-elements-json", default="")
    parser.add_argument("--output-frame-key", default="3d")
    parser.add_argument("--scene-path", required=True)
    parser.add_argument("--bs-list-json", required=True)
    parser.add_argument("--message-frequency-hz", type=float, default=None)
    parser.add_argument("--interval-s", type=float, default=None)
    parser.add_argument("--rf-frame-id", default="map")
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
    parser.add_argument("--synthetic-array", type=_bool_arg, default=True)
    parser.add_argument("--merge-shapes", type=_bool_arg, default=False)
    parser.add_argument("--enable-sys-integration", type=_bool_arg, default=False)
    parser.add_argument("--enable-beamforming", type=_bool_arg, default=False)
    parser.add_argument("--beam-selection-mode", default="exhaustive_sweep")
    parser.add_argument("--beam-codebook-type", default="auto")
    parser.add_argument("--beam-codebook-num-beams", type=int, default=8)
    parser.add_argument("--beam-oversampling-v", type=int, default=1)
    parser.add_argument("--beam-oversampling-h", type=int, default=1)
    parser.add_argument("--beam-manual-index", type=int, default=0)
    parser.add_argument("--beam-normalize-power", type=_bool_arg, default=True)
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
    parser.add_argument("--los", type=_bool_arg, default=True)
    parser.add_argument("--specular-reflection", type=_bool_arg, default=True)
    parser.add_argument("--diffuse-reflection", type=_bool_arg, default=False)
    parser.add_argument("--refraction", type=_bool_arg, default=False)
    parser.add_argument("--diffraction", type=_bool_arg, default=False)
    parser.add_argument("--edge-diffraction", type=_bool_arg, default=False)
    parser.add_argument("--diffraction-lit-region", type=_bool_arg, default=False)
    parser.add_argument("--sionna-python", required=True, help="Python executable that has the Sionna runtime installed")
    parser.add_argument("--worker-script", default="", help="Socket worker script run inside the Sionna Python environment")
    parser.add_argument("--worker-timeout-s", type=float, default=120.0)
    args = parser.parse_args(argv)
    if args.message_frequency_hz is not None:
        if args.message_frequency_hz <= 0.0:
            parser.error("--message-frequency-hz must be > 0")
        args.interval_s = 1.0 / float(args.message_frequency_hz)
    elif args.interval_s is None:
        parser.error("--message-frequency-hz or --interval-s must be provided")
    elif args.interval_s <= 0.0:
        parser.error("--interval-s must be > 0")
    if args.message_frequency_hz is None:
        args.message_frequency_hz = 1.0 / float(args.interval_s)
    if args.worker_timeout_s <= 0.0:
        parser.error("--worker-timeout-s must be > 0")
    args.output_frame_key = normalize_output_frame_key(args.output_frame_key)
    if args.write_sys and not HAS_SIONNA_SYS_MSGS:
        parser.error("--write-sys requested but sionna_sys_msgs is not installed in this ROS Python environment")
    if args.write_sys and not str(args.sys_topic).strip():
        parser.error("--sys-topic must be provided when --write-sys true")
    if args.write_beam and not HAS_SIONNA_BEAM_MSGS:
        parser.error("--write-beam requested but sionna_beam_msgs is not installed in this ROS Python environment")
    if args.write_beam and not str(args.beam_topic).strip():
        parser.error("--beam-topic must be provided when --write-beam true")
    if args.write_beam and not str(args.beam_codebook_topic).strip():
        parser.error("--beam-codebook-topic must be provided when --write-beam true")
    if not args.worker_script:
        args.worker_script = os.path.join(os.path.dirname(os.path.abspath(__file__)), "sionna_resim_socket_worker.py")
    return args



def main(argv: Optional[List[str]] = None) -> int:
    args = parse_args(argv)
    frame_helper = FrameTransformHelper(
        ros_to_scene_matrix_json=args.ros_to_map_matrix_json,
        airsim_to_scene_matrix_json=args.airsim_to_scene_matrix_json,
        ros_frame_id=getattr(args, "rf_frame_id", "ROS"),
        rx_array_facing_direction=args.rx_array_facing_direction,
        uav_to_rx_array_tx=args.uav_to_rx_array_tx,
        uav_to_rx_array_ty=args.uav_to_rx_array_ty,
        uav_to_rx_array_tz=args.uav_to_rx_array_tz,
        uav_to_rx_array_center_matrix_json=args.uav_to_rx_array_center_matrix_json,
        rx_array_elements_json=args.rx_array_elements_json,
        rx_array_num_rows=args.rx_array_num_rows,
        rx_array_num_cols=args.rx_array_num_cols,
        rx_array_vertical_spacing=args.rx_array_vertical_spacing,
        rx_array_horizontal_spacing=args.rx_array_horizontal_spacing,
        carrier_frequency_hz=args.fc_hz,
    )
    print(f"[info] input bag: {args.input_bag}", flush=True)
    print(f"[info] output bag: {args.output_bag}", flush=True)
    print(f"[info] pose topic: {args.pose_topic}", flush=True)
    print(f"[info] rf topic: {args.rf_topic} (write={args.write_rf})", flush=True)
    if str(args.sys_topic).strip():
        print(f"[info] sys topic: {args.sys_topic} (write={args.write_sys})", flush=True)
    if str(args.beam_topic).strip() or str(args.beam_codebook_topic).strip():
        print(f"[info] beam topic: {args.beam_topic or '<disabled>'} (write={args.write_beam})", flush=True)
        print(f"[info] beam codebook topic: {args.beam_codebook_topic or '<disabled>'}", flush=True)
    print(f"[info] ros_to_map_matrix_json: {args.ros_to_map_matrix_json or '<identity>'}", flush=True)
    print(f"[info] airsim_to_scene_matrix_json: {args.airsim_to_scene_matrix_json or '<identity>'}", flush=True)
    print(f"[info] rx_array_facing_direction: {args.rx_array_facing_direction}", flush=True)
    print(f"[info] uav_to_rx_array_offset_uav: [{args.uav_to_rx_array_tx}, {args.uav_to_rx_array_ty}, {args.uav_to_rx_array_tz}]", flush=True)
    print(f"[info] uav_to_rx_array_center_matrix_json: {args.uav_to_rx_array_center_matrix_json or '<generated from facing + tx/ty/tz>'}", flush=True)
    print(f"[info] rx_array_elements_json: {args.rx_array_elements_json or '<none>'}", flush=True)
    print(f"[info] output_frame_key: {args.output_frame_key}", flush=True)
    print(f"[info] message_frequency_hz: {args.message_frequency_hz}", flush=True)
    print(f"[info] interval_s: {args.interval_s}", flush=True)
    print(f"[info] rosbag processor python: {sys.executable}", flush=True)
    print(f"[info] sionna worker python: {args.sionna_python}", flush=True)
    print(f"[info] worker script: {args.worker_script}", flush=True)
    samples, start_time, end_time = _collect_pose_samples(args, frame_helper)
    print(f"[info] selected {len(samples)} pose samples from bag span {start_time:.3f} -> {end_time:.3f}", flush=True)
    generated_messages = _simulate_wireless_messages(args, samples, frame_helper)
    print(f"[info] prepared {len(generated_messages)} generated wireless messages", flush=True)
    _merge_bags(args, generated_messages)
    return 0


if __name__ == "__main__":
    try:
        raise SystemExit(main())
    except Exception as exc:
        print(f"[error] {exc}", flush=True)
        raise
