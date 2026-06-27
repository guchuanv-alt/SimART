#!/usr/bin/env python3
from __future__ import annotations

import csv
import json
import math
import os
import signal
import socket
import struct
import subprocess
import sys
import threading
import time
import traceback
from dataclasses import dataclass
from typing import Any, Dict, List, Optional, Tuple

import numpy as np

from sionna_beam_topic_utils import (
    build_beam_codebook_msg,
    build_beam_observation_array_msg,
)
from sionna_tf_frame_utils import (
    FrameTransformHelper,
    airsim_frame_id,
    clamp_scene_ground,
    normalize_output_frame_key,
    rx_array_center_frame_id,
    scene_frame_id,
    transform_anchor_payloads,
    transform_measurement_rows,
)

try:
    import rospy
    from geometry_msgs.msg import Point, PoseStamped, Vector3
    from nav_msgs.msg import Odometry
    from std_msgs.msg import Header
    from rf_msgs.msg import RfObservationArray, RfAnchorObservation, RfPathObservation
except Exception as exc:
    raise ImportError(
        "This script requires a ROS1 Python environment with rospy, geometry_msgs/nav_msgs/std_msgs, and rf_msgs installed."
    ) from exc

try:
    from sionna_sys_msgs.msg import (
        SysCandidateObservation,
        SysConfig,
        SysObservationArray,
        SysServingLink,
    )
    HAS_SIONNA_SYS_MSGS = True
except Exception:
    SysCandidateObservation = None
    SysConfig = None
    SysObservationArray = None
    SysServingLink = None
    HAS_SIONNA_SYS_MSGS = False

try:
    from sionna_beam_msgs.msg import (
        BeamCodebook,
        BeamEntry,
        BeamObservation,
        BeamObservationArray,
    )
    HAS_SIONNA_BEAM_MSGS = True
except Exception:
    BeamCodebook = None
    BeamEntry = None
    BeamObservation = None
    BeamObservationArray = None
    HAS_SIONNA_BEAM_MSGS = False


_STOP_REQUESTED = False
_PACKET_HEADER = struct.Struct("!Q")
CANDIDATE_BOOL_TRUE = {"1", "true", "yes", "on"}

PATH_TYPE_UNKNOWN = 0
PATH_TYPE_LOS = 1
PATH_TYPE_NLOS = 2
PATH_TYPE_NO_PATH = 3
TB_STATUS_UNKNOWN = 0
TB_STATUS_ACK = 1
TB_STATUS_NACK = 2
SCHED_UNKNOWN = 0
SCHED_BEST_RATE_SINGLE_UAV = 1
GROUND_HEIGHT = 0.0


@dataclass
class OdomSnapshot:
    stamp_s: float
    uav_pos_xyz: np.ndarray
    uav_quat_xyzw: np.ndarray
    pos_xyz: np.ndarray
    rx_quat_xyzw: np.ndarray
    vel_xyz: np.ndarray
    source_frame_id: str
    recv_wall_time_s: float
    seq: int


class LatestOdometryBuffer:
    def __init__(self):
        self._lock = threading.Lock()
        self._latest: Optional[OdomSnapshot] = None
        self._seq = 0

    def update(self,
               stamp_s: float,
               uav_pos_xyz: np.ndarray,
               uav_quat_xyzw: np.ndarray,
               pos_xyz: np.ndarray,
               rx_quat_xyzw: np.ndarray,
               vel_xyz: np.ndarray,
               source_frame_id: str) -> None:
        now_s = time.time()
        with self._lock:
            self._seq += 1
            self._latest = OdomSnapshot(
                stamp_s=float(stamp_s),
                uav_pos_xyz=np.asarray(uav_pos_xyz, dtype=float).reshape(3),
                uav_quat_xyzw=np.asarray(uav_quat_xyzw, dtype=float).reshape(4),
                pos_xyz=np.asarray(pos_xyz, dtype=float).reshape(3),
                rx_quat_xyzw=np.asarray(rx_quat_xyzw, dtype=float).reshape(4),
                vel_xyz=np.asarray(vel_xyz, dtype=float).reshape(3),
                source_frame_id=str(source_frame_id or ""),
                recv_wall_time_s=now_s,
                seq=self._seq,
            )

    def latest(self) -> Optional[OdomSnapshot]:
        with self._lock:
            snap = self._latest
            if snap is None:
                return None
            return OdomSnapshot(
                stamp_s=snap.stamp_s,
                uav_pos_xyz=snap.uav_pos_xyz.copy(),
                uav_quat_xyzw=snap.uav_quat_xyzw.copy(),
                pos_xyz=snap.pos_xyz.copy(),
                rx_quat_xyzw=snap.rx_quat_xyzw.copy(),
                vel_xyz=snap.vel_xyz.copy(),
                source_frame_id=str(snap.source_frame_id or ""),
                recv_wall_time_s=snap.recv_wall_time_s,
                seq=snap.seq,
            )


@dataclass
class WorkerSession:
    proc: subprocess.Popen
    sock: socket.socket
    log_thread: threading.Thread
    hello: Optional[Dict[str, Any]] = None



def _handle_stop(signum, _frame):
    global _STOP_REQUESTED
    _STOP_REQUESTED = True
    print(f"[stop] received signal {signum}; bridge will stop after the current step", flush=True)
    try:
        rospy.signal_shutdown(f"signal {signum}")
    except Exception:
        pass


signal.signal(signal.SIGINT, _handle_stop)
signal.signal(signal.SIGTERM, _handle_stop)



def ros_param_bool(name: str, default: bool) -> bool:
    value = rospy.get_param(name, default)
    if isinstance(value, bool):
        return value
    if isinstance(value, (int, float)):
        return bool(value)
    return str(value).strip().lower() in CANDIDATE_BOOL_TRUE



def require_ros_param(name: str) -> Any:
    value = rospy.get_param(name, None)
    if value is None or value == "":
        raise ValueError(f"Missing required ROS private param: {name}")
    return value



def get_ros_runtime_params() -> Dict[str, Any]:
    params: Dict[str, Any] = {}
    params["scene_path"] = require_ros_param("~scene_path")
    params["bs_list_json"] = require_ros_param("~bs_list_json")
    params["worker_script"] = require_ros_param("~worker_script")
    params["sionna_python"] = str(rospy.get_param("~sionna_python", "python3"))
    params["worker_timeout_s"] = float(rospy.get_param("~worker_timeout_s", 30.0))

    params["fc"] = float(rospy.get_param("~fc", 3.5e9))
    params["pose_topic"] = str(rospy.get_param("~pose_topic", rospy.get_param("~odom_topic", "/odometry")))
    params["pose_topic_type"] = str(rospy.get_param("~pose_topic_type", ""))
    params["ros_to_map_matrix_json"] = str(rospy.get_param("~ros_to_map_matrix_json", ""))
    params["airsim_to_scene_matrix_json"] = str(rospy.get_param("~airsim_to_scene_matrix_json", ""))
    params["rx_array_facing_direction"] = str(rospy.get_param("~rx_array_facing_direction", "front"))
    params["uav_to_rx_array_tx"] = float(rospy.get_param("~uav_to_rx_array_tx", 0.0))
    params["uav_to_rx_array_ty"] = float(rospy.get_param("~uav_to_rx_array_ty", 0.0))
    params["uav_to_rx_array_tz"] = float(rospy.get_param("~uav_to_rx_array_tz", 0.0))
    params["uav_to_rx_array_center_matrix_json"] = str(rospy.get_param("~uav_to_rx_array_center_matrix_json", ""))
    params["rx_array_elements_json"] = str(rospy.get_param("~rx_array_elements_json", ""))
    params["output_frame_key"] = normalize_output_frame_key(str(rospy.get_param("~output_frame_key", "3d")))
    params["sim_hz"] = float(rospy.get_param("~sim_hz", 10.0))
    params["max_odom_staleness_s"] = float(rospy.get_param("~max_odom_staleness_s", 1.0))
    params["measurement_csv"] = str(rospy.get_param("~measurement_csv", ""))
    params["reset_measurement_csv"] = ros_param_bool("~reset_measurement_csv", True)

    params["rf_observation_topic"] = str(rospy.get_param("~rf_observation_topic", "/rf_observations"))
    params["sys_observation_topic"] = str(rospy.get_param("~sys_observation_topic", "/sys_observations"))
    params["beam_observation_topic"] = str(rospy.get_param("~beam_observation_topic", "/beam_observations"))
    params["beam_codebook_topic"] = str(rospy.get_param("~beam_codebook_topic", "/beam_codebook"))
    params["rf_frame_id"] = str(rospy.get_param("~rf_frame_id", "map"))

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
    params["synthetic_array"] = ros_param_bool("~synthetic_array", True)
    params["merge_shapes"] = ros_param_bool("~merge_shapes", False)
    params["enable_sys_integration"] = ros_param_bool("~enable_sys_integration", False)
    params["enable_beamforming"] = ros_param_bool("~enable_beamforming", False)
    params["beam_selection_mode"] = str(rospy.get_param("~beam_selection_mode", "exhaustive_sweep"))
    params["beam_codebook_type"] = str(rospy.get_param("~beam_codebook_type", "auto"))
    params["beam_codebook_num_beams"] = int(rospy.get_param("~beam_codebook_num_beams", 8))
    params["beam_oversampling_v"] = int(rospy.get_param("~beam_oversampling_v", 1))
    params["beam_oversampling_h"] = int(rospy.get_param("~beam_oversampling_h", 1))
    params["beam_manual_index"] = int(rospy.get_param("~beam_manual_index", 0))
    params["beam_normalize_power"] = ros_param_bool("~beam_normalize_power", True)
    params["beam_codebook_file"] = str(rospy.get_param("~beam_codebook_file", ""))
    params["beam_model_checkpoint_path"] = str(rospy.get_param("~beam_model_checkpoint_path", ""))
    params["beam_feature_mode"] = str(rospy.get_param("~beam_feature_mode", "geom_vel_path13"))
    params["beam_top_k"] = int(rospy.get_param("~beam_top_k", 3))
    params["beam_export_training_dataset"] = ros_param_bool("~beam_export_training_dataset", False)
    params["beam_training_dataset_path"] = str(rospy.get_param("~beam_training_dataset_path", "beam_training_samples.csv"))
    params["beam_training_reset_dataset_on_start"] = ros_param_bool("~beam_training_reset_dataset_on_start", False)
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
    params["los"] = ros_param_bool("~los", True)
    params["specular_reflection"] = ros_param_bool("~specular_reflection", True)
    params["diffuse_reflection"] = ros_param_bool("~diffuse_reflection", False)
    params["refraction"] = ros_param_bool("~refraction", False)
    params["diffraction"] = ros_param_bool("~diffraction", False)
    params["edge_diffraction"] = ros_param_bool("~edge_diffraction", False)
    params["diffraction_lit_region"] = ros_param_bool("~diffraction_lit_region", False)
    return params



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
        if math.isnan(f) or math.isinf(f):
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
    if hasattr(value, "tolist"):
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
        assert proc.stdout is not None
        for line in proc.stdout:
            sys.stdout.write(line)
            if not line.endswith("\n"):
                sys.stdout.write("\n")
            sys.stdout.flush()
    except Exception as exc:
        print(f"[warn] failed to forward worker logs: {exc}", flush=True)



def _start_worker(params: Dict[str, Any]) -> WorkerSession:
    server = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    server.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    server.bind(("127.0.0.1", 0))
    server.listen(1)
    server.settimeout(float(params["worker_timeout_s"]))
    host, port = server.getsockname()

    worker_cmd = [
        params["sionna_python"],
        params["worker_script"],
        "--host", str(host),
        "--port", str(port),
        "--scene-path", params["scene_path"],
        "--bs-list-json", params["bs_list_json"],
        "--fc-hz", str(params["fc"]),
        "--mi-variant", params["mi_variant"],
        "--tx-array-num-rows", str(params["tx_array_num_rows"]),
        "--tx-array-num-cols", str(params["tx_array_num_cols"]),
        "--tx-array-vertical-spacing", str(params["tx_array_vertical_spacing"]),
        "--tx-array-horizontal-spacing", str(params["tx_array_horizontal_spacing"]),
        "--tx-array-pattern", params["tx_array_pattern"],
        "--tx-array-polarization", params["tx_array_polarization"],
        "--rx-array-num-rows", str(params["rx_array_num_rows"]),
        "--rx-array-num-cols", str(params["rx_array_num_cols"]),
        "--rx-array-vertical-spacing", str(params["rx_array_vertical_spacing"]),
        "--rx-array-horizontal-spacing", str(params["rx_array_horizontal_spacing"]),
        "--rx-array-pattern", params["rx_array_pattern"],
        "--rx-array-polarization", params["rx_array_polarization"],
        "--max-depth", str(params["max_depth"]),
        "--samples-per-src", str(params["samples_per_src"]),
        "--max-num-paths-per-src", str(params["max_num_paths_per_src"]),
        "--synthetic-array", "true" if params["synthetic_array"] else "false",
        "--merge-shapes", "true" if params["merge_shapes"] else "false",
        "--enable-sys-integration", "true" if params["enable_sys_integration"] else "false",
        "--enable-beamforming", "true" if params["enable_beamforming"] else "false",
        "--beam-selection-mode", params["beam_selection_mode"],
        "--beam-codebook-type", params["beam_codebook_type"],
        "--beam-codebook-num-beams", str(params["beam_codebook_num_beams"]),
        "--beam-oversampling-v", str(params["beam_oversampling_v"]),
        "--beam-oversampling-h", str(params["beam_oversampling_h"]),
        "--beam-manual-index", str(params["beam_manual_index"]),
        "--beam-normalize-power", "true" if params["beam_normalize_power"] else "false",
        "--beam-codebook-file", params["beam_codebook_file"],
        "--beam-model-checkpoint-path", params["beam_model_checkpoint_path"],
        "--beam-feature-mode", params["beam_feature_mode"],
        "--beam-top-k", str(params["beam_top_k"]),
        "--beam-export-training-dataset", "true" if params["beam_export_training_dataset"] else "false",
        "--beam-training-dataset-path", params["beam_training_dataset_path"],
        "--beam-training-reset-dataset-on-start", "true" if params["beam_training_reset_dataset_on_start"] else "false",
        "--beam-training-output-checkpoint-path", params["beam_training_output_checkpoint_path"],
        "--beam-training-epochs", str(params["beam_training_epochs"]),
        "--beam-training-batch-size", str(params["beam_training_batch_size"]),
        "--beam-training-learning-rate", str(params["beam_training_learning_rate"]),
        "--beam-training-validation-split", str(params["beam_training_validation_split"]),
        "--beam-training-hidden-dim", str(params["beam_training_hidden_dim"]),
        "--sys-num-subcarriers", str(params["sys_num_subcarriers"]),
        "--sys-subcarrier-spacing-hz", str(params["sys_subcarrier_spacing_hz"]),
        "--sys-num-ofdm-symbols", str(params["sys_num_ofdm_symbols"]),
        "--sys-temperature-k", str(params["sys_temperature_k"]),
        "--sys-bler-target", str(params["sys_bler_target"]),
        "--sys-mcs-table-index", str(params["sys_mcs_table_index"]),
        "--sys-bs-tx-power-dbm", str(params["sys_bs_tx_power_dbm"]),
        "--los", "true" if params["los"] else "false",
        "--specular-reflection", "true" if params["specular_reflection"] else "false",
        "--diffuse-reflection", "true" if params["diffuse_reflection"] else "false",
        "--refraction", "true" if params["refraction"] else "false",
        "--diffraction", "true" if params["diffraction"] else "false",
        "--edge-diffraction", "true" if params["edge_diffraction"] else "false",
        "--diffraction-lit-region", "true" if params["diffraction_lit_region"] else "false",
    ]
    print("[info] starting internal Sionna worker: " + " ".join(worker_cmd), flush=True)
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



def ensure_dir_for_file(path: str) -> None:
    folder = os.path.dirname(os.path.abspath(path))
    if folder:
        os.makedirs(folder, exist_ok=True)



def reset_output_file(path: str) -> None:
    if not path:
        return
    ensure_dir_for_file(path)
    if os.path.exists(path):
        os.remove(path)



def append_rows_csv(rows: List[Dict[str, Any]], csv_path: str) -> None:
    if not rows or not csv_path:
        return
    ensure_dir_for_file(csv_path)
    file_exists = os.path.exists(csv_path) and os.path.getsize(csv_path) > 0
    fieldnames: List[str] = []
    for row in rows:
        for key in row.keys():
            if key not in fieldnames:
                fieldnames.append(str(key))
    with open(csv_path, "a", newline="", encoding="utf-8") as f:
        writer = csv.DictWriter(f, fieldnames=fieldnames, extrasaction="ignore")
        if not file_exists:
            writer.writeheader()
        for row in rows:
            normalized = {name: _json_compatible(row.get(name)) for name in fieldnames}
            writer.writerow(normalized)
    print(f"[output] measurement csv appended: {csv_path} (+rows={len(rows)})", flush=True)



def log_measurement_rows(rows: List[Dict[str, Any]]) -> None:
    if not rows:
        print("[sim-log] empty measurement rows", flush=True)
        return
    for row in rows:
        print("[sim-log] " + json.dumps(_json_compatible(row), ensure_ascii=False), flush=True)



def path_type_str_to_uint8(path_type: Any) -> int:
    s = str(path_type).strip().upper() if path_type is not None else ""
    if s == "LOS":
        return PATH_TYPE_LOS
    if s == "NLOS":
        return PATH_TYPE_NLOS
    if s == "NO_PATH":
        return PATH_TYPE_NO_PATH
    return PATH_TYPE_UNKNOWN



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



def _make_point(xyz: Any):
    p = Point()
    x, y, z = _xyz_components(xyz)
    p.x = x
    p.y = y
    p.z = z
    return p



def _make_vector3(xyz: Any):
    v = Vector3()
    x, y, z = _xyz_components(xyz)
    v.x = x
    v.y = y
    v.z = z
    return v



def _safe_float(value: Any) -> Optional[float]:
    try:
        f = float(value)
        if not np.isfinite(f):
            return None
        return float(f)
    except Exception:
        return None



def _safe_int(value: Any) -> Optional[int]:
    f = _safe_float(value)
    if f is None:
        return None
    try:
        return int(round(f))
    except Exception:
        return None



def _safe_bool(value: Any) -> Optional[bool]:
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
    if text in CANDIDATE_BOOL_TRUE:
        return True
    if text in {"0", "false", "no", "off"}:
        return False
    return None



def _tb_status_from_value(value: Any) -> int:
    parsed = _safe_bool(value)
    if parsed is None:
        return TB_STATUS_UNKNOWN
    return TB_STATUS_ACK if parsed else TB_STATUS_NACK



def _scheduler_policy_code(name: str) -> int:
    return SCHED_BEST_RATE_SINGLE_UAV if str(name).strip() == "best_rate_single_uav" else SCHED_UNKNOWN



def build_sys_snapshot(rows: List[Dict[str, Any]], params: Dict[str, Any]) -> Dict[str, Any]:
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
            "mcs_index": -1,
            "num_decoded_bits": 0,
            "tb_status": TB_STATUS_UNKNOWN,
            "spectral_efficiency_bpshz": 0.0,
        },
        "candidates": [],
    }
    if rows:
        first = rows[0]
        snapshot["sim_idx"] = int(_safe_int(first.get("sim_idx")) or -1)
        snapshot["odom_stamp_s"] = _safe_float(first.get("odom_stamp_s"))
    for idx, row in enumerate(rows or []):
        candidate = {
            "sim_idx": int(_safe_int(row.get("sim_idx")) or snapshot["sim_idx"]),
            "odom_stamp_s": _safe_float(row.get("odom_stamp_s")),
            "anchor_id": int(_safe_int(row.get("anchor_id")) or -1),
            "anchor_name": str(row.get("bs_name", "")),
            "anchor_position": [
                _safe_float(row.get("bs_x_m")) or 0.0,
                _safe_float(row.get("bs_y_m")) or 0.0,
                _safe_float(row.get("bs_z_m")) or 0.0,
            ],
            "is_serving": bool(_safe_bool(row.get("sys_is_serving_bs")) or False),
            "num_paths": int(_safe_int(row.get("num_paths")) or 0),
            "strongest_path_power_db": _safe_float(row.get("power_db")),
            "strongest_path_type": str(row.get("path_type", "")),
            "strongest_path_order": int(_safe_int(row.get("path_order")) or -1),
            "candidate_rate_bpshz": _safe_float(row.get("sys_candidate_rate_bpshz")),
            "candidate_sinr_eff_db": _safe_float(row.get("sys_candidate_sinr_eff_db")),
            "mcs_index": int(_safe_int(row.get("sys_mcs_index")) or -1),
            "num_decoded_bits": int(_safe_int(row.get("sys_num_decoded_bits")) or 0),
            "tb_status": _tb_status_from_value(row.get("sys_tb_ok")),
            "spectral_efficiency_bpshz": _safe_float(row.get("sys_spectral_efficiency_bpshz")),
            "bler_target": _safe_float(row.get("sys_bler_target")),
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
    params: Dict[str, Any],
    stamp_s: float,
    frame_id: str,
    rx_position: np.ndarray,
    rx_velocity: np.ndarray,
):
    if not HAS_SIONNA_SYS_MSGS:
        return None, None
    snapshot = build_sys_snapshot(rows, params)

    msg = SysObservationArray()
    msg.header = Header()
    msg.header.stamp = rospy.Time.from_sec(float(stamp_s))
    msg.header.frame_id = frame_id
    msg.rx_position = _make_point(rx_position)
    msg.rx_velocity = _make_vector3(rx_velocity)
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
        candidate_msg.anchor_position = _make_point(candidate.get("anchor_position"))
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
    return msg, snapshot


def build_rf_observation_array_msg(
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
    msg.rx_position = _make_point(rx_position)
    msg.rx_velocity = _make_vector3(rx_velocity)

    for anchor_payload in anchor_payloads or []:
        anchor_msg = RfAnchorObservation()
        anchor_msg.anchor_id = int(anchor_payload.get("anchor_id", 0))
        anchor_msg.anchor_name = str(anchor_payload.get("anchor_name", ""))
        anchor_msg.anchor_position = _make_point(anchor_payload.get("anchor_position"))

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
            path_msg.tx_position = _make_point(path_payload.get("tx_position"))
            path_msg.rx_position = _make_point(path_payload.get("rx_position"))
            for xyz in path_payload.get("path_points", []):
                path_msg.path_points.append(_make_point(xyz))
            anchor_msg.paths.append(path_msg)

        msg.anchors.append(anchor_msg)

    return msg



def main() -> int:
    rospy.init_node("sionna_uav_odometry_socket_bridge", anonymous=False)
    params = get_ros_runtime_params()
    frame_helper = FrameTransformHelper(
        ros_to_scene_matrix_json=params["ros_to_map_matrix_json"],
        airsim_to_scene_matrix_json=params["airsim_to_scene_matrix_json"],
        ros_frame_id=params.get("rf_frame_id", "ROS"),
        rx_array_facing_direction=params["rx_array_facing_direction"],
        uav_to_rx_array_tx=params["uav_to_rx_array_tx"],
        uav_to_rx_array_ty=params["uav_to_rx_array_ty"],
        uav_to_rx_array_tz=params["uav_to_rx_array_tz"],
        uav_to_rx_array_center_matrix_json=params["uav_to_rx_array_center_matrix_json"],
        rx_array_elements_json=params["rx_array_elements_json"],
        rx_array_num_rows=params["rx_array_num_rows"],
        rx_array_num_cols=params["rx_array_num_cols"],
        rx_array_vertical_spacing=params["rx_array_vertical_spacing"],
        rx_array_horizontal_spacing=params["rx_array_horizontal_spacing"],
        carrier_frequency_hz=params["fc"],
    )

    print(f"[info] ROS bridge python: {sys.executable}", flush=True)
    print(f"[info] Sionna worker python: {params['sionna_python']}", flush=True)
    print(f"[info] Worker script: {params['worker_script']}", flush=True)
    print(f"[info] Scene: {params['scene_path']}", flush=True)
    print(f"[info] Pose topic: {params['pose_topic']} [{params['pose_topic_type'] or 'nav_msgs/Odometry'}]", flush=True)
    print(f"[info] RF topic: {params['rf_observation_topic']}", flush=True)
    print(f"[info] SYS topic: {params['sys_observation_topic']}", flush=True)
    print(f"[info] Beam topic: {params['beam_observation_topic']}", flush=True)
    print(f"[info] Beam codebook topic: {params['beam_codebook_topic']}", flush=True)
    print(f"[info] Measurement CSV: {params['measurement_csv'] or '<disabled>'}", flush=True)
    print(f"[info] SYS over RT: {'enabled' if params['enable_sys_integration'] else 'disabled'}", flush=True)
    print(f"[info] ROS->3D matrix: {params['ros_to_map_matrix_json'] or '<identity>'}", flush=True)
    print(f"[info] AirSim->3D matrix: {params['airsim_to_scene_matrix_json'] or '<identity>'}", flush=True)
    print(f"[info] RX array facing: {params['rx_array_facing_direction']}", flush=True)
    print(f"[info] UAV COM->RX array offset (UAV frame): [{params['uav_to_rx_array_tx']}, {params['uav_to_rx_array_ty']}, {params['uav_to_rx_array_tz']}]", flush=True)
    print(f"[info] UAV COM->RX array center matrix: {params['uav_to_rx_array_center_matrix_json'] or '<generated from facing + tx/ty/tz>'}", flush=True)
    print(f"[info] RX array elements JSON: {params['rx_array_elements_json'] or '<none>'}", flush=True)
    print(f"[info] Wireless output frame: {params['output_frame_key']}", flush=True)

    if params["measurement_csv"] and params["reset_measurement_csv"]:
        reset_output_file(params["measurement_csv"])

    worker = _start_worker(params)
    rf_obs_pub = rospy.Publisher(params["rf_observation_topic"], RfObservationArray, queue_size=10)
    sys_obs_pub = None
    if HAS_SIONNA_SYS_MSGS:
        sys_obs_pub = rospy.Publisher(params["sys_observation_topic"], SysObservationArray, queue_size=10)
    else:
        rospy.logwarn("sionna_sys_msgs not available; SYS observation topic will not be published.")
    beam_obs_pub = None
    beam_codebook_pub = None
    beam_codebook_payload = worker.hello.get("beam_codebook") if isinstance(worker.hello, dict) else None
    beam_codebook_published = False
    if HAS_SIONNA_BEAM_MSGS:
        beam_obs_pub = rospy.Publisher(params["beam_observation_topic"], BeamObservationArray, queue_size=10)
        beam_codebook_pub = rospy.Publisher(params["beam_codebook_topic"], BeamCodebook, queue_size=1, latch=True)
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
            quat_xyzw = np.array([
                msg.pose.pose.orientation.x,
                msg.pose.pose.orientation.y,
                msg.pose.pose.orientation.z,
                msg.pose.pose.orientation.w,
            ], dtype=float)
        else:
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
        odom_buffer.update(
            stamp_s=stamp_s,
            uav_pos_xyz=scene_uav_pos_xyz,
            uav_quat_xyzw=scene_quat_xyzw,
            pos_xyz=scene_rx_pos_xyz,
            rx_quat_xyzw=scene_rx_quat_xyzw,
            vel_xyz=scene_vel_xyz,
            source_frame_id=resolved_source_frame,
        )

    pose_topic_type = params["pose_topic_type"]
    if pose_topic_type == "geometry_msgs/PoseStamped":
        rospy.Subscriber(params["pose_topic"], PoseStamped, pose_callback, queue_size=1)
    else:
        rospy.Subscriber(params["pose_topic"], Odometry, pose_callback, queue_size=1)

    rospy.loginfo("Subscribed pose topic: %s (type=%s)", params["pose_topic"], pose_topic_type if pose_topic_type else "nav_msgs/Odometry")
    rospy.loginfo("Simulation hz: %.3f", params["sim_hz"])
    rospy.loginfo("Each simulation uses the latest odometry only; no backlog replay.")

    try:
        rospy.loginfo("Waiting for first pose message on topic: %s", params["pose_topic"])
        warn_last_time = 0.0
        while not rospy.is_shutdown() and not _STOP_REQUESTED:
            snap = odom_buffer.latest()
            if snap is not None:
                rospy.loginfo("Received first odometry message: seq=%d stamp=%.6f", snap.seq, snap.stamp_s)
                break
            now_t = time.time()
            if now_t - warn_last_time > 5.0:
                rospy.logwarn("Still waiting for pose on topic: %s", params["pose_topic"])
                warn_last_time = now_t
            rospy.sleep(0.05)

        sim_idx = 0
        rate = rospy.Rate(params["sim_hz"])
        while not rospy.is_shutdown() and not _STOP_REQUESTED:
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
                "[sim] start sim_idx=%d using latest odom seq=%d stamp=%.6f rx_center=[%.3f, %.3f, %.3f] uav_com=[%.3f, %.3f, %.3f] vel=[%.3f, %.3f, %.3f]",
                sim_idx,
                snap.seq,
                snap.stamp_s,
                snap.pos_xyz[0], snap.pos_xyz[1], snap.pos_xyz[2],
                snap.uav_pos_xyz[0], snap.uav_pos_xyz[1], snap.uav_pos_xyz[2],
                snap.vel_xyz[0], snap.vel_xyz[1], snap.vel_xyz[2],
            )
            sim_t0 = time.time()
            request = {
                "kind": "simulate",
                "sim_idx": sim_idx,
                "bag_time_s": snap.stamp_s,
                "stamp_s": snap.stamp_s,
                "pos_xyz": snap.pos_xyz.tolist(),
                "rx_quat_xyzw": snap.rx_quat_xyzw.tolist(),
                "vel_xyz": snap.vel_xyz.tolist(),
            }
            try:
                _send_packet(worker.sock, request)
                response = _recv_packet(worker.sock)
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

                anchor_payloads = response.get("anchor_payloads", [])
                measurement_rows = response.get("measurement_rows", []) or []
                log_measurement_rows(measurement_rows)
                append_rows_csv(measurement_rows, params["measurement_csv"])

                stamp_s = float(response.get("stamp_s", snap.stamp_s))
                scene_rx_position = np.asarray(response.get("rx_position", snap.pos_xyz), dtype=float)
                scene_rx_velocity = np.asarray(response.get("rx_velocity", snap.vel_xyz), dtype=float)
                output_frame_id = frame_helper.frame_id_for_output(
                    params["output_frame_key"],
                    snap.source_frame_id or params.get("rf_frame_id", "ROS"),
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

                if beam_codebook_pub is not None and beam_codebook_payload and not beam_codebook_published:
                    codebook_msg = build_beam_codebook_msg(
                        rospy=rospy,
                        Header=Header,
                        BeamCodebook=BeamCodebook,
                        BeamEntry=BeamEntry,
                        payload=beam_codebook_payload,
                        stamp_s=stamp_s,
                        frame_id=output_frame_id,
                    )
                    if codebook_msg is not None:
                        beam_codebook_pub.publish(codebook_msg)
                        beam_codebook_published = True

                rf_msg = build_rf_observation_array_msg(
                    anchor_payloads=transformed_anchor_payloads,
                    stamp_s=stamp_s,
                    frame_id=output_frame_id,
                    rx_position=rx_position,
                    rx_velocity=rx_velocity,
                )
                rf_obs_pub.publish(rf_msg)
                published_paths = sum(len(anchor.paths) for anchor in rf_msg.anchors)

                published_sys_candidates = 0
                if sys_obs_pub is not None:
                    sys_msg, _sys_snapshot = build_sys_observation_msg(
                        rows=transformed_measurement_rows,
                        params=params,
                        stamp_s=stamp_s,
                        frame_id=output_frame_id,
                        rx_position=rx_position,
                        rx_velocity=rx_velocity,
                    )
                    if sys_msg is not None:
                        sys_obs_pub.publish(sys_msg)
                        published_sys_candidates = len(sys_msg.candidates)

                published_beam_candidates = 0
                if beam_obs_pub is not None:
                    beam_msg = build_beam_observation_array_msg(
                        rospy=rospy,
                        Header=Header,
                        BeamObservationArray=BeamObservationArray,
                        BeamObservation=BeamObservation,
                        rows=transformed_measurement_rows,
                        params=params,
                        stamp_s=stamp_s,
                        frame_id=output_frame_id,
                        codebook_payload=beam_codebook_payload,
                    )
                    if beam_msg is not None:
                        beam_obs_pub.publish(beam_msg)
                        published_beam_candidates = len(beam_msg.observations)

                rospy.loginfo(
                    "[sim] done sim_idx=%d elapsed=%.3f s rows=%d anchors=%d published_paths=%d rf_topic=%s sys_candidates=%d sys_topic=%s beam_candidates=%d beam_topic=%s",
                    sim_idx,
                    time.time() - sim_t0,
                    len(measurement_rows),
                    len(rf_msg.anchors),
                    published_paths,
                    params["rf_observation_topic"],
                    published_sys_candidates,
                    params["sys_observation_topic"],
                    published_beam_candidates,
                    params["beam_observation_topic"],
                )
            except Exception as exc:
                rospy.logerr("[sim] failed sim_idx=%d: %s", sim_idx, str(exc))
                print(traceback.format_exc(), flush=True)
                break
            rate.sleep()
        return 0
    finally:
        _stop_worker(worker)


if __name__ == "__main__":
    try:
        raise SystemExit(main())
    except Exception as exc:
        print(f"[error] {exc}", flush=True)
        print(traceback.format_exc(), flush=True)
        raise
