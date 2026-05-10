#!/usr/bin/env python3
from __future__ import annotations

import copy
import json
from typing import Any, Dict, List, Optional, Sequence

import numpy as np

import rospy
import tf2_geometry_msgs  # noqa: F401
import tf2_ros
from geometry_msgs.msg import PointStamped, PoseStamped, TransformStamped, Vector3Stamped
from tf.transformations import quaternion_from_matrix, quaternion_matrix


FRAME_KEY_ROS = "ros"
FRAME_KEY_3D = "3d"
FRAME_KEY_AIRSIM = "airsim"


def normalize_output_frame_key(key: str) -> str:
    text = str(key or "").strip().lower()
    if text == FRAME_KEY_ROS:
        return FRAME_KEY_ROS
    if text == FRAME_KEY_AIRSIM:
        return FRAME_KEY_AIRSIM
    return FRAME_KEY_3D


def scene_frame_id() -> str:
    return "3D"


def airsim_frame_id() -> str:
    return "airsim"


def uav_com_frame_id() -> str:
    return "uav_com"


def rx_array_center_frame_id() -> str:
    return "rx_array_center"


def default_rx_antenna_frame_id(index: int) -> str:
    return f"rx_ant_{max(int(index), 0)}"


def normalize_rx_array_facing_direction(key: str) -> str:
    text = str(key or "").strip().lower()
    if text in {"back", "left", "right", "up", "down"}:
        return text
    return "front"


def carrier_wavelength_m(carrier_frequency_hz: float) -> float:
    speed_of_light = 299792458.0
    try:
        freq = float(carrier_frequency_hz)
    except Exception:
        freq = 0.0
    if not np.isfinite(freq) or freq <= 0.0:
        return 1.0
    return speed_of_light / freq


def load_transform_matrix(matrix_file: str) -> np.ndarray:
    if not str(matrix_file or "").strip():
        return np.eye(4, dtype=float)
    with open(matrix_file, "r", encoding="utf-8") as f:
        data = json.load(f)
    mat = np.array(data, dtype=float)
    if mat.shape != (4, 4):
        raise ValueError(f"transform matrix must be 4x4, got {mat.shape}")
    return mat


def _normalize_frame_id(frame_id: str, fallback: str = "") -> str:
    text = str(frame_id or "").strip()
    if text:
        return text
    return str(fallback or "").strip()


def _xyz_array(xyz: Any) -> np.ndarray:
    if xyz is None:
        return np.zeros(3, dtype=float)
    if isinstance(xyz, np.ndarray):
        arr = np.asarray(xyz, dtype=float).reshape(-1)
    elif isinstance(xyz, (list, tuple)):
        arr = np.asarray(list(xyz), dtype=float).reshape(-1)
    else:
        arr = np.asarray([
            getattr(xyz, "x", 0.0),
            getattr(xyz, "y", 0.0),
            getattr(xyz, "z", 0.0),
        ], dtype=float).reshape(-1)
    out = np.zeros(3, dtype=float)
    for idx in range(min(3, arr.size)):
        value = float(arr[idx])
        out[idx] = value if np.isfinite(value) else 0.0
    return out


def _safe_float(value: Any, fallback: float = 0.0) -> float:
    try:
        numeric = float(value)
    except Exception:
        return float(fallback)
    return numeric if np.isfinite(numeric) else float(fallback)


def _normalize_axis(axis: Any, fallback: Any) -> np.ndarray:
    vec = _xyz_array(axis)
    norm = float(np.linalg.norm(vec))
    if np.isfinite(norm) and norm > 1e-12:
        return vec / norm
    fallback_vec = _xyz_array(fallback)
    fallback_norm = float(np.linalg.norm(fallback_vec))
    if np.isfinite(fallback_norm) and fallback_norm > 1e-12:
        return fallback_vec / fallback_norm
    return np.array([1.0, 0.0, 0.0], dtype=float)


def _build_uav_to_rx_array_center_matrix(facing_direction: str,
                                         tx: float,
                                         ty: float,
                                         tz: float) -> np.ndarray:
    facing = normalize_rx_array_facing_direction(facing_direction)
    x_axis = np.array([1.0, 0.0, 0.0], dtype=float)
    if facing == "back":
        x_axis = np.array([-1.0, 0.0, 0.0], dtype=float)
    elif facing == "left":
        x_axis = np.array([0.0, 1.0, 0.0], dtype=float)
    elif facing == "right":
        x_axis = np.array([0.0, -1.0, 0.0], dtype=float)
    elif facing == "up":
        x_axis = np.array([0.0, 0.0, 1.0], dtype=float)
    elif facing == "down":
        x_axis = np.array([0.0, 0.0, -1.0], dtype=float)
    x_axis = _normalize_axis(x_axis, [1.0, 0.0, 0.0])

    if facing in {"up", "down"}:
        y_axis = _normalize_axis([0.0, 1.0, 0.0], [0.0, 1.0, 0.0])
        z_axis = _normalize_axis(np.cross(x_axis, y_axis), [1.0, 0.0, 0.0])
        y_axis = _normalize_axis(np.cross(z_axis, x_axis), [0.0, 1.0, 0.0])
    else:
        z_axis = _normalize_axis([0.0, 0.0, 1.0], [0.0, 0.0, 1.0])
        y_axis = _normalize_axis(np.cross(z_axis, x_axis), [0.0, 1.0, 0.0])
        z_axis = _normalize_axis(np.cross(x_axis, y_axis), [0.0, 0.0, 1.0])

    rx_array_in_uav = np.eye(4, dtype=float)
    rx_array_in_uav[0:3, 0] = x_axis
    rx_array_in_uav[0:3, 1] = y_axis
    rx_array_in_uav[0:3, 2] = z_axis
    rx_array_in_uav[0, 3] = _safe_float(tx, 0.0)
    rx_array_in_uav[1, 3] = _safe_float(ty, 0.0)
    rx_array_in_uav[2, 3] = _safe_float(tz, 0.0)
    return np.linalg.inv(rx_array_in_uav)


def _matrix_to_transform_stamped(matrix: np.ndarray, parent_frame: str, child_frame: str) -> TransformStamped:
    transform = TransformStamped()
    transform.header.stamp = rospy.Time(0)
    transform.header.frame_id = str(parent_frame)
    transform.child_frame_id = str(child_frame)
    transform.transform.translation.x = float(matrix[0, 3])
    transform.transform.translation.y = float(matrix[1, 3])
    transform.transform.translation.z = float(matrix[2, 3])
    quaternion = quaternion_from_matrix(matrix)
    transform.transform.rotation.x = float(quaternion[0])
    transform.transform.rotation.y = float(quaternion[1])
    transform.transform.rotation.z = float(quaternion[2])
    transform.transform.rotation.w = float(quaternion[3])
    return transform


def _is_matrix4x4_json_array(value: Any) -> bool:
    if not isinstance(value, list) or len(value) != 4:
        return False
    for row in value:
        if not isinstance(row, list) or len(row) != 4:
            return False
    return True


def _load_named_transform_entries(json_file: str) -> List[Dict[str, Any]]:
    path = str(json_file or "").strip()
    if not path:
        return []
    with open(path, "r", encoding="utf-8") as f:
        data = json.load(f)

    def parse_entry(raw: Any, fallback_frame_id: str) -> Dict[str, Any]:
        if _is_matrix4x4_json_array(raw):
            return {
                "frame_id": _normalize_frame_id(fallback_frame_id, default_rx_antenna_frame_id(0)),
                "matrix": np.asarray(raw, dtype=float).reshape(4, 4),
            }
        if not isinstance(raw, dict):
            raise ValueError("transform entry must be either a 4x4 array or an object with a matrix field")
        frame_id = _normalize_frame_id(
            raw.get("frame_id", ""),
            _normalize_frame_id(raw.get("name", ""), _normalize_frame_id(raw.get("id", ""), fallback_frame_id)),
        )
        matrix_value = None
        for key in ("matrix", "transform", "T"):
            if key in raw:
                matrix_value = raw.get(key)
                break
        if not _is_matrix4x4_json_array(matrix_value):
            raise ValueError("transform entry is missing a valid 4x4 matrix field ('matrix', 'transform', or 'T')")
        return {
            "frame_id": _normalize_frame_id(frame_id, default_rx_antenna_frame_id(0)),
            "matrix": np.asarray(matrix_value, dtype=float).reshape(4, 4),
        }

    entries: List[Dict[str, Any]] = []
    if isinstance(data, list):
        if _is_matrix4x4_json_array(data):
            entries.append(parse_entry(data, default_rx_antenna_frame_id(0)))
            return entries
        for idx, item in enumerate(data):
            entries.append(parse_entry(item, default_rx_antenna_frame_id(idx)))
        return entries

    if not isinstance(data, dict):
        raise ValueError("transform list JSON root must be an object or array")

    if isinstance(data.get("elements"), list):
        for idx, item in enumerate(data["elements"]):
            entries.append(parse_entry(item, default_rx_antenna_frame_id(idx)))
        return entries

    if isinstance(data.get("antennas"), list):
        for idx, item in enumerate(data["antennas"]):
            entries.append(parse_entry(item, default_rx_antenna_frame_id(idx)))
        return entries

    for key, value in data.items():
        if key in {"parent_frame", "source_frame", "frame_prefix"}:
            continue
        entries.append(parse_entry(value, key))
    return entries


def _pose_to_matrix(position_xyz: Any, quaternion_xyzw: Any) -> np.ndarray:
    position = _xyz_array(position_xyz)
    quat = np.asarray(quaternion_xyzw if quaternion_xyzw is not None else [0.0, 0.0, 0.0, 1.0], dtype=float).reshape(-1)
    out = np.eye(4, dtype=float)
    if quat.size >= 4 and np.all(np.isfinite(quat[:4])):
        out = quaternion_matrix([float(quat[0]), float(quat[1]), float(quat[2]), float(quat[3])])
    out = np.asarray(out, dtype=float).reshape(4, 4)
    out[0, 3] = float(position[0])
    out[1, 3] = float(position[1])
    out[2, 3] = float(position[2])
    return out


def _build_planar_rx_array_entries(num_rows: int,
                                   num_cols: int,
                                   vertical_spacing_lambda: float,
                                   horizontal_spacing_lambda: float,
                                   carrier_frequency_hz: float) -> List[Dict[str, Any]]:
    rows = max(int(num_rows), 1)
    cols = max(int(num_cols), 1)
    wavelength = carrier_wavelength_m(carrier_frequency_hz)
    d_v = max(float(vertical_spacing_lambda), 0.0) * wavelength
    d_h = max(float(horizontal_spacing_lambda), 0.0) * wavelength
    entries: List[Dict[str, Any]] = []
    antenna_index = 0
    for i in range(rows):
        for j in range(cols):
            y = d_h * float(j) - (float(cols) - 1.0) * d_h / 2.0
            z = -d_v * float(i) + (float(rows) - 1.0) * d_v / 2.0
            matrix = np.eye(4, dtype=float)
            matrix[1, 3] = -y
            matrix[2, 3] = -z
            entries.append({
                "frame_id": default_rx_antenna_frame_id(antenna_index),
                "matrix": matrix,
            })
            antenna_index += 1
    return entries


class FrameTransformHelper:
    def __init__(self,
                 ros_to_scene_matrix_json: str = "",
                 airsim_to_scene_matrix_json: str = "",
                 ros_frame_id: str = "ROS",
                 rx_array_facing_direction: str = "front",
                 uav_to_rx_array_tx: float = 0.0,
                 uav_to_rx_array_ty: float = 0.0,
                 uav_to_rx_array_tz: float = 0.0,
                 uav_to_rx_array_center_matrix_json: str = "",
                 rx_array_elements_json: str = "",
                 rx_array_num_rows: int = 1,
                 rx_array_num_cols: int = 1,
                 rx_array_vertical_spacing: float = 0.5,
                 rx_array_horizontal_spacing: float = 0.5,
                 carrier_frequency_hz: float = 3.5e9):
        self._ros_to_scene_matrix_json = str(ros_to_scene_matrix_json or "").strip()
        self._airsim_to_scene_matrix_json = str(airsim_to_scene_matrix_json or "").strip()
        self._rx_array_facing_direction = normalize_rx_array_facing_direction(rx_array_facing_direction)
        self._uav_to_rx_array_tx = _safe_float(uav_to_rx_array_tx, 0.0)
        self._uav_to_rx_array_ty = _safe_float(uav_to_rx_array_ty, 0.0)
        self._uav_to_rx_array_tz = _safe_float(uav_to_rx_array_tz, 0.0)
        self._uav_to_rx_array_center_matrix_json = str(uav_to_rx_array_center_matrix_json or "").strip()
        self._rx_array_elements_json = str(rx_array_elements_json or "").strip()
        self._ros_frame_id = _normalize_frame_id(ros_frame_id, "ROS")
        self._rx_array_num_rows = max(int(rx_array_num_rows), 1)
        self._rx_array_num_cols = max(int(rx_array_num_cols), 1)
        self._rx_array_vertical_spacing = float(rx_array_vertical_spacing)
        self._rx_array_horizontal_spacing = float(rx_array_horizontal_spacing)
        self._carrier_frequency_hz = float(carrier_frequency_hz)
        self._runtime_uav_scene_position = np.zeros(3, dtype=float)
        self._runtime_uav_scene_quaternion = np.array([0.0, 0.0, 0.0, 1.0], dtype=float)
        self._runtime_uav_pose_valid = False
        self._antenna_frame_ids: List[str] = []
        self._buffer: Optional[tf2_ros.Buffer] = None
        self._buffer_dirty = True

    def set_ros_frame_id(self, frame_id: str) -> None:
        normalized = _normalize_frame_id(frame_id, self._ros_frame_id or "ROS")
        if normalized == self._ros_frame_id:
            return
        self._ros_frame_id = normalized
        self._buffer_dirty = True

    def resolved_ros_frame_id(self) -> str:
        return _normalize_frame_id(self._ros_frame_id, "ROS")

    def set_ros_to_scene_matrix_json(self, path: str) -> None:
        normalized = str(path or "").strip()
        if normalized == self._ros_to_scene_matrix_json:
            return
        self._ros_to_scene_matrix_json = normalized
        self._buffer_dirty = True

    def set_airsim_to_scene_matrix_json(self, path: str) -> None:
        normalized = str(path or "").strip()
        if normalized == self._airsim_to_scene_matrix_json:
            return
        self._airsim_to_scene_matrix_json = normalized
        self._buffer_dirty = True

    def set_rx_array_facing_direction(self, facing_direction: str) -> None:
        normalized = normalize_rx_array_facing_direction(facing_direction)
        if normalized == self._rx_array_facing_direction:
            return
        self._rx_array_facing_direction = normalized
        self._buffer_dirty = True

    def set_uav_to_rx_array_offset(self, tx: float, ty: float, tz: float) -> None:
        normalized_tx = _safe_float(tx, 0.0)
        normalized_ty = _safe_float(ty, 0.0)
        normalized_tz = _safe_float(tz, 0.0)
        if (
            np.isclose(normalized_tx, self._uav_to_rx_array_tx)
            and np.isclose(normalized_ty, self._uav_to_rx_array_ty)
            and np.isclose(normalized_tz, self._uav_to_rx_array_tz)
        ):
            return
        self._uav_to_rx_array_tx = normalized_tx
        self._uav_to_rx_array_ty = normalized_ty
        self._uav_to_rx_array_tz = normalized_tz
        self._buffer_dirty = True

    def set_uav_to_rx_array_center_matrix_json(self, path: str) -> None:
        normalized = str(path or "").strip()
        if normalized == self._uav_to_rx_array_center_matrix_json:
            return
        self._uav_to_rx_array_center_matrix_json = normalized
        self._buffer_dirty = True

    def set_rx_array_elements_json(self, path: str) -> None:
        normalized = str(path or "").strip()
        if normalized == self._rx_array_elements_json:
            return
        self._rx_array_elements_json = normalized
        self._buffer_dirty = True

    def set_rx_array_geometry(self,
                              num_rows: int,
                              num_cols: int,
                              vertical_spacing_lambda: float,
                              horizontal_spacing_lambda: float,
                              carrier_frequency_hz: float) -> None:
        normalized_rows = max(int(num_rows), 1)
        normalized_cols = max(int(num_cols), 1)
        normalized_v = float(vertical_spacing_lambda)
        normalized_h = float(horizontal_spacing_lambda)
        normalized_fc = float(carrier_frequency_hz)
        if (
            normalized_rows == self._rx_array_num_rows
            and normalized_cols == self._rx_array_num_cols
            and np.isclose(normalized_v, self._rx_array_vertical_spacing)
            and np.isclose(normalized_h, self._rx_array_horizontal_spacing)
            and np.isclose(normalized_fc, self._carrier_frequency_hz)
        ):
            return
        self._rx_array_num_rows = normalized_rows
        self._rx_array_num_cols = normalized_cols
        self._rx_array_vertical_spacing = normalized_v
        self._rx_array_horizontal_spacing = normalized_h
        self._carrier_frequency_hz = normalized_fc
        self._buffer_dirty = True

    def set_runtime_uav_pose_scene(self, position_xyz: Any, quaternion_xyzw: Any) -> None:
        position = _xyz_array(position_xyz)
        quat = np.asarray(quaternion_xyzw if quaternion_xyzw is not None else [0.0, 0.0, 0.0, 1.0], dtype=float).reshape(-1)
        normalized_quat = np.array([0.0, 0.0, 0.0, 1.0], dtype=float)
        for idx in range(min(4, quat.size)):
            value = float(quat[idx])
            normalized_quat[idx] = value if np.isfinite(value) else normalized_quat[idx]
        if (
            self._runtime_uav_pose_valid
            and np.allclose(self._runtime_uav_scene_position, position, atol=1e-12)
            and np.allclose(self._runtime_uav_scene_quaternion, normalized_quat, atol=1e-12)
        ):
            return
        self._runtime_uav_scene_position = position
        self._runtime_uav_scene_quaternion = normalized_quat
        self._runtime_uav_pose_valid = True
        self._buffer_dirty = True

    def clear_runtime_uav_pose_scene(self) -> None:
        if not self._runtime_uav_pose_valid:
            return
        self._runtime_uav_pose_valid = False
        self._buffer_dirty = True

    def antenna_frame_ids(self) -> List[str]:
        self._ensure_buffer()
        return list(self._antenna_frame_ids)

    def frame_id_for_output(self, output_frame_key: str, ros_frame_override: str = "") -> str:
        key = normalize_output_frame_key(output_frame_key)
        if key == FRAME_KEY_ROS:
            return _normalize_frame_id(ros_frame_override, self.resolved_ros_frame_id())
        if key == FRAME_KEY_AIRSIM:
            return airsim_frame_id()
        return scene_frame_id()

    def _ensure_buffer(self) -> None:
        if self._buffer is not None and not self._buffer_dirty:
            return
        ros_to_scene = load_transform_matrix(self._ros_to_scene_matrix_json)
        airsim_to_scene = load_transform_matrix(self._airsim_to_scene_matrix_json)
        if self._uav_to_rx_array_center_matrix_json:
            uav_to_rx_array_center = load_transform_matrix(self._uav_to_rx_array_center_matrix_json)
        else:
            uav_to_rx_array_center = _build_uav_to_rx_array_center_matrix(
                self._rx_array_facing_direction,
                self._uav_to_rx_array_tx,
                self._uav_to_rx_array_ty,
                self._uav_to_rx_array_tz,
            )
        rx_array_center_to_uav = np.linalg.inv(uav_to_rx_array_center)
        antenna_entries = _load_named_transform_entries(self._rx_array_elements_json)
        if not antenna_entries:
            antenna_entries = _build_planar_rx_array_entries(
                self._rx_array_num_rows,
                self._rx_array_num_cols,
                self._rx_array_vertical_spacing,
                self._rx_array_horizontal_spacing,
                self._carrier_frequency_hz,
            )
        buffer = tf2_ros.Buffer(debug=False)
        buffer.set_transform(
            _matrix_to_transform_stamped(ros_to_scene, scene_frame_id(), self.resolved_ros_frame_id()),
            "sionna_tf_frame_utils_ros_to_scene",
        )
        buffer.set_transform(
            _matrix_to_transform_stamped(airsim_to_scene, scene_frame_id(), airsim_frame_id()),
            "sionna_tf_frame_utils_airsim_to_scene",
        )
        buffer.set_transform(
            _matrix_to_transform_stamped(rx_array_center_to_uav, uav_com_frame_id(), rx_array_center_frame_id()),
            "sionna_tf_frame_utils_uav_to_rx_array_center",
        )
        antenna_frame_ids: List[str] = []
        for idx, entry in enumerate(antenna_entries):
            frame_id = _normalize_frame_id(entry.get("frame_id", ""), default_rx_antenna_frame_id(idx))
            matrix = np.asarray(entry.get("matrix", np.eye(4, dtype=float)), dtype=float).reshape(4, 4)
            antenna_to_array = np.linalg.inv(matrix)
            buffer.set_transform(
                _matrix_to_transform_stamped(antenna_to_array, rx_array_center_frame_id(), frame_id),
                f"sionna_tf_frame_utils_rx_array_antenna_{idx}",
            )
            antenna_frame_ids.append(frame_id)
        if self._runtime_uav_pose_valid:
            buffer.set_transform(
                _matrix_to_transform_stamped(
                    _pose_to_matrix(self._runtime_uav_scene_position, self._runtime_uav_scene_quaternion),
                    scene_frame_id(),
                    uav_com_frame_id(),
                ),
                "sionna_tf_frame_utils_scene_to_uav_pose",
            )
        self._buffer = buffer
        self._antenna_frame_ids = antenna_frame_ids
        self._buffer_dirty = False

    def transform_point(self, xyz: Any, source_frame_id: str, target_frame_id: str) -> np.ndarray:
        source = _normalize_frame_id(source_frame_id, scene_frame_id())
        target = _normalize_frame_id(target_frame_id, scene_frame_id())
        point = _xyz_array(xyz)
        if source == target:
            return point
        self._ensure_buffer()
        stamped = PointStamped()
        stamped.header.stamp = rospy.Time(0)
        stamped.header.frame_id = source
        stamped.point.x = float(point[0])
        stamped.point.y = float(point[1])
        stamped.point.z = float(point[2])
        transformed = self._buffer.transform(stamped, target)
        return np.array([
            transformed.point.x,
            transformed.point.y,
            transformed.point.z,
        ], dtype=float)

    def transform_vector(self, xyz: Any, source_frame_id: str, target_frame_id: str) -> np.ndarray:
        source = _normalize_frame_id(source_frame_id, scene_frame_id())
        target = _normalize_frame_id(target_frame_id, scene_frame_id())
        vector = _xyz_array(xyz)
        if source == target:
            return vector
        self._ensure_buffer()
        stamped = Vector3Stamped()
        stamped.header.stamp = rospy.Time(0)
        stamped.header.frame_id = source
        stamped.vector.x = float(vector[0])
        stamped.vector.y = float(vector[1])
        stamped.vector.z = float(vector[2])
        transformed = self._buffer.transform(stamped, target)
        return np.array([
            transformed.vector.x,
            transformed.vector.y,
            transformed.vector.z,
        ], dtype=float)

    def transform_quaternion(self, quaternion_xyzw: Any, source_frame_id: str, target_frame_id: str) -> np.ndarray:
        source = _normalize_frame_id(source_frame_id, scene_frame_id())
        target = _normalize_frame_id(target_frame_id, scene_frame_id())
        quat = np.asarray(quaternion_xyzw if quaternion_xyzw is not None else [0.0, 0.0, 0.0, 1.0], dtype=float).reshape(-1)
        normalized_quat = np.array([0.0, 0.0, 0.0, 1.0], dtype=float)
        for idx in range(min(4, quat.size)):
            value = float(quat[idx])
            normalized_quat[idx] = value if np.isfinite(value) else normalized_quat[idx]
        if source == target:
            return normalized_quat
        self._ensure_buffer()
        stamped = PoseStamped()
        stamped.header.stamp = rospy.Time(0)
        stamped.header.frame_id = source
        stamped.pose.orientation.x = float(normalized_quat[0])
        stamped.pose.orientation.y = float(normalized_quat[1])
        stamped.pose.orientation.z = float(normalized_quat[2])
        stamped.pose.orientation.w = float(normalized_quat[3])
        transformed = self._buffer.transform(stamped, target)
        return np.array([
            transformed.pose.orientation.x,
            transformed.pose.orientation.y,
            transformed.pose.orientation.z,
            transformed.pose.orientation.w,
        ], dtype=float)


def transform_anchor_payloads(anchor_payloads: List[Dict[str, Any]],
                              frame_helper: FrameTransformHelper,
                              source_frame_id: str,
                              target_frame_id: str) -> List[Dict[str, Any]]:
    source = _normalize_frame_id(source_frame_id, scene_frame_id())
    target = _normalize_frame_id(target_frame_id, scene_frame_id())
    if source == target:
        return copy.deepcopy(anchor_payloads or [])
    transformed_payloads: List[Dict[str, Any]] = []
    for anchor_payload in anchor_payloads or []:
        item = dict(anchor_payload or {})
        item["anchor_position"] = frame_helper.transform_point(item.get("anchor_position"), source, target).tolist()
        transformed_paths: List[Dict[str, Any]] = []
        for path_payload in item.get("paths", []) or []:
            path_item = dict(path_payload or {})
            path_item["tx_position"] = frame_helper.transform_point(path_item.get("tx_position"), source, target).tolist()
            path_item["rx_position"] = frame_helper.transform_point(path_item.get("rx_position"), source, target).tolist()
            path_item["path_points"] = [
                frame_helper.transform_point(point_xyz, source, target).tolist()
                for point_xyz in path_item.get("path_points", []) or []
            ]
            transformed_paths.append(path_item)
        item["paths"] = transformed_paths
        transformed_payloads.append(item)
    return transformed_payloads


def transform_measurement_rows(rows: List[Dict[str, Any]],
                               frame_helper: FrameTransformHelper,
                               source_frame_id: str,
                               target_frame_id: str) -> List[Dict[str, Any]]:
    source = _normalize_frame_id(source_frame_id, scene_frame_id())
    target = _normalize_frame_id(target_frame_id, scene_frame_id())
    if source == target:
        return copy.deepcopy(rows or [])
    transformed_rows: List[Dict[str, Any]] = []
    for row in rows or []:
        item = dict(row or {})
        if any(key in item for key in ("bs_x_m", "bs_y_m", "bs_z_m")):
            point = frame_helper.transform_point([
                item.get("bs_x_m", 0.0),
                item.get("bs_y_m", 0.0),
                item.get("bs_z_m", 0.0),
            ], source, target)
            item["bs_x_m"] = float(point[0])
            item["bs_y_m"] = float(point[1])
            item["bs_z_m"] = float(point[2])
        if any(key in item for key in ("uav_true_x_m", "uav_true_y_m", "uav_true_z_m")):
            point = frame_helper.transform_point([
                item.get("uav_true_x_m", 0.0),
                item.get("uav_true_y_m", 0.0),
                item.get("uav_true_z_m", 0.0),
            ], source, target)
            item["uav_true_x_m"] = float(point[0])
            item["uav_true_y_m"] = float(point[1])
            item["uav_true_z_m"] = float(point[2])
        transformed_rows.append(item)
    return transformed_rows


def clamp_scene_ground(xyz: Sequence[float], ground_height: float = 0.0) -> np.ndarray:
    out = _xyz_array(xyz)
    out[2] = max(float(out[2]), float(ground_height))
    return out
