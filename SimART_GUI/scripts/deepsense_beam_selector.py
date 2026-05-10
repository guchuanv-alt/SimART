#!/usr/bin/env python3
from __future__ import annotations

import math
import os
from dataclasses import dataclass
from typing import Any, Dict, Optional, Sequence, Tuple

import numpy as np


DEFAULT_FEATURE_MODE = "geom_vel_path13"
DEEPSENSE_POS_BBOX4_MODE = "deepsense_pos_bbox4"
FEATURE_MODE_ALIASES = {
    "geom4": "geom4",
    "distance_az_el": "geom4",
    "dist4": "geom4",
    "rel_xyzd": "rel_xyzd",
    "xyz4": "rel_xyzd",
    "rel_xyazel": "rel_xyazel",
    "xyang": "rel_xyazel",
    DEEPSENSE_POS_BBOX4_MODE: DEEPSENSE_POS_BBOX4_MODE,
    "deepsense4": DEEPSENSE_POS_BBOX4_MODE,
    "strict_deepsense4": DEEPSENSE_POS_BBOX4_MODE,
    "pos_bbox4": DEEPSENSE_POS_BBOX4_MODE,
    "bbox4": DEEPSENSE_POS_BBOX4_MODE,
    DEFAULT_FEATURE_MODE: DEFAULT_FEATURE_MODE,
    "geom13": DEFAULT_FEATURE_MODE,
    "geom_vp13": DEFAULT_FEATURE_MODE,
    "vel_path13": DEFAULT_FEATURE_MODE,
}
FEATURE_SCHEMAS = {
    "geom4": ("d_xy", "d_3d", "az", "el"),
    "rel_xyzd": ("dx", "dy", "dz", "d_3d"),
    "rel_xyazel": ("dx", "dy", "az", "el"),
    DEEPSENSE_POS_BBOX4_MODE: ("pos_distance", "pos_angle", "bbox_distance", "bbox_angle"),
    DEFAULT_FEATURE_MODE: (
        "dx",
        "dy",
        "dz",
        "d_3d",
        "az",
        "el",
        "rx_vx",
        "rx_vy",
        "rx_vz",
        "num_paths",
        "power_db",
        "tau_std_s",
        "doppler_hz",
    ),
}


def _safe_scalar(value: Any, default: float = 0.0) -> float:
    try:
        out = float(value)
    except Exception:
        return float(default)
    if not np.isfinite(out):
        return float(default)
    return out


def _safe_vector3(values: Optional[Sequence[float]]) -> np.ndarray:
    if values is None:
        return np.zeros(3, dtype=np.float32)
    arr = np.asarray(values, dtype=np.float32).reshape(-1)
    out = np.zeros(3, dtype=np.float32)
    count = min(int(arr.size), 3)
    if count > 0:
        out[:count] = arr[:count]
    out[~np.isfinite(out)] = 0.0
    return out


def _wrap_unit_interval(value: Any, default: float = 0.0) -> float:
    scalar = _safe_scalar(value, default=default)
    if not np.isfinite(scalar):
        return float(default)
    wrapped = math.fmod(scalar, 1.0)
    if wrapped < 0.0:
        wrapped += 1.0
    return float(wrapped)


def _clip_unit_interval(value: Any, default: float = 0.0) -> float:
    scalar = _safe_scalar(value, default=default)
    if not np.isfinite(scalar):
        return float(default)
    return float(min(max(scalar, 0.0), 1.0))


def _normalized_planar_polar(dx: float, dy: float, distance_scale_m: float) -> Tuple[float, float]:
    scale = max(_safe_scalar(distance_scale_m, default=100.0), 1e-6)
    distance = math.hypot(float(dx), float(dy)) / scale
    angle = (math.atan2(float(dy), float(dx)) + 2.0 * math.pi) / (2.0 * math.pi)
    return _clip_unit_interval(distance, default=1.0), _wrap_unit_interval(angle, default=0.0)


@dataclass
class DeepSenseCheckpointMetadata:
    input_size: int = len(FEATURE_SCHEMAS[DEFAULT_FEATURE_MODE])
    hidden_dim: int = 512
    output_size: int = 0
    feature_mode: str = DEFAULT_FEATURE_MODE
    top_k: int = 3
    feature_mean: Optional[np.ndarray] = None
    feature_std: Optional[np.ndarray] = None
    feature_names: Optional[Tuple[str, ...]] = None


class DeepSenseBeamPredictor:
    """
    Online inference adapter for the learned beam-selection MLP.

    It preserves the original 3-hidden-layer ReLU MLP topology from
    main_pos_beam.py while adding robust checkpoint loading for runtime use.

    Supported checkpoints:
      1) plain torch state_dict saved from the model
      2) dict with keys like state_dict/model_state_dict and optional metadata:
         input_size, hidden_dim, output_size, feature_mode, top_k,
         feature_mean, feature_std
    """

    def __init__(self, checkpoint_path: str, device: str = "cpu", default_feature_mode: str = DEFAULT_FEATURE_MODE):
        import torch
        import torch.nn as nn

        self.torch = torch
        self.nn = nn
        self.device = device
        self.checkpoint_path = str(checkpoint_path or "").strip()
        if not self.checkpoint_path:
            raise ValueError("Beam selection checkpoint path is empty")
        if not os.path.exists(self.checkpoint_path):
            raise FileNotFoundError(f"Beam selection checkpoint not found: {self.checkpoint_path}")

        try:
            blob = torch.load(self.checkpoint_path, map_location=device, weights_only=False)
        except TypeError:
            blob = torch.load(self.checkpoint_path, map_location=device)
        metadata, state_dict = self._parse_checkpoint(blob, default_feature_mode=default_feature_mode)
        if metadata.output_size <= 0:
            raise ValueError("Beam selection checkpoint output size could not be inferred")

        model = self._build_model(
            input_size=int(metadata.input_size),
            hidden_dim=int(metadata.hidden_dim),
            output_size=int(metadata.output_size),
        )
        missing, unexpected = model.load_state_dict(state_dict, strict=False)
        if missing:
            raise ValueError(f"Beam selection checkpoint missing parameters: {missing}")
        if unexpected:
            raise ValueError(f"Beam selection checkpoint has unexpected parameters: {unexpected}")
        model.eval()
        model.to(device)

        self.model = model
        self.metadata = metadata

    def _build_model(self, input_size: int, hidden_dim: int, output_size: int):
        nn = self.nn

        class NNBeamPred(nn.Module):
            def __init__(self, num_features: int, node: int, num_output: int):
                super().__init__()
                self.layer_1 = nn.Linear(num_features, node)
                self.layer_2 = nn.Linear(node, node)
                self.layer_3 = nn.Linear(node, node)
                self.layer_out = nn.Linear(node, num_output)
                self.relu = nn.ReLU()

            def forward(self, inputs):
                x = self.relu(self.layer_1(inputs))
                x = self.relu(self.layer_2(x))
                x = self.relu(self.layer_3(x))
                x = self.layer_out(x)
                return x

        return NNBeamPred(input_size, hidden_dim, output_size)

    @staticmethod
    def normalize_feature_mode(feature_mode: str) -> str:
        mode = str(feature_mode or DEFAULT_FEATURE_MODE).strip().lower()
        if not mode:
            return DEFAULT_FEATURE_MODE
        return FEATURE_MODE_ALIASES.get(mode, mode)

    @classmethod
    def feature_schema(cls, feature_mode: str) -> Tuple[str, ...]:
        mode = cls.normalize_feature_mode(feature_mode)
        if mode in FEATURE_SCHEMAS:
            return FEATURE_SCHEMAS[mode]
        raise ValueError(f"Unsupported beam selection feature mode: {feature_mode}")

    @classmethod
    def build_feature_vector(
        cls,
        tx_position: Sequence[float],
        rx_position: Sequence[float],
        feature_mode: str = DEFAULT_FEATURE_MODE,
        rx_velocity: Optional[Sequence[float]] = None,
        link_summary: Optional[Dict[str, Any]] = None,
        proxy_context: Optional[Dict[str, Any]] = None,
    ) -> np.ndarray:
        tx = np.asarray(tx_position, dtype=np.float32).reshape(3)
        rx = np.asarray(rx_position, dtype=np.float32).reshape(3)
        d = rx - tx
        dx, dy, dz = float(d[0]), float(d[1]), float(d[2])
        d_xy = math.hypot(dx, dy)
        d_3d = math.sqrt(max(dx * dx + dy * dy + dz * dz, 0.0))
        az = math.atan2(dy, dx)
        el = math.atan2(dz, max(d_xy, 1e-9))
        mode = cls.normalize_feature_mode(feature_mode)
        if mode == "geom4":
            return np.asarray([d_xy, d_3d, az, el], dtype=np.float32)
        if mode == "rel_xyzd":
            return np.asarray([dx, dy, dz, d_3d], dtype=np.float32)
        if mode == "rel_xyazel":
            return np.asarray([dx, dy, az, el], dtype=np.float32)
        if mode == DEEPSENSE_POS_BBOX4_MODE:
            context = dict(proxy_context or {})
            pos_distance, pos_angle = _normalized_planar_polar(
                dx,
                dy,
                distance_scale_m=context.get("geom_distance_scale_m", 100.0),
            )
            bbox_distance = _clip_unit_interval(context.get("bbox_distance"), default=1.0)
            bbox_angle = _wrap_unit_interval(context.get("bbox_angle"), default=0.0)
            return np.asarray([pos_distance, pos_angle, bbox_distance, bbox_angle], dtype=np.float32)
        if mode == DEFAULT_FEATURE_MODE:
            rx_vel = _safe_vector3(rx_velocity)
            summary = dict(link_summary or {})
            num_paths = _safe_scalar(summary.get("num_paths"), default=0.0)
            power_db = _safe_scalar(summary.get("power_db"), default=-200.0)
            tau_std_s = _safe_scalar(summary.get("tau_std_s"), default=0.0)
            doppler_hz = _safe_scalar(summary.get("doppler_hz"), default=0.0)
            return np.asarray([
                dx,
                dy,
                dz,
                d_3d,
                az,
                el,
                float(rx_vel[0]),
                float(rx_vel[1]),
                float(rx_vel[2]),
                num_paths,
                power_db,
                tau_std_s,
                doppler_hz,
            ], dtype=np.float32)
        raise ValueError(f"Unsupported beam selection feature mode: {feature_mode}")

    def _parse_checkpoint(self, blob: Any, default_feature_mode: str) -> Tuple[DeepSenseCheckpointMetadata, Dict[str, Any]]:
        state_dict = None
        feature_mode = self.normalize_feature_mode(default_feature_mode)
        input_size = len(FEATURE_SCHEMAS.get(feature_mode, FEATURE_SCHEMAS[DEFAULT_FEATURE_MODE]))
        hidden_dim = 512
        output_size = 0
        top_k = 3
        feature_mean = None
        feature_std = None
        feature_names = None

        if isinstance(blob, dict):
            if "state_dict" in blob:
                state_dict = blob["state_dict"]
            elif "model_state_dict" in blob:
                state_dict = blob["model_state_dict"]
            elif all(isinstance(k, str) and k.startswith("layer_") for k in blob.keys()):
                state_dict = blob
            elif all(isinstance(k, str) and (k.startswith("module.layer_") or k.startswith("module.layer_out") or k == "module.relu") for k in blob.keys()):
                state_dict = {k.replace("module.", "", 1): v for k, v in blob.items()}

            input_size = int(blob.get("input_size", input_size)) if isinstance(blob, dict) else input_size
            hidden_dim = int(blob.get("hidden_dim", blob.get("node", hidden_dim))) if isinstance(blob, dict) else hidden_dim
            output_size = int(blob.get("output_size", blob.get("num_output", output_size))) if isinstance(blob, dict) else output_size
            feature_mode = self.normalize_feature_mode(str(blob.get("feature_mode", feature_mode))) if isinstance(blob, dict) else feature_mode
            top_k = int(blob.get("top_k", top_k)) if isinstance(blob, dict) else top_k
            if isinstance(blob, dict) and blob.get("feature_mean") is not None:
                feature_mean = np.asarray(blob.get("feature_mean"), dtype=np.float32).reshape(-1)
            if isinstance(blob, dict) and blob.get("feature_std") is not None:
                feature_std = np.asarray(blob.get("feature_std"), dtype=np.float32).reshape(-1)
            if isinstance(blob, dict) and blob.get("feature_names") is not None:
                feature_names = tuple(str(v) for v in np.asarray(blob.get("feature_names")).reshape(-1).tolist())
        else:
            state_dict = blob

        if state_dict is None:
            raise ValueError("Unsupported beam selection checkpoint format")

        if "module.layer_1.weight" in state_dict:
            state_dict = {k.replace("module.", "", 1): v for k, v in state_dict.items()}

        if "layer_1.weight" in state_dict:
            w1 = state_dict["layer_1.weight"]
            hidden_dim = int(getattr(w1, "shape", [hidden_dim, input_size])[0])
            input_size = int(getattr(w1, "shape", [hidden_dim, input_size])[1])
        if "layer_out.weight" in state_dict:
            wout = state_dict["layer_out.weight"]
            output_size = int(getattr(wout, "shape", [output_size, hidden_dim])[0])
        if feature_names is None:
            try:
                schema = self.feature_schema(feature_mode)
            except ValueError:
                schema = tuple()
            if len(schema) == int(input_size):
                feature_names = tuple(schema)

        meta = DeepSenseCheckpointMetadata(
            input_size=input_size,
            hidden_dim=hidden_dim,
            output_size=output_size,
            feature_mode=feature_mode,
            top_k=max(int(top_k), 1),
            feature_mean=feature_mean,
            feature_std=feature_std,
            feature_names=feature_names,
        )
        return meta, state_dict

    def _normalize_features(self, features: np.ndarray) -> np.ndarray:
        x = np.asarray(features, dtype=np.float32).reshape(-1)
        if self.metadata.feature_mean is not None and self.metadata.feature_std is not None:
            if self.metadata.feature_mean.shape == x.shape and self.metadata.feature_std.shape == x.shape:
                denom = np.where(np.abs(self.metadata.feature_std) < 1e-12, 1.0, self.metadata.feature_std)
                x = (x - self.metadata.feature_mean) / denom
        return x

    @staticmethod
    def geometry_features(tx_position: Sequence[float], rx_position: Sequence[float], feature_mode: str = DEFAULT_FEATURE_MODE) -> np.ndarray:
        return DeepSenseBeamPredictor.build_feature_vector(
            tx_position=tx_position,
            rx_position=rx_position,
            feature_mode=feature_mode,
            rx_velocity=None,
            link_summary=None,
        )

    def predict(self, features: Sequence[float], top_k: Optional[int] = None) -> Dict[str, Any]:
        torch = self.torch
        x_np = self._normalize_features(np.asarray(features, dtype=np.float32).reshape(-1))
        if int(x_np.size) != int(self.metadata.input_size):
            raise ValueError(
                f"Beam selection feature dimension mismatch: got {int(x_np.size)}, checkpoint expects {int(self.metadata.input_size)}"
            )
        x = torch.from_numpy(x_np.reshape(1, -1)).to(self.device)
        with torch.no_grad():
            logits = self.model(x)
            probs = torch.softmax(logits, dim=-1)
        logits_np = logits.detach().cpu().numpy().reshape(-1)
        probs_np = probs.detach().cpu().numpy().reshape(-1)
        k = max(1, int(top_k if top_k is not None else self.metadata.top_k))
        k = min(k, int(probs_np.size))
        topk_idx = np.argsort(-probs_np)[:k].astype(int)
        topk_probs = probs_np[topk_idx]
        pred_idx = int(topk_idx[0]) if topk_idx.size > 0 else -1
        pred_conf = float(topk_probs[0]) if topk_probs.size > 0 else float("nan")
        return {
            "pred_index": pred_idx,
            "pred_confidence": pred_conf,
            "topk_indices": [int(v) for v in topk_idx.tolist()],
            "topk_probabilities": [float(v) for v in topk_probs.tolist()],
            "feature_vector": [float(v) for v in x_np.tolist()],
            "logits": [float(v) for v in logits_np.tolist()],
        }
