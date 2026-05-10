#!/usr/bin/env python3
from __future__ import annotations

import json
import math
from typing import Any, Dict, List, Optional, Tuple

import numpy as np


def _get(obj: Any, key: str, default: Any = None) -> Any:
    if isinstance(obj, dict):
        return obj.get(key, default)
    return getattr(obj, key, default)


def _safe_float(value: Any) -> Optional[float]:
    try:
        out = float(value)
        if not np.isfinite(out):
            return None
        return out
    except Exception:
        return None


def _safe_int(value: Any) -> Optional[int]:
    out = _safe_float(value)
    if out is None:
        return None
    try:
        return int(round(out))
    except Exception:
        return None


def _safe_bool(value: Any) -> Optional[bool]:
    if isinstance(value, bool):
        return value
    if value is None:
        return None
    if isinstance(value, (int, float)):
        try:
            out = float(value)
        except Exception:
            return None
        if not np.isfinite(out):
            return None
        return bool(int(round(out)))
    text = str(value).strip().lower()
    if text in {"1", "true", "yes", "on"}:
        return True
    if text in {"0", "false", "no", "off"}:
        return False
    return None


SUPPORTED_CODEBOOK_NUM_BEAMS = (8, 64, 128)


def normalize_codebook_num_beams(value: Any, default: int = 8) -> int:
    requested = _safe_int(value)
    if requested is None:
        requested = int(default)
    if requested not in SUPPORTED_CODEBOOK_NUM_BEAMS:
        raise ValueError(
            f"beam_codebook_num_beams must be one of {SUPPORTED_CODEBOOK_NUM_BEAMS}, got {requested}"
        )
    return int(requested)


def effective_dft_oversampling_for_num_beams(codebook_type: str,
                                             tx_rows: int,
                                             tx_cols: int,
                                             target_num_beams: int,
                                             min_oversampling_v: int = 1,
                                             min_oversampling_h: int = 1) -> Tuple[int, int]:
    codebook_type = str(codebook_type or "").strip().lower()
    tx_rows = max(int(tx_rows), 1)
    tx_cols = max(int(tx_cols), 1)
    target_num_beams = normalize_codebook_num_beams(target_num_beams)
    min_oversampling_v = max(int(min_oversampling_v), 1)
    min_oversampling_h = max(int(min_oversampling_h), 1)

    if codebook_type == "dft_ura" and tx_rows > 1 and tx_cols > 1:
        best_v = min_oversampling_v
        best_h = min_oversampling_h
        best_score: Optional[Tuple[int, int, int]] = None
        max_search = max(target_num_beams, min_oversampling_v, min_oversampling_h, 1)
        for ov_v in range(min_oversampling_v, max_search + 1):
            for ov_h in range(min_oversampling_h, max_search + 1):
                num_v = tx_rows * ov_v
                num_h = tx_cols * ov_h
                total = num_v * num_h
                if total < target_num_beams:
                    continue
                score = (total - target_num_beams, abs(num_v - num_h), ov_v + ov_h)
                if best_score is None or score < best_score:
                    best_score = score
                    best_v = ov_v
                    best_h = ov_h
        return best_v, best_h

    tx_ant = max(tx_rows * tx_cols, 1)
    required_oversampling = max(int(math.ceil(float(target_num_beams) / float(tx_ant))), 1)
    if tx_cols > 1:
        return min_oversampling_v, max(min_oversampling_h, required_oversampling)
    return max(min_oversampling_v, required_oversampling), min_oversampling_h


def select_codebook_num_beams(codebook: Any, target_num_beams: int) -> Tuple[Any, List[int]]:
    target_num_beams = normalize_codebook_num_beams(target_num_beams)
    total_beams = int(codebook.shape[0]) if hasattr(codebook, "shape") and len(codebook.shape) >= 1 else 0
    if total_beams <= 0:
        raise ValueError("generated codebook has no beams")
    if total_beams < target_num_beams:
        raise ValueError(f"generated codebook has {total_beams} beams, cannot select {target_num_beams}")
    if total_beams == target_num_beams:
        return codebook, list(range(total_beams))

    indices = np.linspace(0, total_beams - 1, target_num_beams, dtype=np.int64).tolist()
    if hasattr(codebook, "index_select"):
        try:
            import torch
            index_tensor = torch.as_tensor(indices, dtype=torch.long, device=codebook.device)
            return codebook.index_select(0, index_tensor), [int(v) for v in indices]
        except Exception:
            pass
    return codebook[indices], [int(v) for v in indices]


def make_codebook_id(codebook_type: str,
                     tx_rows: int,
                     tx_cols: int,
                     oversampling_v: int,
                     oversampling_h: int,
                     normalize_power: bool,
                     num_beams: int) -> str:
    return (
        f"{str(codebook_type or 'unknown').strip().lower()}"
        f"-r{max(int(tx_rows), 1)}"
        f"-c{max(int(tx_cols), 1)}"
        f"-ov{max(int(oversampling_v), 1)}"
        f"-oh{max(int(oversampling_h), 1)}"
        f"-np{1 if normalize_power else 0}"
        f"-nb{max(int(num_beams), 0)}"
    )


def _beam_grid_dims(codebook_type: str,
                    tx_rows: int,
                    tx_cols: int,
                    oversampling_v: int,
                    oversampling_h: int,
                    num_beams: int) -> Tuple[int, int]:
    codebook_type = str(codebook_type or "").strip().lower()
    tx_rows = max(int(tx_rows), 1)
    tx_cols = max(int(tx_cols), 1)
    oversampling_v = max(int(oversampling_v), 1)
    oversampling_h = max(int(oversampling_h), 1)
    num_beams = max(int(num_beams), 0)
    if codebook_type == "dft_ura":
        num_v = max(tx_rows * oversampling_v, 1)
        num_h = max(tx_cols * oversampling_h, 1)
        return num_v, num_h
    num_v = 1
    if tx_cols > 1:
        num_h = max(tx_cols * oversampling_h, 1)
    else:
        num_h = max(tx_rows * oversampling_v, 1)
    if num_beams > 0:
        num_h = max(num_h, num_beams)
    return num_v, num_h


def build_codebook_payload(beam_runtime: Optional[Dict[str, Any]], cfg: Any) -> Optional[Dict[str, Any]]:
    if not beam_runtime:
        return None
    codebook = beam_runtime.get("codebook")
    if codebook is None:
        return None
    try:
        if hasattr(codebook, "detach"):
            codebook_np = codebook.detach().cpu().numpy()
        else:
            codebook_np = np.asarray(codebook)
    except Exception:
        codebook_np = np.asarray(codebook)
    if codebook_np.ndim != 2:
        codebook_np = np.asarray(codebook_np).reshape(int(getattr(codebook_np, "shape", [0])[0]), -1)
    codebook_np = np.asarray(codebook_np)

    codebook_type = str(beam_runtime.get("codebook_type", ""))
    tx_rows = max(int(_get(cfg, "tx_array_num_rows", 1)), 1)
    tx_cols = max(int(_get(cfg, "tx_array_num_cols", 1)), 1)
    oversampling_v = max(int(beam_runtime.get("beam_oversampling_v", _get(cfg, "beam_oversampling_v", 1))), 1)
    oversampling_h = max(int(beam_runtime.get("beam_oversampling_h", _get(cfg, "beam_oversampling_h", 1))), 1)
    normalize_power = bool(_get(cfg, "beam_normalize_power", True))
    num_beams = int(codebook_np.shape[0]) if codebook_np.ndim >= 1 else 0
    tx_ant = int(codebook_np.shape[1]) if codebook_np.ndim >= 2 else 0
    requested_num_beams = _safe_int(beam_runtime.get("beam_codebook_num_beams", _get(cfg, "beam_codebook_num_beams", num_beams)))
    num_v, num_h = _beam_grid_dims(codebook_type, tx_rows, tx_cols, oversampling_v, oversampling_h, num_beams)
    codebook_id = make_codebook_id(codebook_type, tx_rows, tx_cols, oversampling_v, oversampling_h, normalize_power, num_beams)

    entries: List[Dict[str, Any]] = []
    for beam_index in range(num_beams):
        weights = np.asarray(codebook_np[beam_index]).reshape(-1)
        beam_index_v = beam_index // num_h if (num_v > 1 and num_h > 0) else -1
        beam_index_h = beam_index % num_h if num_h > 0 else beam_index
        entries.append({
            "beam_index": int(beam_index),
            "beam_index_v": int(beam_index_v),
            "beam_index_h": int(beam_index_h),
            "weights_real": [float(np.real(v)) for v in weights.tolist()],
            "weights_imag": [float(np.imag(v)) for v in weights.tolist()],
        })

    return {
        "codebook_id": codebook_id,
        "codebook_type": codebook_type,
        "tx_array_num_rows": tx_rows,
        "tx_array_num_cols": tx_cols,
        "beam_oversampling_v": oversampling_v,
        "beam_oversampling_h": oversampling_h,
        "beam_codebook_num_beams": int(requested_num_beams or num_beams),
        "normalize_power": normalize_power,
        "num_beams": num_beams,
        "tx_ant": tx_ant,
        "beams": entries,
    }


def _normalize_codebook_payload(payload: Dict[str, Any]) -> Tuple[Dict[str, Any], np.ndarray]:
    if not isinstance(payload, dict):
        raise ValueError("codebook payload must be a JSON object")
    beams = payload.get("beams")
    if not isinstance(beams, list) or not beams:
        raise ValueError("codebook payload must contain a non-empty 'beams' array")

    weight_rows: List[np.ndarray] = []
    normalized_beams: List[Dict[str, Any]] = []
    tx_ant: Optional[int] = None
    for idx, item in enumerate(beams):
        if not isinstance(item, dict):
            raise ValueError(f"beam entry {idx} must be an object")
        weights_real = item.get("weights_real")
        weights_imag = item.get("weights_imag")
        if not isinstance(weights_real, list) or not isinstance(weights_imag, list):
            raise ValueError(f"beam entry {idx} must contain list fields 'weights_real' and 'weights_imag'")
        if len(weights_real) != len(weights_imag) or len(weights_real) == 0:
            raise ValueError(f"beam entry {idx} has inconsistent weight lengths")
        if tx_ant is None:
            tx_ant = len(weights_real)
        elif tx_ant != len(weights_real):
            raise ValueError("all beam entries must have the same antenna dimension")
        real = np.asarray(weights_real, dtype=np.float32)
        imag = np.asarray(weights_imag, dtype=np.float32)
        weights = real.astype(np.complex64) + 1j * imag.astype(np.complex64)
        weight_rows.append(weights.reshape(1, -1))
        beam_index = _safe_int(item.get("beam_index", idx))
        beam_index_v = _safe_int(item.get("beam_index_v", -1))
        beam_index_h = _safe_int(item.get("beam_index_h", idx))
        normalized_beams.append({
            "beam_index": int(idx if beam_index is None else beam_index),
            "beam_index_v": int(-1 if beam_index_v is None else beam_index_v),
            "beam_index_h": int(idx if beam_index_h is None else beam_index_h),
            "weights_real": [float(v) for v in real.tolist()],
            "weights_imag": [float(v) for v in imag.tolist()],
        })

    codebook_np = np.concatenate(weight_rows, axis=0).astype(np.complex64)
    num_beams = int(codebook_np.shape[0])
    tx_ant = int(codebook_np.shape[1])
    codebook_type = str(payload.get("codebook_type", "custom_file") or "custom_file")
    tx_rows = max(int(_safe_int(payload.get("tx_array_num_rows", 1)) or 1), 1)
    tx_cols = max(int(_safe_int(payload.get("tx_array_num_cols", tx_ant)) or tx_ant), 1)
    oversampling_v = max(int(_safe_int(payload.get("beam_oversampling_v", 1)) or 1), 1)
    oversampling_h = max(int(_safe_int(payload.get("beam_oversampling_h", 1)) or 1), 1)
    normalize_power = bool(_safe_bool(payload.get("normalize_power", False)) or False)
    codebook_id = str(payload.get("codebook_id", "")).strip()
    if not codebook_id:
        codebook_id = make_codebook_id(codebook_type, tx_rows, tx_cols, oversampling_v, oversampling_h, normalize_power, num_beams)

    normalized_payload = {
        "codebook_id": codebook_id,
        "codebook_type": codebook_type,
        "tx_array_num_rows": tx_rows,
        "tx_array_num_cols": tx_cols,
        "beam_codebook_num_beams": num_beams,
        "beam_oversampling_v": oversampling_v,
        "beam_oversampling_h": oversampling_h,
        "normalize_power": normalize_power,
        "num_beams": num_beams,
        "tx_ant": tx_ant,
        "beams": normalized_beams,
    }
    return normalized_payload, codebook_np



def load_codebook_payload_from_file(path: str) -> Tuple[Dict[str, Any], np.ndarray]:
    with open(path, "r", encoding="utf-8") as f:
        payload = json.load(f)
    return _normalize_codebook_payload(payload)


def build_beam_snapshot_from_rows(rows: List[Dict[str, Any]],
                                  params: Any,
                                  codebook_payload: Optional[Dict[str, Any]] = None) -> Dict[str, Any]:
    if codebook_payload is None:
        codebook_type = str(_get(params, "beam_codebook_type", ""))
        tx_rows = max(int(_get(params, "tx_array_num_rows", 1)), 1)
        tx_cols = max(int(_get(params, "tx_array_num_cols", 1)), 1)
        oversampling_v = max(int(_get(params, "beam_oversampling_v", 1)), 1)
        oversampling_h = max(int(_get(params, "beam_oversampling_h", 1)), 1)
        normalize_power = bool(_get(params, "beam_normalize_power", True))
        num_beams = _safe_int(_get(params, "beam_num_beams", 0)) or 0
        codebook_payload = {
            "codebook_id": make_codebook_id(codebook_type, tx_rows, tx_cols, oversampling_v, oversampling_h, normalize_power, num_beams),
            "codebook_type": codebook_type,
            "tx_array_num_rows": tx_rows,
            "tx_array_num_cols": tx_cols,
            "beam_oversampling_v": oversampling_v,
            "beam_oversampling_h": oversampling_h,
            "normalize_power": normalize_power,
            "num_beams": num_beams,
        }

    snapshot: Dict[str, Any] = {
        "enabled": bool(_get(params, "enable_beamforming", False)),
        "sim_idx": -1,
        "odom_stamp_s": None,
        "frame_id": "",
        "codebook_id": str(codebook_payload.get("codebook_id", "")),
        "selection_mode": "",
        "codebook_type": str(codebook_payload.get("codebook_type", "")),
        "num_beams": int(codebook_payload.get("num_beams", 0) or 0),
        "manual_beam_index": max(int(_get(params, "beam_manual_index", 0)), 0),
        "feature_mode": str(_get(params, "beam_feature_mode", "geom_vel_path13")),
        "top_k": max(int(_get(params, "beam_top_k", 1)), 1),
        "predictor_available": False,
        "predictor_status": "",
        "predictor_error": "",
        "serving_source": "",
        "observations": [],
    }

    if rows:
        first = rows[0]
        snapshot["sim_idx"] = int(_safe_int(first.get("sim_idx")) or -1)
        snapshot["odom_stamp_s"] = _safe_float(first.get("odom_stamp_s"))
        snapshot["selection_mode"] = str(first.get("beam_selection_mode", ""))
        snapshot["codebook_type"] = str(first.get("beam_codebook_type", snapshot["codebook_type"]))
        snapshot["num_beams"] = int(_safe_int(first.get("beam_num_beams")) or snapshot["num_beams"])
        snapshot["feature_mode"] = str(first.get("beam_feature_mode", snapshot["feature_mode"]))
        snapshot["top_k"] = int(_safe_int(first.get("beam_top_k")) or snapshot["top_k"])
        snapshot["enabled"] = bool(_safe_bool(first.get("beam_enabled")) if _safe_bool(first.get("beam_enabled")) is not None else snapshot["enabled"])

    for bs_index, row in enumerate(rows or []):
        predictor_status = str(row.get("beam_predictor_status", ""))
        selected_beam_index = int(_safe_int(row.get("beam_selected_index")) or -1)
        oracle_beam_index = int(_safe_int(row.get("beam_oracle_index")) or -1)
        predicted_beam_index = int(_safe_int(row.get("beam_predicted_index")) or -1)
        raw_beam_hit = _safe_bool(row.get("beam_hit"))
        raw_oracle_in_topk = _safe_bool(row.get("beam_oracle_in_topk"))
        topk_text = str(row.get("beam_topk_indices", "")).strip()
        parsed_indices: List[int] = []
        if topk_text:
            for token in topk_text.split(","):
                token = token.strip()
                if not token:
                    continue
                maybe_int = _safe_int(token)
                if maybe_int is not None:
                    parsed_indices.append(int(maybe_int))

        if selected_beam_index >= 0 and oracle_beam_index >= 0:
            beam_hit = selected_beam_index == oracle_beam_index
        else:
            beam_hit = bool(raw_beam_hit) if raw_beam_hit is not None else False

        if oracle_beam_index >= 0 and parsed_indices:
            oracle_in_topk = oracle_beam_index in parsed_indices
        elif oracle_beam_index >= 0 and predicted_beam_index >= 0 and int(_safe_int(row.get("beam_top_k")) or snapshot["top_k"]) == 1:
            oracle_in_topk = predicted_beam_index == oracle_beam_index
        else:
            oracle_in_topk = bool(raw_oracle_in_topk) if raw_oracle_in_topk is not None else False

        obs = {
            "sim_idx": int(_safe_int(row.get("sim_idx")) or snapshot["sim_idx"]),
            "odom_stamp_s": _safe_float(row.get("odom_stamp_s")) if _safe_float(row.get("odom_stamp_s")) is not None else snapshot["odom_stamp_s"],
            "bs_index": int(bs_index),
            "bs_name": str(row.get("bs_name", "")),
            "anchor_id": int(_safe_int(row.get("anchor_id")) or -1),
            "tx_idx": int(bs_index),
            "beamforming_enabled": bool(_safe_bool(row.get("beam_enabled")) if _safe_bool(row.get("beam_enabled")) is not None else snapshot["enabled"]),
            "selection_mode": str(row.get("beam_selection_mode", snapshot["selection_mode"])),
            "selection_source": str(row.get("beam_selection_source", "")),
            "serving_source": str(row.get("beam_serving_source", "")),
            "codebook_id": snapshot["codebook_id"],
            "codebook_type": str(row.get("beam_codebook_type", snapshot["codebook_type"])),
            "num_beams": int(_safe_int(row.get("beam_num_beams")) or snapshot["num_beams"]),
            "manual_beam_index": int(_safe_int(row.get("beam_manual_index")) or snapshot["manual_beam_index"]),
            "feature_mode": str(row.get("beam_feature_mode", snapshot["feature_mode"])),
            "top_k": int(_safe_int(row.get("beam_top_k")) or snapshot["top_k"]),
            "predictor_available": False,
            "predictor_status": predictor_status,
            "predictor_error": str(row.get("beam_predictor_error", "")),
            "selected_beam_index": selected_beam_index,
            "selected_beam_gain_db": _safe_float(row.get("beam_selected_gain_db")),
            "oracle_beam_index": oracle_beam_index,
            "oracle_beam_gain_db": _safe_float(row.get("beam_oracle_gain_db")),
            "predicted_beam_index": predicted_beam_index,
            "predicted_beam_confidence": _safe_float(row.get("beam_predicted_confidence")),
            "is_serving_bs": bool(_safe_bool(row.get("sys_is_serving_bs")) or False),
            "beam_hit": bool(beam_hit),
            "oracle_in_topk": bool(oracle_in_topk),
            "topk_indices": parsed_indices,
            "topk_probabilities": [],
            "feature_vector": [],
        }
        snapshot["observations"].append(obs)
        if obs["is_serving_bs"]:
            snapshot["predictor_status"] = predictor_status
            snapshot["predictor_error"] = obs["predictor_error"]
            snapshot["serving_source"] = obs["serving_source"]
            snapshot["predictor_available"] = predictor_status == "ready"

    return snapshot


def _make_point(Point, xyz: Optional[Any]):
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


def build_beam_codebook_msg(rospy: Any,
                            Header: Any,
                            BeamCodebook: Any,
                            BeamEntry: Any,
                            payload: Optional[Dict[str, Any]],
                            stamp_s: float,
                            frame_id: str):
    if not payload:
        return None
    msg = BeamCodebook()
    msg.header = Header()
    msg.header.stamp = rospy.Time.from_sec(float(stamp_s))
    msg.header.frame_id = frame_id
    msg.codebook_id = str(payload.get("codebook_id", ""))
    msg.codebook_type = str(payload.get("codebook_type", ""))
    msg.tx_array_num_rows = int(payload.get("tx_array_num_rows", 0) or 0)
    msg.tx_array_num_cols = int(payload.get("tx_array_num_cols", 0) or 0)
    msg.beam_oversampling_v = int(payload.get("beam_oversampling_v", 0) or 0)
    msg.beam_oversampling_h = int(payload.get("beam_oversampling_h", 0) or 0)
    msg.normalize_power = bool(payload.get("normalize_power", False))
    msg.num_beams = int(payload.get("num_beams", 0) or 0)
    msg.tx_ant = int(payload.get("tx_ant", 0) or 0)
    for item in payload.get("beams", []) or []:
        beam_msg = BeamEntry()
        beam_msg.beam_index = int(item.get("beam_index", 0) or 0)
        beam_msg.beam_index_v = int(item.get("beam_index_v", -1) if item.get("beam_index_v") is not None else -1)
        beam_msg.beam_index_h = int(item.get("beam_index_h", -1) if item.get("beam_index_h") is not None else -1)
        beam_msg.weights_real = [float(v) for v in item.get("weights_real", []) or []]
        beam_msg.weights_imag = [float(v) for v in item.get("weights_imag", []) or []]
        msg.beams.append(beam_msg)
    return msg


def build_beam_observation_array_msg(rospy: Any,
                                     Header: Any,
                                     BeamObservationArray: Any,
                                     BeamObservation: Any,
                                     rows: List[Dict[str, Any]],
                                     params: Any,
                                     stamp_s: float,
                                     frame_id: str,
                                     codebook_payload: Optional[Dict[str, Any]] = None):
    snapshot = build_beam_snapshot_from_rows(rows=rows, params=params, codebook_payload=codebook_payload)
    msg = BeamObservationArray()
    msg.header = Header()
    msg.header.stamp = rospy.Time.from_sec(float(stamp_s))
    msg.header.frame_id = frame_id
    msg.sim_idx = max(int(snapshot.get("sim_idx", -1)), 0)
    msg.odom_stamp_s = float(snapshot.get("odom_stamp_s") or stamp_s)
    msg.frame_id = str(frame_id)
    msg.codebook_id = str(snapshot.get("codebook_id", ""))
    msg.selection_mode = str(snapshot.get("selection_mode", ""))

    for item in snapshot.get("observations", []) or []:
        obs = BeamObservation()
        obs.header = Header()
        obs.header.stamp = msg.header.stamp
        obs.header.frame_id = frame_id
        obs.sim_idx = int(item.get("sim_idx", -1))
        obs.odom_stamp_s = float(item.get("odom_stamp_s") or stamp_s)
        obs.frame_id = str(frame_id)
        obs.bs_index = int(item.get("bs_index", 0) or 0)
        obs.bs_name = str(item.get("bs_name", ""))
        obs.anchor_id = int(item.get("anchor_id", 0) if item.get("anchor_id", None) is not None and int(item.get("anchor_id", -1)) >= 0 else 0)
        obs.tx_idx = int(item.get("tx_idx", -1))
        obs.beamforming_enabled = bool(item.get("beamforming_enabled", False))
        obs.selection_mode = str(item.get("selection_mode", ""))
        obs.selection_source = str(item.get("selection_source", ""))
        obs.serving_source = str(item.get("serving_source", ""))
        obs.codebook_id = str(item.get("codebook_id", ""))
        obs.codebook_type = str(item.get("codebook_type", ""))
        obs.num_beams = int(item.get("num_beams", 0) or 0)
        obs.manual_beam_index = int(item.get("manual_beam_index", -1))
        obs.feature_mode = str(item.get("feature_mode", ""))
        obs.top_k = int(item.get("top_k", 0) or 0)
        obs.predictor_available = bool(item.get("predictor_available", False))
        obs.predictor_status = str(item.get("predictor_status", ""))
        obs.predictor_error = str(item.get("predictor_error", ""))
        obs.selected_beam_index = int(item.get("selected_beam_index", -1))
        obs.selected_beam_gain_db = float(item.get("selected_beam_gain_db") if item.get("selected_beam_gain_db") is not None else float("nan"))
        obs.oracle_beam_index = int(item.get("oracle_beam_index", -1))
        obs.oracle_beam_gain_db = float(item.get("oracle_beam_gain_db") if item.get("oracle_beam_gain_db") is not None else float("nan"))
        obs.predicted_beam_index = int(item.get("predicted_beam_index", -1))
        obs.predicted_beam_confidence = float(item.get("predicted_beam_confidence") if item.get("predicted_beam_confidence") is not None else float("nan"))
        obs.is_serving_bs = bool(item.get("is_serving_bs", False))
        obs.beam_hit = bool(item.get("beam_hit", False))
        obs.oracle_in_topk = bool(item.get("oracle_in_topk", False))
        obs.topk_indices = [int(v) for v in item.get("topk_indices", []) or []]
        obs.topk_probabilities = [float(v) for v in item.get("topk_probabilities", []) or []]
        obs.feature_vector = [float(v) for v in item.get("feature_vector", []) or []]
        msg.observations.append(obs)
    return msg
