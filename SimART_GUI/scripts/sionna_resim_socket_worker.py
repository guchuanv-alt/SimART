#!/usr/bin/env python3
from __future__ import annotations

import argparse
import json
import signal
import socket
import struct
import sys
import time
import traceback
from typing import Any, Dict, List, Optional

import numpy as np

from sionna_sim_only_topic2 import OfflineSionnaSimulator, SimulationConfig, load_bs_list_from_json
from sionna_beam_topic_utils import build_codebook_payload

_STOP_REQUESTED = False
_PACKET_HEADER = struct.Struct("!Q")


def _handle_stop(signum, _frame):
    global _STOP_REQUESTED
    _STOP_REQUESTED = True
    print(f"[stop] received signal {signum}; worker will stop after the current request", flush=True)


signal.signal(signal.SIGINT, _handle_stop)
signal.signal(signal.SIGTERM, _handle_stop)



def _bool_arg(text: str) -> bool:
    return str(text).strip().lower() in {"1", "true", "yes", "on"}


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



def _connect(host: str, port: int, timeout_s: float = 30.0) -> socket.socket:
    deadline = time.time() + timeout_s
    last_error: Optional[BaseException] = None
    while time.time() < deadline and not _STOP_REQUESTED:
        sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        try:
            sock.connect((host, port))
            return sock
        except OSError as exc:
            last_error = exc
            sock.close()
            time.sleep(0.2)
    raise RuntimeError(f"Could not connect to socket controller at {host}:{port}: {last_error}")



def _build_simulator(args: argparse.Namespace) -> OfflineSionnaSimulator:
    bs_list = load_bs_list_from_json(args.bs_list_json)
    sim_cfg = SimulationConfig(
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
        beam_selection_mode=args.beam_selection_mode,
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
        beam_export_training_dataset=args.beam_export_training_dataset,
        beam_training_dataset_path=args.beam_training_dataset_path,
        beam_training_reset_dataset_on_start=args.beam_training_reset_dataset_on_start,
        beam_training_output_checkpoint_path=args.beam_training_output_checkpoint_path,
        beam_training_epochs=args.beam_training_epochs,
        beam_training_batch_size=args.beam_training_batch_size,
        beam_training_learning_rate=args.beam_training_learning_rate,
        beam_training_validation_split=args.beam_training_validation_split,
        beam_training_hidden_dim=args.beam_training_hidden_dim,
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
    return OfflineSionnaSimulator(sim_cfg)



def parse_args(argv: Optional[List[str]] = None) -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="Socket worker for offline Sionna rosbag re-simulation")
    parser.add_argument("--host", required=True)
    parser.add_argument("--port", type=int, required=True)
    parser.add_argument("--scene-path", required=True)
    parser.add_argument("--bs-list-json", required=True)
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
    parser.add_argument("--beam-export-training-dataset", type=_bool_arg, default=False)
    parser.add_argument("--beam-training-dataset-path", default="beam_training_samples.csv")
    parser.add_argument("--beam-training-reset-dataset-on-start", type=_bool_arg, default=False)
    parser.add_argument("--beam-training-output-checkpoint-path", default="beam_trained_model.pt")
    parser.add_argument("--beam-training-epochs", type=int, default=30)
    parser.add_argument("--beam-training-batch-size", type=int, default=256)
    parser.add_argument("--beam-training-learning-rate", type=float, default=1e-3)
    parser.add_argument("--beam-training-validation-split", type=float, default=0.2)
    parser.add_argument("--beam-training-hidden-dim", type=int, default=512)
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
    return parser.parse_args(argv)



def main(argv: Optional[List[str]] = None) -> int:
    args = parse_args(argv)
    print(f"[info] worker python: {sys.executable}", flush=True)
    print("[info] initializing OfflineSionnaSimulator", flush=True)
    simulator = _build_simulator(args)
    sock = _connect(args.host, args.port)
    try:
        _send_packet(sock, {
            "kind": "hello",
            "message": f"OfflineSionnaSimulator ready with {len(simulator.cfg.bs_list)} base stations",
            "beam_codebook": _json_compatible(build_codebook_payload(simulator.beam_runtime, simulator.cfg)),
        })
        while not _STOP_REQUESTED:
            request = _recv_packet(sock)
            if not isinstance(request, dict):
                raise RuntimeError(f"Unsupported request payload: {request!r}")
            kind = request.get("kind")
            if kind == "stop":
                print("[info] stop requested by controller", flush=True)
                break
            if kind != "simulate":
                raise RuntimeError(f"Unsupported request kind: {request!r}")
            sim_idx = int(request.get("sim_idx", 0))
            bag_time_s = float(request.get("bag_time_s", 0.0))
            stamp_s = float(request.get("stamp_s", bag_time_s))
            pos_xyz = np.asarray(request.get("pos_xyz", [0.0, 0.0, 0.0]), dtype=float)
            rx_quat_xyzw = np.asarray(request.get("rx_quat_xyzw", [0.0, 0.0, 0.0, 1.0]), dtype=float)
            vel_xyz = np.asarray(request.get("vel_xyz", [0.0, 0.0, 0.0]), dtype=float)
            print(f"[sim] snapshot {sim_idx}: stamp={stamp_s:.6f} pos={pos_xyz.tolist()}", flush=True)
            try:
                meas_df, anchor_payloads = simulator.simulate_from_odometry_snapshot(
                    sim_idx=sim_idx,
                    odom_stamp_s=stamp_s,
                    pos_xyz=pos_xyz,
                    rx_quat_xyzw=rx_quat_xyzw,
                    vel_xyz=vel_xyz,
                )
                _send_packet(sock, {
                    "kind": "result",
                    "sim_idx": sim_idx,
                    "bag_time_s": bag_time_s,
                    "stamp_s": stamp_s,
                    "rx_position": pos_xyz.tolist(),
                    "rx_velocity": vel_xyz.tolist(),
                    "anchor_payloads": _json_compatible(anchor_payloads),
                    "measurement_rows": _json_compatible(meas_df.to_dict(orient="records")),
                })
            except Exception as exc:
                tb = traceback.format_exc()
                print(f"[sim-error] snapshot {sim_idx}: {exc}", flush=True)
                print(tb, flush=True)
                _send_packet(sock, {
                    "kind": "error",
                    "sim_idx": sim_idx,
                    "message": str(exc),
                    "traceback": tb,
                })
        return 0
    finally:
        try:
            sock.shutdown(socket.SHUT_RDWR)
        except Exception:
            pass
        sock.close()


if __name__ == "__main__":
    try:
        raise SystemExit(main())
    except Exception as exc:
        print(f"[error] {exc}", flush=True)
        print(traceback.format_exc(), flush=True)
        raise
