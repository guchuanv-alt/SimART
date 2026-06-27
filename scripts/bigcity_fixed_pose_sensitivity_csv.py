#!/usr/bin/env python3
from __future__ import annotations

import argparse
import csv
import json
import math
import sys
import time
from pathlib import Path
from types import SimpleNamespace
from typing import Any, Dict, Iterable, List, Optional

REPO_ROOT = Path(__file__).resolve().parents[1]
SIMART_SCRIPT_DIR = REPO_ROOT / "SimART_GUI" / "scripts"
sys.path.insert(0, str(SIMART_SCRIPT_DIR))

import reprocess_rosbag_with_sionna as rp  # noqa: E402


DEFAULT_VARIANTS = [
    (
        "00_baseline",
        "BigCitySample_sensitivity_00_baseline_concrete_placeholder.xml",
    ),
    (
        "01_small_low",
        "BigCitySample_sensitivity_01_small_low_interaction.xml",
    ),
    (
        "02_small_medium",
        "BigCitySample_sensitivity_02_small_medium_interaction.xml",
    ),
    (
        "03_small_high",
        "BigCitySample_sensitivity_03_small_high_interaction.xml",
    ),
]


SUMMARY_FIELDS = [
    "variant",
    "sample_id",
    "stamp_s",
    "bag_time_s",
    "rx_x",
    "rx_y",
    "rx_z",
    "total_paths",
    "los_paths",
    "nlos_paths",
    "anchors_with_paths",
    "strongest_power_db",
    "strongest_anchor_id",
    "strongest_anchor_name",
    "strongest_path_type",
    "strongest_path_order",
    "strongest_tau_s",
    "strongest_doppler_hz",
    "best_nlos_power_db",
    "best_nlos_anchor_id",
    "best_nlos_anchor_name",
    "best_nlos_tau_s",
    "elapsed_s",
]

ANCHOR_FIELDS = [
    "variant",
    "sample_id",
    "stamp_s",
    "rx_x",
    "rx_y",
    "rx_z",
    "anchor_id",
    "anchor_name",
    "num_paths",
    "anchor_best_power_db",
    "anchor_best_path_type",
    "anchor_best_path_order",
    "anchor_best_tau_s",
]

DELTA_FIELDS = [
    "variant",
    "sample_id",
    "rx_x",
    "rx_y",
    "rx_z",
    "baseline_total_paths",
    "variant_total_paths",
    "delta_total_paths",
    "baseline_strongest_power_db",
    "variant_strongest_power_db",
    "delta_strongest_power_db",
    "baseline_best_nlos_power_db",
    "variant_best_nlos_power_db",
    "delta_best_nlos_power_db",
    "baseline_strongest_anchor",
    "variant_strongest_anchor",
]


def _bool_arg(text: str) -> bool:
    return str(text).strip().lower() in {"1", "true", "yes", "on"}


def _float_or_none(value: Any) -> Optional[float]:
    try:
        if value is None or value == "":
            return None
        out = float(value)
        return out if math.isfinite(out) else None
    except Exception:
        return None


def _fmt(value: Any, digits: int = 9) -> str:
    number = _float_or_none(value)
    if number is None:
        return ""
    return f"{number:.{digits}f}"


def _fmt_sci(value: Any) -> str:
    number = _float_or_none(value)
    if number is None:
        return ""
    return f"{number:.12g}"


def _variant_specs(raw: Iterable[str]) -> List[tuple[str, str]]:
    specs: List[tuple[str, str]] = []
    for item in raw:
        if "=" not in item:
            raise ValueError(f"Variant must be label=xml, got: {item}")
        label, xml = item.split("=", 1)
        label = label.strip()
        xml = xml.strip()
        if not label or not xml:
            raise ValueError(f"Variant must be label=xml, got: {item}")
        specs.append((label, xml))
    return specs or list(DEFAULT_VARIANTS)


def _worker_args(args: argparse.Namespace, scene_path: Path) -> SimpleNamespace:
    return SimpleNamespace(
        input_bag=str(args.input_bag),
        pose_topic=args.pose_topic,
        interval_s=args.interval_s,
        scene_path=str(scene_path),
        bs_list_json=str(args.bs_list_json),
        sionna_python=str(args.sionna_python),
        worker_script=str(args.worker_script),
        worker_timeout_s=float(args.worker_timeout_s),
        fc_hz=float(args.fc_hz),
        mi_variant=args.mi_variant,
        tx_array_num_rows=int(args.tx_array_num_rows),
        tx_array_num_cols=int(args.tx_array_num_cols),
        tx_array_vertical_spacing=float(args.tx_array_vertical_spacing),
        tx_array_horizontal_spacing=float(args.tx_array_horizontal_spacing),
        tx_array_pattern=args.tx_array_pattern,
        tx_array_polarization=args.tx_array_polarization,
        rx_array_num_rows=int(args.rx_array_num_rows),
        rx_array_num_cols=int(args.rx_array_num_cols),
        rx_array_vertical_spacing=float(args.rx_array_vertical_spacing),
        rx_array_horizontal_spacing=float(args.rx_array_horizontal_spacing),
        rx_array_pattern=args.rx_array_pattern,
        rx_array_polarization=args.rx_array_polarization,
        max_depth=int(args.max_depth),
        samples_per_src=int(args.samples_per_src),
        max_num_paths_per_src=int(args.max_num_paths_per_src),
        synthetic_array=bool(args.synthetic_array),
        merge_shapes=bool(args.merge_shapes),
        enable_sys_integration=bool(args.enable_sys_integration),
        enable_beamforming=bool(args.enable_beamforming),
        beam_selection_mode=args.beam_selection_mode,
        beam_codebook_type=args.beam_codebook_type,
        beam_codebook_num_beams=int(args.beam_codebook_num_beams),
        beam_oversampling_v=int(args.beam_oversampling_v),
        beam_oversampling_h=int(args.beam_oversampling_h),
        beam_manual_index=int(args.beam_manual_index),
        beam_normalize_power=bool(args.beam_normalize_power),
        beam_codebook_file=args.beam_codebook_file,
        beam_model_checkpoint_path=args.beam_model_checkpoint_path,
        beam_feature_mode=args.beam_feature_mode,
        beam_top_k=int(args.beam_top_k),
        sys_num_subcarriers=int(args.sys_num_subcarriers),
        sys_subcarrier_spacing_hz=float(args.sys_subcarrier_spacing_hz),
        sys_num_ofdm_symbols=int(args.sys_num_ofdm_symbols),
        sys_temperature_k=float(args.sys_temperature_k),
        sys_bler_target=float(args.sys_bler_target),
        sys_mcs_table_index=int(args.sys_mcs_table_index),
        sys_bs_tx_power_dbm=float(args.sys_bs_tx_power_dbm),
        los=bool(args.los),
        specular_reflection=bool(args.specular_reflection),
        diffuse_reflection=bool(args.diffuse_reflection),
        refraction=bool(args.refraction),
        diffraction=bool(args.diffraction),
        edge_diffraction=bool(args.edge_diffraction),
        diffraction_lit_region=bool(args.diffraction_lit_region),
    )


def _frame_helper(args: argparse.Namespace) -> rp.FrameTransformHelper:
    return rp.FrameTransformHelper(
        ros_to_scene_matrix_json=args.ros_to_map_matrix_json,
        airsim_to_scene_matrix_json=args.airsim_to_scene_matrix_json,
        ros_frame_id=args.rf_frame_id,
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


def _best_path(anchor_payload: Dict[str, Any]) -> Optional[Dict[str, Any]]:
    best = None
    for path in anchor_payload.get("paths", []) or []:
        power = _float_or_none(path.get("power_db"))
        if power is None:
            continue
        if best is None or power > best["power_db"]:
            best = {
                "power_db": power,
                "path_type": str(path.get("path_type", "")),
                "path_order": int(path.get("path_order", -1) or -1),
                "tau_s": _float_or_none(path.get("tau_s")),
                "doppler_hz": _float_or_none(path.get("doppler_hz")),
            }
    return best


def _summarize_response(
    variant: str,
    sample_id: int,
    snap: Any,
    response: Dict[str, Any],
    elapsed_s: float,
) -> tuple[Dict[str, Any], List[Dict[str, Any]]]:
    anchor_payloads = response.get("anchor_payloads", []) or []
    strongest = None
    best_nlos = None
    total_paths = 0
    los_paths = 0
    nlos_paths = 0
    anchors_with_paths = 0
    anchor_rows: List[Dict[str, Any]] = []

    for anchor in anchor_payloads:
        paths = anchor.get("paths", []) or []
        if paths:
            anchors_with_paths += 1
        total_paths += len(paths)
        for path in paths:
            ptype = str(path.get("path_type", "")).upper()
            if ptype == "LOS":
                los_paths += 1
            elif ptype == "NLOS":
                nlos_paths += 1
            power = _float_or_none(path.get("power_db"))
            if power is None:
                continue
            candidate = {
                "power_db": power,
                "anchor_id": int(anchor.get("anchor_id", -1) or -1),
                "anchor_name": str(anchor.get("anchor_name", "")),
                "path_type": ptype,
                "path_order": int(path.get("path_order", -1) or -1),
                "tau_s": _float_or_none(path.get("tau_s")),
                "doppler_hz": _float_or_none(path.get("doppler_hz")),
            }
            if strongest is None or power > strongest["power_db"]:
                strongest = candidate
            if ptype == "NLOS" and (best_nlos is None or power > best_nlos["power_db"]):
                best_nlos = candidate

        anchor_best = _best_path(anchor)
        if anchor_best is not None:
            anchor_rows.append({
                "variant": variant,
                "sample_id": sample_id,
                "stamp_s": _fmt_sci(snap.stamp_s),
                "rx_x": _fmt(snap.pos_xyz[0]),
                "rx_y": _fmt(snap.pos_xyz[1]),
                "rx_z": _fmt(snap.pos_xyz[2]),
                "anchor_id": int(anchor.get("anchor_id", -1) or -1),
                "anchor_name": str(anchor.get("anchor_name", "")),
                "num_paths": len(paths),
                "anchor_best_power_db": _fmt(anchor_best["power_db"]),
                "anchor_best_path_type": anchor_best["path_type"],
                "anchor_best_path_order": anchor_best["path_order"],
                "anchor_best_tau_s": _fmt_sci(anchor_best["tau_s"]),
            })

    summary = {
        "variant": variant,
        "sample_id": sample_id,
        "stamp_s": _fmt_sci(snap.stamp_s),
        "bag_time_s": _fmt_sci(snap.bag_time_s),
        "rx_x": _fmt(snap.pos_xyz[0]),
        "rx_y": _fmt(snap.pos_xyz[1]),
        "rx_z": _fmt(snap.pos_xyz[2]),
        "total_paths": total_paths,
        "los_paths": los_paths,
        "nlos_paths": nlos_paths,
        "anchors_with_paths": anchors_with_paths,
        "strongest_power_db": "" if strongest is None else _fmt(strongest["power_db"]),
        "strongest_anchor_id": "" if strongest is None else strongest["anchor_id"],
        "strongest_anchor_name": "" if strongest is None else strongest["anchor_name"],
        "strongest_path_type": "" if strongest is None else strongest["path_type"],
        "strongest_path_order": "" if strongest is None else strongest["path_order"],
        "strongest_tau_s": "" if strongest is None else _fmt_sci(strongest["tau_s"]),
        "strongest_doppler_hz": "" if strongest is None else _fmt(strongest["doppler_hz"]),
        "best_nlos_power_db": "" if best_nlos is None else _fmt(best_nlos["power_db"]),
        "best_nlos_anchor_id": "" if best_nlos is None else best_nlos["anchor_id"],
        "best_nlos_anchor_name": "" if best_nlos is None else best_nlos["anchor_name"],
        "best_nlos_tau_s": "" if best_nlos is None else _fmt_sci(best_nlos["tau_s"]),
        "elapsed_s": _fmt(elapsed_s, digits=3),
    }
    return summary, anchor_rows


def _write_delta_csv(summary_rows: List[Dict[str, Any]], path: Path) -> None:
    baseline = {
        int(row["sample_id"]): row
        for row in summary_rows
        if row["variant"] == "00_baseline"
    }
    delta_rows: List[Dict[str, Any]] = []
    for row in summary_rows:
        if row["variant"] == "00_baseline":
            continue
        sid = int(row["sample_id"])
        base = baseline.get(sid)
        if not base:
            continue
        base_power = _float_or_none(base["strongest_power_db"])
        row_power = _float_or_none(row["strongest_power_db"])
        base_nlos = _float_or_none(base["best_nlos_power_db"])
        row_nlos = _float_or_none(row["best_nlos_power_db"])
        delta_rows.append({
            "variant": row["variant"],
            "sample_id": sid,
            "rx_x": row["rx_x"],
            "rx_y": row["rx_y"],
            "rx_z": row["rx_z"],
            "baseline_total_paths": base["total_paths"],
            "variant_total_paths": row["total_paths"],
            "delta_total_paths": int(row["total_paths"]) - int(base["total_paths"]),
            "baseline_strongest_power_db": base["strongest_power_db"],
            "variant_strongest_power_db": row["strongest_power_db"],
            "delta_strongest_power_db": "" if base_power is None or row_power is None else _fmt(row_power - base_power),
            "baseline_best_nlos_power_db": base["best_nlos_power_db"],
            "variant_best_nlos_power_db": row["best_nlos_power_db"],
            "delta_best_nlos_power_db": "" if base_nlos is None or row_nlos is None else _fmt(row_nlos - base_nlos),
            "baseline_strongest_anchor": base["strongest_anchor_name"],
            "variant_strongest_anchor": row["strongest_anchor_name"],
        })

    with path.open("w", newline="", encoding="utf-8") as handle:
        writer = csv.DictWriter(handle, fieldnames=DELTA_FIELDS)
        writer.writeheader()
        writer.writerows(delta_rows)


def _load_existing_summary(path: Path) -> tuple[List[Dict[str, Any]], set[tuple[str, int]]]:
    rows: List[Dict[str, Any]] = []
    completed: set[tuple[str, int]] = set()
    if not path.exists():
        return rows, completed
    with path.open("r", newline="", encoding="utf-8") as handle:
        reader = csv.DictReader(handle)
        for row in reader:
            variant = str(row.get("variant", "")).strip()
            try:
                sample_id = int(row.get("sample_id", ""))
            except Exception:
                continue
            if not variant:
                continue
            rows.append(row)
            completed.add((variant, sample_id))
    return rows, completed


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="Run BigCity fixed-pose Sionna sensitivity and write CSV summaries only")
    parser.add_argument("--input-bag", default="SimART_sample_rosbags/BigCitySample_circular_flight.bag")
    parser.add_argument("--pose-topic", default="/airsim_node/PX4/odom_local_ned")
    parser.add_argument("--scene-dir", default="SimART_sample_maps/BigCitySample/BigCitySample_strict_review_sionna")
    parser.add_argument("--bs-list-json", default="SimART_sample_maps/BigCitySample/config/bs_list_simplified.json")
    parser.add_argument("--out-dir", default="SimART_sample_maps/BigCitySample/output/fixed_pose_sensitivity_csv")
    parser.add_argument("--variant", action="append", default=[], help="label=xml. Defaults to the four BigCity sensitivity XMLs.")
    parser.add_argument("--resume", type=_bool_arg, default=False, help="Append to existing CSVs and skip completed variant/sample rows.")
    parser.add_argument("--interval-s", type=float, default=10.0)
    parser.add_argument("--sionna-python", default="/home/ubuntu2004/miniconda3/envs/SimART/bin/python")
    parser.add_argument("--worker-script", default=str(SIMART_SCRIPT_DIR / "sionna_resim_socket_worker.py"))
    parser.add_argument("--worker-timeout-s", type=float, default=900.0)
    parser.add_argument("--fc-hz", type=float, default=3.5e9)
    parser.add_argument("--mi-variant", default="llvm_ad_mono_polarized")
    parser.add_argument("--tx-array-num-rows", type=int, default=8)
    parser.add_argument("--tx-array-num-cols", type=int, default=8)
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
    parser.add_argument("--max-depth", type=int, default=2)
    parser.add_argument("--samples-per-src", type=int, default=64)
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
    parser.add_argument("--ros-to-map-matrix-json", default="")
    parser.add_argument("--airsim-to-scene-matrix-json", default="")
    parser.add_argument("--rx-array-facing-direction", default="front")
    parser.add_argument("--uav-to-rx-array-tx", type=float, default=0.0)
    parser.add_argument("--uav-to-rx-array-ty", type=float, default=0.0)
    parser.add_argument("--uav-to-rx-array-tz", type=float, default=0.0)
    parser.add_argument("--uav-to-rx-array-center-matrix-json", default="")
    parser.add_argument("--rx-array-elements-json", default="")
    parser.add_argument("--output-frame-key", default="3d")
    parser.add_argument("--rf-frame-id", default="map")
    return parser.parse_args()


def main() -> int:
    args = parse_args()
    args.output_frame_key = rp.normalize_output_frame_key(args.output_frame_key)
    out_dir = Path(args.out_dir)
    out_dir.mkdir(parents=True, exist_ok=True)
    variants = _variant_specs(args.variant)
    scene_dir = Path(args.scene_dir)
    frame_helper = _frame_helper(args)
    sample_args = _worker_args(args, scene_dir / variants[0][1])

    print("[info] fixed-pose CSV sensitivity run, not CKM", flush=True)
    print(f"[info] input bag: {args.input_bag}", flush=True)
    print(f"[info] interval_s: {args.interval_s}", flush=True)
    print(f"[info] max_depth: {args.max_depth}", flush=True)
    print(f"[info] samples_per_src: {args.samples_per_src}", flush=True)
    samples, start_time, end_time = rp._collect_pose_samples(sample_args, frame_helper)
    print(f"[info] selected {len(samples)} fixed poses from bag span {start_time:.3f} -> {end_time:.3f}", flush=True)

    summary_path = out_dir / "rf_summary.csv"
    anchor_path = out_dir / "rf_anchor_summary.csv"
    delta_path = out_dir / "rf_deltas_vs_baseline.csv"
    meta_path = out_dir / "meta.json"

    summary_rows: List[Dict[str, Any]] = []
    completed: set[tuple[str, int]] = set()
    if args.resume:
        summary_rows, completed = _load_existing_summary(summary_path)
        print(f"[resume] loaded {len(completed)} completed variant/sample rows from {summary_path}", flush=True)

    summary_mode = "a" if args.resume and summary_path.exists() else "w"
    anchor_mode = "a" if args.resume and anchor_path.exists() else "w"
    with summary_path.open(summary_mode, newline="", encoding="utf-8") as summary_handle, \
            anchor_path.open(anchor_mode, newline="", encoding="utf-8") as anchor_handle:
        summary_writer = csv.DictWriter(summary_handle, fieldnames=SUMMARY_FIELDS)
        anchor_writer = csv.DictWriter(anchor_handle, fieldnames=ANCHOR_FIELDS)
        if summary_mode == "w":
            summary_writer.writeheader()
        if anchor_mode == "w":
            anchor_writer.writeheader()

        for variant_index, (variant, xml_name) in enumerate(variants, start=1):
            scene_path = scene_dir / xml_name
            print(f"[variant] {variant_index}/{len(variants)} {variant}: {scene_path}", flush=True)
            pending = [
                (sample_id, snap)
                for sample_id, snap in enumerate(samples, start=1)
                if (variant, sample_id) not in completed
            ]
            if not pending:
                print(f"[resume] skipping {variant}: all {len(samples)} samples already complete", flush=True)
                continue
            worker_args = _worker_args(args, scene_path)
            session = None
            try:
                session = rp._start_worker(worker_args)
                for sample_id, snap in pending:
                    request = {
                        "kind": "simulate",
                        "sim_idx": sample_id,
                        "bag_time_s": snap.bag_time_s,
                        "stamp_s": snap.stamp_s,
                        "pos_xyz": snap.pos_xyz.tolist(),
                        "rx_quat_xyzw": snap.rx_quat_xyzw.tolist(),
                        "vel_xyz": snap.vel_xyz.tolist(),
                    }
                    t0 = time.time()
                    rp._send_packet(session.sock, request)
                    response = rp._recv_packet(session.sock)
                    elapsed_s = time.time() - t0
                    if not isinstance(response, dict):
                        raise RuntimeError(f"Unexpected worker response: {response!r}")
                    if response.get("kind") == "error":
                        detail = str(response.get("message", "Unknown Sionna worker error"))
                        tb = str(response.get("traceback", "")).strip()
                        if tb:
                            detail = f"{detail}\n{tb}"
                        raise RuntimeError(detail)
                    if response.get("kind") != "result":
                        raise RuntimeError(f"Unexpected worker response kind: {response!r}")
                    summary, anchor_rows = _summarize_response(variant, sample_id, snap, response, elapsed_s)
                    summary_writer.writerow(summary)
                    summary_handle.flush()
                    summary_rows.append(summary)
                    completed.add((variant, sample_id))
                    for anchor_row in anchor_rows:
                        anchor_writer.writerow(anchor_row)
                    anchor_handle.flush()
                    print(
                        f"[progress] {variant} sample {sample_id}/{len(samples)} "
                        f"paths={summary['total_paths']} los={summary['los_paths']} "
                        f"nlos={summary['nlos_paths']} strongest={summary['strongest_power_db'] or 'NA'} "
                        f"elapsed={elapsed_s:.1f}s",
                        flush=True,
                    )
            finally:
                rp._stop_worker(session)

    _write_delta_csv(summary_rows, delta_path)
    meta = {
        "input_bag": str(args.input_bag),
        "pose_topic": args.pose_topic,
        "interval_s": args.interval_s,
        "sample_count": len(samples),
        "variants": [{"label": label, "xml": xml} for label, xml in variants],
        "max_depth": args.max_depth,
        "samples_per_src": args.samples_per_src,
        "max_num_paths_per_src": args.max_num_paths_per_src,
        "mi_variant": args.mi_variant,
        "los": args.los,
        "specular_reflection": args.specular_reflection,
        "diffuse_reflection": args.diffuse_reflection,
        "refraction": args.refraction,
        "diffraction": args.diffraction,
        "outputs": {
            "summary_csv": str(summary_path),
            "anchor_summary_csv": str(anchor_path),
            "delta_csv": str(delta_path),
        },
    }
    meta_path.write_text(json.dumps(meta, ensure_ascii=False, indent=2), encoding="utf-8")
    print(f"[done] summary: {summary_path}", flush=True)
    print(f"[done] deltas: {delta_path}", flush=True)
    print(f"[done] anchor summary: {anchor_path}", flush=True)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
