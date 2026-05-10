#!/usr/bin/env python3
from __future__ import annotations

import argparse
import json
import time
from types import SimpleNamespace

from sionna_beam_topic_utils import (
    build_codebook_payload,
    effective_dft_oversampling_for_num_beams,
    normalize_codebook_num_beams,
    select_codebook_num_beams,
)


def parse_bool(value: str) -> bool:
    text = str(value).strip().lower()
    if text in {"1", "true", "yes", "on"}:
        return True
    if text in {"0", "false", "no", "off"}:
        return False
    raise argparse.ArgumentTypeError(f"invalid boolean value: {value}")


def build_arg_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description="Export a Sionna-generated beam codebook to JSON.")
    parser.add_argument("--output", required=True)
    parser.add_argument("--tx-array-num-rows", type=int, default=1)
    parser.add_argument("--tx-array-num-cols", type=int, default=1)
    parser.add_argument("--beam-codebook-type", default="auto")
    parser.add_argument("--beam-codebook-num-beams", type=int, default=8)
    parser.add_argument("--beam-oversampling-v", type=int, default=1)
    parser.add_argument("--beam-oversampling-h", type=int, default=1)
    parser.add_argument("--beam-normalize-power", type=parse_bool, default=True)
    return parser


def main() -> int:
    args = build_arg_parser().parse_args()

    try:
        from sionna.phy.mimo import (
            grid_of_beams_dft,
            grid_of_beams_dft_ula,
            normalize_precoding_power,
        )
    except Exception as exc:
        raise RuntimeError(
            "Exporting the Sionna codebook requires the current Python environment to import sionna.phy.mimo."
        ) from exc

    tx_rows = max(int(args.tx_array_num_rows), 1)
    tx_cols = max(int(args.tx_array_num_cols), 1)
    tx_ant = tx_rows * tx_cols
    requested_type = str(args.beam_codebook_type or "").strip().lower() or "auto"
    if requested_type == "auto":
        codebook_type = "dft_ura" if (tx_rows > 1 and tx_cols > 1) else "dft_ula"
    else:
        codebook_type = requested_type
    if codebook_type == "dft_ura" and (tx_rows <= 1 or tx_cols <= 1):
        codebook_type = "dft_ula"
    target_num_beams = normalize_codebook_num_beams(args.beam_codebook_num_beams)
    oversampling_v, oversampling_h = effective_dft_oversampling_for_num_beams(
        codebook_type=codebook_type,
        tx_rows=tx_rows,
        tx_cols=tx_cols,
        target_num_beams=target_num_beams,
        min_oversampling_v=max(int(args.beam_oversampling_v), 1),
        min_oversampling_h=max(int(args.beam_oversampling_h), 1),
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

    if bool(args.beam_normalize_power):
        codebook = normalize_precoding_power(codebook)

    cfg = SimpleNamespace(
        tx_array_num_rows=tx_rows,
        tx_array_num_cols=tx_cols,
        beam_codebook_num_beams=target_num_beams,
        beam_oversampling_v=oversampling_v,
        beam_oversampling_h=oversampling_h,
        beam_normalize_power=bool(args.beam_normalize_power),
    )
    runtime = {
        "codebook_type": codebook_type,
        "codebook": codebook,
        "beam_codebook_num_beams": target_num_beams,
        "beam_oversampling_v": oversampling_v,
        "beam_oversampling_h": oversampling_h,
        "codebook_source_indices": selected_source_indices,
    }
    payload = build_codebook_payload(runtime, cfg)
    if payload is None:
        raise RuntimeError("failed to build codebook payload")
    payload["exported_utc"] = time.strftime("%Y-%m-%dT%H:%M:%SZ", time.gmtime())
    with open(args.output, "w", encoding="utf-8") as f:
        json.dump(payload, f, ensure_ascii=False, indent=2)
    print(json.dumps({"ok": True, "output": args.output, "num_beams": payload.get("num_beams", 0), "tx_ant": payload.get("tx_ant", 0)}, ensure_ascii=False))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
