#!/usr/bin/env python3
from __future__ import annotations

import argparse
import copy
import json
import math
import os
import random
import re
from dataclasses import dataclass
from typing import Dict, List, Sequence, Tuple

import numpy as np
import pandas as pd

from deepsense_beam_selector import DEFAULT_FEATURE_MODE, DeepSenseBeamPredictor


FEATURE_COLUMN_RE = re.compile(r"^feature_(\d+)$")


def ensure_dir_for_file(path: str) -> None:
    folder = os.path.dirname(os.path.abspath(path))
    if folder:
        os.makedirs(folder, exist_ok=True)


class NNBeamPred:
    @staticmethod
    def build(torch, nn, input_size: int, hidden_dim: int, output_size: int):
        class _Model(nn.Module):
            def __init__(self):
                super().__init__()
                self.layer_1 = nn.Linear(input_size, hidden_dim)
                self.layer_2 = nn.Linear(hidden_dim, hidden_dim)
                self.layer_3 = nn.Linear(hidden_dim, hidden_dim)
                self.layer_out = nn.Linear(hidden_dim, output_size)
                self.relu = nn.ReLU()

            def forward(self, inputs):
                x = self.relu(self.layer_1(inputs))
                x = self.relu(self.layer_2(x))
                x = self.relu(self.layer_3(x))
                return self.layer_out(x)

        return _Model()


@dataclass
class DatasetBundle:
    x: np.ndarray
    y: np.ndarray
    feature_mode: str
    feature_columns: List[str]
    feature_names: List[str]
    num_beams: int
    metadata: Dict[str, object]


@dataclass
class EvalMetrics:
    loss: float
    top1_acc: float
    topk_acc: float


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="Train a beam-selection model from exported Sionna oracle samples")
    parser.add_argument("--dataset", required=True, action="append", help="Path to a training CSV. Repeat the flag to combine multiple CSV files.")
    parser.add_argument("--output-checkpoint", required=True)
    parser.add_argument("--feature-mode", default=DEFAULT_FEATURE_MODE)
    parser.add_argument("--epochs", type=int, default=30)
    parser.add_argument("--batch-size", type=int, default=256)
    parser.add_argument("--learning-rate", type=float, default=1e-3)
    parser.add_argument("--val-split", type=float, default=0.2)
    parser.add_argument("--hidden-dim", type=int, default=512)
    parser.add_argument("--top-k", type=int, default=3)
    parser.add_argument("--seed", type=int, default=42)
    return parser.parse_args()


def set_seed(seed: int) -> None:
    random.seed(seed)
    np.random.seed(seed)
    import torch

    torch.manual_seed(seed)
    if torch.cuda.is_available():
        torch.cuda.manual_seed_all(seed)


def split_dataset_paths(values: Sequence[str]) -> List[str]:
    paths: List[str] = []
    seen = set()
    for value in values:
        if value is None:
            continue
        normalized = str(value).replace("\r", "\n")
        for token in normalized.split("\n"):
            for subtoken in token.split(";"):
                path = subtoken.strip()
                if not path or path in seen:
                    continue
                seen.add(path)
                paths.append(path)
    return paths


def infer_feature_columns(frame: pd.DataFrame, feature_mode: str) -> Tuple[List[str], List[str]]:
    indexed_columns: List[Tuple[int, str]] = []
    for column in frame.columns:
        match = FEATURE_COLUMN_RE.match(str(column))
        if match and not frame[str(column)].isna().all():
            indexed_columns.append((int(match.group(1)), str(column)))
    if not indexed_columns:
        raise ValueError("Dataset does not contain any feature_* columns")
    indexed_columns.sort(key=lambda item: item[0])
    expected_idx = list(range(len(indexed_columns)))
    actual_idx = [idx for idx, _ in indexed_columns]
    if actual_idx != expected_idx:
        raise ValueError(f"Feature columns must be contiguous from feature_0. Found indices: {actual_idx}")
    feature_columns = [column for _, column in indexed_columns]
    schema = list(DeepSenseBeamPredictor.feature_schema(feature_mode))
    if schema and len(schema) != len(feature_columns):
        raise ValueError(
            f"Feature column count mismatch for mode {DeepSenseBeamPredictor.normalize_feature_mode(feature_mode)}: "
            f"dataset has {len(feature_columns)} feature_* columns but the mode expects {len(schema)}"
        )
    feature_names = schema if schema else list(feature_columns)
    return feature_columns, feature_names


def load_dataset(paths: Sequence[str], feature_mode: str) -> DatasetBundle:
    dataset_paths = split_dataset_paths(paths)
    if not dataset_paths:
        raise ValueError("No dataset CSV paths were provided")

    frames = []
    rows_per_file: Dict[str, int] = {}
    for path in dataset_paths:
        if not os.path.exists(path):
            raise FileNotFoundError(f"Dataset not found: {path}")
        df = pd.read_csv(path)
        if df.empty:
            continue
        abs_path = os.path.abspath(path)
        df = df.copy()
        df["source_dataset_path"] = abs_path
        frames.append(df)
        rows_per_file[abs_path] = int(df.shape[0])

    if not frames:
        raise ValueError("All provided datasets are empty")

    requested_feature_mode = DeepSenseBeamPredictor.normalize_feature_mode(feature_mode)
    df = pd.concat(frames, ignore_index=True)
    missing = [c for c in ["oracle_beam_index"] if c not in df.columns]
    if missing:
        raise ValueError(f"Dataset is missing required columns: {missing}")

    work = df.copy()
    if "feature_mode" in work.columns:
        normalized_modes = work["feature_mode"].astype(str).map(DeepSenseBeamPredictor.normalize_feature_mode)
        mode_mask = normalized_modes == requested_feature_mode
        if mode_mask.any():
            work = work.loc[mode_mask].copy()
        else:
            available_modes = sorted({str(v) for v in normalized_modes.dropna().tolist() if str(v).strip()})
            if available_modes:
                raise ValueError(
                    f"Dataset feature_mode values {available_modes} do not match requested mode {requested_feature_mode}"
                )
    feature_columns, feature_names = infer_feature_columns(work, requested_feature_mode)
    work = work.replace([np.inf, -np.inf], np.nan)
    work = work.dropna(subset=feature_columns + ["oracle_beam_index"])
    if work.empty:
        raise ValueError("No usable rows remain after filtering NaN/Inf rows")

    x = work[feature_columns].to_numpy(dtype=np.float32)
    y = work["oracle_beam_index"].to_numpy(dtype=np.int64)
    valid_y = np.isfinite(y)
    x = x[valid_y]
    y = y[valid_y].astype(np.int64)
    if x.shape[0] == 0:
        raise ValueError("No usable labels remain")

    num_beams = None
    if "num_beams" in work.columns:
        beam_values = sorted({int(v) for v in work["num_beams"].dropna().astype(int).tolist() if int(v) > 0})
        if len(beam_values) == 1:
            num_beams = int(beam_values[0])
        elif len(beam_values) > 1:
            raise ValueError(f"Dataset contains multiple num_beams values: {beam_values}. Please train one codebook configuration at a time.")
    if num_beams is None:
        num_beams = int(np.max(y)) + 1

    label_mask = (y >= 0) & (y < num_beams)
    x = x[label_mask]
    y = y[label_mask]
    if x.shape[0] == 0:
        raise ValueError("All labels are outside the valid beam range")

    metadata: Dict[str, object] = {
        "rows": int(x.shape[0]),
        "feature_mode": str(requested_feature_mode),
        "feature_columns": list(feature_columns),
        "feature_names": list(feature_names),
        "feature_count": int(len(feature_columns)),
        "num_beams": int(num_beams),
        "dataset_paths": [os.path.abspath(path) for path in dataset_paths],
        "dataset_rows_per_file": rows_per_file,
        "codebook_type": work["codebook_type"].dropna().astype(str).mode().iloc[0] if "codebook_type" in work.columns and not work["codebook_type"].dropna().empty else "",
        "scene_path": work["scene_path"].dropna().astype(str).mode().iloc[0] if "scene_path" in work.columns and not work["scene_path"].dropna().empty else "",
        "tx_array_num_rows": int(work["tx_array_num_rows"].dropna().astype(int).mode().iloc[0]) if "tx_array_num_rows" in work.columns and not work["tx_array_num_rows"].dropna().empty else 0,
        "tx_array_num_cols": int(work["tx_array_num_cols"].dropna().astype(int).mode().iloc[0]) if "tx_array_num_cols" in work.columns and not work["tx_array_num_cols"].dropna().empty else 0,
        "beam_oversampling_v": int(work["beam_oversampling_v"].dropna().astype(int).mode().iloc[0]) if "beam_oversampling_v" in work.columns and not work["beam_oversampling_v"].dropna().empty else 0,
        "beam_oversampling_h": int(work["beam_oversampling_h"].dropna().astype(int).mode().iloc[0]) if "beam_oversampling_h" in work.columns and not work["beam_oversampling_h"].dropna().empty else 0,
    }
    return DatasetBundle(
        x=x,
        y=y,
        feature_mode=str(requested_feature_mode),
        feature_columns=list(feature_columns),
        feature_names=list(feature_names),
        num_beams=int(num_beams),
        metadata=metadata,
    )


def train_val_split(n_rows: int, val_split: float, seed: int) -> Tuple[np.ndarray, np.ndarray]:
    indices = np.arange(n_rows, dtype=np.int64)
    rng = np.random.default_rng(seed)
    rng.shuffle(indices)
    if n_rows <= 1 or val_split <= 0.0:
        return indices, np.array([], dtype=np.int64)
    val_count = int(round(n_rows * val_split))
    val_count = max(1, min(val_count, n_rows - 1))
    return indices[val_count:], indices[:val_count]


def build_loader(torch, x: np.ndarray, y: np.ndarray, batch_size: int, shuffle: bool):
    dataset = torch.utils.data.TensorDataset(
        torch.from_numpy(x.astype(np.float32)),
        torch.from_numpy(y.astype(np.int64)),
    )
    return torch.utils.data.DataLoader(dataset, batch_size=batch_size, shuffle=shuffle)


def evaluate(model, loader, criterion, top_k: int):
    import torch

    model.eval()
    total_loss = 0.0
    total = 0
    top1 = 0
    topk = 0
    with torch.no_grad():
        for batch_x, batch_y in loader:
            logits = model(batch_x)
            loss = criterion(logits, batch_y)
            batch_size = int(batch_y.shape[0])
            total_loss += float(loss.item()) * batch_size
            total += batch_size
            preds = torch.argmax(logits, dim=-1)
            top1 += int((preds == batch_y).sum().item())
            k = min(max(int(top_k), 1), int(logits.shape[-1]))
            topk_idx = torch.topk(logits, k=k, dim=-1).indices
            topk += int((topk_idx == batch_y.unsqueeze(-1)).any(dim=-1).sum().item())
    if total <= 0:
        return EvalMetrics(loss=float("nan"), top1_acc=float("nan"), topk_acc=float("nan"))
    return EvalMetrics(loss=total_loss / total, top1_acc=top1 / total, topk_acc=topk / total)


def main() -> int:
    args = parse_args()
    set_seed(int(args.seed))
    import torch
    import torch.nn as nn

    dataset_paths = split_dataset_paths(args.dataset)
    bundle = load_dataset(dataset_paths, feature_mode=args.feature_mode)
    n_rows = int(bundle.x.shape[0])
    print(f"[train] dataset files: {len(dataset_paths)}", flush=True)
    for dataset_path in dataset_paths:
        print(f"[train]   file: {dataset_path}", flush=True)
    print(f"[train] dataset rows: {n_rows}", flush=True)
    print(f"[train] num_beams: {bundle.num_beams}", flush=True)
    print(f"[train] feature_mode: {bundle.feature_mode}", flush=True)
    print(f"[train] feature_dim: {len(bundle.feature_columns)}", flush=True)

    train_idx, val_idx = train_val_split(n_rows, float(args.val_split), int(args.seed))
    x_train = bundle.x[train_idx]
    y_train = bundle.y[train_idx]
    if val_idx.size > 0:
        x_val = bundle.x[val_idx]
        y_val = bundle.y[val_idx]
    else:
        x_val = np.zeros((0, bundle.x.shape[1]), dtype=np.float32)
        y_val = np.zeros((0,), dtype=np.int64)

    feature_mean = x_train.mean(axis=0).astype(np.float32)
    feature_std = x_train.std(axis=0).astype(np.float32)
    feature_std = np.where(np.abs(feature_std) < 1e-6, 1.0, feature_std).astype(np.float32)
    x_train_norm = ((x_train - feature_mean) / feature_std).astype(np.float32)
    x_val_norm = ((x_val - feature_mean) / feature_std).astype(np.float32)

    train_loader = build_loader(torch, x_train_norm, y_train, batch_size=max(int(args.batch_size), 1), shuffle=True)
    val_loader = build_loader(torch, x_val_norm, y_val, batch_size=max(int(args.batch_size), 1), shuffle=False) if x_val_norm.shape[0] > 0 else None

    model = NNBeamPred.build(torch, nn, input_size=bundle.x.shape[1], hidden_dim=max(int(args.hidden_dim), 8), output_size=bundle.num_beams)
    criterion = nn.CrossEntropyLoss()
    optimizer = torch.optim.Adam(model.parameters(), lr=float(args.learning_rate))

    best_state = copy.deepcopy(model.state_dict())
    best_metric = -math.inf
    history = []
    for epoch in range(1, max(int(args.epochs), 1) + 1):
        model.train()
        epoch_loss = 0.0
        seen = 0
        for batch_x, batch_y in train_loader:
            optimizer.zero_grad(set_to_none=True)
            logits = model(batch_x)
            loss = criterion(logits, batch_y)
            loss.backward()
            optimizer.step()
            batch_size = int(batch_y.shape[0])
            epoch_loss += float(loss.item()) * batch_size
            seen += batch_size
        train_metrics = evaluate(model, train_loader, criterion, top_k=int(args.top_k))
        if val_loader is not None:
            val_metrics = evaluate(model, val_loader, criterion, top_k=int(args.top_k))
            select_metric = float(val_metrics.top1_acc)
        else:
            val_metrics = EvalMetrics(loss=float("nan"), top1_acc=float("nan"), topk_acc=float("nan"))
            select_metric = float(train_metrics.top1_acc)
        if select_metric >= best_metric:
            best_metric = select_metric
            best_state = copy.deepcopy(model.state_dict())
        history.append({
            "epoch": epoch,
            "train_loss": float(epoch_loss / max(seen, 1)),
            "train_top1_acc": float(train_metrics.top1_acc),
            "train_topk_acc": float(train_metrics.topk_acc),
            "val_loss": float(val_metrics.loss) if np.isfinite(val_metrics.loss) else None,
            "val_top1_acc": float(val_metrics.top1_acc) if np.isfinite(val_metrics.top1_acc) else None,
            "val_topk_acc": float(val_metrics.topk_acc) if np.isfinite(val_metrics.topk_acc) else None,
        })
        print(
            f"[train] epoch {epoch:03d}/{int(args.epochs):03d} "
            f"train_top1={train_metrics.top1_acc:.4f} train_top{int(args.top_k)}={train_metrics.topk_acc:.4f} "
            f"val_top1={val_metrics.top1_acc if np.isfinite(val_metrics.top1_acc) else float('nan'):.4f} "
            f"val_top{int(args.top_k)}={val_metrics.topk_acc if np.isfinite(val_metrics.topk_acc) else float('nan'):.4f}",
            flush=True,
        )

    model.load_state_dict(best_state)
    final_train = evaluate(model, train_loader, criterion, top_k=int(args.top_k))
    final_val = evaluate(model, val_loader, criterion, top_k=int(args.top_k)) if val_loader is not None else EvalMetrics(loss=float("nan"), top1_acc=float("nan"), topk_acc=float("nan"))

    ensure_dir_for_file(args.output_checkpoint)
    checkpoint = {
        "state_dict": best_state,
        "input_size": int(bundle.x.shape[1]),
        "hidden_dim": int(max(int(args.hidden_dim), 8)),
        "output_size": int(bundle.num_beams),
        "feature_mode": str(bundle.feature_mode),
        "feature_names": list(bundle.feature_names),
        "top_k": int(args.top_k),
        "feature_mean": feature_mean.tolist(),
        "feature_std": feature_std.tolist(),
        "dataset_path": os.path.abspath(dataset_paths[0]) if len(dataset_paths) == 1 else ";".join(os.path.abspath(path) for path in dataset_paths),
        "dataset_paths": [os.path.abspath(path) for path in dataset_paths],
        "metadata": {
            **bundle.metadata,
            "rows": int(n_rows),
            "train_rows": int(len(train_idx)),
            "val_rows": int(len(val_idx)),
            "seed": int(args.seed),
        },
        "metrics": {
            "train_loss": float(final_train.loss),
            "train_top1_acc": float(final_train.top1_acc),
            f"train_top{int(args.top_k)}_acc": float(final_train.topk_acc),
            "val_loss": float(final_val.loss) if np.isfinite(final_val.loss) else None,
            "val_top1_acc": float(final_val.top1_acc) if np.isfinite(final_val.top1_acc) else None,
            f"val_top{int(args.top_k)}_acc": float(final_val.topk_acc) if np.isfinite(final_val.topk_acc) else None,
        },
        "history": history,
    }
    torch.save(checkpoint, args.output_checkpoint)

    summary_path = args.output_checkpoint + ".json"
    with open(summary_path, "w", encoding="utf-8") as f:
        json.dump({k: v for k, v in checkpoint.items() if k != "state_dict"}, f, ensure_ascii=False, indent=2)

    print(f"[train] saved checkpoint: {args.output_checkpoint}", flush=True)
    print(f"[train] saved summary: {summary_path}", flush=True)
    print(
        f"[train] final train top1={final_train.top1_acc:.4f} top{int(args.top_k)}={final_train.topk_acc:.4f}; "
        f"val top1={(final_val.top1_acc if np.isfinite(final_val.top1_acc) else float('nan')):.4f} "
        f"top{int(args.top_k)}={(final_val.topk_acc if np.isfinite(final_val.topk_acc) else float('nan')):.4f}",
        flush=True,
    )
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
