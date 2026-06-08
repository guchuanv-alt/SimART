"""Compare RF observation power from two SimART rosbag files.
对比两个 SimART rosbag 中 RF observation 的功率结果。

This script intentionally shells out to ``rostopic echo -p`` instead of using
the rosbag Python API, because some environments lack the optional Cryptodome
dependency required by ``import rosbag``.
这个脚本故意调用 ``rostopic echo -p``，而不是直接导入 rosbag Python API，
因为有些环境缺少 ``import rosbag`` 所需的 Cryptodome 依赖。
"""

from __future__ import annotations

import argparse
import csv
import math
import subprocess
from dataclasses import dataclass
from pathlib import Path

import yaml


RF_TOPIC = "/airsim_gui_UErealtime/rf_observations"


@dataclass
class Series:
    label: str
    times: list[float]
    strongest_power_db: list[float]
    strongest_by_anchor: dict[int, list[float]]


def _run_rostopic_yaml(bag_path: Path, topic: str) -> str:
    result = subprocess.run(
        ["rostopic", "echo", "-b", str(bag_path), topic],
        check=True,
        text=True,
        capture_output=True,
    )
    return result.stdout


def _to_float(value: str | None) -> float | None:
    if value is None or value == "":
        return None
    try:
        parsed = float(value)
    except ValueError:
        return None
    return parsed if math.isfinite(parsed) else None


def load_series(label: str, bag_path: Path, topic: str = RF_TOPIC) -> Series:
    """Load strongest power per RF message and per anchor.
    加载每条 RF 消息整体最强功率，以及每个 anchor 的最强功率。
    """
    text = _run_rostopic_yaml(bag_path, topic)
    start_time: float | None = None
    times: list[float] = []
    strongest_power_db: list[float] = []
    strongest_by_anchor: dict[int, list[float]] = {}
    seen_anchor_ids: set[int] = set()

    for message in yaml.safe_load_all(text):
        if not isinstance(message, dict):
            continue
        stamp = message.get("header", {}).get("stamp", {})
        secs = _to_float(stamp.get("secs"))
        nsecs = _to_float(stamp.get("nsecs"))
        if secs is None or nsecs is None:
            continue
        raw_time = secs + nsecs / 1e9
        if start_time is None:
            start_time = raw_time
        times.append(raw_time - start_time)

        by_anchor: dict[int, list[float]] = {}
        for anchor in message.get("anchors", []):
            if not isinstance(anchor, dict):
                continue
            anchor_id = anchor.get("anchor_id")
            if not isinstance(anchor_id, int):
                continue
            seen_anchor_ids.add(anchor_id)
            for path in anchor.get("paths", []):
                if not isinstance(path, dict):
                    continue
                if not path.get("is_valid", False) or not path.get("has_power_db", False):
                    continue
                value = _to_float(path.get("power_db"))
                if value is not None:
                    by_anchor.setdefault(anchor_id, []).append(value)

        all_powers = [value for values in by_anchor.values() for value in values]
        strongest_power_db.append(max(all_powers) if all_powers else float("nan"))

        for anchor_id in sorted(seen_anchor_ids):
            values = by_anchor.get(anchor_id, [])
            strongest_by_anchor.setdefault(anchor_id, []).append(max(values) if values else float("nan"))

    return Series(label=label, times=times, strongest_power_db=strongest_power_db, strongest_by_anchor=strongest_by_anchor)


def write_summary_csv(original: Series, changed: Series, output_path: Path) -> None:
    """Write frame-by-frame strongest power comparison.
    写出逐帧最强功率对比 CSV。
    """
    count = min(len(original.times), len(changed.times))
    output_path.parent.mkdir(parents=True, exist_ok=True)
    with output_path.open("w", newline="", encoding="utf-8") as file:
        writer = csv.writer(file)
        writer.writerow(["frame", "time_s", "original_strongest_db", "changed_strongest_db", "delta_db"])
        for index in range(count):
            before = original.strongest_power_db[index]
            after = changed.strongest_power_db[index]
            delta = after - before if math.isfinite(before) and math.isfinite(after) else float("nan")
            writer.writerow([index, original.times[index], before, after, delta])


def _polyline(points: list[tuple[float, float]]) -> str:
    return " ".join(f"{x:.2f},{y:.2f}" for x, y in points)


def write_svg(original: Series, changed: Series, output_path: Path) -> None:
    """Write a small SVG line chart for strongest RF power.
    写出最强 RF 功率的 SVG 折线图。
    """
    count = min(len(original.times), len(changed.times))
    if count == 0:
        raise RuntimeError("No overlapping RF frames to plot.")

    width, height = 960, 520
    margin_left, margin_right, margin_top, margin_bottom = 72, 24, 42, 58
    plot_w = width - margin_left - margin_right
    plot_h = height - margin_top - margin_bottom

    xs = original.times[:count]
    y_values = original.strongest_power_db[:count] + changed.strongest_power_db[:count]
    finite_y = [value for value in y_values if math.isfinite(value)]
    if not finite_y:
        raise RuntimeError("No finite power values to plot.")
    x_min, x_max = min(xs), max(xs) or 1.0
    y_min, y_max = min(finite_y), max(finite_y)
    if abs(y_max - y_min) < 1e-9:
        y_min -= 1.0
        y_max += 1.0

    def map_point(x_value: float, y_value: float) -> tuple[float, float]:
        x = margin_left + (x_value - x_min) / (x_max - x_min) * plot_w
        y = margin_top + (y_max - y_value) / (y_max - y_min) * plot_h
        return x, y

    original_points = [
        map_point(xs[index], original.strongest_power_db[index])
        for index in range(count)
        if math.isfinite(original.strongest_power_db[index])
    ]
    changed_points = [
        map_point(xs[index], changed.strongest_power_db[index])
        for index in range(count)
        if math.isfinite(changed.strongest_power_db[index])
    ]

    output_path.parent.mkdir(parents=True, exist_ok=True)
    output_path.write_text(
        f"""<svg xmlns="http://www.w3.org/2000/svg" width="{width}" height="{height}" viewBox="0 0 {width} {height}">
  <rect width="100%" height="100%" fill="#f8fafc"/>
  <text x="{width / 2:.0f}" y="24" text-anchor="middle" font-family="sans-serif" font-size="18" fill="#0f172a">Strongest RF Power Comparison</text>
  <line x1="{margin_left}" y1="{margin_top}" x2="{margin_left}" y2="{margin_top + plot_h}" stroke="#334155"/>
  <line x1="{margin_left}" y1="{margin_top + plot_h}" x2="{margin_left + plot_w}" y2="{margin_top + plot_h}" stroke="#334155"/>
  <text x="18" y="{margin_top + plot_h / 2:.0f}" transform="rotate(-90 18,{margin_top + plot_h / 2:.0f})" font-family="sans-serif" font-size="13" fill="#334155">power_db</text>
  <text x="{margin_left + plot_w / 2:.0f}" y="{height - 16}" text-anchor="middle" font-family="sans-serif" font-size="13" fill="#334155">time (s)</text>
  <text x="{margin_left - 8}" y="{margin_top + 4}" text-anchor="end" font-family="monospace" font-size="12" fill="#334155">{y_max:.2f}</text>
  <text x="{margin_left - 8}" y="{margin_top + plot_h}" text-anchor="end" font-family="monospace" font-size="12" fill="#334155">{y_min:.2f}</text>
  <polyline fill="none" stroke="#2563eb" stroke-width="2.5" points="{_polyline(original_points)}"/>
  <polyline fill="none" stroke="#dc2626" stroke-width="2.5" points="{_polyline(changed_points)}"/>
  <rect x="{width - 244}" y="42" width="220" height="58" rx="4" fill="#ffffff" stroke="#cbd5e1"/>
  <line x1="{width - 228}" y1="62" x2="{width - 188}" y2="62" stroke="#2563eb" stroke-width="3"/>
  <text x="{width - 178}" y="66" font-family="sans-serif" font-size="13" fill="#0f172a">{original.label}</text>
  <line x1="{width - 228}" y1="84" x2="{width - 188}" y2="84" stroke="#dc2626" stroke-width="3"/>
  <text x="{width - 178}" y="88" font-family="sans-serif" font-size="13" fill="#0f172a">{changed.label}</text>
</svg>
""",
        encoding="utf-8",
    )


def print_anchor_summary(original: Series, changed: Series) -> None:
    count = min(len(original.times), len(changed.times))
    print(f"Compared RF frames: {count}")
    print("Average strongest power delta by anchor (changed - original):")
    for anchor_id in sorted(set(original.strongest_by_anchor) | set(changed.strongest_by_anchor)):
        before_values = original.strongest_by_anchor.get(anchor_id, [])[:count]
        after_values = changed.strongest_by_anchor.get(anchor_id, [])[:count]
        deltas = [
            after - before
            for before, after in zip(before_values, after_values)
            if math.isfinite(before) and math.isfinite(after)
        ]
        if not deltas:
            continue
        avg_delta = sum(deltas) / len(deltas)
        max_abs_delta = max(deltas, key=lambda value: abs(value))
        print(f"  BS {anchor_id}: avg_delta={avg_delta:+.3f} dB, max_abs_delta={max_abs_delta:+.3f} dB")


def main() -> int:
    parser = argparse.ArgumentParser(description="Compare RF power in two SimART rosbag files.")
    parser.add_argument("original_bag", type=Path)
    parser.add_argument("changed_bag", type=Path)
    parser.add_argument("--topic", default=RF_TOPIC)
    parser.add_argument("--output-dir", type=Path, default=Path("output/rf_compare"))
    args = parser.parse_args()

    original = load_series("original", args.original_bag, args.topic)
    changed = load_series("changed", args.changed_bag, args.topic)

    csv_path = args.output_dir / "strongest_power_comparison.csv"
    svg_path = args.output_dir / "strongest_power_comparison.svg"
    write_summary_csv(original, changed, csv_path)
    write_svg(original, changed, svg_path)
    print_anchor_summary(original, changed)
    print(f"Wrote CSV: {csv_path}")
    print(f"Wrote SVG: {svg_path}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
