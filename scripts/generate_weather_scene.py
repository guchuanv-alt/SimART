#!/usr/bin/env python3
from __future__ import annotations

import argparse
import json
import os
from pathlib import Path
import re
import xml.etree.ElementTree as ET

import yaml


FLOAT_NAMES = {
    "relative_permittivity",
    "conductivity",
    "thickness",
    "scattering_coefficient",
    "xpd_coefficient",
}


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="Generate weather-specific Sionna XMLs from one template scene."
    )
    parser.add_argument("--template", required=True, help="Input template XML")
    parser.add_argument("--profiles", required=True, help="Weather profile YAML")
    parser.add_argument(
        "--weather",
        action="append",
        default=[],
        help="Weather profile name to generate. Repeat or omit to generate all.",
    )
    parser.add_argument("--output-dir", required=True)
    parser.add_argument("--prefix", default="BigCitySample_weather")
    return parser.parse_args()


def fmt_float(value: float) -> str:
    return f"{float(value):.12g}"


def find_or_add(parent: ET.Element, tag: str, name: str) -> ET.Element:
    for child in parent:
        if child.tag == tag and child.get("name") == name:
            return child
    child = ET.Element(tag, {"name": name})
    # Place material parameters before color when possible.
    insert_at = len(parent)
    for i, existing in enumerate(parent):
        if existing.tag == "rgb":
            insert_at = i
            break
    parent.insert(insert_at, child)
    return child


def remove_itu_type_string(bsdf: ET.Element) -> None:
    for child in list(bsdf):
        if child.tag == "string" and child.get("name") == "type":
            bsdf.remove(child)


def classify_material(material_id: str, rules: list[dict]) -> str | None:
    normalized = material_id.lower()
    for rule in rules:
        for needle in rule.get("contains", []):
            if str(needle).lower() in normalized:
                return str(rule["class"])
    return None


def weather_material_id(original_id: str) -> str:
    base = re.sub(r"^mat-(itu|review)_", "", original_id)
    base = re.sub(r"[^A-Za-z0-9_.-]+", "_", base)
    return f"mat-weather_{base}"


def interpolate(dry: float, wet: float, wetness: float) -> float:
    return float(dry) + float(wetness) * (float(wet) - float(dry))


def update_bsdf(
    bsdf: ET.Element,
    material_class: str,
    class_profiles: dict,
    weather_profile: dict,
) -> dict:
    params = class_profiles[material_class]
    wetness = float(weather_profile["wetness"])
    dry = params["dry"]
    wet = params["wet"]
    computed = {
        "relative_permittivity": interpolate(
            dry["relative_permittivity"], wet["relative_permittivity"], wetness
        ),
        "conductivity": interpolate(dry["conductivity"], wet["conductivity"], wetness),
        "thickness": float(params.get("thickness", 0.1)),
        "scattering_coefficient": float(params.get("scattering_coefficient", 0.0)),
        "xpd_coefficient": float(params.get("xpd_coefficient", 0.0)),
    }

    bsdf.set("type", "radio-material")
    remove_itu_type_string(bsdf)
    for name in FLOAT_NAMES:
        node = find_or_add(bsdf, "float", name)
        node.set("value", fmt_float(computed[name]))

    rgb = find_or_add(bsdf, "rgb", "color")
    rgb.set("value", str(weather_profile["color"]))
    return computed


def generate_one(
    template: Path,
    profiles: dict,
    weather_name: str,
    output_dir: Path,
    prefix: str,
) -> dict:
    tree = ET.parse(template)
    root = tree.getroot()
    weather_profile = profiles["weather_profiles"][weather_name]
    class_profiles = profiles["material_classes"]
    rules = profiles["material_id_rules"]

    updated: list[dict] = []
    id_map: dict[str, str] = {}
    for bsdf in root.findall("bsdf"):
        material_id = bsdf.get("id") or ""
        material_class = classify_material(material_id, rules)
        if not material_class:
            continue
        new_id = weather_material_id(material_id)
        id_map[material_id] = new_id
        bsdf.set("id", new_id)
        bsdf.set("name", new_id)
        computed = update_bsdf(bsdf, material_class, class_profiles, weather_profile)
        updated.append(
            {
                "original_material_id": material_id,
                "weather_material_id": new_id,
                "material_class": material_class,
                **computed,
            }
        )

    for ref in root.findall(".//ref"):
        if ref.get("name") == "bsdf" and ref.get("id") in id_map:
            ref.set("id", id_map[ref.get("id")])

    output_dir.mkdir(parents=True, exist_ok=True)
    template_dir = template.parent.resolve()
    output_dir_resolved = output_dir.resolve()
    for string_node in root.findall(".//string"):
        if string_node.get("name") != "filename":
            continue
        filename = string_node.get("value") or ""
        source_path = (template_dir / filename).resolve()
        rel_path = os.path.relpath(source_path, output_dir_resolved)
        string_node.set("value", Path(rel_path).as_posix())

    out_xml = output_dir / f"{prefix}_{weather_name}.xml"
    tree.write(out_xml, encoding="utf-8", xml_declaration=True)

    manifest = {
        "template": str(template),
        "weather": weather_name,
        "wetness": float(weather_profile["wetness"]),
        "color": str(weather_profile["color"]),
        "output_xml": str(out_xml),
        "updated_material_count": len(updated),
        "updated_materials": updated,
    }
    (output_dir / f"{prefix}_{weather_name}.materials.json").write_text(
        json.dumps(manifest, indent=2), encoding="utf-8"
    )
    return manifest


def main() -> int:
    args = parse_args()
    template = Path(args.template)
    output_dir = Path(args.output_dir)
    with Path(args.profiles).open("r", encoding="utf-8") as f:
        profiles = yaml.safe_load(f)

    weather_names = args.weather or list(profiles["weather_profiles"].keys())
    manifests = [
        generate_one(template, profiles, weather, output_dir, args.prefix)
        for weather in weather_names
    ]
    summary_path = output_dir / f"{args.prefix}_summary.json"
    summary_path.write_text(json.dumps(manifests, indent=2), encoding="utf-8")
    print(f"generated={len(manifests)}")
    print(f"summary={summary_path}")
    for manifest in manifests:
        print(
            f"{manifest['weather']}: {manifest['output_xml']} "
            f"materials={manifest['updated_material_count']}"
        )
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
