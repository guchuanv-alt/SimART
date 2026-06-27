"""Edit Blender/FBX visual assets and export Sionna Mitsuba XML.

This script keeps material changes at the asset level: update the visual source
asset first, then export both the visual FBX used by the main GUI and the
Mitsuba XML used by Sionna RT.
"""

from __future__ import annotations

import argparse
import csv
import json
import os
import shutil
import subprocess
import sys
import tempfile
import zipfile
from pathlib import Path


REPO_ROOT = Path(__file__).resolve().parents[1]
DEFAULT_BIGCITY_VISUAL_BLEND = Path("SimART_sample_maps/BigCitySample/BigCitySample.blend")
DEFAULT_BIGCITY_VISUAL_FBX = Path("SimART_sample_maps/BigCitySample/BigCitySample_fbx/BigCitySampleScene.fbx")
DEFAULT_BIGCITY_SIONNA_BLEND = Path("SimART_sample_maps/BigCitySample/BigCitySample_simptest.blend")
DEFAULT_BIGCITY_SIONNA_XML = Path(
    "SimART_sample_maps/BigCitySample/BigCitySample_simptest_sionna/BigCitySample_simptest.xml"
)
DEFAULT_BIGCITY_AUTO_SIONNA_BLEND = Path(
    "SimART_sample_maps/BigCitySample/BigCitySample_auto_sionna/BigCitySample_auto_sionna.blend"
)
DEFAULT_BIGCITY_AUTO_SIONNA_XML = Path(
    "SimART_sample_maps/BigCitySample/BigCitySample_auto_sionna/BigCitySample_auto_sionna.xml"
)
DEFAULT_BIGCITY_FBX_MATERIAL_BLEND = Path(
    "SimART_sample_maps/BigCitySample/BigCitySample_fbx_material_slots/BigCitySample_fbx_material_slots.blend"
)
DEFAULT_BIGCITY_STRICT_REVIEW_BLEND = Path(
    "SimART_sample_maps/BigCitySample/BigCitySample_strict_review_sionna/BigCitySample_strict_review.blend"
)
DEFAULT_BIGCITY_STRICT_REVIEW_XML = Path(
    "SimART_sample_maps/BigCitySample/BigCitySample_strict_review_sionna/BigCitySample_strict_review.xml"
)
DEFAULT_AGENT_OUTPUT_DIR = Path("agent_outputs")
DEFAULT_BIGCITY_CONFIRMED_MATERIALS_CSV = DEFAULT_AGENT_OUTPUT_DIR / "bigcity_confirmed_materials.csv"
DEFAULT_BIGCITY_UNCERTAIN_MATERIALS_CSV = DEFAULT_AGENT_OUTPUT_DIR / "bigcity_uncertain_materials.csv"
DEFAULT_BIGCITY_STRICT_REVIEW_MAPPING_CSV = DEFAULT_AGENT_OUTPUT_DIR / "bigcity_strict_review_material_mapping.csv"
DEFAULT_BIGCITY_STRICT_REVIEW_MAPPING_JSON = DEFAULT_AGENT_OUTPUT_DIR / "bigcity_strict_review_material_mapping.json"
STRICT_UNCERTAIN_MATERIAL = "simart_uncertain_review"
DEFAULT_BLENDER_USER_SCRIPTS = REPO_ROOT / ".tools/blender_user_scripts"
DEFAULT_SAMPLE_MAPS_ARCHIVE = Path("SimART_sample_maps.zip")
BIGCITY_RESTORE_MEMBERS = (
    "SimART_sample_maps/BigCitySample/BigCitySample_fbx/BigCitySampleScene.fbx",
    "SimART_sample_maps/BigCitySample/BigCitySample_simptest.blend",
    "SimART_sample_maps/BigCitySample/BigCitySample_simptest_sionna/BigCitySample_simptest.xml",
)


class AssetToolError(RuntimeError):
    """Raised when Blender asset processing cannot continue."""


def _repo_path(path: str | Path, must_exist: bool = False) -> Path:
    """Resolve a path relative to the repository and reject outside paths."""
    resolved = Path(path).expanduser()
    if not resolved.is_absolute():
        resolved = REPO_ROOT / resolved
    resolved = resolved.resolve()
    try:
        resolved.relative_to(REPO_ROOT)
    except ValueError as exc:
        raise AssetToolError(f"Refusing to use a path outside the repo: {resolved}") from exc
    if must_exist and not resolved.exists():
        raise AssetToolError(f"Path does not exist: {resolved}")
    return resolved


def _find_blender(explicit_path: str | None = None) -> Path:
    """Find the Blender executable from --blender, BLENDER, or PATH."""
    local_blenders = sorted((REPO_ROOT / ".tools").glob("blender-*-linux-x64/blender"), reverse=True)
    candidates = [explicit_path, os.environ.get("BLENDER"), *local_blenders, shutil.which("blender")]
    for candidate in candidates:
        if not candidate:
            continue
        path = Path(candidate).expanduser()
        if path.exists() and os.access(path, os.X_OK):
            return path.resolve()
        found = shutil.which(str(candidate))
        if found:
            return Path(found).resolve()
    raise AssetToolError(
        "Could not find Blender. Install Blender 4.2 LTS or set BLENDER=/path/to/blender."
    )


def _run_blender_payload(payload: dict[str, object], blender_path: str | None = None) -> list[dict[str, object]]:
    """Run the bundled Blender job script and parse its JSON result."""
    blender = _find_blender(blender_path)
    with tempfile.TemporaryDirectory(prefix="simart_blender_job_") as tmpdir:
        job_script = Path(tmpdir) / "job.py"
        job_script.write_text(_blender_job_source(), encoding="utf-8")
        payload_text = json.dumps(payload, ensure_ascii=False)
        command = [str(blender), "--background", "--python", str(job_script), "--", payload_text]
        env = os.environ.copy()
        env.setdefault("BLENDER_USER_SCRIPTS", str(DEFAULT_BLENDER_USER_SCRIPTS))
        completed = subprocess.run(command, text=True, capture_output=True, check=False, env=env)

    if completed.returncode != 0:
        raise AssetToolError(
            "Blender job failed.\n"
            f"stdout:\n{completed.stdout}\n"
            f"stderr:\n{completed.stderr}"
        )

    result_line = ""
    for line in completed.stdout.splitlines():
        if line.startswith("SIMART_ASSET_TOOLS_RESULT="):
            result_line = line.split("=", 1)[1]
    if not result_line:
        raise AssetToolError(f"Blender job finished but did not return a result:\n{completed.stdout}")
    return json.loads(result_line)


def _load_material_overrides(path: str | Path | None) -> dict[str, object]:
    """Load optional material-name overrides from a JSON mapping file."""
    if not path:
        return {}
    override_path = _repo_path(path, must_exist=True)
    with override_path.open("r", encoding="utf-8") as handle:
        data = json.load(handle)
    if not isinstance(data, dict):
        raise AssetToolError("Material override file must be a JSON object.")
    return data


def _write_material_inventory_csv(path: Path, materials: list[dict[str, object]]) -> None:
    """Write a compact material inventory for human review."""
    path.parent.mkdir(parents=True, exist_ok=True)
    fieldnames = [
        "material_name",
        "suggested_sionna_material",
        "confidence",
        "reason",
        "slot_count",
        "object_count",
        "face_count",
        "texture_files",
        "objects_preview",
    ]
    with path.open("w", newline="", encoding="utf-8") as handle:
        writer = csv.DictWriter(handle, fieldnames=fieldnames)
        writer.writeheader()
        for material in materials:
            row = dict(material)
            row["texture_files"] = ";".join(row.get("texture_files", []))
            row["objects_preview"] = ";".join(row.get("objects_preview", []))
            writer.writerow({name: row.get(name, "") for name in fieldnames})


def _load_strict_review_mapping(
    confirmed_csv: str | Path,
    uncertain_csv: str | Path,
    uncertain_material: str = STRICT_UNCERTAIN_MATERIAL,
) -> tuple[dict[str, str], list[dict[str, str]]]:
    """Build a no-guess material map from reviewed CSV files.

    Confirmed rows are mapped to their chosen Sionna material. Uncertain rows are
    mapped to one deliberately visible review material so they stand out in the
    generated scene.
    """
    confirmed_path = _repo_path(confirmed_csv, must_exist=True)
    uncertain_path = _repo_path(uncertain_csv, must_exist=True)
    mapping: dict[str, str] = {}
    rows: list[dict[str, str]] = []

    with confirmed_path.open("r", encoding="utf-8", newline="") as handle:
        for row in csv.DictReader(handle):
            visual_material = (row.get("visual_material") or "").strip()
            target_material = (row.get("sionna_material") or "").strip()
            if not visual_material or not target_material:
                continue
            mapping[visual_material] = target_material
            rows.append(
                {
                    "visual_material": visual_material,
                    "target_material": target_material,
                    "decision": "confirmed",
                    "reason": row.get("strict_reason", ""),
                }
            )

    with uncertain_path.open("r", encoding="utf-8", newline="") as handle:
        for row in csv.DictReader(handle):
            visual_material = (row.get("visual_material") or "").strip()
            if not visual_material:
                continue
            mapping[visual_material] = uncertain_material
            rows.append(
                {
                    "visual_material": visual_material,
                    "target_material": uncertain_material,
                    "decision": "uncertain_review",
                    "reason": row.get("strict_reason", ""),
                }
            )

    return mapping, rows


def _write_strict_review_mapping(
    rows: list[dict[str, str]],
    output_csv: str | Path | None,
    output_json: str | Path | None,
) -> dict[str, str]:
    """Write the exact material mapping used for the review export."""
    written: dict[str, str] = {}
    if output_csv:
        csv_path = _repo_path(output_csv)
        csv_path.parent.mkdir(parents=True, exist_ok=True)
        fieldnames = ["visual_material", "target_material", "decision", "reason"]
        with csv_path.open("w", encoding="utf-8", newline="") as handle:
            writer = csv.DictWriter(handle, fieldnames=fieldnames)
            writer.writeheader()
            writer.writerows(rows)
        written["csv"] = str(csv_path.relative_to(REPO_ROOT))
    if output_json:
        json_path = _repo_path(output_json)
        json_path.parent.mkdir(parents=True, exist_ok=True)
        json_path.write_text(json.dumps(rows, ensure_ascii=False, indent=2), encoding="utf-8")
        written["json"] = str(json_path.relative_to(REPO_ROOT))
    return written


def _blender_job_source() -> str:
    """Return the Python code executed inside Blender."""
    return r'''
import json
import hashlib
import struct
import sys
from pathlib import Path
import xml.etree.ElementTree as ET

import bpy


def enable_mitsuba_blender():
    try:
        import addon_utils
    except Exception as exc:
        print(f"[WARN] Could not import addon_utils: {exc}")
        return False

    for module_name in ("mitsuba_blender", "mitsuba-blender"):
        try:
            addon_utils.enable(module_name)
            if bpy.ops.export_scene.mitsuba.poll():
                return True
        except Exception as exc:
            print(f"[WARN] Could not enable {module_name}: {exc}")
    return False


STRICT_UNCERTAIN_MATERIAL = "simart_uncertain_review"
CONFIRMED_MATERIAL_SEPARATOR = "__src_"

MATERIAL_COLORS = {
    "itu_concrete": (0.55, 0.55, 0.52, 1.0),
    "itu_glass": (0.15, 0.55, 0.80, 0.55),
    "itu_brick": (0.58, 0.20, 0.12, 1.0),
    "itu_metal": (0.50, 0.50, 0.55, 1.0),
    "itu_wood": (0.45, 0.28, 0.12, 1.0),
    "itu_very_dry_ground": (0.42, 0.36, 0.27, 1.0),
    "itu_medium_dry_ground": (0.30, 0.34, 0.25, 1.0),
    "itu_wet_ground": (0.07, 0.11, 0.13, 1.0),
    "simart_uncertain_review": (1.00, 0.00, 0.85, 1.0),
    "simart_review_roof": (0.18, 0.17, 0.15, 1.0),
    "simart_review_asphalt": (0.09, 0.09, 0.085, 1.0),
}

IMAGE_AVERAGE_COLOR_CACHE = {}


def normalize_name(name):
    value = (name or "").strip().lower()
    if value.startswith("mat-"):
        value = value[4:]
    return value


def material_matches(material_name, wanted_name):
    current = normalize_name(material_name)
    wanted = normalize_name(wanted_name)
    aliases = {
        "itu_concrete": ["itu_concrete", "concrete", "cement", "stucco"],
        "itu_glass": ["itu_glass", "glass", "window"],
        "itu_brick": ["itu_brick", "brick"],
        "itu_metal": ["itu_metal", "metal", "iron", "steel"],
        "itu_wood": ["itu_wood", "wood"],
        "itu_very_dry_ground": ["itu_very_dry_ground", "ground", "terrain", "asphalt", "road"],
        "itu_medium_dry_ground": ["itu_medium_dry_ground", "ground", "terrain", "asphalt", "road"],
        "itu_wet_ground": ["itu_wet_ground", "wet_ground", "ground", "terrain", "asphalt", "road"],
    }
    candidates = aliases.get(wanted, [wanted])
    return any(candidate == current or candidate in current for candidate in candidates)


def sanitize_name(value):
    cleaned = []
    for char in str(value):
        if char.isalnum() or char in "_-":
            cleaned.append(char)
        else:
            cleaned.append("_")
    return "".join(cleaned).strip("_") or "unnamed"


def material_texture_names(material):
    if not material or not material.use_nodes or not material.node_tree:
        return []
    names = []
    for node in material.node_tree.nodes:
        if getattr(node, "type", "") != "TEX_IMAGE":
            continue
        image = getattr(node, "image", None)
        if not image:
            continue
        path = image.filepath or image.name
        if path:
            names.append(Path(path).name)
    return sorted(set(names))


def infer_sionna_material(material_name, texture_names=None, overrides=None):
    texture_names = texture_names or []
    overrides = overrides or {}
    raw_name = material_name or ""
    normalized = normalize_name(raw_name)
    for key, value in overrides.items():
        if normalize_name(key) == normalized:
            target = value["target"] if isinstance(value, dict) else value
            return target, "manual", f"manual override for {raw_name}"

    haystack = " ".join([raw_name, *texture_names]).lower()
    rules = [
        ("itu_glass", "high", ["glass", "window", "transparent"], "glass/window visual material"),
        ("itu_brick", "high", ["brick", "bricks"], "brick visual material"),
        ("itu_metal", "high", ["metal", "iron", "steel"], "metal visual material"),
        ("itu_wood", "high", ["wood", "trunk", "bark"], "wood/tree visual material"),
        (
            "itu_medium_dry_ground",
            "medium",
            ["asphalt", "bitumen", "road", "lane", "pavement", "sidewalk"],
            "road/asphalt-like surface",
        ),
        ("itu_concrete", "high", ["concrete", "cement", "stucco", "foundation"], "concrete/cement visual material"),
        ("itu_concrete", "medium", ["stone", "rock"], "stone approximated by concrete"),
        ("itu_medium_dry_ground", "medium", ["terrain", "ground", "grass", "soil"], "outdoor ground surface"),
        ("itu_concrete", "low", ["roof", "trim", "building", "facade", "sandiego"], "generic building material fallback"),
    ]
    for target, confidence, keywords, reason in rules:
        if any(keyword in haystack for keyword in keywords):
            return target, confidence, reason
    return "itu_concrete", "low", "unknown material fallback"


def get_or_create_material(name):
    material = bpy.data.materials.get(name)
    if material is None:
        material = bpy.data.materials.new(name=name)
    material.use_nodes = True
    color = MATERIAL_COLORS.get(normalize_name(name), (0.80, 0.80, 0.80, 1.0))
    material.diffuse_color = color
    bsdf = material.node_tree.nodes.get("Principled BSDF")
    if bsdf:
        if "Base Color" in bsdf.inputs:
            bsdf.inputs["Base Color"].default_value = color
        if "Alpha" in bsdf.inputs:
            bsdf.inputs["Alpha"].default_value = color[3]
    if color[3] < 1.0:
        material.blend_method = "BLEND"
        material.use_screen_refraction = True
    return material


def get_or_create_colored_material(name, color):
    material = bpy.data.materials.get(name)
    if material is None:
        material = bpy.data.materials.new(name=name)
    material.use_nodes = True
    material.diffuse_color = color
    bsdf = material.node_tree.nodes.get("Principled BSDF")
    if bsdf:
        if "Base Color" in bsdf.inputs:
            bsdf.inputs["Base Color"].default_value = color
        if "Alpha" in bsdf.inputs:
            bsdf.inputs["Alpha"].default_value = color[3]
        if "Roughness" in bsdf.inputs:
            bsdf.inputs["Roughness"].default_value = 0.75
    if color[3] < 1.0:
        material.blend_method = "BLEND"
        material.use_screen_refraction = True
    return material


def color_to_prop(color):
    if color is None:
        return ""
    return " ".join(f"{float(color[i]):.6g}" for i in range(min(4, len(color))))


def color_from_prop(value, fallback):
    try:
        parts = [float(part) for part in str(value).replace(",", " ").split()]
    except Exception:
        return fallback
    if len(parts) < 3:
        return fallback
    return (parts[0], parts[1], parts[2], parts[3] if len(parts) > 3 else 1.0)


def image_average_color(image, fallback=None):
    if image is None:
        return fallback
    key = image.filepath or image.name
    if key in IMAGE_AVERAGE_COLOR_CACHE:
        return IMAGE_AVERAGE_COLOR_CACHE[key]
    try:
        width, height = image.size
        pixel_count = int(width) * int(height)
        if pixel_count <= 0:
            return fallback
        pixels = image.pixels
        step = max(1, pixel_count // 4096)
        red = green = blue = weight = 0.0
        for pixel_index in range(0, pixel_count, step):
            offset = pixel_index * 4
            alpha = float(pixels[offset + 3]) if offset + 3 < len(pixels) else 1.0
            red += float(pixels[offset]) * alpha
            green += float(pixels[offset + 1]) * alpha
            blue += float(pixels[offset + 2]) * alpha
            weight += alpha
        if weight <= 0.0:
            return fallback
        color = (red / weight, green / weight, blue / weight, 1.0)
        IMAGE_AVERAGE_COLOR_CACHE[key] = color
        return color
    except Exception:
        return fallback


def source_material_display_color(material, fallback=(0.80, 0.80, 0.80, 1.0)):
    if material is None:
        return fallback
    base_color = tuple(float(material.diffuse_color[i]) for i in range(4))
    if material.use_nodes and material.node_tree:
        bsdf = material.node_tree.nodes.get("Principled BSDF")
        if bsdf and "Base Color" in bsdf.inputs:
            try:
                value = bsdf.inputs["Base Color"].default_value
                color = (float(value[0]), float(value[1]), float(value[2]), float(value[3]))
                if color[:3] != (1.0, 1.0, 1.0):
                    return color
            except Exception:
                pass
    return base_color


def semantic_material_color(source_name, target_name, texture_names, fallback):
    text = " ".join([str(source_name), *[str(name) for name in texture_names]]).lower()
    target = normalize_name(target_name)

    if target == "itu_brick":
        if "white" in text:
            return (0.76, 0.72, 0.64, 1.0)
        if "tan" in text:
            return (0.62, 0.48, 0.34, 1.0)
        return (0.50, 0.17, 0.10, 1.0)

    if target == "itu_concrete":
        if "bluegray" in text or "blue_gray" in text:
            return (0.48, 0.56, 0.62, 1.0)
        if "brown" in text:
            return (0.48, 0.34, 0.25, 1.0)
        if "olive" in text:
            return (0.45, 0.48, 0.32, 1.0)
        if "white" in text:
            return (0.78, 0.76, 0.70, 1.0)
        if "cement" in text or "foundation" in text:
            return (0.58, 0.56, 0.52, 1.0)
        return (0.56, 0.55, 0.51, 1.0)

    if target == "itu_glass":
        if "transparent" in text:
            return (0.55, 0.82, 0.95, 1.0)
        if "dark" in text or "black" in text:
            return (0.08, 0.18, 0.24, 1.0)
        return (0.24, 0.58, 0.78, 1.0)

    if target == "itu_metal":
        if "iron" in text or "steel" in text:
            return (0.35, 0.36, 0.38, 1.0)
        if "shiny" in text or "emissive" in text:
            return (0.62, 0.62, 0.58, 1.0)
        return (0.44, 0.42, 0.38, 1.0)

    if target == "itu_wood":
        if "floor" in text:
            return (0.48, 0.30, 0.16, 1.0)
        return (0.36, 0.23, 0.12, 1.0)

    if target in {"itu_medium_dry_ground", "itu_very_dry_ground", "itu_wet_ground"}:
        if "lane" in text and "yellow" in text:
            return (0.95, 0.78, 0.18, 1.0)
        if "lane" in text or "marking" in text:
            return (0.95, 0.92, 0.78, 1.0)
        if "road" in text or "asphalt" in text or "bitumen" in text:
            return (0.17, 0.17, 0.16, 1.0)
        if "grass" in text:
            return (0.30, 0.45, 0.24, 1.0)
        return (0.35, 0.34, 0.29, 1.0)

    return fallback


def review_palette_color(name):
    digest = hashlib.sha1(str(name).encode("utf-8", errors="replace")).digest()
    hue = digest[0] / 255.0
    sector = int(hue * 6.0)
    frac = hue * 6.0 - sector
    value = 1.0
    chroma = 0.82
    x = chroma * (1.0 - abs(frac % 2.0 - 1.0))
    colors = [
        (chroma, x, 0.0),
        (x, chroma, 0.0),
        (0.0, chroma, x),
        (0.0, x, chroma),
        (x, 0.0, chroma),
        (chroma, 0.0, x),
    ]
    r, g, b = colors[sector % 6]
    m = value - chroma
    return (r + m, g + m, b + m, 1.0)


def collect_material_inventory(name_contains="", overrides=None, object_preview_limit=20):
    materials = {}
    object_entries = []
    mesh_objects = [
        obj
        for obj in bpy.context.scene.objects
        if obj.type == "MESH" and (not name_contains or name_contains.lower() in obj.name.lower())
    ]

    for obj in mesh_objects:
        slot_entries = []
        slots = list(getattr(obj, "material_slots", []))
        if not slots:
            key = "<none>"
            entry = materials.setdefault(
                key,
                {
                    "material_name": key,
                    "suggested_sionna_material": "itu_concrete",
                    "confidence": "low",
                    "reason": "object has no material slot",
                    "slot_count": 0,
                    "object_count": 0,
                    "face_count": 0,
                    "texture_files": set(),
                    "objects": set(),
                },
            )
            entry["object_count"] += 1
            entry["objects"].add(obj.name)
            object_entries.append({"object_name": obj.name, "material_slots": []})
            continue

        for slot_index, slot in enumerate(slots):
            material = slot.material
            material_name = material.name if material else "<none>"
            textures = material_texture_names(material)
            target, confidence, reason = infer_sionna_material(material_name, textures, overrides)
            face_count = 0
            mesh = getattr(obj, "data", None)
            if mesh:
                face_count = sum(1 for polygon in mesh.polygons if polygon.material_index == slot_index)
            entry = materials.setdefault(
                material_name,
                {
                    "material_name": material_name,
                    "suggested_sionna_material": target,
                    "confidence": confidence,
                    "reason": reason,
                    "slot_count": 0,
                    "object_count": 0,
                    "face_count": 0,
                    "texture_files": set(),
                    "objects": set(),
                },
            )
            entry["slot_count"] += 1
            entry["face_count"] += face_count
            entry["texture_files"].update(textures)
            entry["objects"].add(obj.name)
            slot_entries.append(
                {
                    "slot_index": slot_index,
                    "material_name": material_name,
                    "suggested_sionna_material": target,
                    "confidence": confidence,
                    "reason": reason,
                    "face_count": face_count,
                    "texture_files": textures,
                }
            )
        object_entries.append({"object_name": obj.name, "material_slots": slot_entries})

    material_rows = []
    for entry in materials.values():
        objects = sorted(entry["objects"])
        material_rows.append(
            {
                "material_name": entry["material_name"],
                "suggested_sionna_material": entry["suggested_sionna_material"],
                "confidence": entry["confidence"],
                "reason": entry["reason"],
                "slot_count": entry["slot_count"],
                "object_count": len(objects),
                "face_count": entry["face_count"],
                "texture_files": sorted(entry["texture_files"]),
                "objects_preview": objects[:object_preview_limit],
            }
        )
    material_rows.sort(key=lambda row: (row["suggested_sionna_material"], row["material_name"].lower()))
    return {
        "mesh_object_count": len(mesh_objects),
        "material_count": len(material_rows),
        "materials": material_rows,
        "objects_preview": object_entries[:object_preview_limit],
    }


def apply_sionna_material_mapping(name_contains="", overrides=None):
    changed_slots = 0
    changed_objects = set()
    mapping_counts = {}
    for obj in bpy.context.scene.objects:
        if obj.type != "MESH" or (name_contains and name_contains.lower() not in obj.name.lower()):
            continue
        for slot_index, slot in enumerate(getattr(obj, "material_slots", [])):
            material = slot.material
            material_name = material.name if material else "<none>"
            source_color = material.diffuse_color if material else None
            textures = material_texture_names(material)
            target_name, confidence, reason = infer_sionna_material(material_name, textures, overrides)
            target = get_or_create_material(target_name)
            obj.data.materials[slot_index] = target
            obj["simart_source_material"] = material_name
            obj["simart_source_color"] = color_to_prop(source_color)
            obj["simart_target_material"] = target_name
            obj["simart_mapping_confidence"] = confidence
            obj["simart_mapping_reason"] = reason
            changed_slots += 1
            changed_objects.add(obj.name)
            mapping_counts[target_name] = mapping_counts.get(target_name, 0) + 1
            target["simart_inferred_from"] = material_name
            target["simart_inference_confidence"] = confidence
            target["simart_inference_reason"] = reason
    return {
        "changed_object_count": len(changed_objects),
        "changed_slot_count": changed_slots,
        "changed_objects_preview": sorted(changed_objects)[:20],
        "mapping_counts": mapping_counts,
    }


def annotate_sionna_material_mapping(name_contains="", overrides=None):
    annotated_slots = 0
    annotated_objects = set()
    mapping_counts = {}
    for obj in bpy.context.scene.objects:
        if obj.type != "MESH" or (name_contains and name_contains.lower() not in obj.name.lower()):
            continue
        for slot_index, slot in enumerate(getattr(obj, "material_slots", [])):
            material = slot.material
            material_name = material.name if material else "<none>"
            source_color = material.diffuse_color if material else None
            textures = material_texture_names(material)
            target_name, confidence, reason = infer_sionna_material(material_name, textures, overrides)
            obj["simart_source_material"] = material_name
            obj["simart_source_color"] = color_to_prop(source_color)
            obj["simart_target_material"] = target_name
            obj["simart_mapping_confidence"] = confidence
            obj["simart_mapping_reason"] = reason
            annotated_slots += 1
            annotated_objects.add(obj.name)
            mapping_counts[target_name] = mapping_counts.get(target_name, 0) + 1
    return {
        "annotated_object_count": len(annotated_objects),
        "annotated_slot_count": annotated_slots,
        "annotated_objects_preview": sorted(annotated_objects)[:20],
        "mapping_counts": mapping_counts,
    }


def split_scene_by_material(name_contains=""):
    split_source_count = 0
    errors = []
    candidates = [
        obj
        for obj in bpy.context.scene.objects
        if obj.type == "MESH"
        and len(obj.data.materials) > 1
        and (not name_contains or name_contains.lower() in obj.name.lower())
    ]
    for obj in candidates:
        try:
            bpy.ops.object.mode_set(mode="OBJECT")
        except Exception:
            pass
        bpy.ops.object.select_all(action="DESELECT")
        obj.select_set(True)
        bpy.context.view_layer.objects.active = obj
        try:
            bpy.ops.object.mode_set(mode="EDIT")
            bpy.ops.mesh.select_all(action="SELECT")
            bpy.ops.mesh.separate(type="MATERIAL")
            bpy.ops.object.mode_set(mode="OBJECT")
            split_source_count += 1
        except Exception as exc:
            errors.append(f"{obj.name}: {exc}")
            try:
                bpy.ops.object.mode_set(mode="OBJECT")
            except Exception:
                pass

    renamed_count = 0
    for obj in bpy.context.scene.objects:
        if (
            obj.type != "MESH"
            or not obj.data.materials
            or (name_contains and name_contains.lower() not in obj.name.lower())
        ):
            continue
        material = obj.data.materials[0]
        if not material:
            continue
        suffix = sanitize_name(material.name)
        if suffix and suffix.lower() not in obj.name.lower():
            obj.name = f"{sanitize_name(obj.name)}-{suffix}"
            obj.data.name = obj.name
            renamed_count += 1
    return {"split_source_count": split_source_count, "renamed_count": renamed_count, "errors": errors[:20]}


def object_matches(obj, from_material, name_contains):
    if name_contains and name_contains.lower() not in obj.name.lower():
        return False
    if not from_material:
        return True
    for slot in getattr(obj, "material_slots", []):
        if slot.material and material_matches(slot.material.name, from_material):
            return True
    return material_matches(obj.name, from_material)


def replace_materials(from_material, to_material, name_contains):
    target = get_or_create_material(to_material)
    changed_objects = []
    changed_slots = 0

    for obj in bpy.context.scene.objects:
        if obj.type != "MESH" or not object_matches(obj, from_material, name_contains):
            continue
        slots = obj.data.materials
        if len(slots) == 0:
            slots.append(target)
            changed_slots += 1
        elif from_material:
            for index, material in enumerate(slots):
                if material and material_matches(material.name, from_material):
                    slots[index] = target
                    changed_slots += 1
        else:
            for index in range(len(slots)):
                slots[index] = target
                changed_slots += 1
        changed_objects.append(obj.name)

    return changed_objects, changed_slots


def export_fbx(path):
    Path(path).parent.mkdir(parents=True, exist_ok=True)
    bpy.ops.export_scene.fbx(filepath=str(path), use_selection=False, path_mode="COPY")


def reset_scene():
    bpy.ops.object.select_all(action="SELECT")
    bpy.ops.object.delete()


def review_material_for_source(source_material, target_name, uncertain_color_mode):
    source_name = source_material.name if source_material else "<none>"
    source_textures = material_texture_names(source_material)
    source_color = source_material_display_color(source_material, (1.0, 0.0, 0.85, 1.0))
    normalized_target = normalize_name(target_name)
    if normalized_target == normalize_name(STRICT_UNCERTAIN_MATERIAL):
        material_name = f"review_{sanitize_name(source_name)}"
        structural_color = review_structural_placeholder_color(source_name)
        if structural_color is not None:
            color = structural_color
        elif uncertain_color_mode == "magenta":
            color = MATERIAL_COLORS[STRICT_UNCERTAIN_MATERIAL]
        elif uncertain_color_mode == "palette":
            color = review_palette_color(source_name)
        else:
            color = tuple(source_color[i] for i in range(min(4, len(source_color))))
            if len(color) < 4:
                color = (color[0], color[1], color[2], 1.0)
    else:
        material_name = f"{normalized_target}{CONFIRMED_MATERIAL_SEPARATOR}{sanitize_name(source_name)}"
        color = semantic_material_color(source_name, target_name, source_textures, source_color)
    material = get_or_create_colored_material(material_name, color)
    material["simart_source_material"] = source_name
    material["simart_target_material"] = target_name
    material["simart_review_color"] = color_to_prop(color)
    return material, color


def review_structural_placeholder_color(source_name):
    """Color large unresolved structures without changing their RF placeholder."""
    normalized = normalize_name(source_name)
    if "roof" in normalized:
        return MATERIAL_COLORS["simart_review_roof"]
    if (
        ("asphalt" in normalized or "road" in normalized)
        and "lane" not in normalized
        and "marking" not in normalized
    ):
        return MATERIAL_COLORS["simart_review_asphalt"]
    if "ground" in normalized and "water" not in normalized:
        return MATERIAL_COLORS["itu_concrete"]
    return None


def rebuild_scene_as_clean_single_material_meshes(name_contains="", overrides=None, uncertain_color_mode="palette"):
    """Bake the current scene into clean per-material meshes for Mitsuba export."""
    def is_valid_triangle(coords):
        if len({tuple(co) for co in coords}) < 3:
            return False
        ax, ay, az = coords[0]
        bx, by, bz = coords[1]
        cx, cy, cz = coords[2]
        ux, uy, uz = bx - ax, by - ay, bz - az
        vx, vy, vz = cx - ax, cy - ay, cz - az
        nx = uy * vz - uz * vy
        ny = uz * vx - ux * vz
        nz = ux * vy - uy * vx
        return (nx * nx + ny * ny + nz * nz) > 1e-18

    depsgraph = bpy.context.evaluated_depsgraph_get()
    source_objects = [
        obj
        for obj in bpy.context.scene.objects
        if obj.type == "MESH" and (not name_contains or name_contains.lower() in obj.name.lower())
    ]
    original_objects = list(bpy.context.scene.objects)
    created = []
    skipped = []
    material_counts = {}
    total_faces = 0

    for obj in source_objects:
        eval_obj = obj.evaluated_get(depsgraph)
        try:
            mesh = eval_obj.to_mesh()
        except Exception as exc:
            skipped.append(f"{obj.name}: to_mesh failed: {exc}")
            continue
        if mesh is None:
            skipped.append(f"{obj.name}: empty evaluated mesh")
            continue
        try:
            mesh.validate(clean_customdata=True)
            mesh.update(calc_edges=True)
            mesh.calc_loop_triangles()
            groups = {}
            for tri in mesh.loop_triangles:
                poly = mesh.polygons[tri.polygon_index]
                groups.setdefault(poly.material_index, []).append(tri)
            for mat_index, triangles in groups.items():
                if not triangles:
                    continue
                source_material = None
                if 0 <= mat_index < len(obj.material_slots):
                    source_material = obj.material_slots[mat_index].material
                source_name = source_material.name if source_material else "<none>"
                textures = material_texture_names(source_material)
                target_name, confidence, reason = infer_sionna_material(source_name, textures, overrides)
                review_material, review_color = review_material_for_source(
                    source_material, target_name, uncertain_color_mode
                )

                vertex_map = {}
                baked_vertices = {}
                vertices = []
                faces = []
                dropped_faces = 0
                for tri in triangles:
                    coords = []
                    for vertex_index in tri.vertices:
                        if vertex_index not in baked_vertices:
                            baked = eval_obj.matrix_world @ mesh.vertices[vertex_index].co
                            baked_vertices[vertex_index] = (float(baked.x), float(baked.y), float(baked.z))
                        coords.append(baked_vertices[vertex_index])
                    if not is_valid_triangle(coords):
                        dropped_faces += 1
                        continue
                    face = []
                    for vertex_index in tri.vertices:
                        if vertex_index not in vertex_map:
                            vertex_map[vertex_index] = len(vertices)
                            vertices.append(baked_vertices[vertex_index])
                        face.append(vertex_map[vertex_index])
                    faces.append(tuple(face))
                if not faces:
                    continue

                clean_mesh = bpy.data.meshes.new(
                    sanitize_name(f"{obj.name}_{source_name}_mesh")[:60]
                )
                clean_mesh.from_pydata(vertices, [], faces)
                clean_mesh.materials.append(review_material)
                clean_mesh.validate(clean_customdata=True)
                clean_mesh.update(calc_edges=True)

                clean_name = sanitize_name(f"{obj.name}-{source_name}")[:90]
                clean_obj = bpy.data.objects.new(clean_name, clean_mesh)
                clean_obj["simart_source_material"] = source_name
                clean_obj["simart_source_color"] = color_to_prop(source_material.diffuse_color if source_material else None)
                clean_obj["simart_target_material"] = target_name
                clean_obj["simart_mapping_confidence"] = confidence
                clean_obj["simart_mapping_reason"] = reason
                clean_obj["simart_review_color"] = color_to_prop(review_color)
                clean_obj["simart_dropped_degenerate_faces"] = dropped_faces
                bpy.context.scene.collection.objects.link(clean_obj)
                created.append(clean_obj.name)
                material_counts[target_name] = material_counts.get(target_name, 0) + 1
                total_faces += len(faces)
        finally:
            try:
                eval_obj.to_mesh_clear()
            except Exception:
                pass

    bpy.ops.object.select_all(action="DESELECT")
    for obj in original_objects:
        if obj.name in bpy.context.scene.objects:
            obj.select_set(True)
    if any(obj.select_get() for obj in original_objects if obj.name in bpy.context.scene.objects):
        bpy.ops.object.delete()

    return {
        "created_object_count": len(created),
        "created_objects_preview": sorted(created)[:20],
        "skipped_count": len(skipped),
        "skipped_preview": skipped[:20],
        "material_counts": material_counts,
        "total_face_count": total_faces,
    }


def postprocess_sionna_xml_materials(path):
    """Convert exported visual materials into Sionna radio-material BSDFs."""
    xml_path = Path(path)
    tree = ET.parse(xml_path)
    root = tree.getroot()
    converted = 0
    review_count = 0

    def first_rgb_value(node, fallback):
        for name in ("color", "reflectance", "base_color"):
            elem = node.find(f".//rgb[@name='{name}']")
            if elem is not None and elem.get("value"):
                return elem.get("value")
        return fallback

    def itu_type_from_material_name(normalized_name):
        if not normalized_name.startswith("itu_"):
            return None
        base_name = normalized_name.split(CONFIRMED_MATERIAL_SEPARATOR, 1)[0]
        return base_name[4:]

    for bsdf in root.findall("./bsdf"):
        mat_id = bsdf.attrib.get("id", "")
        mat_name = mat_id[4:] if mat_id.startswith("mat-") else mat_id
        normalized = normalize_name(mat_name)
        itu_type = itu_type_from_material_name(normalized)
        if itu_type is not None:
            base_name = normalized.split(CONFIRMED_MATERIAL_SEPARATOR, 1)[0]
            color = MATERIAL_COLORS.get(base_name, (0.80, 0.80, 0.80, 1.0))
            color_value = first_rgb_value(bsdf, f"{color[0]:.6g} {color[1]:.6g} {color[2]:.6g}")
        elif normalized.startswith("review_"):
            itu_type = "concrete"
            color_value = first_rgb_value(bsdf, "1 0 0.85")
            review_count += 1
        else:
            itu_type = "concrete"
            color_value = first_rgb_value(bsdf, "0.8 0.8 0.8")

        bsdf.clear()
        bsdf.attrib["type"] = "itu-radio-material"
        bsdf.attrib["id"] = mat_id
        bsdf.attrib["name"] = mat_id
        bsdf.append(ET.Element("string", {"name": "type", "value": itu_type}))
        bsdf.append(ET.Element("float", {"name": "thickness", "value": "0.1"}))
        bsdf.append(ET.Element("rgb", {"name": "color", "value": color_value}))
        converted += 1

    tree.write(xml_path, encoding="utf-8", xml_declaration=False)
    return {
        "converted_material_count": converted,
        "review_material_count": review_count,
    }


def load_job_source(job):
    source_type = job.get("source_type", "blend")
    source_path = job["source_path"]
    if source_type == "blend":
        bpy.ops.wm.open_mainfile(filepath=source_path)
    elif source_type == "fbx":
        reset_scene()
        bpy.ops.import_scene.fbx(filepath=source_path)
    else:
        raise RuntimeError(f"Unsupported source_type: {source_type}")


def export_mitsuba(path):
    Path(path).parent.mkdir(parents=True, exist_ok=True)
    attempts = [
        (
            "export_scene.mitsuba",
            lambda: bpy.ops.export_scene.mitsuba(
                filepath=str(path),
                export_ids=True,
                split_files=False,
                ignore_background=True,
            ),
        ),
        ("wm.mitsuba_export", lambda: bpy.ops.wm.mitsuba_export(filepath=str(path))),
        ("wm.export_mitsuba", lambda: bpy.ops.wm.export_mitsuba(filepath=str(path))),
    ]
    errors = []
    for name, call in attempts:
        try:
            call()
            return name
        except Exception as exc:
            errors.append(f"{name}: {exc}")
    raise RuntimeError(
        "Mitsuba Blender export operator was not found or failed. "
        "Install/enable mitsuba-blender v0.4.0, then export once from Blender UI if needed. "
        + " | ".join(errors)
    )


def install_mitsuba_invalid_normal_skipper():
    """Let the official exporter continue when only a few meshes have invalid normals."""
    try:
        from mitsuba_blender.io.exporter import geometry
    except Exception as exc:
        return {
            "installed": False,
            "error": f"Could not import Mitsuba geometry exporter: {exc}",
            "skipped": [],
        }

    original_convert_mesh = geometry.convert_mesh
    skipped = []

    def safe_convert_mesh(export_ctx, b_mesh, matrix_world, name, mat_nr):
        try:
            return original_convert_mesh(export_ctx, b_mesh, matrix_world, name, mat_nr)
        except RuntimeError as exc:
            message = str(exc)
            if "invalid normals" not in message:
                raise
            skipped.append({"name": name, "error": message})
            try:
                export_ctx.log(
                    f"Skipping mesh {name!r} because Mitsuba reported invalid normals.",
                    "WARN",
                )
            except Exception:
                print(f"[WARN] Skipping mesh {name!r}: {message}")
            return None

    geometry.convert_mesh = safe_convert_mesh
    return {
        "installed": True,
        "skipped": skipped,
    }


def export_simple_mitsuba_xml(path, uncertain_color_mode="palette", binary_ply=True):
    """Export a review-friendly Mitsuba XML with one PLY per mesh object."""
    from bpy_extras.io_utils import axis_conversion
    from xml.sax.saxutils import escape

    xml_path = Path(path)
    xml_path.parent.mkdir(parents=True, exist_ok=True)
    meshes_dir = xml_path.parent / "meshes"
    meshes_dir.mkdir(parents=True, exist_ok=True)
    for stale in meshes_dir.glob("*.ply"):
        stale.unlink()

    axis_mat = axis_conversion(to_forward="-Z", to_up="Y").to_4x4()

    def clean_id(value):
        cleaned = sanitize_name(value)
        return cleaned.replace(".", "_")

    def unique_name(base, used):
        candidate = base
        index = 1
        while candidate in used:
            candidate = f"{base}_{index:03d}"
            index += 1
        used.add(candidate)
        return candidate

    def itu_type_for_review_material(material_name):
        normalized = normalize_name(material_name)
        if normalized == normalize_name(STRICT_UNCERTAIN_MATERIAL):
            return "concrete"
        if normalized.startswith("itu_"):
            return normalized[4:]
        return "concrete"

    material_defs = {}
    shape_lines = []
    used_shape_ids = set()
    exported_shapes = 0

    mesh_objects = [obj for obj in bpy.context.scene.objects if obj.type == "MESH"]
    for obj in mesh_objects:
        mesh = obj.data
        mesh.validate(clean_customdata=True)
        mesh.update()
        mesh.calc_loop_triangles()
        if not mesh.loop_triangles:
            continue

        material = mesh.materials[0] if mesh.materials else get_or_create_material(STRICT_UNCERTAIN_MATERIAL)
        material_name = material.name if material else STRICT_UNCERTAIN_MATERIAL
        source_material_name = obj.get("simart_source_material", material_name)
        source_material_color = obj.get("simart_source_color", "")
        target_material_name = obj.get("simart_target_material", material_name)
        mapping_reason = obj.get("simart_mapping_reason", "")
        normalized_target = normalize_name(target_material_name)
        if normalized_target == normalize_name(STRICT_UNCERTAIN_MATERIAL):
            material_id = f"mat-review_{clean_id(source_material_name)}"
        elif normalized_target.startswith("itu_"):
            material_id = f"mat-{clean_id(target_material_name)}"
        else:
            material_id = f"mat-{clean_id(material_name)}"
        if material_id not in material_defs:
            color = material.diffuse_color if material else (1.0, 0.0, 0.85, 1.0)
            if (
                normalized_target == normalize_name(STRICT_UNCERTAIN_MATERIAL)
                and uncertain_color_mode == "magenta"
            ):
                color = MATERIAL_COLORS[STRICT_UNCERTAIN_MATERIAL]
            structural_color = None
            if normalized_target == normalize_name(STRICT_UNCERTAIN_MATERIAL):
                structural_color = review_structural_placeholder_color(source_material_name)
            if structural_color is not None:
                color = structural_color
            elif (
                normalized_target == normalize_name(STRICT_UNCERTAIN_MATERIAL)
                and uncertain_color_mode == "palette"
            ):
                color = review_palette_color(source_material_name)
            elif normalized_target == normalize_name(STRICT_UNCERTAIN_MATERIAL):
                color = color_from_prop(source_material_color, color)
            elif normalized_target in MATERIAL_COLORS:
                color = MATERIAL_COLORS[normalized_target]
            material_defs[material_id] = {
                "name": material_id,
                "itu_type": itu_type_for_review_material(target_material_name),
                "color": tuple(float(color[i]) for i in range(3)),
            }

        shape_id = unique_name(f"mesh-{clean_id(obj.name)}", used_shape_ids)
        filename = f"{shape_id}.ply"
        ply_path = meshes_dir / filename
        transform = axis_mat @ obj.matrix_world

        if binary_ply:
            with ply_path.open("wb") as handle:
                header = (
                    "ply\n"
                    "format binary_little_endian 1.0\n"
                    f"element vertex {len(mesh.vertices)}\n"
                    "property float x\n"
                    "property float y\n"
                    "property float z\n"
                    f"element face {len(mesh.loop_triangles)}\n"
                    "property list uchar int vertex_indices\n"
                    "end_header\n"
                )
                handle.write(header.encode("ascii"))
                for vertex in mesh.vertices:
                    co = transform @ vertex.co
                    handle.write(struct.pack("<fff", float(co.x), float(co.y), float(co.z)))
                for tri in mesh.loop_triangles:
                    v0, v1, v2 = tri.vertices
                    handle.write(struct.pack("<Biii", 3, int(v0), int(v1), int(v2)))
        else:
            with ply_path.open("w", encoding="ascii", newline="\n") as handle:
                handle.write("ply\n")
                handle.write("format ascii 1.0\n")
                handle.write(f"element vertex {len(mesh.vertices)}\n")
                handle.write("property float x\n")
                handle.write("property float y\n")
                handle.write("property float z\n")
                handle.write(f"element face {len(mesh.loop_triangles)}\n")
                handle.write("property list uchar int vertex_indices\n")
                handle.write("end_header\n")
                for vertex in mesh.vertices:
                    co = transform @ vertex.co
                    handle.write(f"{co.x:.9g} {co.y:.9g} {co.z:.9g}\n")
                for tri in mesh.loop_triangles:
                    v0, v1, v2 = tri.vertices
                    handle.write(f"3 {v0} {v1} {v2}\n")

        shape_lines.append(
            "\n"
            f"\t<!-- source_material=\"{escape(str(source_material_name))}\" "
            f"target_material=\"{escape(str(target_material_name))}\" "
            f"reason=\"{escape(str(mapping_reason))}\" -->\n"
            f'\t<shape type="ply" id="{escape(shape_id)}" name="{escape(shape_id)}">\n'
            f'\t\t<string name="filename" value="meshes/{escape(filename)}"/>\n'
            f'\t\t<boolean name="face_normals" value="true"/>\n'
            f'\t\t<ref id="{escape(material_id)}" name="bsdf"/>\n'
            "\t</shape>\n"
        )
        exported_shapes += 1

    material_lines = []
    for material_id, info in sorted(material_defs.items()):
        r, g, b = info["color"]
        itu_type = info["itu_type"]
        material_lines.append(
            f'\t<bsdf type="itu-radio-material" id="{escape(material_id)}" name="{escape(material_id)}">\n'
            f'\t\t<string name="type" value="{escape(itu_type)}"/>\n'
            '\t\t<float name="thickness" value="0.1"/>\n'
            f'\t\t<rgb value="{r:.6g} {g:.6g} {b:.6g}" name="color"/>\n'
            "\t</bsdf>\n"
        )

    xml_text = (
        '<scene version="2.1.0">\n\n'
        "<!-- Review scene generated by agent.sionna_asset_tools. -->\n\n"
        "<!-- Camera and Rendering Parameters -->\n\n"
        '\t<integrator type="path" id="elm__0" name="elm__0">\n'
        '\t\t<integer name="max_depth" value="12"/>\n'
        "\t</integrator>\n\n"
        "<!-- Materials -->\n\n"
        + "".join(material_lines)
        + "\n<!-- Emitters -->\n\n"
        '\t<emitter type="constant" id="World" name="World">\n'
        '\t\t<rgb value="0.771761 0.771761 0.771761" name="radiance"/>\n'
        "\t</emitter>\n\n"
        "<!-- Shapes -->\n"
        + "".join(shape_lines)
        + "\n</scene>\n"
    )
    xml_path.write_text(xml_text, encoding="utf-8")
    return {
        "operator": "simple_binary_ply" if binary_ply else "simple_ascii_ply",
        "binary_ply": binary_ply,
        "uncertain_color_mode": uncertain_color_mode,
        "exported_shape_count": exported_shapes,
        "exported_material_count": len(material_defs),
        "mesh_dir": str(meshes_dir),
    }


def run_one_job(job):
    load_job_source(job)
    changed_objects, changed_slots = replace_materials(
        job["from_material"],
        job["to_material"],
        job.get("name_contains", ""),
    )
    result = {
        "source_type": job.get("source_type", "blend"),
        "source_path": job["source_path"],
        "from_material": job["from_material"],
        "to_material": job["to_material"],
        "changed_object_count": len(changed_objects),
        "changed_slot_count": changed_slots,
        "changed_objects_preview": changed_objects[:20],
        "saved_source_blend": False,
        "exported_fbx": None,
        "exported_xml": None,
        "mitsuba_operator": None,
    }
    if not job["apply"]:
        return result

    if job.get("source_type", "blend") == "blend" and job.get("save_blend", True):
        bpy.ops.wm.save_as_mainfile(filepath=job["source_path"])
        result["saved_source_blend"] = True
    if job.get("export_fbx"):
        export_fbx(job["export_fbx"])
        result["exported_fbx"] = job["export_fbx"]
    if job.get("export_xml"):
        result["mitsuba_operator"] = export_mitsuba(job["export_xml"])
        result["exported_xml"] = job["export_xml"]
    return result


def run_inspect_fbx(payload):
    reset_scene()
    bpy.ops.import_scene.fbx(filepath=payload["fbx_path"])
    inventory = collect_material_inventory(
        name_contains=payload.get("name_contains", ""),
        overrides=payload.get("material_overrides", {}),
        object_preview_limit=payload.get("object_preview_limit", 20),
    )
    inventory["source_path"] = payload["fbx_path"]
    return inventory


def run_import_fbx_to_blend(payload):
    reset_scene()
    bpy.ops.import_scene.fbx(filepath=payload["fbx_path"])
    inventory = collect_material_inventory(
        name_contains=payload.get("name_contains", ""),
        overrides=payload.get("material_overrides", {}),
        object_preview_limit=payload.get("object_preview_limit", 20),
    )
    result = {
        "source_path": payload["fbx_path"],
        "mode": "applied" if payload.get("apply", False) else "dry_run",
        "mesh_object_count": inventory["mesh_object_count"],
        "material_count": inventory["material_count"],
        "materials_preview": inventory["materials"][: payload.get("material_preview_limit", 20)],
        "saved_blend": None,
    }
    if payload.get("apply", False):
        blend_path = payload["output_blend"]
        Path(blend_path).parent.mkdir(parents=True, exist_ok=True)
        bpy.ops.wm.save_as_mainfile(filepath=blend_path)
        result["saved_blend"] = blend_path
    return result


def run_export_sionna_from_blend(payload):
    bpy.ops.wm.open_mainfile(filepath=payload["blend_path"])
    before = collect_material_inventory(
        name_contains=payload.get("name_contains", ""),
        overrides=payload.get("material_overrides", {}),
        object_preview_limit=payload.get("object_preview_limit", 20),
    )
    split_result = {"split_source_count": 0, "renamed_count": 0, "errors": []}
    rebuild_result = None
    if payload.get("exporter", "mitsuba") == "mitsuba":
        rebuild_result = rebuild_scene_as_clean_single_material_meshes(
            name_contains=payload.get("name_contains", ""),
            overrides=payload.get("material_overrides", {}),
            uncertain_color_mode=payload.get("uncertain_color_mode", "palette"),
        )
        annotated = {
            "annotated_object_count": rebuild_result["created_object_count"],
            "annotated_slot_count": rebuild_result["created_object_count"],
            "annotated_objects_preview": rebuild_result["created_objects_preview"],
            "mapping_counts": rebuild_result["material_counts"],
        }
    else:
        if payload.get("split_by_material", True):
            split_result = split_scene_by_material(payload.get("name_contains", ""))
        annotated = annotate_sionna_material_mapping(
            name_contains=payload.get("name_contains", ""),
            overrides=payload.get("material_overrides", {}),
        )

    result = {
        "source_path": payload["blend_path"],
        "source_type": "blend",
        "mode": "applied" if payload.get("apply", False) else "dry_run",
        "input_mesh_object_count": before["mesh_object_count"],
        "input_material_count": before["material_count"],
        "annotated_object_count": annotated["annotated_object_count"],
        "annotated_slot_count": annotated["annotated_slot_count"],
        "annotated_objects_preview": annotated["annotated_objects_preview"],
        "mapping_counts": annotated["mapping_counts"],
        "split_by_material": payload.get("split_by_material", True),
        "split_result": split_result,
        "rebuild_result": rebuild_result,
        "saved_blend": None,
        "exported_xml": None,
        "mitsuba_operator": None,
        "materials_preview": before["materials"][: payload.get("material_preview_limit", 20)],
    }
    if not payload.get("apply", False):
        return result

    blend_path = payload.get("output_blend", "")
    xml_path = payload.get("output_xml", "")
    if blend_path:
        Path(blend_path).parent.mkdir(parents=True, exist_ok=True)
        bpy.ops.wm.save_as_mainfile(filepath=blend_path)
        result["saved_blend"] = blend_path
    if xml_path:
        if payload.get("exporter", "mitsuba") == "mitsuba":
            invalid_normal_skipper = install_mitsuba_invalid_normal_skipper()
            operator = export_mitsuba(xml_path)
            postprocess_result = postprocess_sionna_xml_materials(xml_path)
            result["mitsuba_operator"] = operator
            result["official_xml_postprocess"] = postprocess_result
            result["official_skipped_invalid_normals_count"] = len(
                invalid_normal_skipper.get("skipped", [])
            )
            result["official_skipped_invalid_normals_preview"] = invalid_normal_skipper.get(
                "skipped", []
            )[:20]
            if not invalid_normal_skipper.get("installed", False):
                result["official_invalid_normal_skipper_error"] = invalid_normal_skipper.get("error")
        else:
            simple_result = export_simple_mitsuba_xml(
                xml_path,
                uncertain_color_mode=payload.get("uncertain_color_mode", "source"),
                binary_ply=payload.get("binary_ply", True),
            )
            result["mitsuba_operator"] = simple_result["operator"]
            result["simple_xml_export"] = simple_result
        result["exported_xml"] = xml_path
    return result


def run_export_sionna_from_fbx(payload):
    reset_scene()
    bpy.ops.import_scene.fbx(filepath=payload["fbx_path"])
    before = collect_material_inventory(
        name_contains=payload.get("name_contains", ""),
        overrides=payload.get("material_overrides", {}),
        object_preview_limit=payload.get("object_preview_limit", 20),
    )
    split_result = {"split_source_count": 0, "renamed_count": 0, "errors": []}
    if payload.get("split_by_material", True) and payload.get("preserve_source_material_names", False):
        split_result = split_scene_by_material(payload.get("name_contains", ""))
    applied = apply_sionna_material_mapping(
        name_contains=payload.get("name_contains", ""),
        overrides=payload.get("material_overrides", {}),
    )
    if payload.get("split_by_material", True) and not payload.get("preserve_source_material_names", False):
        split_result = split_scene_by_material(payload.get("name_contains", ""))

    result = {
        "source_path": payload["fbx_path"],
        "mode": "applied" if payload.get("apply", False) else "dry_run",
        "input_mesh_object_count": before["mesh_object_count"],
        "input_material_count": before["material_count"],
        "changed_object_count": applied["changed_object_count"],
        "changed_slot_count": applied["changed_slot_count"],
        "changed_objects_preview": applied["changed_objects_preview"],
        "mapping_counts": applied["mapping_counts"],
        "split_by_material": payload.get("split_by_material", True),
        "split_result": split_result,
        "saved_blend": None,
        "exported_xml": None,
        "mitsuba_operator": None,
        "materials_preview": before["materials"][: payload.get("material_preview_limit", 20)],
    }
    if not payload.get("apply", False):
        return result

    blend_path = payload.get("output_blend", "")
    xml_path = payload.get("output_xml", "")
    if blend_path:
        Path(blend_path).parent.mkdir(parents=True, exist_ok=True)
        bpy.ops.wm.save_as_mainfile(filepath=blend_path)
        result["saved_blend"] = blend_path
    if xml_path:
        if payload.get("simple_xml_export", False):
            simple_result = export_simple_mitsuba_xml(
                xml_path,
                uncertain_color_mode=payload.get("uncertain_color_mode", "source"),
                binary_ply=payload.get("binary_ply", True),
            )
            result["mitsuba_operator"] = simple_result["operator"]
            result["simple_xml_export"] = simple_result
        else:
            result["mitsuba_operator"] = export_mitsuba(xml_path)
        result["exported_xml"] = xml_path
    return result


def main():
    marker = "--"
    payload_arg = sys.argv[sys.argv.index(marker) + 1]
    payload = json.loads(payload_arg)
    enable_mitsuba_blender()
    if payload.get("mode") == "inspect_fbx_materials":
        results = [run_inspect_fbx(payload)]
    elif payload.get("mode") == "import_fbx_to_blend":
        results = [run_import_fbx_to_blend(payload)]
    elif payload.get("mode") == "export_sionna_from_blend":
        results = [run_export_sionna_from_blend(payload)]
    elif payload.get("mode") == "export_sionna_from_fbx":
        results = [run_export_sionna_from_fbx(payload)]
    else:
        results = [run_one_job(job) for job in payload["jobs"]]
    print("SIMART_ASSET_TOOLS_RESULT=" + json.dumps(results, ensure_ascii=False))


if __name__ == "__main__":
    main()
'''


def replace_material_pipeline(
    from_material: str,
    to_material: str,
    visual_source: str = "fbx",
    visual_blend: str | Path = DEFAULT_BIGCITY_VISUAL_BLEND,
    visual_fbx_input: str | Path = DEFAULT_BIGCITY_VISUAL_FBX,
    visual_fbx: str | Path = DEFAULT_BIGCITY_VISUAL_FBX,
    sionna_blend: str | Path = DEFAULT_BIGCITY_SIONNA_BLEND,
    sionna_xml: str | Path = DEFAULT_BIGCITY_SIONNA_XML,
    visual_from_material: str | None = None,
    visual_to_material: str | None = None,
    sionna_from_material: str | None = None,
    sionna_to_material: str | None = None,
    name_contains: str = "",
    apply_changes: bool = False,
    blender_path: str | None = None,
) -> str:
    """Replace material in visual and Sionna Blender sources, then export outputs."""
    if visual_source not in {"blend", "fbx"}:
        raise AssetToolError("visual_source must be 'blend' or 'fbx'.")
    visual_blend_path = _repo_path(visual_blend, must_exist=True)
    visual_fbx_input_path = _repo_path(visual_fbx_input, must_exist=True)
    visual_fbx_path = _repo_path(visual_fbx)
    sionna_blend_path = _repo_path(sionna_blend, must_exist=True)
    sionna_xml_path = _repo_path(sionna_xml)
    visual_input_path = visual_blend_path if visual_source == "blend" else visual_fbx_input_path
    visual_from = visual_from_material or from_material
    visual_to = visual_to_material or to_material
    sionna_from = sionna_from_material or from_material
    sionna_to = sionna_to_material or to_material

    jobs = [
        {
            "source_type": visual_source,
            "source_path": str(visual_input_path),
            "from_material": visual_from,
            "to_material": visual_to,
            "name_contains": name_contains,
            "apply": apply_changes,
            "export_fbx": str(visual_fbx_path),
            "save_blend": visual_source == "blend",
        },
        {
            "source_type": "blend",
            "source_path": str(sionna_blend_path),
            "from_material": sionna_from,
            "to_material": sionna_to,
            "name_contains": name_contains,
            "apply": apply_changes,
            "export_xml": str(sionna_xml_path),
            "save_blend": True,
        },
    ]

    mode = "applied" if apply_changes else "dry_run"
    results = _run_blender_payload({"jobs": jobs}, blender_path=blender_path)
    return json.dumps(
        {
            "mode": mode,
            "requested_from_material": from_material,
            "requested_to_material": to_material,
            "visual_material_mapping": {"from": visual_from, "to": visual_to},
            "sionna_material_mapping": {"from": sionna_from, "to": sionna_to},
            "visual_source": visual_source,
            "visual_output": str(visual_fbx_path.relative_to(REPO_ROOT)),
            "sionna_output": str(sionna_xml_path.relative_to(REPO_ROOT)),
            "jobs": results,
        },
        ensure_ascii=False,
        indent=2,
    )


def inspect_fbx_materials(
    fbx_path: str | Path = DEFAULT_BIGCITY_VISUAL_FBX,
    output_csv: str | Path | None = DEFAULT_AGENT_OUTPUT_DIR / "bigcity_fbx_material_inventory.csv",
    output_json: str | Path | None = DEFAULT_AGENT_OUTPUT_DIR / "bigcity_fbx_material_inventory.json",
    material_overrides: str | Path | None = None,
    name_contains: str = "",
    blender_path: str | None = None,
) -> str:
    """Read an FBX with Blender and report visual material slots plus Sionna suggestions."""
    fbx = _repo_path(fbx_path, must_exist=True)
    overrides = _load_material_overrides(material_overrides)
    results = _run_blender_payload(
        {
            "mode": "inspect_fbx_materials",
            "fbx_path": str(fbx),
            "name_contains": name_contains,
            "material_overrides": overrides,
        },
        blender_path=blender_path,
    )
    inventory = results[0]

    written: dict[str, str] = {}
    if output_csv:
        csv_path = _repo_path(output_csv)
        _write_material_inventory_csv(csv_path, inventory["materials"])
        written["csv"] = str(csv_path.relative_to(REPO_ROOT))
    if output_json:
        json_path = _repo_path(output_json)
        json_path.parent.mkdir(parents=True, exist_ok=True)
        json_path.write_text(json.dumps(inventory, ensure_ascii=False, indent=2), encoding="utf-8")
        written["json"] = str(json_path.relative_to(REPO_ROOT))

    return json.dumps(
        {
            "source": str(fbx.relative_to(REPO_ROOT)),
            "mesh_object_count": inventory["mesh_object_count"],
            "material_count": inventory["material_count"],
            "written": written,
            "materials_preview": inventory["materials"][:20],
        },
        ensure_ascii=False,
        indent=2,
    )


def import_fbx_to_blend(
    fbx_path: str | Path = DEFAULT_BIGCITY_VISUAL_FBX,
    output_blend: str | Path = DEFAULT_BIGCITY_FBX_MATERIAL_BLEND,
    name_contains: str = "",
    apply_changes: bool = False,
    blender_path: str | None = None,
) -> str:
    """Import the visual FBX into Blender without changing its material slots."""
    fbx = _repo_path(fbx_path, must_exist=True)
    blend = _repo_path(output_blend)
    results = _run_blender_payload(
        {
            "mode": "import_fbx_to_blend",
            "fbx_path": str(fbx),
            "output_blend": str(blend),
            "name_contains": name_contains,
            "apply": apply_changes,
        },
        blender_path=blender_path,
    )
    result = results[0]
    result["source"] = str(fbx.relative_to(REPO_ROOT))
    result["output_blend"] = str(blend.relative_to(REPO_ROOT))
    return json.dumps(result, ensure_ascii=False, indent=2)


def export_sionna_from_fbx(
    fbx_path: str | Path = DEFAULT_BIGCITY_VISUAL_FBX,
    output_blend: str | Path = DEFAULT_BIGCITY_AUTO_SIONNA_BLEND,
    output_xml: str | Path = DEFAULT_BIGCITY_AUTO_SIONNA_XML,
    material_overrides: str | Path | None = None,
    name_contains: str = "",
    split_by_material: bool = True,
    apply_changes: bool = False,
    blender_path: str | None = None,
) -> str:
    """Generate a material-preserving Sionna scene from the visual FBX."""
    fbx = _repo_path(fbx_path, must_exist=True)
    blend = _repo_path(output_blend)
    xml = _repo_path(output_xml)
    overrides = _load_material_overrides(material_overrides)
    results = _run_blender_payload(
        {
            "mode": "export_sionna_from_fbx",
            "fbx_path": str(fbx),
            "output_blend": str(blend),
            "output_xml": str(xml),
            "name_contains": name_contains,
            "material_overrides": overrides,
            "split_by_material": split_by_material,
            "apply": apply_changes,
        },
        blender_path=blender_path,
    )
    result = results[0]
    result["source"] = str(fbx.relative_to(REPO_ROOT))
    result["output_blend"] = str(blend.relative_to(REPO_ROOT))
    result["output_xml"] = str(xml.relative_to(REPO_ROOT))
    return json.dumps(result, ensure_ascii=False, indent=2)


def export_strict_review_scene(
    source_blend: str | Path = DEFAULT_BIGCITY_FBX_MATERIAL_BLEND,
    fbx_path: str | Path = DEFAULT_BIGCITY_VISUAL_FBX,
    output_blend: str | Path = DEFAULT_BIGCITY_STRICT_REVIEW_BLEND,
    output_xml: str | Path = DEFAULT_BIGCITY_STRICT_REVIEW_XML,
    confirmed_csv: str | Path = DEFAULT_BIGCITY_CONFIRMED_MATERIALS_CSV,
    uncertain_csv: str | Path = DEFAULT_BIGCITY_UNCERTAIN_MATERIALS_CSV,
    mapping_output_csv: str | Path | None = DEFAULT_BIGCITY_STRICT_REVIEW_MAPPING_CSV,
    mapping_output_json: str | Path | None = DEFAULT_BIGCITY_STRICT_REVIEW_MAPPING_JSON,
    uncertain_material: str = STRICT_UNCERTAIN_MATERIAL,
    uncertain_color_mode: str = "palette",
    binary_ply: bool = True,
    name_contains: str = "",
    split_by_material: bool = True,
    exporter: str = "mitsuba",
    apply_changes: bool = False,
    blender_path: str | None = None,
) -> str:
    """Export a review scene: confirmed materials use ITU ids, unknowns are highlighted."""
    blend_source = _repo_path(source_blend, must_exist=True) if source_blend else None
    fbx = _repo_path(fbx_path, must_exist=True)
    blend = _repo_path(output_blend)
    xml = _repo_path(output_xml)
    material_map, mapping_rows = _load_strict_review_mapping(
        confirmed_csv=confirmed_csv,
        uncertain_csv=uncertain_csv,
        uncertain_material=uncertain_material,
    )
    written_mapping = _write_strict_review_mapping(mapping_rows, mapping_output_csv, mapping_output_json)
    decision_counts: dict[str, int] = {}
    target_counts: dict[str, int] = {}
    for row in mapping_rows:
        decision_counts[row["decision"]] = decision_counts.get(row["decision"], 0) + 1
        target_counts[row["target_material"]] = target_counts.get(row["target_material"], 0) + 1

    if blend_source:
        payload = {
            "mode": "export_sionna_from_blend",
            "blend_path": str(blend_source),
            "output_blend": str(blend),
            "output_xml": str(xml),
            "name_contains": name_contains,
            "material_overrides": material_map,
            "split_by_material": split_by_material,
            "uncertain_color_mode": uncertain_color_mode,
            "binary_ply": binary_ply,
            "exporter": exporter,
            "apply": apply_changes,
        }
    else:
        payload = {
            "mode": "export_sionna_from_fbx",
            "fbx_path": str(fbx),
            "output_blend": str(blend),
            "output_xml": str(xml),
            "name_contains": name_contains,
            "material_overrides": material_map,
            "split_by_material": split_by_material,
            "preserve_source_material_names": True,
            "simple_xml_export": True,
            "uncertain_color_mode": uncertain_color_mode,
            "binary_ply": binary_ply,
            "exporter": exporter,
            "apply": apply_changes,
        }
    results = _run_blender_payload(payload, blender_path=blender_path)
    result = results[0]
    result["source"] = str((blend_source or fbx).relative_to(REPO_ROOT))
    result["output_blend"] = str(blend.relative_to(REPO_ROOT))
    result["output_xml"] = str(xml.relative_to(REPO_ROOT))
    result["strict_review_mapping"] = written_mapping
    result["strict_review_decision_counts"] = decision_counts
    result["strict_review_target_counts"] = target_counts
    result["strict_review_uncertain_material"] = uncertain_material
    result["strict_review_uncertain_color_mode"] = uncertain_color_mode
    return json.dumps(result, ensure_ascii=False, indent=2)


def restore_bigcity_assets(
    apply_changes: bool = False,
    archive_path: str | Path = DEFAULT_SAMPLE_MAPS_ARCHIVE,
) -> str:
    """Restore BigCity visual/Sionna assets from the downloaded sample-maps zip."""
    archive = _repo_path(archive_path, must_exist=True)
    if archive.suffix.lower() != ".zip":
        raise AssetToolError(f"Expected a zip archive, got: {archive}")

    restored: list[dict[str, object]] = []
    with zipfile.ZipFile(archive) as zip_file:
        available = set(zip_file.namelist())
        missing = [member for member in BIGCITY_RESTORE_MEMBERS if member not in available]
        if missing:
            raise AssetToolError(f"Archive is missing restore members: {missing}")

        for member in BIGCITY_RESTORE_MEMBERS:
            target = _repo_path(member)
            info = zip_file.getinfo(member)
            if apply_changes:
                target.parent.mkdir(parents=True, exist_ok=True)
                target.write_bytes(zip_file.read(member))
            restored.append(
                {
                    "path": str(target.relative_to(REPO_ROOT)),
                    "archive_size": info.file_size,
                    "written": apply_changes,
                }
            )

    return json.dumps(
        {
            "mode": "applied" if apply_changes else "dry_run",
            "archive": str(archive.relative_to(REPO_ROOT)),
            "restored_files": restored,
        },
        ensure_ascii=False,
        indent=2,
    )


def _main() -> int:
    parser = argparse.ArgumentParser(
        description="Edit Blender material sources and export SimART visual FBX plus Sionna XML."
    )
    parser.add_argument("--blender", default=None, help="Path to Blender executable. Defaults to BLENDER or PATH.")
    subparsers = parser.add_subparsers(dest="command", required=True)

    replace_parser = subparsers.add_parser("replace-material", help="Replace material in visual and Sionna assets.")
    replace_parser.add_argument("from_material", help="Material to replace, e.g. itu_concrete.")
    replace_parser.add_argument("to_material", help="Target material, e.g. itu_glass.")
    replace_parser.add_argument(
        "--visual-source",
        choices=["fbx", "blend"],
        default="fbx",
        help="Use the visual FBX directly, or use the visual Blender source file.",
    )
    replace_parser.add_argument("--visual-blend", default=str(DEFAULT_BIGCITY_VISUAL_BLEND))
    replace_parser.add_argument("--visual-fbx-input", default=str(DEFAULT_BIGCITY_VISUAL_FBX))
    replace_parser.add_argument("--visual-fbx", default=str(DEFAULT_BIGCITY_VISUAL_FBX))
    replace_parser.add_argument("--sionna-blend", default=str(DEFAULT_BIGCITY_SIONNA_BLEND))
    replace_parser.add_argument("--sionna-xml", default=str(DEFAULT_BIGCITY_SIONNA_XML))
    replace_parser.add_argument(
        "--visual-from",
        default=None,
        help="Override source material name for the visual FBX/Blend, e.g. Concrete.",
    )
    replace_parser.add_argument(
        "--visual-to",
        default=None,
        help="Override target material name for the visual FBX/Blend.",
    )
    replace_parser.add_argument(
        "--sionna-from",
        default=None,
        help="Override source material name for the Sionna Blender scene, e.g. itu_concrete.",
    )
    replace_parser.add_argument(
        "--sionna-to",
        default=None,
        help="Override target material name for the Sionna Blender scene.",
    )
    replace_parser.add_argument("--name-contains", default="", help="Only edit mesh objects whose name contains this text.")
    replace_parser.add_argument("--apply", action="store_true", help="Save .blend and export FBX/XML.")

    restore_parser = subparsers.add_parser(
        "restore-bigcity",
        help="Restore BigCity visual FBX, Sionna Blender source, and Sionna XML from SimART_sample_maps.zip.",
    )
    restore_parser.add_argument("--archive", default=str(DEFAULT_SAMPLE_MAPS_ARCHIVE))
    restore_parser.add_argument("--apply", action="store_true", help="Actually overwrite BigCity files.")

    inspect_parser = subparsers.add_parser(
        "inspect-fbx-materials",
        help="Inspect FBX material slots and suggest Sionna radio material mappings.",
    )
    inspect_parser.add_argument("--fbx", default=str(DEFAULT_BIGCITY_VISUAL_FBX))
    inspect_parser.add_argument("--output-csv", default=str(DEFAULT_AGENT_OUTPUT_DIR / "bigcity_fbx_material_inventory.csv"))
    inspect_parser.add_argument("--output-json", default=str(DEFAULT_AGENT_OUTPUT_DIR / "bigcity_fbx_material_inventory.json"))
    inspect_parser.add_argument(
        "--material-overrides",
        default=None,
        help="Optional JSON object mapping FBX material names to Sionna material names.",
    )
    inspect_parser.add_argument("--name-contains", default="", help="Only inspect mesh objects whose name contains this text.")

    import_blend_parser = subparsers.add_parser(
        "import-fbx-to-blend",
        help="Import the visual FBX into a Blender file without changing material slots.",
    )
    import_blend_parser.add_argument("--fbx", default=str(DEFAULT_BIGCITY_VISUAL_FBX))
    import_blend_parser.add_argument("--output-blend", default=str(DEFAULT_BIGCITY_FBX_MATERIAL_BLEND))
    import_blend_parser.add_argument("--name-contains", default="", help="Only summarize mesh objects whose name contains this text.")
    import_blend_parser.add_argument("--apply", action="store_true", help="Actually save the imported FBX as a .blend file.")

    export_parser = subparsers.add_parser(
        "export-sionna-from-fbx",
        help="Create a material-preserving Sionna blend/XML scene directly from the visual FBX.",
    )
    export_parser.add_argument("--fbx", default=str(DEFAULT_BIGCITY_VISUAL_FBX))
    export_parser.add_argument("--output-blend", default=str(DEFAULT_BIGCITY_AUTO_SIONNA_BLEND))
    export_parser.add_argument("--output-xml", default=str(DEFAULT_BIGCITY_AUTO_SIONNA_XML))
    export_parser.add_argument(
        "--material-overrides",
        default=None,
        help="Optional JSON object mapping FBX material names to Sionna material names.",
    )
    export_parser.add_argument("--name-contains", default="", help="Only export mesh objects whose name contains this text.")
    export_parser.add_argument(
        "--no-split-by-material",
        action="store_true",
        help="Keep multi-material meshes intact instead of separating objects by material.",
    )
    export_parser.add_argument("--apply", action="store_true", help="Actually save the new blend and export XML.")

    strict_review_parser = subparsers.add_parser(
        "export-strict-review-scene",
        help="Export a review XML/Blend: confirmed materials use ITU ids; uncertain materials are highlighted.",
    )
    strict_review_parser.add_argument("--source-blend", default=str(DEFAULT_BIGCITY_FBX_MATERIAL_BLEND))
    strict_review_parser.add_argument("--fbx", default=str(DEFAULT_BIGCITY_VISUAL_FBX))
    strict_review_parser.add_argument("--output-blend", default=str(DEFAULT_BIGCITY_STRICT_REVIEW_BLEND))
    strict_review_parser.add_argument("--output-xml", default=str(DEFAULT_BIGCITY_STRICT_REVIEW_XML))
    strict_review_parser.add_argument("--confirmed-csv", default=str(DEFAULT_BIGCITY_CONFIRMED_MATERIALS_CSV))
    strict_review_parser.add_argument("--uncertain-csv", default=str(DEFAULT_BIGCITY_UNCERTAIN_MATERIALS_CSV))
    strict_review_parser.add_argument(
        "--mapping-output-csv",
        default=str(DEFAULT_BIGCITY_STRICT_REVIEW_MAPPING_CSV),
        help="Write the exact visual-material to review-material mapping used for this export.",
    )
    strict_review_parser.add_argument(
        "--mapping-output-json",
        default=str(DEFAULT_BIGCITY_STRICT_REVIEW_MAPPING_JSON),
        help="Write the exact visual-material to review-material mapping used for this export.",
    )
    strict_review_parser.add_argument(
        "--uncertain-material",
        default=STRICT_UNCERTAIN_MATERIAL,
        help="Material name used to highlight unresolved materials.",
    )
    strict_review_parser.add_argument(
        "--uncertain-color-mode",
        choices=["source", "magenta", "palette"],
        default="palette",
        help=(
            "Color unresolved materials with their original visual color, one fixed review color per source material, "
            "or a single magenta highlight."
        ),
    )
    strict_review_parser.add_argument(
        "--ascii-ply",
        action="store_true",
        help="Write ASCII PLY files instead of binary PLY. ASCII is easier to inspect but much slower to load.",
    )
    strict_review_parser.add_argument("--name-contains", default="", help="Only export mesh objects whose name contains this text.")
    strict_review_parser.add_argument(
        "--no-split-by-material",
        action="store_true",
        help="Keep multi-material meshes intact instead of separating objects by material.",
    )
    strict_review_parser.add_argument(
        "--exporter",
        choices=["mitsuba", "simple"],
        default="mitsuba",
        help="Use the Mitsuba Blender exporter after rebuilding clean meshes, or the built-in simple PLY exporter.",
    )
    strict_review_parser.add_argument("--apply", action="store_true", help="Actually save the review blend and export XML.")

    args = parser.parse_args()
    try:
        if args.command == "replace-material":
            print(
                replace_material_pipeline(
                    args.from_material,
                    args.to_material,
                    visual_source=args.visual_source,
                    visual_blend=args.visual_blend,
                    visual_fbx_input=args.visual_fbx_input,
                    visual_fbx=args.visual_fbx,
                    sionna_blend=args.sionna_blend,
                    sionna_xml=args.sionna_xml,
                    visual_from_material=args.visual_from,
                    visual_to_material=args.visual_to,
                    sionna_from_material=args.sionna_from,
                    sionna_to_material=args.sionna_to,
                    name_contains=args.name_contains,
                    apply_changes=args.apply,
                    blender_path=args.blender,
                )
            )
        elif args.command == "restore-bigcity":
            print(restore_bigcity_assets(apply_changes=args.apply, archive_path=args.archive))
        elif args.command == "inspect-fbx-materials":
            print(
                inspect_fbx_materials(
                    fbx_path=args.fbx,
                    output_csv=args.output_csv,
                    output_json=args.output_json,
                    material_overrides=args.material_overrides,
                    name_contains=args.name_contains,
                    blender_path=args.blender,
                )
            )
        elif args.command == "import-fbx-to-blend":
            print(
                import_fbx_to_blend(
                    fbx_path=args.fbx,
                    output_blend=args.output_blend,
                    name_contains=args.name_contains,
                    apply_changes=args.apply,
                    blender_path=args.blender,
                )
            )
        elif args.command == "export-sionna-from-fbx":
            print(
                export_sionna_from_fbx(
                    fbx_path=args.fbx,
                    output_blend=args.output_blend,
                    output_xml=args.output_xml,
                    material_overrides=args.material_overrides,
                    name_contains=args.name_contains,
                    split_by_material=not args.no_split_by_material,
                    apply_changes=args.apply,
                    blender_path=args.blender,
                )
            )
        elif args.command == "export-strict-review-scene":
            print(
                export_strict_review_scene(
                    source_blend=args.source_blend,
                    fbx_path=args.fbx,
                    output_blend=args.output_blend,
                    output_xml=args.output_xml,
                    confirmed_csv=args.confirmed_csv,
                    uncertain_csv=args.uncertain_csv,
                    mapping_output_csv=args.mapping_output_csv,
                    mapping_output_json=args.mapping_output_json,
                    uncertain_material=args.uncertain_material,
                    uncertain_color_mode=args.uncertain_color_mode,
                    binary_ply=not args.ascii_ply,
                    name_contains=args.name_contains,
                    split_by_material=not args.no_split_by_material,
                    exporter=args.exporter,
                    apply_changes=args.apply,
                    blender_path=args.blender,
                )
            )
    except AssetToolError as exc:
        print(f"error: {exc}", file=sys.stderr)
        return 1
    return 0


if __name__ == "__main__":
    raise SystemExit(_main())
