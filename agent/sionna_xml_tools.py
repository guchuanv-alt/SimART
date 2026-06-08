"""Inspect/edit Sionna RT XML materials safely.
安全地查看和编辑 Sionna RT XML 材质。

This module is deliberately usable without LangChain. When learning agents,
keep real-world actions as normal, testable Python functions first.
这个模块故意不依赖 LangChain。学习 agent 时，先把真实操作写成普通、
可测试的 Python 函数。
"""

from __future__ import annotations

import argparse
import json
import re
from dataclasses import dataclass
from pathlib import Path
from typing import Iterable


REPO_ROOT = Path(__file__).resolve().parents[1]
DEFAULT_SCENE = Path("SimART_sample_maps/Paris_Small/Paris_Small_sionna/Paris_Small.xml")
SCENE_ALIASES = {
    "parissmall": DEFAULT_SCENE,
    "paris": Path("SimART_sample_maps/Paris/Paris_sionna/Paris.xml"),
    "bigcitysample": Path("SimART_sample_maps/BigCitySample/BigCitySample_simptest_sionna/BigCitySample_simptest.xml"),
    "bigcitysamplewet": Path(
        "SimART_sample_maps/BigCitySample/BigCitySample_simptest_wet_sionna/BigCitySample_simptest_wet.xml"
    ),
    "tokyoginza": Path("SimART_sample_maps/Tokyo/Tokyo_Ginza_sionna/Tokyo_Ginza.xml"),
}


class XmlToolError(ValueError):
    """Raised when an XML edit is unsafe or impossible.
    当 XML 修改不安全或无法完成时抛出。
    """


@dataclass(frozen=True)
class MaterialInfo:
    """Structured description of one material in a Sionna scene.
    Sionna 场景中单个材质的结构化描述。
    """

    material_id: str
    kind: str
    color_name: str | None
    color_value: str | None
    ref_count: int


def resolve_scene_path(scene_path: str | Path = DEFAULT_SCENE) -> Path:
    """Resolve a scene path and keep it inside this repository.
    解析场景路径，并确保它仍然位于当前项目仓库内。
    """
    if isinstance(scene_path, str):
        alias_key = re.sub(r"[^a-z0-9]+", "", scene_path.lower())
        scene_path = SCENE_ALIASES.get(alias_key, scene_path)

    path = Path(scene_path).expanduser()
    if not path.is_absolute():
        path = REPO_ROOT / path
    path = path.resolve()

    try:
        path.relative_to(REPO_ROOT)
    except ValueError as exc:
        raise XmlToolError(f"Refusing to access a file outside the repo: {path}") from exc
    if path.suffix.lower() != ".xml":
        raise XmlToolError(f"Expected a Sionna scene XML file, got: {path}")
    if not path.exists():
        raise XmlToolError(f"Scene XML does not exist: {path}")
    return path


def _read_text(scene_path: str | Path) -> tuple[Path, str]:
    """Resolve a scene path and read its XML text as UTF-8.
    解析场景路径，并以 UTF-8 读取 XML 文本。

    Returns:
        The resolved absolute path and the file content.
        解析后的绝对路径和文件内容。
    """
    path = resolve_scene_path(scene_path)
    return path, path.read_text(encoding="utf-8")


def _write_text(path: Path, text: str, apply_changes: bool) -> None:
    """Write XML text only when changes are explicitly enabled.
    只有在明确允许写入时，才把 XML 文本写回文件。

    Args:
        path: Resolved XML file path.
            已解析的 XML 文件路径。
        text: New XML content to write.
            要写入的新 XML 内容。
        apply_changes: If false, keep this as a dry run.
            如果为 false，则保持预演模式，不写文件。
    """
    if apply_changes:
        path.write_text(text, encoding="utf-8")


def _material_blocks(text: str) -> Iterable[tuple[str, str, int, int]]:
    """Yield material XML blocks and their positions in the full text.
    逐个产出材质 XML 块，以及它们在完整文本中的位置。

    Yields:
        Tuples of material id, block text, start index, and end index.
        依次返回材质 ID、块文本、起始位置和结束位置。
    """
    pattern = re.compile(
        r'(?P<block><bsdf\b[^>]*\bid="(?P<id>mat-[^"]+)"[^>]*>.*?</bsdf>\s*)',
        re.DOTALL,
    )
    for match in pattern.finditer(text):
        yield match.group("id"), match.group("block"), match.start("block"), match.end("block")


def _extract_color(block: str) -> tuple[str | None, str | None]:
    """Extract the display color field from one material block.
    从单个材质块中提取显示颜色字段。

    Returns:
        The color field name and RGB value, or ``(None, None)`` if absent.
        颜色字段名和 RGB 值；如果不存在，则返回 ``(None, None)``。
    """
    color_match = re.search(
        r'<rgb\b[^>]*\bvalue="(?P<value>[^"]+)"[^>]*\bname="(?P<name>reflectance|base_color)"',
        block,
    )
    if not color_match:
        return None, None
    return color_match.group("name"), color_match.group("value")


def _normalize_material_id(material_id: str) -> str:
    """Return a non-empty material id with the ``mat-`` prefix.
    返回带有 ``mat-`` 前缀的非空材质 ID。
    """
    material_id = material_id.strip()
    if not material_id:
        raise XmlToolError("Material id cannot be empty.")
    return material_id if material_id.startswith("mat-") else f"mat-{material_id}"


def _normalize_rgb(rgb: str) -> str:
    """Validate and format an RGB string as three floats in ``[0, 1]``.
    校验 RGB 字符串，并格式化为三个 ``[0, 1]`` 范围内的浮点数。
    """
    parts = rgb.replace(",", " ").split()
    if len(parts) != 3:
        raise XmlToolError("RGB must contain exactly three numbers, e.g. '1 0 0'.")
    values = []
    for part in parts:
        value = float(part)
        if value < 0.0 or value > 1.0:
            raise XmlToolError("RGB values must be in [0, 1].")
        values.append(f"{value:.6f}")
    return " ".join(values)


def list_materials(scene_path: str | Path = DEFAULT_SCENE) -> list[dict[str, object]]:
    """Return material definitions and their shape reference counts.
    返回材质定义，以及每种材质被多少个 shape 引用。
    """
    _, text = _read_text(scene_path)
    materials: list[dict[str, object]] = []
    for material_id, block, _, _ in _material_blocks(text):
        color_name, color_value = _extract_color(block)
        ref_count = len(re.findall(rf'<ref\b[^>]*\bid="{re.escape(material_id)}"[^>]*/>', text))
        materials.append(
            {
                "material_id": material_id,
                "kind": "principled" if "type=\"principled\"" in block else "diffuse",
                "color_name": color_name,
                "color_value": color_value,
                "ref_count": ref_count,
            }
        )
    return materials


def summarize_scene(scene_path: str | Path = DEFAULT_SCENE) -> str:
    """Return a compact JSON summary of material usage in a scene XML.
    返回场景 XML 中材质使用情况的简洁 JSON 摘要。
    """
    path, text = _read_text(scene_path)
    shape_count = len(re.findall(r"<shape\b", text))
    summary = {
        "scene_path": str(path.relative_to(REPO_ROOT)),
        "shape_count": shape_count,
        "materials": list_materials(path),
    }
    return json.dumps(summary, ensure_ascii=False, indent=2)


def set_material_rgb(
    scene_path: str | Path,
    material_id: str,
    rgb: str,
    apply_changes: bool = False,
) -> str:
    """Change a material's display RGB color.
    修改某个材质的显示 RGB 颜色。

    Args:
        scene_path: Sionna scene XML path.
            Sionna 场景 XML 路径。
        material_id: Material id, e.g. mat-itu_concrete or itu_concrete.
            材质 ID，例如 mat-itu_concrete 或 itu_concrete。
        rgb: Three floats in [0, 1], e.g. "1 0 0".
            三个 0 到 1 之间的数字，例如 "1 0 0" 表示红色。
        apply_changes: If false, return a dry-run report without writing.
            如果为 false，只返回预演报告，不真正写入文件。
    """
    path, text = _read_text(scene_path)
    material_id = _normalize_material_id(material_id)
    rgb = _normalize_rgb(rgb)

    for found_id, block, start, end in _material_blocks(text):
        if found_id != material_id:
            continue
        new_block, replacements = re.subn(
            r'(<rgb\b[^>]*\bvalue=")([^"]+)("[^>]*\bname="(?:reflectance|base_color)"[^>]*/>)',
            rf"\g<1>{rgb}\g<3>",
            block,
            count=1,
        )
        if replacements != 1:
            raise XmlToolError(f"Could not find an RGB color entry inside {material_id}.")
        new_text = text[:start] + new_block + text[end:]
        _write_text(path, new_text, apply_changes)
        mode = "applied" if apply_changes else "dry_run"
        return f"{mode}: set {material_id} color to {rgb} in {path.relative_to(REPO_ROOT)}"

    raise XmlToolError(f"Material not found: {material_id}")


def replace_material_refs(
    scene_path: str | Path,
    from_material: str,
    to_material: str,
    limit: int = 0,
    name_contains: str = "",
    apply_changes: bool = False,
) -> str:
    """Replace shape material references from one material to another.
    把 shape 上引用的一种材质替换成另一种材质。

    Args:
        scene_path: Sionna scene XML path.
            Sionna 场景 XML 路径。
        from_material: Existing material id, e.g. itu_concrete.
            当前已有材质 ID，例如 itu_concrete。
        to_material: Target material id, e.g. itu_glass.
            目标材质 ID，例如 itu_glass。
        limit: Maximum number of shapes to modify. 0 means all matching shapes.
            最多修改多少个 shape；0 表示修改所有匹配项。
        name_contains: Optional substring that must appear inside the shape block.
            可选过滤条件：shape 块里必须包含这个字符串。
        apply_changes: If false, return a dry-run report without writing.
            如果为 false，只返回预演报告，不真正写入文件。
    """
    path, text = _read_text(scene_path)
    from_material = _normalize_material_id(from_material)
    to_material = _normalize_material_id(to_material)
    if limit < 0:
        raise XmlToolError("limit must be >= 0.")
    known = {item["material_id"] for item in list_materials(path)}
    if from_material not in known:
        raise XmlToolError(f"Source material does not exist: {from_material}")
    if to_material not in known:
        raise XmlToolError(f"Target material does not exist: {to_material}")

    shape_pattern = re.compile(r"(?P<block><shape\b.*?</shape>\s*)", re.DOTALL)
    changed = 0
    output_parts: list[str] = []
    cursor = 0
    changed_names: list[str] = []

    for match in shape_pattern.finditer(text):
        output_parts.append(text[cursor:match.start()])
        block = match.group("block")
        should_change = (
            f'id="{from_material}"' in block
            and (not name_contains or name_contains in block)
            and (limit == 0 or changed < limit)
        )
        if should_change:
            new_block = block.replace(f'id="{from_material}"', f'id="{to_material}"', 1)
            changed += 1
            name_match = re.search(r'<shape\b[^>]*\bname="([^"]+)"', block)
            changed_names.append(name_match.group(1) if name_match else f"shape#{changed}")
            output_parts.append(new_block)
        else:
            output_parts.append(block)
        cursor = match.end()
    output_parts.append(text[cursor:])

    if changed == 0:
        return "dry_run: no matching shape material references found"

    new_text = "".join(output_parts)
    _write_text(path, new_text, apply_changes)
    mode = "applied" if apply_changes else "dry_run"
    preview = ", ".join(changed_names[:8])
    if len(changed_names) > 8:
        preview += f", ... (+{len(changed_names) - 8} more)"
    return (
        f"{mode}: replaced {changed} refs from {from_material} to {to_material} "
        f"in {path.relative_to(REPO_ROOT)}; shapes: {preview}"
    )


def _main() -> int:
    """Run the command-line interface for inspecting or editing materials.
    运行用于查看或编辑材质的命令行接口。
    """
    parser = argparse.ArgumentParser(description="Inspect or edit Sionna XML materials.")
    parser.add_argument("--scene", default=str(DEFAULT_SCENE), help="Scene XML path relative to repo root.")
    subparsers = parser.add_subparsers(dest="command", required=True)

    subparsers.add_parser("summary", help="Print scene material summary.")

    color_parser = subparsers.add_parser("set-color", help="Set a material RGB color.")
    color_parser.add_argument("material_id")
    color_parser.add_argument("rgb")
    color_parser.add_argument("--apply", action="store_true")

    replace_parser = subparsers.add_parser("replace-refs", help="Replace material refs on matching shapes.")
    replace_parser.add_argument("from_material")
    replace_parser.add_argument("to_material")
    replace_parser.add_argument("--limit", type=int, default=0)
    replace_parser.add_argument("--name-contains", default="")
    replace_parser.add_argument("--apply", action="store_true")

    args = parser.parse_args()
    if args.command == "summary":
        print(summarize_scene(args.scene))
    elif args.command == "set-color":
        print(set_material_rgb(args.scene, args.material_id, args.rgb, apply_changes=args.apply))
    elif args.command == "replace-refs":
        print(
            replace_material_refs(
                args.scene,
                args.from_material,
                args.to_material,
                limit=args.limit,
                name_contains=args.name_contains,
                apply_changes=args.apply,
            )
        )
    return 0


if __name__ == "__main__":
    raise SystemExit(_main())
