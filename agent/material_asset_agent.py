"""Natural-language material agent for SimART asset editing.

The agent never edits FBX/XML directly. It only calls the tested asset pipeline
in ``sionna_asset_tools.py``.
"""

from __future__ import annotations

import argparse
import os
import re
import sys
from pathlib import Path

from langchain.agents import create_agent
from langchain.tools import tool
from langchain_openai import ChatOpenAI

from agent.sionna_asset_tools import AssetToolError, replace_material_pipeline, restore_bigcity_assets


REPO_ROOT = Path(__file__).resolve().parents[1]
DEFAULT_MODEL = "deepseek-chat"
DEFAULT_BASE_URL = "https://api.deepseek.com"

MATERIAL_HINTS = {
    "混凝土": "itu_concrete",
    "水泥": "itu_concrete",
    "玻璃": "itu_glass",
    "砖": "itu_brick",
    "金属": "itu_metal",
    "木头": "itu_wood",
    "木材": "itu_wood",
    "干地面": "itu_very_dry_ground",
    "潮湿地面": "itu_wet_ground",
    "湿地面": "itu_wet_ground",
}

SYSTEM_PROMPT = """You are the SimART material asset agent.

Your job is to help edit SimART BigCitySample materials through tools.

Important rules:
- Never claim that you edited files unless a tool result says mode=applied.
- Use preview_replace_material for ordinary requests.
- Use apply_replace_material only when the command-line session allows apply.
- Material names must be Sionna material ids such as itu_concrete, itu_glass,
  itu_brick, itu_metal, itu_wood, itu_very_dry_ground, itu_wet_ground.
- Chinese material hints: 混凝土/水泥 -> itu_concrete, 玻璃 -> itu_glass,
  砖 -> itu_brick, 金属 -> itu_metal, 木材 -> itu_wood.
- Explain that the tool updates the visual FBX, the Sionna Blender source, and
  the exported Mitsuba/Sionna XML.
- If the user asks to restore/reset/recover BigCity to its initial downloaded
  state, use the restore tools instead of material replacement tools.
- Keep answers concise and practical.
"""

APPLY_MODE_PROMPT = """This session was started with --apply.

If the user asks to directly execute, directly modify, apply changes, or says
no confirmation is needed, you must call apply_replace_material for a material
replacement request.

If the user asks to restore/reset/recover BigCity to the initial downloaded
state, you must call apply_restore_initial_bigcity.

Do not call preview_replace_material first in that case.
"""

DRY_RUN_MODE_PROMPT = """This session was started without --apply.

You must only call preview tools. Use preview_replace_material for material
replacement requests and preview_restore_initial_bigcity for restore requests.
If the user asks to directly execute, explain that they need to rerun the
command with --apply.
"""


def _load_env_file(path: Path) -> None:
    """Load KEY=VALUE pairs from a local env file without extra dependencies."""
    if not path.exists():
        return
    for raw_line in path.read_text(encoding="utf-8").splitlines():
        line = raw_line.strip()
        if not line or line.startswith("#") or "=" not in line:
            continue
        key, value = line.split("=", 1)
        key = key.strip()
        value = value.strip().strip("'\"")
        if key and key not in os.environ:
            os.environ[key] = value


def load_local_env() -> None:
    """Load repo-local secrets from ignored files."""
    _load_env_file(REPO_ROOT / ".env")
    _load_env_file(REPO_ROOT / "agent/.env")


def _normalize_material(text: str) -> str:
    """Convert simple Chinese/English material words to tool material ids."""
    value = text.strip()
    if not value:
        raise ValueError("material name cannot be empty")
    lowered = value.lower()
    compact = re.sub(r"[^a-z0-9_]+", "", lowered)
    if compact.startswith("itu_"):
        return compact
    if compact.startswith("mat_itu_"):
        return compact[4:]
    if compact.startswith("mat-itu_"):
        return compact[4:]
    for word, material_id in MATERIAL_HINTS.items():
        if word in value:
            return material_id
    aliases = {
        "concrete": "itu_concrete",
        "cement": "itu_concrete",
        "stucco": "itu_concrete",
        "glass": "itu_glass",
        "window": "itu_glass",
        "brick": "itu_brick",
        "metal": "itu_metal",
        "wood": "itu_wood",
        "wetground": "itu_wet_ground",
        "verydryground": "itu_very_dry_ground",
        "mediumdryground": "itu_medium_dry_ground",
    }
    return aliases.get(compact, value)


def build_tools(allow_apply: bool):
    """Create LangChain tools with an apply safety gate."""

    @tool
    def preview_replace_material(from_material: str, to_material: str, name_contains: str = "") -> str:
        """Preview replacing one material in BigCity visual FBX and Sionna XML without writing files."""
        try:
            return replace_material_pipeline(
                from_material=_normalize_material(from_material),
                to_material=_normalize_material(to_material),
                name_contains=name_contains,
                apply_changes=False,
            )
        except (AssetToolError, ValueError) as exc:
            return f"preview failed: {exc}"

    @tool
    def apply_replace_material(from_material: str, to_material: str, name_contains: str = "") -> str:
        """Apply material replacement to BigCity visual FBX, Sionna Blender source, and Sionna XML."""
        if not allow_apply:
            return (
                "apply blocked: this agent session was started without --apply. "
                "Run with --apply if you really want to write FBX/Blend/XML files."
            )
        try:
            return replace_material_pipeline(
                from_material=_normalize_material(from_material),
                to_material=_normalize_material(to_material),
                name_contains=name_contains,
                apply_changes=True,
            )
        except (AssetToolError, ValueError) as exc:
            return f"apply failed: {exc}"

    @tool
    def preview_restore_initial_bigcity() -> str:
        """Preview restoring BigCity assets from the original downloaded SimART_sample_maps.zip."""
        try:
            return restore_bigcity_assets(apply_changes=False)
        except AssetToolError as exc:
            return f"restore preview failed: {exc}"

    @tool
    def apply_restore_initial_bigcity() -> str:
        """Restore BigCity visual FBX, Sionna Blender source, and Sionna XML from the original zip."""
        if not allow_apply:
            return (
                "restore blocked: this agent session was started without --apply. "
                "Run with --apply if you really want to overwrite BigCity files."
            )
        try:
            return restore_bigcity_assets(apply_changes=True)
        except AssetToolError as exc:
            return f"restore failed: {exc}"

    return [
        preview_replace_material,
        apply_replace_material,
        preview_restore_initial_bigcity,
        apply_restore_initial_bigcity,
    ]


def _content_from_result(result: dict) -> str:
    """Extract the final assistant text from a LangChain agent result."""
    messages = result.get("messages", [])
    if not messages:
        return str(result)
    last = messages[-1]
    content = getattr(last, "content", last.get("content", "") if isinstance(last, dict) else "")
    if isinstance(content, list):
        return "\n".join(str(item) for item in content)
    return str(content)


def run_agent(request: str, apply_changes: bool = False, model_name: str = DEFAULT_MODEL) -> str:
    """Run the DeepSeek/LangChain material agent for one user request."""
    load_local_env()
    api_key = os.environ.get("DEEPSEEK_API_KEY")
    if not api_key:
        raise RuntimeError(
            "DEEPSEEK_API_KEY is not set. Put it in agent/.env or export it in the terminal."
        )

    llm = ChatOpenAI(
        model=model_name,
        api_key=api_key,
        base_url=os.environ.get("DEEPSEEK_BASE_URL", DEFAULT_BASE_URL),
        temperature=0,
    )
    agent = create_agent(
        model=llm,
        tools=build_tools(allow_apply=apply_changes),
        system_prompt=SYSTEM_PROMPT + "\n\n" + (APPLY_MODE_PROMPT if apply_changes else DRY_RUN_MODE_PROMPT),
    )
    result = agent.invoke({"messages": [{"role": "user", "content": request}]})
    return _content_from_result(result)


def _main() -> int:
    parser = argparse.ArgumentParser(description="Chat with the SimART material asset agent.")
    parser.add_argument("request", help="Natural-language material edit request.")
    parser.add_argument("--apply", action="store_true", help="Allow the agent to write FBX/Blend/XML files.")
    parser.add_argument("--model", default=DEFAULT_MODEL, help="DeepSeek model name.")
    args = parser.parse_args()

    try:
        print(run_agent(args.request, apply_changes=args.apply, model_name=args.model))
    except Exception as exc:
        print(f"error: {exc}", file=sys.stderr)
        return 1
    return 0


if __name__ == "__main__":
    raise SystemExit(_main())
