"""A minimal LangChain agent for SimART Sionna RT material XML.
一个用于 SimART Sionna RT 材质 XML 的最小 LangChain agent。

Run examples:
运行示例：

    python -m agent.material_agent "查看 Paris_Small 场景有哪些材质"
    python -m agent.material_agent --apply "把 concrete 改成红色"

The important learning point:
重要学习点：
    The LLM does not edit files directly. It calls small, safe Python tools.
    大模型不直接改文件，而是调用小而安全的 Python 工具。
"""

from __future__ import annotations

import argparse
import os
from typing import Annotated

from langchain.agents import create_agent
from langchain.tools import tool

from agent.sionna_xml_tools import (
    DEFAULT_SCENE,
    XmlToolError,
    replace_material_refs,
    set_material_rgb,
    summarize_scene,
)


DEEPSEEK_BASE_URL = "https://api.deepseek.com"
DEFAULT_MODEL = os.environ.get(
    "SIMART_AGENT_MODEL",
    "openai:deepseek-v4-pro" if os.environ.get("DEEPSEEK_API_KEY") else "openai:gpt-5.4",
)


def configure_model_environment(model: str) -> None:
    """Configure provider-specific environment variables before LangChain starts.
    在 LangChain 启动前配置不同模型提供商需要的环境变量。

    DeepSeek exposes an OpenAI-compatible API. LangChain's OpenAI provider reads
    ``OPENAI_API_KEY`` and ``OPENAI_BASE_URL``, so we map ``DEEPSEEK_API_KEY`` to
    those variables when the selected model is a DeepSeek model.
    DeepSeek 提供 OpenAI 兼容接口。LangChain 的 OpenAI provider 会读取
    ``OPENAI_API_KEY`` 和 ``OPENAI_BASE_URL``，所以当选择 DeepSeek 模型时，
    这里把 ``DEEPSEEK_API_KEY`` 映射过去。
    """
    model_name = model.split(":", 1)[-1]
    uses_deepseek = model_name.startswith("deepseek-") or bool(os.environ.get("DEEPSEEK_API_KEY"))
    if not uses_deepseek:
        return

    deepseek_key = os.environ.get("DEEPSEEK_API_KEY")
    if deepseek_key and not os.environ.get("OPENAI_API_KEY"):
        os.environ["OPENAI_API_KEY"] = deepseek_key
    os.environ.setdefault("OPENAI_BASE_URL", os.environ.get("DEEPSEEK_BASE_URL", DEEPSEEK_BASE_URL))


def build_tools(apply_changes: bool):
    """Create LangChain tools.
    创建 LangChain 工具。

    The closure captures `apply_changes`, so the CLI controls whether tools are
    allowed to write files. This is a simple beginner-friendly safety gate.
    这里用闭包捕获 `apply_changes`，所以命令行可以控制工具是否允许写文件。
    这是一个适合新手的简单安全开关。
    """

    @tool
    def inspect_sionna_scene(
        scene_path: Annotated[str, "Scene XML path relative to the SimART repo root."] = str(DEFAULT_SCENE),
    ) -> str:
        """Inspect a Sionna RT XML scene and return material usage as JSON.
        查看 Sionna RT XML 场景，并以 JSON 返回材质使用情况。
        """
        try:
            return summarize_scene(scene_path)
        except XmlToolError as exc:
            return f"error: {exc}"

    @tool
    def set_sionna_material_color(
        material_id: Annotated[str, "Material id, e.g. mat-itu_concrete or itu_concrete."],
        rgb: Annotated[str, "Three floats in [0, 1], e.g. '1 0 0' for red."],
        scene_path: Annotated[str, "Scene XML path relative to the SimART repo root."] = str(DEFAULT_SCENE),
    ) -> str:
        """Set the display RGB color of a Sionna XML material.
        设置 Sionna XML 材质的显示 RGB 颜色。
        """
        try:
            return set_material_rgb(scene_path, material_id, rgb, apply_changes=apply_changes)
        except XmlToolError as exc:
            return f"error: {exc}"

    @tool
    def replace_sionna_material_references(
        from_material: Annotated[str, "Existing material id, e.g. itu_concrete."],
        to_material: Annotated[str, "Target material id, e.g. itu_glass."],
        limit: Annotated[int, "Maximum matching shapes to edit. 0 means all."] = 0,
        name_contains: Annotated[str, "Only edit shape blocks containing this substring. Empty means no filter."] = "",
        scene_path: Annotated[str, "Scene XML path relative to the SimART repo root."] = str(DEFAULT_SCENE),
    ) -> str:
        """Replace shape material references in a Sionna XML scene.
        替换 Sionna XML 场景中 shape 引用的材质。
        """
        try:
            return replace_material_refs(
                scene_path,
                from_material,
                to_material,
                limit=limit,
                name_contains=name_contains,
                apply_changes=apply_changes,
            )
        except XmlToolError as exc:
            return f"error: {exc}"

    return [inspect_sionna_scene, set_sionna_material_color, replace_sionna_material_references]


SYSTEM_PROMPT = """你是 SimART 项目的 Sionna RT 材质配置助手。

你的任务：
- 帮用户理解和修改 Sionna RT XML 场景材质。
- 优先调用工具检查场景，再决定修改。
- 默认场景是 Paris_Small XML，除非用户指定别的场景。
- 如果用户说“雨天”，可考虑把地面材质从 very_dry/medium_dry 改到 wet_ground；如果场景没有 wet_ground，要先说明。
- 如果用户说“玻璃更多”，可把部分 concrete 引用替换成 glass，先用较小 limit，除非用户要求大面积。
- 修改后要用中文总结：改了哪个文件、哪个材质、影响范围、如何预览。

安全规则：
- 不要编造工具没有返回的信息。
- 不要声称已经写文件，除非工具结果里是 applied。
- 如果工具结果是 dry_run，要明确告诉用户这只是预演。
"""


def run_agent(user_request: str, apply_changes: bool = False, model: str = DEFAULT_MODEL) -> str:
    configure_model_environment(model)
    tools = build_tools(apply_changes=apply_changes)
    agent = create_agent(
        model=model,
        tools=tools,
        system_prompt=SYSTEM_PROMPT,
    )
    result = agent.invoke({"messages": [{"role": "user", "content": user_request}]})
    final_message = result["messages"][-1]
    return str(final_message.content)


def _main() -> int:
    parser = argparse.ArgumentParser(description="Run the beginner SimART LangChain material agent.")
    parser.add_argument("request", help="Natural-language request for the material agent.")
    parser.add_argument("--apply", action="store_true", help="Actually write XML changes. Default is dry-run.")
    parser.add_argument("--model", default=DEFAULT_MODEL, help="LangChain model id, e.g. openai:gpt-5.4.")
    args = parser.parse_args()

    configure_model_environment(args.model)
    if not os.environ.get("OPENAI_API_KEY"):
        raise SystemExit(
            "No API key is set. Export OPENAI_API_KEY for OpenAI, or DEEPSEEK_API_KEY for DeepSeek."
        )

    print(run_agent(args.request, apply_changes=args.apply, model=args.model))
    return 0


if __name__ == "__main__":
    raise SystemExit(_main())
