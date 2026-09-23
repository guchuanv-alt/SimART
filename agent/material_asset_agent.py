"""Natural-language material and weather agent for SimART asset editing."""

from __future__ import annotations

import argparse
import hashlib
import importlib.util
import json
import os
import re
import subprocess
import sys
import time
from pathlib import Path

from agent.sionna_asset_tools import AssetToolError, replace_material_pipeline, restore_bigcity_assets
from agent.weather_material_model import generate_dynamic_weather_scene


REPO_ROOT = Path(__file__).resolve().parents[1]
DEFAULT_MODEL = "deepseek-chat"
DEFAULT_BASE_URL = "https://api.deepseek.com"
AGENT_ACTION_PREFIX = "SIMART_AGENT_ACTION_JSON="
AGENT_SCENE_DIR = (
    REPO_ROOT
    / "SimART_sample_maps/BigCitySample/BigCitySample_strict_review_slot_autogen_sionna/generated_agent_scenes"
)
WEATHER_TEMPLATE_XML = (
    REPO_ROOT
    / "SimART_sample_maps/BigCitySample/BigCitySample_strict_review_slot_autogen_sionna"
    / "BigCitySample_sensitivity_00_baseline_concrete_placeholder.xml"
)
WEATHER_PROFILE_PATH = REPO_ROOT / "agent/weather_profiles/bigcity_weather_profiles.yaml"
WEATHER_GENERATOR = REPO_ROOT / "scripts/generate_weather_scene.py"
UNCERTAIN_MATERIAL_CLASSES = {
    "roof_bitumen",
    "road_asphalt",
    "grass_ground",
    "concrete_ground",
    "brick_paved_ground",
}
MATERIAL_ENVIRONMENT_RESPONSE = {
    "roof_bitumen": {"wetness_scale": 1.0, "surface_water_scale": 0.35},
    "road_asphalt": {"wetness_scale": 0.9, "surface_water_scale": 0.45},
    "grass_ground": {"wetness_scale": 1.15, "surface_water_scale": 0.2},
    "concrete_ground": {"wetness_scale": 0.85, "surface_water_scale": 0.35},
    "brick_paved_ground": {"wetness_scale": 0.9, "surface_water_scale": 0.3},
}
SURFACE_WATER_EPS_GAIN = 0.35
SURFACE_WATER_SIGMA_GAIN = 0.5

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
- If the user asks whether you are DeepSeek or whether DeepSeek was called,
  say that your role is the SimART material asset agent and your backend LLM is
  DeepSeek via the DeepSeek API.
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
- If the user asks to switch weather, set weather, use rainy/cloudy/clear
  conditions, rain, standing water, ice or snow, infer physical environment
  variables and call generate_environment_material_scene. Never choose
  relative_permittivity or conductivity values yourself.
- The weather tool updates every material slot in the active BigCity XML,
  including ITU/Sionna materials. It classifies slots from their exported source
  names, applies the fixed dry/wet endpoint formula to material bulk, and applies water/ice/snow
  only to horizontal ground or roof roles. Vertical facades have no millimetre
  scale pooling or snow cover in this model, but may have bulk moisture.
- Do not choose a pre-generated weather XML. Generate exactly one
  request-specific XML through generate_environment_material_scene. It does not
  overwrite the base scene or visual FBX.
- Use environment_json fields only: temperature_c, material_moisture (0..1),
  ground_water_mm, roof_water_mm, snow_mm, ice_mm, snow_density_g_cm3. If the
  request is only a weather word, infer conservative defaults: clear moisture
  0.0; cloudy 0.1; light rain 0.35; moderate rain 0.65; heavy rain 0.85;
  temperature 20 C unless stated. If a puddle depth is stated, put it in
  ground_water_mm. Do not invent a puddle depth when none is stated.
- The local deterministic model computes relative_permittivity/conductivity at
  3.5 GHz; keep thickness, scattering_coefficient and xpd_coefficient fixed.
- When generate_environment_material_scene returns a SIMART_AGENT_ACTION_JSON
  line, include that line exactly in your final answer so the GUI can switch to
  the new XML.
- XML switching changes the GUI's Sionna simulation XML path only. It does not
  edit the FBX visual scene, so the main 3D visual mesh may not change.
- Keep answers concise and practical.
"""

APPLY_MODE_PROMPT = """This session was started with --apply.

If the user asks to directly execute, directly modify, apply changes, or says
no confirmation is needed, you must call apply_replace_material for a material
replacement request.

If the user asks to restore/reset/recover BigCity to the initial downloaded
state, you must call apply_restore_initial_bigcity.

If the user asks for weather/environment XML generation, call
generate_environment_material_scene. It creates a new generated XML instead of
overwriting the base XML.

Do not call preview_replace_material first in that case.
"""

DRY_RUN_MODE_PROMPT = """This session was started without --apply.

You must only call preview tools. Use preview_replace_material for material
replacement requests and preview_restore_initial_bigcity for restore requests.
If the user asks to directly execute, explain that they need to rerun the
command with --apply.

Exception: weather/environment requests may call
generate_environment_material_scene because it creates a new derived XML and asks
the GUI to switch to it; it does not overwrite the base XML/FBX.
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


def _normalize_proxy_env() -> None:
    """Make common desktop SOCKS proxy env vars compatible with httpx/OpenAI."""
    proxy_keys = [
        "HTTP_PROXY",
        "HTTPS_PROXY",
        "ALL_PROXY",
        "http_proxy",
        "https_proxy",
        "all_proxy",
    ]
    uses_socks_proxy = False
    for key in proxy_keys:
        value = os.environ.get(key, "").strip()
        if value.lower().startswith("socks://"):
            os.environ[key] = "socks5h://" + value[len("socks://"):]
            uses_socks_proxy = True
        elif value.lower().startswith(("socks5://", "socks5h://")):
            uses_socks_proxy = True
    if uses_socks_proxy and importlib.util.find_spec("socksio") is None:
        raise RuntimeError(
            "A SOCKS proxy is configured, but Python package 'socksio' is not installed. "
            "Install it with: python3 -m pip install socksio"
        )


def _static_agent_reply(request: str) -> str | None:
    text = request.strip().lower()
    compact = re.sub(r"[\s!！。,.，?？]+", "", text)
    if compact in {"你好", "您好", "hello", "hi", "hey"}:
        return (
            "你好，我是 SimART 材质/天气 agent。\n"
            "可以直接描述天气、温度、材料湿润程度、地面积水/屋顶水膜/冰雪厚度。\n"
            "天气请求会由 DeepSeek 提取环境变量，再由本地物理公式计算全部材质槽参数、生成一份 XML 并切换。\n"
            "也可以说：把 BigCitySample 里的混凝土换成玻璃。"
        )
    if compact in {"帮助", "help", "usage", "怎么用"}:
        return (
            "可用示例：\n"
            "- 刚下完大雨，20度，地面有约1毫米积水\n"
            "- 零下5度，地面有2毫米冰和10毫米积雪\n"
            "- 把 BigCitySample 里的混凝土换成玻璃\n"
            "- 恢复 BigCity 到初始下载状态\n"
            "天气切换会让 DeepSeek 提取物理环境变量，本地公式计算全部材质槽参数并切换 Sionna XML；不会改变 FBX 主视觉。"
        )
    return None


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


def _safe_slug(text: str) -> str:
    slug = re.sub(r"[^A-Za-z0-9_.-]+", "_", text.strip())
    slug = slug.strip("_.-")
    return slug[:48] or "agent_scene"


def _load_profile_template() -> dict:
    if not WEATHER_PROFILE_PATH.exists():
        raise FileNotFoundError(f"weather profile template does not exist: {WEATHER_PROFILE_PATH}")
    import yaml

    with WEATHER_PROFILE_PATH.open("r", encoding="utf-8") as f:
        return yaml.safe_load(f)


def _coerce_float(value: object, name: str) -> float:
    try:
        return float(value)
    except (TypeError, ValueError) as exc:
        raise ValueError(f"{name} must be a number, got {value!r}") from exc


def _clamp01(value: object, name: str) -> float:
    return max(0.0, min(1.0, _coerce_float(value, name)))


def _normalize_rgb_color(color: str) -> str:
    parts = str(color).replace(",", " ").split()
    if len(parts) != 3:
        raise ValueError("color must contain three RGB values in 0..1, for example '0.05 0.10 0.45'")
    values = []
    for part in parts:
        value = max(0.0, min(1.0, _coerce_float(part, "color")))
        values.append(f"{value:.6f}")
    return " ".join(values)


def _default_exact_params(class_profile: dict) -> dict:
    dry = class_profile["dry"]
    return {
        "relative_permittivity": float(dry["relative_permittivity"]),
        "conductivity": float(dry["conductivity"]),
        "thickness": float(class_profile.get("thickness", 0.1)),
        "scattering_coefficient": float(class_profile.get("scattering_coefficient", 0.0)),
        "xpd_coefficient": float(class_profile.get("xpd_coefficient", 0.0)),
    }


def _parse_material_parameters(raw_json: str) -> dict:
    try:
        parsed = json.loads(raw_json)
    except json.JSONDecodeError as exc:
        raise ValueError(f"material_parameters_json is not valid JSON: {exc}") from exc
    if not isinstance(parsed, dict):
        raise ValueError("material_parameters_json must be a JSON object")
    return parsed


def _parse_environment(raw_json: str) -> dict:
    try:
        parsed = json.loads(raw_json)
    except json.JSONDecodeError as exc:
        raise ValueError(f"environment_json is not valid JSON: {exc}") from exc
    if not isinstance(parsed, dict):
        raise ValueError("environment_json must be a JSON object")
    return parsed


def _environment_to_material_parameters(environment: dict, profile_template: dict) -> tuple[dict, dict]:
    global_wetness = _clamp01(
        environment.get("global_wetness", environment.get("wetness", 0.0)),
        "global_wetness",
    )
    global_surface_water = _clamp01(
        environment.get("surface_water", 0.0),
        "surface_water",
    )
    material_wetness = environment.get("material_wetness", {})
    material_surface_water = environment.get("material_surface_water", {})
    if material_wetness is None:
        material_wetness = {}
    if material_surface_water is None:
        material_surface_water = {}
    if not isinstance(material_wetness, dict):
        raise ValueError("material_wetness must be a JSON object when provided")
    if not isinstance(material_surface_water, dict):
        raise ValueError("material_surface_water must be a JSON object when provided")

    unknown = sorted((set(material_wetness) | set(material_surface_water)) - UNCERTAIN_MATERIAL_CLASSES)
    if unknown:
        raise ValueError(
            "unsupported material classes in environment overrides: "
            + ", ".join(unknown)
            + ". Allowed: "
            + ", ".join(sorted(UNCERTAIN_MATERIAL_CLASSES))
        )

    source_classes = profile_template["material_classes"]
    material_parameters: dict[str, dict[str, float]] = {}
    formula_summary = {
        "model": "bounded_dry_wet_interpolation_with_surface_water_correction",
        "inputs": {
            "global_wetness": global_wetness,
            "surface_water": global_surface_water,
            "material_wetness": material_wetness,
            "material_surface_water": material_surface_water,
        },
        "constants": {
            "surface_water_eps_gain": SURFACE_WATER_EPS_GAIN,
            "surface_water_sigma_gain": SURFACE_WATER_SIGMA_GAIN,
            "material_environment_response": MATERIAL_ENVIRONMENT_RESPONSE,
        },
        "formula": {
            "w_class": "clamp01(material_wetness[class] if provided else global_wetness * wetness_scale[class])",
            "sw_class": "clamp01(material_surface_water[class] if provided else surface_water * surface_water_scale[class])",
            "eps": "eps_dry + w_class*(eps_wet-eps_dry) + sw_class*surface_water_eps_gain*(eps_wet-eps_dry)",
            "sigma": "sigma_dry + w_class*(sigma_wet-sigma_dry) + sw_class*surface_water_sigma_gain*(sigma_wet-sigma_dry)",
        },
        "computed": {},
    }

    for material_class in sorted(UNCERTAIN_MATERIAL_CLASSES):
        if material_class not in source_classes:
            raise ValueError(f"profile template is missing material class: {material_class}")
        response = MATERIAL_ENVIRONMENT_RESPONSE[material_class]
        if material_class in material_wetness:
            w_class = _clamp01(material_wetness[material_class], f"material_wetness.{material_class}")
        else:
            w_class = max(0.0, min(1.0, global_wetness * response["wetness_scale"]))
        if material_class in material_surface_water:
            sw_class = _clamp01(
                material_surface_water[material_class],
                f"material_surface_water.{material_class}",
            )
        else:
            sw_class = max(0.0, min(1.0, global_surface_water * response["surface_water_scale"]))

        class_profile = source_classes[material_class]
        dry = class_profile["dry"]
        wet = class_profile["wet"]
        eps_dry = float(dry["relative_permittivity"])
        eps_wet = float(wet["relative_permittivity"])
        sigma_dry = float(dry["conductivity"])
        sigma_wet = float(wet["conductivity"])
        eps_delta = eps_wet - eps_dry
        sigma_delta = sigma_wet - sigma_dry
        eps = eps_dry + w_class * eps_delta + sw_class * SURFACE_WATER_EPS_GAIN * eps_delta
        sigma = sigma_dry + w_class * sigma_delta + sw_class * SURFACE_WATER_SIGMA_GAIN * sigma_delta
        params = {
            "relative_permittivity": max(1.0, eps),
            "conductivity": max(0.0, sigma),
            "thickness": float(class_profile.get("thickness", 0.1)),
            "scattering_coefficient": float(class_profile.get("scattering_coefficient", 0.0)),
            "xpd_coefficient": float(class_profile.get("xpd_coefficient", 0.0)),
        }
        material_parameters[material_class] = params
        formula_summary["computed"][material_class] = {
            "w_class": w_class,
            "surface_water_class": sw_class,
            "relative_permittivity": params["relative_permittivity"],
            "conductivity": params["conductivity"],
        }

    return material_parameters, formula_summary


def generate_uncertain_material_scene_xml(
    scene_label: str,
    material_parameters_json: str,
    color: str,
    rationale: str = "",
    formula_metadata: dict | None = None,
) -> str:
    """Generate one request-specific XML by editing only uncertain materials."""
    if not WEATHER_TEMPLATE_XML.exists():
        raise FileNotFoundError(f"template XML does not exist: {WEATHER_TEMPLATE_XML}")
    if not WEATHER_GENERATOR.exists():
        raise FileNotFoundError(f"weather XML generator does not exist: {WEATHER_GENERATOR}")

    template = _load_profile_template()
    source_classes = template["material_classes"]
    requested = _parse_material_parameters(material_parameters_json)
    unknown = sorted(set(requested) - UNCERTAIN_MATERIAL_CLASSES)
    if unknown:
        raise ValueError(
            "unsupported material classes: "
            + ", ".join(unknown)
            + ". Allowed: "
            + ", ".join(sorted(UNCERTAIN_MATERIAL_CLASSES))
        )

    material_classes = {}
    updated_summary = {}
    for material_class in sorted(UNCERTAIN_MATERIAL_CLASSES):
        if material_class not in source_classes:
            raise ValueError(f"profile template is missing material class: {material_class}")
        exact = _default_exact_params(source_classes[material_class])
        overrides = requested.get(material_class, {})
        if overrides is None:
            overrides = {}
        if not isinstance(overrides, dict):
            raise ValueError(f"{material_class} parameters must be a JSON object")
        for key in [
            "relative_permittivity",
            "conductivity",
            "thickness",
            "scattering_coefficient",
            "xpd_coefficient",
        ]:
            if key in overrides:
                exact[key] = _coerce_float(overrides[key], f"{material_class}.{key}")
        exact["relative_permittivity"] = max(1.0, exact["relative_permittivity"])
        exact["conductivity"] = max(0.0, exact["conductivity"])
        exact["thickness"] = max(0.0, exact["thickness"])
        exact["scattering_coefficient"] = max(0.0, min(1.0, exact["scattering_coefficient"]))
        exact["xpd_coefficient"] = max(0.0, min(1.0, exact["xpd_coefficient"]))
        material_classes[material_class] = {
            "dry": {
                "relative_permittivity": exact["relative_permittivity"],
                "conductivity": exact["conductivity"],
            },
            "wet": {
                "relative_permittivity": exact["relative_permittivity"],
                "conductivity": exact["conductivity"],
            },
            "thickness": exact["thickness"],
            "scattering_coefficient": exact["scattering_coefficient"],
            "xpd_coefficient": exact["xpd_coefficient"],
        }
        updated_summary[material_class] = exact

    label = scene_label.strip() or "agent_weather"
    slug_source = f"{label}:{material_parameters_json}:{color}"
    slug_hash = hashlib.sha1(slug_source.encode("utf-8")).hexdigest()[:8]
    slug = f"{_safe_slug(label)}_{slug_hash}"
    run_dir = AGENT_SCENE_DIR / slug
    run_dir.mkdir(parents=True, exist_ok=True)

    dynamic_profile = {
        "weather_profiles": {
            "agent": {
                "wetness": 0.0,
                "color": _normalize_rgb_color(color),
            }
        },
        "material_classes": material_classes,
        "material_id_rules": template["material_id_rules"],
        "agent_request": {
            "scene_label": label,
            "rationale": rationale,
            "created_unix_time": time.time(),
            "updated_material_classes": updated_summary,
            "formula_metadata": formula_metadata or {},
        },
    }
    profile_path = run_dir / "profile.json"
    profile_path.write_text(json.dumps(dynamic_profile, indent=2, ensure_ascii=False), encoding="utf-8")

    prefix = f"BigCitySample_agent_{slug}"
    cmd = [
        sys.executable,
        str(WEATHER_GENERATOR),
        "--template",
        str(WEATHER_TEMPLATE_XML),
        "--profiles",
        str(profile_path),
        "--weather",
        "agent",
        "--output-dir",
        str(run_dir),
        "--prefix",
        prefix,
    ]
    result = subprocess.run(
        cmd,
        cwd=str(REPO_ROOT),
        text=True,
        stdout=subprocess.PIPE,
        stderr=subprocess.PIPE,
        check=False,
    )
    if result.returncode != 0:
        raise RuntimeError(
            "agent scene generation failed:\n"
            f"command: {' '.join(cmd)}\n"
            f"stdout:\n{result.stdout}\n"
            f"stderr:\n{result.stderr}"
        )

    scene_path = run_dir / f"{prefix}_agent.xml"
    if not scene_path.exists():
        raise FileNotFoundError(f"agent scene XML was not generated: {scene_path}")
    payload = {
        "action": "set_scene_xml",
        "weather": label,
        "scene_xml": str(scene_path.resolve()),
        "note": "Formula-generated uncertain material parameters; FBX visual scene is unchanged.",
    }
    return (
        f"{AGENT_ACTION_PREFIX}{json.dumps(payload, ensure_ascii=False, separators=(',', ':'))}\n"
        f"Generated request-specific XML: {scene_path.resolve()}\n"
        f"Updated uncertain material classes: {', '.join(sorted(UNCERTAIN_MATERIAL_CLASSES))}\n"
        f"Profile: {profile_path.resolve()}\n"
        "Only currently uncertain roof/ground material classes were modified. Confirmed materials were left unchanged."
    )


def generate_environment_material_scene_xml(
    scene_label: str,
    environment_json: str,
    rationale: str = "",
) -> str:
    """Generate one all-material XML from weather inputs, without preset scenes."""
    environment = _parse_environment(environment_json)
    label = scene_label.strip() or str(environment.get("scene_label", "")).strip() or "environment_scene"
    scene_path, manifest_path, manifest = generate_dynamic_weather_scene(
        template_xml=WEATHER_TEMPLATE_XML,
        output_root=AGENT_SCENE_DIR,
        scene_label=label,
        environment=environment,
        rationale=rationale,
    )
    payload = {
        "action": "set_scene_xml",
        "weather": label,
        "scene_xml": str(scene_path.resolve()),
        "note": "Dynamic physics-model XML; the FBX visual scene is unchanged.",
    }
    return (
        f"{AGENT_ACTION_PREFIX}{json.dumps(payload, ensure_ascii=False, separators=(',', ':'))}\n"
        f"Generated one dynamic XML: {scene_path.resolve()}\n"
        f"Updated all {len(manifest['updated_materials'])} material slots at 3.5 GHz.\n"
        f"Manifest: {manifest_path.resolve()}\n"
        "The generated manifest records source classification, assumptions, layer thicknesses, and every computed epsilon/conductivity value."
    )


def build_tools(allow_apply: bool):
    """Create LangChain tools with an apply safety gate."""
    from langchain.tools import tool

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
    def generate_environment_material_scene(
        scene_label: str,
        environment_json: str,
        rationale: str = "",
    ) -> str:
        """Generate and switch to a request-specific BigCity XML from environment variables.

        Call this for weather or environment requests, for example clear,
        cloudy, light rain, moderate rain, heavy rain, wet roof, flooded ground,
        or seasonal roof/ground adjustments.

        environment_json must be a JSON object. Use these fields:
        - material_moisture: 0..1 normalized bulk moisture/saturation.
          Defaults: clear=0.0, cloudy=0.1, light_rain=0.35,
          moderate_rain=0.65, heavy_rain=0.85.
        - ground_water_mm: standing water depth on horizontal ground only.
        - roof_water_mm: water-film/ponding depth on roofs only.
        - snow_mm and ice_mm: coverage thicknesses on ground and roofs only.
        - temperature_c: air/surface temperature, default 20.
        - snow_density_g_cm3: optional, default 0.30.

        The tool updates all XML BSDF material slots, not merely previously
        uncertain slots. The LLM must never supply epsilon/conductivity values;
        the local model uses fixed dry/wet endpoint linear interpolation,
        ITU-R P.527-6 water/ice/snow dielectric models, and a
        normal-incidence layered equivalent for the single-material XML limit.

        The local model chooses a deterministic display color from
        material_moisture: warm yellow for dry/clear weather, then gray-blue,
        pale blue, saturated blue, and dark storm blue as moisture increases.
        The returned SIMART_AGENT_ACTION_JSON line must be included exactly in
        the final answer so the GUI can switch scene XML.
        """
        try:
            return generate_environment_material_scene_xml(
                scene_label=scene_label,
                environment_json=environment_json,
                rationale=rationale,
            )
        except Exception as exc:
            return f"agent scene generation failed: {exc}"

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
        generate_environment_material_scene,
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
    static_reply = _static_agent_reply(request)
    if static_reply:
        return static_reply

    load_local_env()
    api_key = os.environ.get("DEEPSEEK_API_KEY")
    if not api_key:
        raise RuntimeError(
            "DEEPSEEK_API_KEY is not set. Put it in agent/.env or export it in the terminal."
        )
    _normalize_proxy_env()

    from langchain.agents import create_agent
    from langchain_openai import ChatOpenAI

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
    content = _content_from_result(result).strip()
    backend_note = f"Backend LLM: DeepSeek API ({model_name})"
    if backend_note not in content:
        content = f"{content}\n\n{backend_note}" if content else backend_note
    return content


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
