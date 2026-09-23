"""Deterministic weather-to-radio-material model for BigCitySample.

The LLM is deliberately not allowed to invent radio parameters.  It only
extracts weather inputs.  This module classifies every exported material slot,
computes its complex permittivity at the configured frequency, and writes the
single XML scene requested by the user.

The model is an engineering equivalent-material approximation: Sionna XML has
one homogeneous radio material per BSDF, whereas liquid water, ice and snow
are physical surface layers.  We therefore match their normal-incidence
reflection with one equivalent homogeneous material.  The approximation and
all non-measured defaults are recorded in the generated manifest.
"""

from __future__ import annotations

from dataclasses import asdict, dataclass
import cmath
import json
import math
import os
from pathlib import Path
import re
import time
import xml.etree.ElementTree as ET


EPSILON_0 = 8.854187817e-12
DEFAULT_FREQUENCY_HZ = 3.5e9


# Fixed dry/wet endpoints for the Word-specified two-function model.
# Values are for 3.5 GHz and 20 C. The LLM never chooses these values.
MATERIAL_ENDPOINTS: dict[str, tuple[tuple[float, float], tuple[float, float]]] = {
    "concrete": ((5.24, 0.12308695), (10.38365162, 0.30020109)),
    "brick": ((3.91, 0.02908224), (15.39746534, 0.37918714)),
    "wood": ((1.99, 0.01799824), (24.11975099, 0.78794860)),
    "glass": ((6.31, 0.01927646), (6.31, 0.01927646)),
    "metal": ((1.0, 1.0e7), (1.0, 1.0e7)),
    "asphalt_concrete": ((4.83, 0.06214875), (6.69358466, 0.11552963)),
    "vegetated_ground": ((13.23379658, 0.26971118), (50.94386830, 1.58356371)),
}

# Human-readable weather colors, interpolated by normalized material moisture.
WEATHER_COLOR_STOPS: tuple[tuple[float, tuple[float, float, float]], ...] = (
    (0.00, (0.95, 0.78, 0.25)),  # clear: warm sunlight
    (0.10, (0.55, 0.60, 0.65)),  # cloudy: gray-blue
    (0.35, (0.35, 0.65, 0.95)),  # light rain: pale blue
    (0.65, (0.15, 0.35, 0.85)),  # moderate rain: saturated blue
    (0.85, (0.05, 0.10, 0.45)),  # heavy rain: dark storm blue
)


@dataclass(frozen=True)
class MaterialProfile:
    physical_class: str
    baseline: str
    role: str
    confidence: str
    reason: str


@dataclass(frozen=True)
class SurfaceState:
    temperature_c: float
    moisture: float
    ground_water_mm: float
    roof_water_mm: float
    snow_mm: float
    ice_mm: float
    snow_density_g_cm3: float


def clamp(value: float, low: float, high: float) -> float:
    return max(low, min(high, value))


def _number(value: object, name: str, low: float | None = None) -> float:
    try:
        result = float(value)
    except (TypeError, ValueError) as exc:
        raise ValueError(f"{name} must be numeric, got {value!r}") from exc
    if low is not None and result < low:
        raise ValueError(f"{name} must be at least {low}, got {result}")
    return result


def parse_surface_state(environment: dict) -> SurfaceState:
    """Parse the intentionally small natural-language-derived input schema."""
    moisture = environment.get("material_moisture", environment.get("global_wetness", 0.0))
    return SurfaceState(
        temperature_c=_number(environment.get("temperature_c", 20.0), "temperature_c"),
        moisture=clamp(_number(moisture, "material_moisture"), 0.0, 1.0),
        ground_water_mm=_number(environment.get("ground_water_mm", 0.0), "ground_water_mm", 0.0),
        roof_water_mm=_number(environment.get("roof_water_mm", 0.0), "roof_water_mm", 0.0),
        snow_mm=_number(environment.get("snow_mm", 0.0), "snow_mm", 0.0),
        ice_mm=_number(environment.get("ice_mm", 0.0), "ice_mm", 0.0),
        snow_density_g_cm3=clamp(
            _number(environment.get("snow_density_g_cm3", 0.30), "snow_density_g_cm3", 0.05),
            0.05,
            0.60,
        ),
    )


def pure_water(frequency_hz: float, temperature_c: float) -> complex:
    """ITU-R P.527-6 equations (5)-(13), pure liquid water."""
    temperature_c = clamp(temperature_c, -4.0, 40.0)
    f_ghz = frequency_hz / 1e9
    theta = 300.0 / (temperature_c + 273.15) - 1.0
    eps_s = 77.66 + 103.3 * theta
    eps_1 = 0.0671 * eps_s
    eps_inf = 3.52 - 7.52 * theta
    f_1 = 20.20 - 146.4 * theta + 316.0 * theta * theta
    f_2 = 39.8 * f_1
    eps_real = (eps_s - eps_1) / (1.0 + (f_ghz / f_1) ** 2)
    eps_real += (eps_1 - eps_inf) / (1.0 + (f_ghz / f_2) ** 2) + eps_inf
    eps_imag = (f_ghz / f_1) * (eps_s - eps_1) / (1.0 + (f_ghz / f_1) ** 2)
    eps_imag += (f_ghz / f_2) * (eps_1 - eps_inf) / (1.0 + (f_ghz / f_2) ** 2)
    return complex(eps_real, -eps_imag)


def pure_ice(frequency_hz: float, temperature_c: float) -> complex:
    """ITU-R P.527-6 equations (28)-(34), pure ice."""
    temperature_c = clamp(temperature_c, -60.0, 0.0)
    f_ghz = frequency_hz / 1e9
    theta = 300.0 / (temperature_c + 273.15) - 1.0
    tau = 335.0 / (temperature_c + 273.15)
    a = (0.00504 + 0.0062 * theta) * math.exp(-22.1 * theta)
    b = (
        0.0207 / (temperature_c + 273.15) * math.exp(-tau) / (math.exp(-tau) - 1.0) ** 2
        + 1.16e-11 * f_ghz * f_ghz
        + math.exp(-9.963 + 0.0372 * temperature_c)
    )
    return complex(3.1884 + 0.00091 * temperature_c, -(a / f_ghz + b * f_ghz))


def dry_snow(frequency_hz: float, temperature_c: float, density_g_cm3: float) -> complex:
    """ITU-R P.527-6 equations (51)-(53), dry snow at a configured density."""
    density = clamp(density_g_cm3, 0.05, 0.60)
    ice = pure_ice(frequency_hz, min(temperature_c, 0.0))
    eps_real = 1.0 + 1.9 * density if density <= 0.5 else 0.51 + 2.88 * density
    ice_fraction = density / 0.916
    eps_imag = (
        3.0
        * abs(ice.imag)
        * ice_fraction
        * eps_real**2
        * (2.0 * eps_real + 1.0)
        / ((ice.real + 2.0 * eps_real) * (ice.real + 2.0 * eps_real**2))
    )
    return complex(eps_real, -eps_imag)


def profile_for_material_id(material_id: str) -> MaterialProfile:
    """Use exported source names and ITU labels as auditable physical evidence."""
    source = material_id.lower()
    if "roof_bitumen" in source:
        return MaterialProfile("bituminous_roof", "asphalt_concrete", "roof", "medium", "source slot name Roof_Bitumen; asphalt-concrete proxy")
    if "roof_asphalt" in source:
        return MaterialProfile("asphalt_roof", "asphalt_concrete", "roof", "high", "source slot name Roof_Asphalt")
    if "asphalt" in source and "roof" not in source:
        return MaterialProfile("asphalt_road", "asphalt_concrete", "ground", "high", "source slot name Asphalt/Road")
    if "grass" in source:
        return MaterialProfile("vegetated_ground", "vegetated_ground", "ground", "medium", "source slot name Grass/Ground; medium-dry-ground proxy")
    if "itu_concrete" in source:
        role = "ground" if any(word in source for word in ("ground", "sidewalk", "gutter", "curb")) else "wall"
        return MaterialProfile("concrete", "concrete", role, "high", "ITU concrete label and source slot role")
    if "itu_brick" in source:
        role = "ground" if "ground" in source else "wall"
        return MaterialProfile("brick", "brick", role, "high", "ITU brick label and source slot role")
    if "itu_wood" in source:
        return MaterialProfile("wood", "wood", "wall", "high", "ITU wood label")
    if "itu_glass" in source:
        return MaterialProfile("glass", "glass", "wall", "high", "ITU glass label; non-porous")
    if "itu_metal" in source:
        return MaterialProfile("metal", "metal", "wall", "high", "ITU metal label; non-porous")
    return MaterialProfile("unresolved", "concrete", "wall", "low", "no reliable source-name evidence; conservative concrete fallback")


def bulk_material(profile: MaterialProfile, state: SurfaceState, frequency_hz: float) -> complex:
    """Word-spec bulk formula: linear interpolation of fixed dry/wet endpoints."""
    try:
        (eps_dry, sigma_dry), (eps_wet, sigma_wet) = MATERIAL_ENDPOINTS[profile.baseline]
    except KeyError as exc:
        raise KeyError(f"no dry/wet endpoint profile for {profile.baseline}") from exc
    eps_r = eps_dry + state.moisture * (eps_wet - eps_dry)
    sigma = sigma_dry + state.moisture * (sigma_wet - sigma_dry)
    return complex(eps_r, -sigma / (2.0 * math.pi * frequency_hz * EPSILON_0))


def coverage_for_role(role: str, state: SurfaceState) -> tuple[float, float, float]:
    """Return water, ice and snow layer depths in metres for exposed surfaces.

    Vertical facades do not receive millimetre-scale pooling or snow load in
    this first model. They still receive the bulk-moisture correction above.
    Roof water is separate because rainwater drains from tilted roof surfaces.
    """
    if role == "ground":
        water_mm, ice_mm, snow_mm = state.ground_water_mm, state.ice_mm, state.snow_mm
    elif role == "roof":
        water_mm, ice_mm, snow_mm = state.roof_water_mm, state.ice_mm * 0.7, state.snow_mm * 0.7
    else:
        return 0.0, 0.0, 0.0
    if state.temperature_c < 0.0 and water_mm > 0.0:
        ice_mm += water_mm
        water_mm = 0.0
    return water_mm * 1e-3, ice_mm * 1e-3, snow_mm * 1e-3


def equivalent_layered_material(substrate: complex, layers_bottom_to_top: list[tuple[complex, float]], frequency_hz: float) -> complex:
    """Bounded single-material equivalent for sub-wavelength surface layers.

    XML radio materials cannot encode a finite water/ice/snow layer. A strict
    multilayer reflection match can have no passive homogeneous solution for a
    finite film, so the single-material approximation uses a bounded electrical
    thickness weight instead. It preserves a positive real permittivity and
    passive loss while making the layer effect grow monotonically with thickness.
    """
    effective = substrate
    free_space_wavelength = 299792458.0 / frequency_hz
    for epsilon, thickness_m in layers_bottom_to_top:
        if thickness_m <= 0.0:
            continue
        refractive_index = max(1.0, cmath.sqrt(epsilon).real)
        electrical_thickness = 2.0 * math.pi * refractive_index * thickness_m / free_space_wavelength
        weight = 1.0 - math.exp(-electrical_thickness)
        effective = (1.0 - weight) * effective + weight * epsilon
    return effective


def effective_complex_permittivity(material_id: str, environment: dict, frequency_hz: float = DEFAULT_FREQUENCY_HZ) -> tuple[complex, dict]:
    """Core physical function used by both public epsilon and sigma functions."""
    state = parse_surface_state(environment)
    profile = profile_for_material_id(material_id)
    bulk = bulk_material(profile, state, frequency_hz)
    water_m, ice_m, snow_m = coverage_for_role(profile.role, state)
    layered = equivalent_layered_material(
        bulk,
        [
            (pure_ice(frequency_hz, state.temperature_c), ice_m),
            (pure_water(frequency_hz, state.temperature_c), water_m),
            (dry_snow(frequency_hz, state.temperature_c, state.snow_density_g_cm3), snow_m),
        ],
        frequency_hz,
    )
    details = {
        "physical_class": profile.physical_class,
        "baseline": profile.baseline,
        "role": profile.role,
        "confidence": profile.confidence,
        "classification_reason": profile.reason,
        "material_moisture": state.moisture,
        "dry_endpoint": {
            "relative_permittivity": MATERIAL_ENDPOINTS[profile.baseline][0][0],
            "conductivity": MATERIAL_ENDPOINTS[profile.baseline][0][1],
        },
        "wet_endpoint": {
            "relative_permittivity": MATERIAL_ENDPOINTS[profile.baseline][1][0],
            "conductivity": MATERIAL_ENDPOINTS[profile.baseline][1][1],
        },
        "bulk_complex_permittivity": [bulk.real, bulk.imag],
        "surface_layers_m": {"water": water_m, "ice": ice_m, "snow": snow_m},
    }
    return layered, details


def relative_permittivity(material_id: str, environment: dict, frequency_hz: float = DEFAULT_FREQUENCY_HZ) -> float:
    """First requested core output function: effective relative permittivity."""
    epsilon, _ = effective_complex_permittivity(material_id, environment, frequency_hz)
    return max(1.0, epsilon.real)


def conductivity(material_id: str, environment: dict, frequency_hz: float = DEFAULT_FREQUENCY_HZ) -> float:
    """Second requested core output function: equivalent conductivity in S/m."""
    epsilon, _ = effective_complex_permittivity(material_id, environment, frequency_hz)
    return max(0.0, -epsilon.imag * 2.0 * math.pi * frequency_hz * EPSILON_0)


def _find_or_add(parent: ET.Element, tag: str, name: str) -> ET.Element:
    for child in parent:
        if child.tag == tag and child.get("name") == name:
            return child
    child = ET.Element(tag, {"name": name})
    parent.append(child)
    return child


def _set_float(bsdf: ET.Element, name: str, value: float) -> None:
    node = _find_or_add(bsdf, "float", name)
    node.set("value", f"{value:.12g}")


def weather_visual_color(moisture: float) -> str:
    """Return a deterministic, human-readable RGB color for weather wetness."""
    value = clamp(float(moisture), 0.0, 1.0)
    for (low_value, low_rgb), (high_value, high_rgb) in zip(WEATHER_COLOR_STOPS, WEATHER_COLOR_STOPS[1:]):
        if value <= high_value:
            ratio = (value - low_value) / (high_value - low_value)
            rgb = tuple(low + ratio * (high - low) for low, high in zip(low_rgb, high_rgb))
            return " ".join(f"{channel:.6f}" for channel in rgb)
    return " ".join(f"{channel:.6f}" for channel in WEATHER_COLOR_STOPS[-1][1])


def generate_dynamic_weather_scene(
    *,
    template_xml: Path,
    output_root: Path,
    scene_label: str,
    environment: dict,
    rationale: str = "",
    frequency_hz: float = DEFAULT_FREQUENCY_HZ,
) -> tuple[Path, Path, dict]:
    """Generate exactly one request-specific XML and a reproducibility manifest."""
    if not template_xml.exists():
        raise FileNotFoundError(f"template XML does not exist: {template_xml}")
    state = parse_surface_state(environment)
    visual_color = weather_visual_color(state.moisture)
    digest_source = json.dumps({"label": scene_label, "environment": environment, "frequency_hz": frequency_hz}, sort_keys=True, ensure_ascii=False)
    digest = __import__("hashlib").sha1(digest_source.encode("utf-8")).hexdigest()[:10]
    slug = re.sub(r"[^A-Za-z0-9_.-]+", "_", scene_label.strip()).strip("_.-")[:48] or "weather"
    run_dir = output_root / f"dynamic_{slug}_{digest}"
    run_dir.mkdir(parents=True, exist_ok=True)

    tree = ET.parse(template_xml)
    root = tree.getroot()
    updated: list[dict] = []
    id_map: dict[str, str] = {}
    for index, bsdf in enumerate(root.findall("bsdf")):
        material_id = bsdf.get("id") or ""
        epsilon, details = effective_complex_permittivity(material_id, environment, frequency_hz)
        eps_r = max(1.0, epsilon.real)
        sigma = max(0.0, -epsilon.imag * 2.0 * math.pi * frequency_hz * EPSILON_0)
        # Sionna automatically converts IDs beginning with mat-itu_ to its
        # immutable ITU plugin. Dynamic materials must have a distinct ID.
        weather_id = f"mat-weather-{index:03d}-{re.sub(r'[^A-Za-z0-9_.-]+', '_', material_id)}"
        id_map[material_id] = weather_id
        bsdf.set("id", weather_id)
        bsdf.set("name", weather_id)
        bsdf.set("type", "radio-material")
        for child in list(bsdf):
            if child.tag == "string" and child.get("name") == "type":
                bsdf.remove(child)
        _set_float(bsdf, "relative_permittivity", eps_r)
        _set_float(bsdf, "conductivity", sigma)
        _set_float(bsdf, "thickness", 0.1)
        _set_float(bsdf, "scattering_coefficient", 0.0)
        _set_float(bsdf, "xpd_coefficient", 0.0)
        rgb = _find_or_add(bsdf, "rgb", "color")
        rgb.set("value", visual_color)
        updated.append({"material_id": material_id, "relative_permittivity": eps_r, "conductivity": sigma, **details})

    for ref in root.findall(".//ref"):
        if ref.get("name") == "bsdf" and ref.get("id") in id_map:
            ref.set("id", id_map[ref.get("id")])

    # Paths in the base XML are relative to its directory; the dynamic XML is not.
    template_dir = template_xml.parent.resolve()
    for node in root.findall(".//string"):
        if node.get("name") == "filename" and node.get("value"):
            original = (template_dir / node.get("value")).resolve()
            node.set("value", os.path.relpath(original, run_dir.resolve()).replace(os.sep, "/"))

    scene_path = run_dir / "BigCitySample_agent_dynamic_weather.xml"
    tree.write(scene_path, encoding="utf-8", xml_declaration=True)
    manifest = {
        "scene_xml": str(scene_path.resolve()),
        "template_xml": str(template_xml.resolve()),
        "created_unix_time": time.time(),
        "frequency_hz": frequency_hz,
        "environment": asdict(state),
        "visual_color_rgb": visual_color,
        "rationale": rationale,
        "model": {
            "bulk": "fixed dry/wet endpoint linear interpolation",
            "water_ice_snow": "ITU-R P.527-6",
            "dry_building_baselines": "ITU-R P.2040-4 values used in the local endpoint table",
            "surface_layers": "bounded electrical-thickness single-material equivalent",
            "limitation": "one XML BSDF cannot represent finite-layer angle/polarization behaviour exactly",
        },
        "profile_defaults": {
            "dry_wet_endpoints": "fixed local endpoint table at 3.5 GHz and 20 C; replace with measurements when available",
            "vertical_surface_water_ice_snow": "0 mm; vertical facades receive bulk moisture only",
            "roof_snow_ice_retention": "70% of provided horizontal-surface thickness",
        },
        "updated_materials": updated,
    }
    manifest_path = run_dir / "BigCitySample_agent_dynamic_weather.materials.json"
    manifest_path.write_text(json.dumps(manifest, indent=2, ensure_ascii=False), encoding="utf-8")
    return scene_path, manifest_path, manifest
