# SimART Asset Tools

This folder currently keeps only the asset-level material workflow.

For moving this work to a new machine, see:

```text
agent/CONTINUE_ON_NEW_MACHINE.md
```

The goal is:

1. Edit the visual asset used by the main SimART GUI.
2. Export the updated FBX for visual simulation.
3. Export the updated Mitsuba/Sionna XML for RT simulation.

The old XML-only tools and LangChain agent prototype were removed because they
did not update the visual FBX layer used by the main simulator.

## Main Script

```bash
python3 -m agent.sionna_asset_tools replace-material itu_concrete itu_glass
```

Dry-run mode is the default. It starts Blender, loads the configured BigCity
assets, and reports what would be changed.

To actually write files:

```bash
python3 -m agent.sionna_asset_tools replace-material itu_concrete itu_glass --apply
```

By default this targets:

- Visual FBX input/output:
  `SimART_sample_maps/BigCitySample/BigCitySample_fbx/BigCitySampleScene.fbx`
- Sionna Blender source:
  `SimART_sample_maps/BigCitySample/BigCitySample_simptest.blend`
- Sionna XML output:
  `SimART_sample_maps/BigCitySample/BigCitySample_simptest_sionna/BigCitySample_simptest.xml`

## Blender

The script first looks for:

1. `--blender /path/to/blender`
2. `BLENDER=/path/to/blender`
3. `.tools/blender-*-linux-x64/blender`
4. `blender` on `PATH`

The local `.tools/` directory is ignored by git.

## Natural-Language Agent

The agent wraps the same asset pipeline as LangChain tools. It reads the
DeepSeek key from `DEEPSEEK_API_KEY`, `.env`, or `agent/.env`.

Example `agent/.env`:

```bash
DEEPSEEK_API_KEY=your_key_here
```

Preview only:

```bash
python3 -m agent.material_asset_agent "change concrete to glass"
```

Allow writing files:

```bash
python3 -m agent.material_asset_agent --apply "change concrete to glass directly"
```

### Dynamic Weather Materials

Weather is a separate XML-only workflow: it does not edit the visual FBX or
overwrite the baseline XML. For every request it creates exactly one derived
scene XML, returns a `SIMART_AGENT_ACTION_JSON` line, and the embedded GUI uses
that line to select the generated XML.

For example:

```bash
python3 -m agent.material_asset_agent --apply \
  "It just stopped raining heavily. It is 20 C, the ground has about 1 mm of standing water. Generate and apply the scene."
```

The LLM only converts language into this small physical input schema. It must
not invent permittivity or conductivity values:

```json
{
  "temperature_c": 20,
  "material_moisture": 0.85,
  "ground_water_mm": 1.0,
  "roof_water_mm": 0.0,
  "snow_mm": 0.0,
  "ice_mm": 0.0,
  "snow_density_g_cm3": 0.30
}
```

`agent/weather_material_model.py` deterministically updates all 149 current
BigCity material slots at 3.5 GHz. It classifies the exported source slots
(`Roof_Bitumen`, `Asphalt1_Road`, `Grass1_Ground`, and the ITU-labelled slots),
then applies the following rules:

- Every slot uses the documented dry/wet endpoint interpolation. Glass and
  metal have identical dry and wet endpoints, so their internal parameters do
  not change in this first model.
- Ground slots can receive `ground_water_mm`, ice, and snow.
- Roof slots can receive `roof_water_mm`, ice, and snow.
- Vertical wall, glass, metal, wood, and prop slots receive no millimetre-scale
  standing water or snow in this first model, although porous walls can still
  receive the internal-moisture correction.

The generated `*.materials.json` beside the XML records every material-slot
classification, input, assumption, calculated relative permittivity, and
conductivity. The formulas and source references are in
`docs/SimART_天气材质两参数简化模型.docx`.

## Restore BigCity To Downloaded State

Preview:

```bash
python3 -m agent.sionna_asset_tools restore-bigcity
```

Overwrite the BigCity visual FBX, Sionna Blender source, and Sionna XML from
`SimART_sample_maps.zip`:

```bash
python3 -m agent.sionna_asset_tools restore-bigcity --apply
```

The same action is also available through the agent:

```bash
python3 -m agent.material_asset_agent --apply "restore BigCity to the initial downloaded state"
```
