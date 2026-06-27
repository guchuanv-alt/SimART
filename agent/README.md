# SimART Asset Tools

This folder currently keeps only the asset-level material workflow.

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
