# Continue The BigCity Material Task On A New Machine

This note records the current BigCity material-agent workflow so the work can
continue after switching machines.

## What Was Pushed To GitHub

The current checkpoint was pushed on branch `LLMdev`.

Latest material checkpoint commit before this note:

```text
ab56b061ec36edd6c64eb877e1b110626d8c706f
Add BigCity material sensitivity assets
```

The pushed files include:

- `agent/material_asset_agent.py`
- `agent/sionna_asset_tools.py`
- `scripts/*.sh`
- `scripts/bigcity_fixed_pose_sensitivity_csv.py`
- `agent_outputs/*.csv`
- `agent_outputs/*.json`
- the five review XML files:
  - `SimART_sample_maps/BigCitySample/BigCitySample_strict_review_sionna/BigCitySample_strict_review.xml`
  - `SimART_sample_maps/BigCitySample/BigCitySample_strict_review_sionna/BigCitySample_sensitivity_00_baseline_concrete_placeholder.xml`
  - `SimART_sample_maps/BigCitySample/BigCitySample_strict_review_sionna/BigCitySample_sensitivity_01_small_low_interaction.xml`
  - `SimART_sample_maps/BigCitySample/BigCitySample_strict_review_sionna/BigCitySample_sensitivity_02_small_medium_interaction.xml`
  - `SimART_sample_maps/BigCitySample/BigCitySample_strict_review_sionna/BigCitySample_sensitivity_03_small_high_interaction.xml`

Important: only the XML files were pushed. The large mesh files referenced by
those XML files were not pushed.

## Files That Were Not Pushed

These files/directories are intentionally ignored by git:

- `SimART_sample_maps/`
- `SimART_sample_maps.zip`
- `SimART_sample_rosbags/`
- `.tools/`
- `.env`
- `agent/.env`

The strict review scene also has many generated `.ply` mesh files under:

```text
SimART_sample_maps/BigCitySample/BigCitySample_strict_review_sionna/meshes/
```

That directory is required if you want to directly open/run the strict review
XML files. It is large, so it should be copied separately or regenerated.

## The Two INI Files

These two files do not need to be uploaded for the material task:

```text
.polyscope.ini
imgui.ini
```

They only store local viewer/window state such as window size and position.
They do not contain material mappings, Sionna solver settings, RF results, or
agent logic.

Current recommendation:

- Do not commit the local changes in these two files.
- It is OK if they remain modified locally.
- If they become annoying in `git status`, restore them with:

```bash
git restore .polyscope.ini imgui.ini
```

Only do that if you do not care about the current local viewer layout.

## New Machine Setup

Clone the repo and switch to the working branch:

```bash
cd /home/ubuntu2004/catkin_ws/src
git clone git@github.com:guchuanv-alt/SimART.git
cd SimART
git checkout LLMdev
```

If the repo already exists:

```bash
cd /home/ubuntu2004/catkin_ws/src/SimART
git fetch origin
git checkout LLMdev
git pull origin LLMdev
```

Load ROS and the SimART catkin workspace:

```bash
source /opt/ros/noetic/setup.bash
source /home/ubuntu2004/catkin_ws/devel/setup.bash
```

Activate the SimART Python environment if needed:

```bash
conda activate SimART
```

## Restore Sample Maps

If `SimART_sample_maps/` is missing, use the project sample-map download flow:

```bash
cd /home/ubuntu2004/catkin_ws/src/SimART
./download_sample_maps.sh
```

If you already copied `SimART_sample_maps.zip` to the new machine, unzip it:

```bash
cd /home/ubuntu2004/catkin_ws/src/SimART
unzip -o SimART_sample_maps.zip
```

## Restore The Strict Review Scene Meshes

The pushed XML files refer to generated PLY meshes. On a new machine you need
one of these two options.

### Option A: Copy The Generated Folder

Copy this whole folder from the old machine:

```text
/home/ubuntu2004/catkin_ws/src/SimART/SimART_sample_maps/BigCitySample/BigCitySample_strict_review_sionna/
```

The important subfolder is:

```text
SimART_sample_maps/BigCitySample/BigCitySample_strict_review_sionna/meshes/
```

### Option B: Regenerate From FBX/Blend

If the normal BigCity assets exist, regenerate the strict review scene:

```bash
cd /home/ubuntu2004/catkin_ws/src/SimART

python3 -m agent.sionna_asset_tools export-strict-review-scene \
  --uncertain-color-mode palette \
  --apply
```

This creates or refreshes the strict review XML/Blend outputs and their mesh
files. Use this when the XML is present but the scene fails to load because the
`meshes/*.ply` files are missing.

## Visualize A Review XML

Use the embedded Sionna viewer:

```bash
cd /home/ubuntu2004/catkin_ws/src/SimART

python3 SimART_GUI/scripts/sionna_embed_launcher.py \
  --scene /home/ubuntu2004/catkin_ws/src/SimART/SimART_sample_maps/BigCitySample/BigCitySample_strict_review_sionna/BigCitySample_strict_review.xml \
  --title "BigCity Strict Review"
```

For one of the sensitivity XML files, replace the `--scene` path, for example:

```bash
python3 SimART_GUI/scripts/sionna_embed_launcher.py \
  --scene /home/ubuntu2004/catkin_ws/src/SimART/SimART_sample_maps/BigCitySample/BigCitySample_strict_review_sionna/BigCitySample_sensitivity_03_small_high_interaction.xml \
  --title "BigCity Small High Interaction"
```

## Run The Material Agent

Set the DeepSeek API key outside git:

```bash
export DEEPSEEK_API_KEY="your_key_here"
```

Preview only:

```bash
python3 -m agent.material_asset_agent "change concrete to glass"
```

Actually modify files:

```bash
python3 -m agent.material_asset_agent --apply "change concrete to glass directly"
```

Restore BigCity to downloaded state:

```bash
python3 -m agent.sionna_asset_tools restore-bigcity --apply
```

## Run The Four XML Sensitivity Experiments

The newer fixed-pose CSV workflow is:

```bash
cd /home/ubuntu2004/catkin_ws/src/SimART

source /opt/ros/noetic/setup.bash
source /home/ubuntu2004/catkin_ws/devel/setup.bash

./scripts/run_bigcity_fixed_pose_sensitivity_csv.sh
```

Default output directory:

```text
SimART_sample_maps/BigCitySample/output/fixed_pose_sensitivity_csv/
```

Main result files:

```text
SimART_sample_maps/BigCitySample/output/fixed_pose_sensitivity_csv/rf_summary.csv
SimART_sample_maps/BigCitySample/output/fixed_pose_sensitivity_csv/rf_deltas_vs_baseline.csv
SimART_sample_maps/BigCitySample/output/fixed_pose_sensitivity_csv/rf_anchor_summary.csv
```

The reflection bag workflow is:

```bash
cd /home/ubuntu2004/catkin_ws/src/SimART
./scripts/run_bigcity_reflection_sensitivity_bags.sh
```

Default output directory:

```text
SimART_sample_maps/BigCitySample/output/reflection_sensitivity_bags/
```

This workflow writes four bag files, one per XML variant.

## Current Experimental Meaning Of The Four XML Files

The confirmed building materials are kept fixed across all four XML files.
Roof and asphalt/ground placeholders are currently kept as concrete-like radio
materials for stability. The small unresolved objects are varied across the four
files:

- `00_baseline`: unresolved small objects use the concrete placeholder.
- `01_small_low`: unresolved small objects use a low-interaction custom material.
- `02_small_medium`: unresolved small objects use a medium-interaction custom material.
- `03_small_high`: unresolved small objects use a high-interaction custom material.

Use `agent_outputs/bigcity_small_object_material_sensitivity_manifest.csv` for
the exact parameter table.

## Before Continuing Research

Useful source files to read:

- `SimART_GUI/scripts/reprocess_rosbag_with_sionna.py`
- `SimART_GUI/scripts/sionna_resim_socket_worker.py`
- `SimART_GUI/scripts/sionna_sim_only_topic2.py`
- `scripts/bigcity_fixed_pose_sensitivity_csv.py`
- `agent/sionna_asset_tools.py`

The key question for the next stage is not only whether an object exists in the
XML. It is whether Sionna's valid paths actually interact with that object. If
that needs to be proven, add logging around the path extraction code so the
output records hit shape/material names for each valid path.
