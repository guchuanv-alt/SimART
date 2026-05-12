[中文](CreateMap.zh-CN.md) | English

# Create Map Guide

## 1. Real-World Map Scene Adaptation

### 1.1 Convert Real-World OpenStreetMap Data to a Sionna RT Map

#### 1.1.1 Install Required Software

- Install Blender 4.2 LTS from [Blender 4.2 LTS Download](https://www.blender.org/download/lts/4-2/).
- Install JOSM from [JOSM Quick Start](https://wiki.openstreetmap.org/wiki/Zh-hans:JOSM/Quick_start).
- Install `blosm` v2.7.25 ([GitHub](https://github.com/vvoovv/blosm)) and `mitsuba-blender` v0.4.0 ([GitHub](https://github.com/mitsuba-renderer/mitsuba-blender)).

#### 1.1.2 Download the OSM Map in JOSM

Use JOSM to download the target OpenStreetMap area and save it as an `.osm` file.

#### 1.1.3 Import the OSM File with the blosm Plugin

- Open the `blosm` plugin panel in Blender. Set `Import` to `File`.
- Select the `.osm` file. Enable only `Import buildings`. Disable `Import as a single object`. Click `Import` to load the OSM map into Blender.
- Click `Add` -> `Mesh` -> `Plane`. This creates a small plane at the world origin, then adjust its scale factor.
- Rename the imported building material names to follow the Sionna RT material naming convention.

#### 1.1.4 Export as a Mitsuba File

Export the Blender scene as a Mitsuba file.

- Enable `Export IDs`.
- Set the axis orientation to `Y Forward` and `Z Up`.

### 1.2 Convert Real-World OpenStreetMap Data to an Unreal Engine Map

#### 1.2.1 Install Required Software

- Install Unreal Engine `4.27.2` from [EpicGames/UnrealEngine](https://github.com/EpicGames/UnrealEngine).
- Install OSM2World `0.4.0` from [OSM2World Download](https://osm2world.org/download/).

#### 1.2.2 Convert the OSM File to a GLB File with OSM2World

In the downloaded OSM2World source directory, run:

```bash
./osm2world.sh -i /path/to/input_map.osm -o /path/to/output_map.glb
```

#### 1.2.3 Enable the UE Plugin and Import the GLB/glTF File

- In Unreal Engine, search for the plugin `Datasmith glTF Importer` and enable it.
- In the top toolbar, click `Datasmith`, then select the exported `.glb` file.
- Set `Import Uniform Scale` to `100`.

#### 1.2.4 Save as a UE4 Level

- Before building, select `SkyLight` and `Light Source`, then set `Mobility` to `Movable` in the `Details` panel.
- Save the current map as a UE4 level.
- Set this level as the default level: open `Edit` -> `Project Settings` -> `Maps & Modes`, then set both `Editor Startup Map` and `Game Default Map` to the saved map. This makes the UE4 project open and run this map by default.

#### 1.2.5 Integrate the UE4 Map with AirSim

- Copy the AirSim plugin into the UE4 project:

```bash
mkdir -p /path/to/YourUEProject/Plugins
cp -r /path/to/AirSim/Unreal/Plugins/AirSim /path/to/YourUEProject/Plugins/
```

- Close Unreal Engine, then build the UE4 project so the AirSim modules are compiled:

```bash
/path/to/UnrealEngine/Engine/Build/BatchFiles/Linux/Build.sh \
  YourProjectEditor Linux Development \
  -Project="/path/to/YourUEProject/YourProject.uproject" \
  -WaitMutex
```

- Reopen the UE4 project and set `GameMode Override` to `AirSimGameMode` in `Window` -> `World Settings`.
- Add a `Player Start` actor to the scene and place it slightly above the map surface.
- In the `Content Browser`, open the `Geometries` folder. Select all static meshes, right-click, and choose `Asset Actions` -> `Bulk Edit via Property Matrix`. Search for `collision`, set `Collision Complexity` to `Use Complex Collision As Simple`, then save. This gives all selected static meshes collision.
- Click `Play` to launch the map with AirSim.

## 2. User-Defined Map Scene Creation

### 2.1 Import a RoadRunner Map into UE4

#### 2.1.1 Create the Map in RoadRunner

- Create the map in RoadRunner. For details, refer to the [RoadRunner documentation](https://ww2.mathworks.cn/help/roadrunner/index.html?s_tid=CRUX_lftnav).
- Export the map as an `.fbx` file.

#### 2.1.2 Import the FBX File into UE4

- In the UE4 `Content Browser`, click `Import` or `Add/Import`, then select the exported `.fbx` file.
- For the detailed workflow, refer to section `1.2`. The only difference is that `.fbx` files can be imported directly without enabling an additional plugin.

### 2.2 Generate the Corresponding Sionna RT Map

#### 2.2.1 Import the FBX File into Blender

- In Blender, click `File` -> `Import` -> `FBX`, then select the exported `.fbx` file.

#### 2.2.2 Run the Blender Processing Scripts

Open the required script in Blender's `Scripting` workspace and run it as needed.

- [simplify_map_to_boxes.py](blender_scripts/simplify_map_to_boxes.py): replace selected map objects, typically buildings, with box proxies.
- [replace_selected_material.py](blender_scripts/replace_selected_material.py): replace all material slots of selected objects with `itu_concrete`.
- [mesh_statistics_report.py](blender_scripts/mesh_statistics_report.py): generate a mesh statistics report for the scene.
- [clean_invalid_normals.py](blender_scripts/clean_invalid_normals.py): clean invalid normals and degenerate mesh data where possible.

#### After your UE4 project, fbx/glb maps are ready, you can head to [Use SimART with Your Own Maps](../Usage/Usage.md) for the next step.