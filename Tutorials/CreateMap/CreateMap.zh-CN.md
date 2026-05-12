中文 | [English](CreateMap.md)

# 地图创建指南

## 1. 真实世界地图场景适配

### 1.1 将真实世界的 OpenStreetMap 数据转换为 Sionna RT 地图

#### 1.1.1 安装必要软件

- 从 [Blender 4.2 LTS Download](https://www.blender.org/download/lts/4-2/) 安装 Blender 4.2 LTS。
- 从 [JOSM Quick Start](https://wiki.openstreetmap.org/wiki/Zh-hans:JOSM/Quick_start) 安装 JOSM。
- 安装 `blosm` v2.7.25（[GitHub](https://github.com/vvoovv/blosm)）和 `mitsuba-blender` v0.4.0（[GitHub](https://github.com/mitsuba-renderer/mitsuba-blender)）。

#### 1.1.2 在 JOSM 中下载 OSM 地图

使用 JOSM 下载目标 OpenStreetMap 区域，并将其保存为 `.osm` 文件。

#### 1.1.3 使用 blosm 插件导入 OSM 文件

- 在 Blender 中打开 `blosm` 插件面板，将 `Import` 设置为 `File`。
- 选择 `.osm` 文件。只勾选 `Import buildings`，取消勾选 `Import as a single object`，然后点击 `Import` 将 OSM 地图导入 Blender。
- 点击 `Add` -> `Mesh` -> `Plane`，在世界原点创建一个小平面，然后调整其缩放比例。
- 将导入建筑对象的材质名称修改为符合 Sionna RT 规范的材质名。

#### 1.1.4 导出为 Mitsuba 文件

将 Blender 场景导出为 Mitsuba 文件。

- 勾选 `Export IDs`。
- 将轴向设置为 `Y Forward` 和 `Z Up`。

### 1.2 将真实世界的 OpenStreetMap 数据转换为 Unreal Engine 地图

#### 1.2.1 安装必要软件

- 从 [EpicGames/UnrealEngine](https://github.com/EpicGames/UnrealEngine) 安装 Unreal Engine `4.27.2`。
- 从 [OSM2World Download](https://osm2world.org/download/) 安装 OSM2World `0.4.0`。

#### 1.2.2 使用 OSM2World 将 OSM 文件转换为 GLB 文件

在下载的 OSM2World 源码目录下运行：

```bash
./osm2world.sh -i /path/to/input_map.osm -o /path/to/output_map.glb
```

#### 1.2.3 启用 UE 插件并导入 GLB/glTF 文件

- 在 Unreal Engine 中搜索插件 `Datasmith glTF Importer` 并启用它。
- 在顶部工具栏点击 `Datasmith`，然后选择刚刚导出的 `.glb` 文件。
- 将 `Import Uniform Scale` 设置为 `100`。

#### 1.2.4 保存为 UE4 关卡

- 在 Build 之前，选中 `SkyLight` 和 `Light Source`，在 `Details` 面板中将 `Mobility` 设置为 `Movable`。
- 将当前地图保存为 UE4 关卡。
- 将该关卡设置为默认关卡：打开 `Edit` -> `Project Settings` -> `Maps & Modes`，将 `Editor Startup Map` 和 `Game Default Map` 都设置为保存好的地图。这样 UE4 工程启动和运行时都会默认打开该地图。

#### 1.2.5 将 UE4 地图接入 AirSim

- 将 AirSim 插件复制到 UE4 工程中：

```bash
mkdir -p /path/to/YourUEProject/Plugins
cp -r /path/to/AirSim/Unreal/Plugins/AirSim /path/to/YourUEProject/Plugins/
```

- 关闭 Unreal Engine，然后编译 UE4 工程，使 AirSim 模块完成编译：

```bash
/path/to/UnrealEngine/Engine/Build/BatchFiles/Linux/Build.sh \
  YourProjectEditor Linux Development \
  -Project="/path/to/YourUEProject/YourProject.uproject" \
  -WaitMutex
```

- 重新打开 UE4 工程，在 `Window` -> `World Settings` 中将 `GameMode Override` 设置为 `AirSimGameMode`。
- 在场景中添加一个 `Player Start`，并将其放在地图表面上方一点的位置。
- 在 `Content Browser` 中打开 `Geometries` 文件夹。全选所有静态网格体，右键选择 `Asset Actions` -> `Bulk Edit via Property Matrix`。搜索 `collision`，将 `Collision Complexity` 设置为 `Use Complex Collision As Simple`，然后保存。这样所有选中的静态网格体都会具有碰撞属性。
- 点击 `Play`，使用 AirSim 启动该地图。

## 2. 用户自定义地图场景创建

### 2.1 从 RoadRunner 地图导入 UE4

#### 2.1.1 在 RoadRunner 中创建地图

- 在 RoadRunner 中创建地图。具体步骤可参考 [RoadRunner 官方教程](https://ww2.mathworks.cn/help/roadrunner/index.html?s_tid=CRUX_lftnav)。
- 将地图导出为 `.fbx` 文件。

#### 2.1.2 将 FBX 文件导入 UE4

- 在 UE4 的 `Content Browser` 中点击 `Import` 或 `Add/Import`，然后选择导出的 `.fbx` 文件。
- 具体流程可参考 `1.2` 小节。唯一区别是 `.fbx` 文件不需要启用额外插件，可以直接导入。

### 2.2 生成对应的 Sionna RT 地图

#### 2.2.1 将 FBX 文件导入 Blender

- 在 Blender 中点击 `File` -> `Import` -> `FBX`，然后选择导出的 `.fbx` 文件。

#### 2.2.2 运行 Blender 处理脚本

在 Blender 的 `Scripting` 工作区中打开所需脚本，并根据需要运行。

- [simplify_map_to_boxes.py](blender_scripts/simplify_map_to_boxes.py)：将选中的地图对象（通常是建筑物）替换为立方体代理模型。
- [replace_selected_material.py](blender_scripts/replace_selected_material.py)：将选中对象的所有材质槽替换为 `itu_concrete`。
- [mesh_statistics_report.py](blender_scripts/mesh_statistics_report.py)：生成场景网格统计报告。
- [clean_invalid_normals.py](blender_scripts/clean_invalid_normals.py)：尽可能清理无效法线和退化网格数据。

#### 当您已经准备好了您的UE4工程文件、fbx/glb地图文件，你可以前往[用您的地图来使用SimART](../Usage/Usage.md)来进行下一步。