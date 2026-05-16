<div align="center">

<p>
  <img src="Tutorials/images/simart_wordmark.svg" alt="SimART" width="420">
</p>

<p><strong>An open-source software platform for UAV wireless communication and sensing simulation.</strong></p>

<p>
  <a href="https://arxiv.org/abs/2605.13309">
    <img alt="arXiv" src="https://img.shields.io/badge/arXiv-2605.13309-B31B1B?style=for-the-badge">
  </a>
  <a href="https://kangyan.lat/simart/">
    <img alt="Project Page" src="https://img.shields.io/badge/Project-Page-0F766E?style=for-the-badge">
  </a>
</p>

</div>

---

SimART is an open-source software platform for UAV wireless communication and sensing simulation. Built around ROS1, it integrates a C++/Qt/VTK graphical interface, Sionna-based ray tracing and link simulation, AirSim/Unreal Engine live visualization, and rosbag recording and replay tools. It helps users place base stations, visualize UAV trajectories, inspect wireless channel observations, and evaluate beam selection workflows in 3D scenes.

The project is designed for UAV communication research, digital-twin wireless simulation, visual network planning, Sionna simulation data collection, and communication-sensing experiments in AirSim/Unreal Engine environments.

## Contents

- [System Architecture](#system-architecture)
- [Scene Construction and Map Adaptation](#scene-construction-and-map-adaptation)
- [Multimodal Data and CKM](#multimodal-data-and-ckm)
- [Key Features](#key-features)
- [Preview](#preview)
- [Tutorials](#tutorials)
- [Quick Start](#quick-start)
- [Try SimART](#try-simart)
- [Use SimART with UE4 and AirSim](#use-simart-with-ue4-and-airsim)
- [Further Exploration](#8-further-exploration)

## System Architecture

<p align="center">
  <img src="Tutorials/images/system_architecture.png" alt="SimART system architecture" width="100%">
</p>

SimART consists of four functional modules coordinated by ROS:

| Module | Role |
| --- | --- |
| Physics and Sensing Module | Provides platform motion, RGB/depth/semantic cameras, LiDAR, IMU, GPS, and ground-truth poses through ROS-compatible simulators such as AirSim, Gazebo, Isaac Sim, or CARLA. |
| Ray Tracing Module | Uses Sionna RT to compute site-specific propagation paths, delays, angles, Doppler shifts, interaction points, and channel impulse responses. |
| Link and System Module | Uses Sionna SYS to evaluate OFDM, PHY/MAC behavior, multi-antenna links, beamforming codebooks, SINR, BLER, achievable rate, and optimal beam index. |
| CKM Generator | Scans dense receiver grids to generate multi-layer channel knowledge maps for path loss, delay/angular spread, SINR, rate, and beam-selection priors. |

The ray tracing module supports online simulation, offline rosbag replay, and dense grid scan modes. This allows users to visualize propagation during a live session, reproduce experiments from recorded trajectories, or generate CKM layers over a region of interest.

## Scene Construction and Map Adaptation

SimART provides two complementary scene construction pipelines. Both produce a high-fidelity visual scene for physics/sensing and a simplified, material-aware scene for ray tracing, while preserving a shared coordinate frame.

<table>
  <tr>
    <td align="center">
      <img src="Tutorials/images/louvre_map_pipeline.png" alt="Real-world OpenStreetMap based scene construction pipeline" width="100%">
    </td>
  </tr>
  <tr>
    <td align="center"><sub>Real-world map adaptation: OpenStreetMap data are converted into aligned visual and ray-tracing assets.</sub></td>
  </tr>
  <tr>
    <td align="center">
      <img src="Tutorials/images/roadrunner_map_pipeline.png" alt="User-defined RoadRunner scene construction pipeline" width="100%">
    </td>
  </tr>
  <tr>
    <td align="center"><sub>User-defined scene interface: RoadRunner or Unreal Engine scenes are simplified into propagation-ready meshes.</sub></td>
  </tr>
</table>

| Pipeline | Workflow |
| --- | --- |
| Real-world map adaptation | OpenStreetMap extracts provide building footprints, heights, roads, and land-use polygons. OSM2World creates the visual asset for Unreal Engine/AirSim, while Blender exports a Mitsuba/Sionna RT scene with electromagnetic material annotations. |
| User-defined scene interface | RoadRunner, Unreal Engine, or other custom scene assets are imported for physics and sensing. A Blender conversion script removes fine visual details, applies mesh decimation, preserves dominant facades and ground planes, and assigns Sionna RT materials. |

## Multimodal Data and CKM

During a simulation session, every module publishes data under the shared ROS clock. A single rosbag can preserve the complete synchronized session for replay, inspection, dataset export, and downstream learning tasks.

<table>
  <tr>
    <td align="center" width="50%">
      <img src="Tutorials/images/multimodal_uav_sensor_streams.png" alt="Multimodal UAV sensor streams" width="100%">
    </td>
    <td align="center" width="50%">
      <img src="Tutorials/images/multimodal_ckm_layers.png" alt="Dense CKM layers generated by SimART" width="100%">
    </td>
  </tr>
  <tr>
    <td align="center"><sub>Multimodal sensor streams: RGB, depth, LiDAR, and semantic segmentation.</sub></td>
    <td align="center"><sub>Dense CKM layers: path loss, received power, best BS rate, and effective SINR.</sub></td>
  </tr>
</table>

| Data Stream | Examples |
| --- | --- |
| Sensing and mobility | RGB images, depth images, semantic images, LiDAR point clouds, IMU, GPS, ground-truth poses, and navigation messages. |
| Wireless channel | Per-link CIRs, propagation paths, path loss, delays, angles of arrival/departure, Doppler, and interaction points. |
| Link/system KPIs | SINR, BLER, achievable rate, PHY/MAC outputs, and optimal beam index from a configured codebook. |
| CKM layers | Path loss, RMS delay spread, angular spread, average SINR, achievable rate, and beam-selection priors over dense spatial grids. |

## Key Features

| Capability | Description |
| --- | --- |
| 3D scene loading and preview | Load local scene meshes in the GUI to inspect maps, UAV trajectories, base stations, and ray paths. |
| Sionna wireless simulation | Run ray tracing, OFDM/SYS link adaptation, and beam codebook selection from exported Mitsuba/XML scenes. |
| ROS1 data-stream integration | Subscribe to UAV pose topics and publish RF, SYS, beam, codebook, and related simulation observations. |
| AirSim/UE live integration | Display UAVs, trajectories, base stations, and rays directly in Unreal Engine scenes, with support for base-station camera previews. |
| Base-station editing and configuration | Add, select, edit, save, and load base-station JSON configurations from the GUI. |
| Rosbag tools | Record, replay, and re-simulate wireless data for offline analysis and dataset generation. |
| Coordinate-frame configuration | Configure transforms among the ROS frame, 3D scene frame, and AirSim frame. |

## Preview

<table>
  <tr>
    <td align="center" width="33%">
      <img src="Tutorials/images/planform.gif" alt="SimART main simulation interface" width="100%">
    </td>
    <td align="center" width="33%">
      <img src="Tutorials/images/drone_view.gif" alt="UAV view and ray tracing visualization" width="100%">
    </td>
    <td align="center" width="33%">
      <img src="Tutorials/images/base_station.gif" alt="Base-station layout and preview" width="100%">
    </td>
  </tr>
  <tr>
    <td align="center"><sub>Main Simulation Interface</sub></td>
    <td align="center"><sub>UAV Perspective</sub></td>
    <td align="center"><sub>Base-Station Perspective</sub></td>
  </tr>
</table>

## Tutorials

The following brings you a quick start. For other tutorials, click here:

| Tutorial | Link |
| --- | --- |
| Create Your Own Maps | [Tutorials/CreateMap/CreateMap.md](Tutorials/CreateMap/CreateMap.md) |
| Use SimART with Your Own Maps | [Tutorials/Usage/Usage.md](Tutorials/Usage/Usage.md) |

## Quick Start

### Building and Compiling

This project is a ROS 1 catkin workspace package set and was tested under
`Ubuntu 20.04 + ROS 1 Noetic`. We recommend working with the same environment.
The repository should be placed under a catkin workspace, for example:

```bash
mkdir -p ~/catkin_ws/src
cd ~/catkin_ws/src
git clone https://github.com/guchuanv-alt/SimART.git
```

The repository contains these catkin packages:

| Package | Description |
| --- | --- |
| `airsim_gui_UErealtime` | the C++/Qt/VTK GUI package |
| `rf_msgs` | RF observation message definitions |
| `sionna_sys_msgs` | Sionna SYS message definitions |
| `sionna_beam_msgs` | Sionna beam/codebook message definitions |

### 1. System and ROS Dependencies

Install ROS 1 Noetic for Ubuntu 20.04 first.

For a new Ubuntu machine, we recommend installing ROS with the FishROS
installer: <https://github.com/fishros/install>

```bash
wget http://fishros.com/install -O fishros && bash fishros
```

After ROS 1 Noetic is installed, source the ROS environment and install the
remaining system dependencies:

```bash
source /opt/ros/noetic/setup.bash
sudo apt update
sudo apt install -y \
  build-essential \
  cmake \
  python3-pip \
  python3-venv \
  python3-rosdep \
  python3-catkin-tools \
  qtbase5-dev \
  qtdeclarative5-dev \
  libqt5opengl5-dev \
  qml-module-qtquick2 \
  libassimp-dev \
  libvtk7-dev
```

If your Ubuntu version does not provide `libvtk7-dev`, install the available
VTK development package instead, for example:

```bash
sudo apt install -y libvtk9-dev
```

Then install ROS package dependencies declared by the catkin packages:

```bash
cd ~/catkin_ws
sudo rosdep init 2>/dev/null || true
rosdep update
rosdep install --from-paths src/SimART --ignore-src -r -y
```

`rosdep` can now install the Qt, VTK, and Assimp dependencies declared by
`SimART_GUI/package.xml`. The explicit `apt install` command above is still
listed so a new machine has the core tools before running `rosdep`.

### 2. Python Dependencies

We recommend installing the Python dependencies with conda. The tested conda
environment is named `SimART` and uses Python 3.12:

```bash
cd ~/catkin_ws/src/SimART
conda create -n SimART python=3.12
conda activate SimART
python -m pip install --upgrade pip
python -m pip install -r requirements.txt
```

`requirements.txt` includes sionna. You can also refer to [Sionna](https://github.com/NVlabs/sionna.git) to install it.

### 3. Build the Default GUI

Build the default GUI with `catkin build`:

```bash
cd ~/catkin_ws
source /opt/ros/noetic/setup.bash
catkin init
catkin config --extend /opt/ros/noetic --cmake-args -DCMAKE_BUILD_TYPE=RelWithDebInfo
catkin build airsim_gui_UErealtime
source devel/setup.bash
```

Run SimART:

```bash
rosrun airsim_gui_UErealtime airsim_gui_UErealtime
```

## Try SimART

4 sample maps and 1 sample rosbag for one of them are provided. The sample
rosbag already contains the UAV pose data recorded from AirSim, so it can be
used directly for simulation without installing AirSim first. The map
BigCitySample can be used for a quick start while the rest can be used for
further exploration.

### 4. Download the Sample Maps and Rosbags

In the root directory of the repository you just cloned, run:

```bash
cd ~/catkin_ws/src/SimART
chmod +x download_sample_maps.sh
./download_sample_maps.sh
```

and:

```bash
cd ~/catkin_ws/src/SimART
chmod +x download_sample_rosbags.sh
./download_sample_rosbags.sh
```

The folder SimART_sample_maps contains 4 sample maps.<br>
The folder SimART_sample_rosbags contains 1 rosbag for the map BigCitySample.

### 5. Load the Config File in SimART and Start the Simulation

Run SimART by using the command:

```bash
rosrun airsim_gui_UErealtime airsim_gui_UErealtime
```

- In SimART, click "Open Existing Config" and select `~/catkin_ws/src/SimART/SimART_sample_maps/BigCitySample/BigCitySample.agcfg`.
- Click "Simulation Settings", select the `SimART` conda environment in the Python environment field, and click "Test Environment".

<p align="center">
  <img src="Tutorials/images/SwitchPythonEnv.png" alt="Select the SimART Python environment in Simulation Settings" width="100%">
</p>

- Click "Open Rosbag Tools", select the downloaded sample rosbag in Replay panel. Click "Start Playback".

<p align="center">
  <img src="Tutorials/images/OpenRosbag.png" alt="Open the sample rosbag in Rosbag Tools" width="100%">
</p>

- Click "Start Simulation" to start the Sionna simulation. RF, SYS, and beam simulation will all be started. The data can be viewed in the Wireless Data and Sionna SYS panels. The raw data is available on ROS topics and can be inspected with the `rostopic` CLI.

<p align="center">
  <img src="Tutorials/images/StartSim.png" alt="Start the Sionna simulation in SimART" width="100%">
</p>

<p align="center">
  <img src="Tutorials/images/WirelessData.png" alt="Wireless Data panel in SimART" width="100%">
</p>

<p align="center">
  <img src="Tutorials/images/SionnaSYSData.png" alt="Sionna SYS panel in SimART" width="100%">
</p>

## Use SimART with UE4 and AirSim

If you need to view the scene in UE4 or customize UAV flight, follow the steps
below to set up Unreal Engine and AirSim.

### 6. Download and Compile Unreal Engine

Clone and build Unreal Engine 4 and AirSim. We recommend you to use Unreal Engine 4.27 version. You can refer to [Unreal Engine Documentation](https://dev.epicgames.com/documentation/unreal-engine/downloading-source-code-in-unreal-engine) and [AirSim](https://github.com/microsoft/AirSim.git). 

Make sure that you can use AirSim to perform UAV simulations first. Then move the AirSim ros wrapper package to your AirSim path. In the root directory of the repository, run (Remember to replace the path/to/your/AirSim with your actual AirSim path):

```bash
cp -r third_party/airsim_ros_pkgs_sa path/to/your/AirSim/ros/src
```

This will copy the modified AirSim ros wrapper package to the AirSim ros workspace. Then build the new AirSim ros wrapper in the root directory of the AirSim ros workspace:

```bash
cd path/to/your/AirSim/ros
catkin build
```

When you use the new ros wrapper to publish the rostopics in AirSim, you can run:
```bash
roslaunch airsim_ros_pkgs_sa airsim_node.launch is_vulkan:="false
```

### 7. Optional: Enable AirSim C++ Live View

The normal build can load local meshes and use ROS topics without the AirSim
C++ SDK. Enable AirSim C++ support only when you need UE live-view integration(The UE Live View panel).

First make sure that Unreal Engine 4 and AirSim have been cloned and built, including AirSim's RPC library. 
Then rebuild the catkin workspace with(Remember to replace /path/to/your/AirSim with your actual AirSim path)

```bash
cd ~/catkin_ws
source /opt/ros/noetic/setup.bash
catkin build airsim_gui_UErealtime --cmake-args \
  -DAIRSIM_GUI_ENABLE_AIRSIM=ON \
  -DAIRSIM_CLIENT_ROOT=/path/to/your/AirSim
source devel/setup.bash
```

With `catkin_make`, use:

```bash
cd ~/catkin_ws
source /opt/ros/noetic/setup.bash
catkin_make \
  -DAIRSIM_GUI_ENABLE_AIRSIM=ON \
  -DAIRSIM_CLIENT_ROOT=/path/to/your/AirSim
source devel/setup.bash
```

You can also export the AirSim path before building:

```bash
export AIRSIM_CLIENT_ROOT=/path/to/your/AirSim
catkin build airsim_gui_UErealtime --cmake-args -DAIRSIM_GUI_ENABLE_AIRSIM=ON
```

### 8. Further Exploration

The rest of the maps should work with a UAV simulation software, e.g., AirSim (Recommended), Gazebo, etc, and a matched map (Take AirSim for instance, a corresponding Unreal Engine project is required). The required output of the UAV simulation software is a rostopic containing the pose of the UAV (data type is nav_msgs/Odometry or geometry_msgs/PoseStamped). If you decide to use AirSim, please follow [Create your own maps](Tutorials/CreateMap/CreateMap.md).

---

## Contact

For questions, feedback, or collaboration, please contact us at kangyan@std.uestc.edu.cn, yuqicao@std.uestc.edu.cn.

---

## Citation

If you use SimART in your research, please cite our paper:

```bibtex
@article{yan2026simart,
  title={SimART: A Unified and Open Real-world Multimodal Simulation Platform for 6G Integrated Sensing and Communication},
  author={Yan, Kang and Cao, Yuqi and Li, Jiaqi and Xiang, Luping and Yang, Kun},
  journal={arXiv preprint arXiv:2605.13309},
  year={2026}
}
```
