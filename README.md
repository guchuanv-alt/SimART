# SimART

SimART is an open-source software platform for UAV wireless communication and sensing simulation. Built around ROS1, it integrates a C++/Qt/VTK graphical interface, Sionna-based ray tracing and link simulation, AirSim/Unreal Engine live visualization, and rosbag recording and replay tools. It helps users place base stations, visualize UAV trajectories, inspect wireless channel observations, and evaluate beam selection workflows in 3D scenes.

The project is designed for UAV communication research, digital-twin wireless simulation, visual network planning, Sionna simulation data collection, and communication-sensing experiments in AirSim/Unreal Engine environments.

## Key Features

- 3D scene loading and preview: Load local scene meshes in the GUI to inspect maps, UAV trajectories, base stations, and ray paths.
- Sionna wireless simulation: Run ray tracing, OFDM/SYS link adaptation, and beam codebook selection from exported Mitsuba/XML scenes.
- ROS1 data-stream integration: Subscribe to UAV pose topics and publish RF, SYS, beam, codebook, and related simulation observations.
- AirSim/UE live integration: Display UAVs, trajectories, base stations, and rays directly in Unreal Engine scenes, with support for base-station camera previews.
- Base-station editing and configuration: Add, select, edit, save, and load base-station JSON configurations from the GUI.
- Rosbag tools: Record, replay, and re-simulate wireless data for offline analysis and dataset generation.
- Coordinate-frame configuration: Configure transforms among the ROS frame, 3D scene frame, and AirSim frame.

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
    <td align="center">Main Simulation Interface</td>
    <td align="center">UAV and Ray Visualization</td>
    <td align="center">Base-Station Perspective</td>
  </tr>
</table>

### The following brings you a quick start. For other tutorials, click here:

- [Create Your Own Maps](Tutorials/CreateMap/CreateMap.md)

- [Use SimART with Your Own Maps](Tutorials/Usage/Usage.md)


## Quick Start

### Building and Compiling

This project is a ROS1 catkin workspace package set. The repository should be
placed under a catkin workspace, for example:

```bash
mkdir -p ~/catkin_ws/src
cd ~/catkin_ws/src
git clone https://github.com/guchuanv-alt/SimART.git
```

The repository contains these catkin packages:

- `airsim_gui_UErealtime`: the C++/Qt/VTK GUI package
- `rf_msgs`: RF observation message definitions
- `sionna_sys_msgs`: Sionna SYS message definitions
- `sionna_beam_msgs`: Sionna beam/codebook message definitions

### 1. System and ROS Dependencies

Install ROS1 first. The commands below assume Ubuntu with ROS Noetic. If you
use another ROS1 distribution, replace `noetic` with your own ROS distribution
name, or use `$ROS_DISTRO` after sourcing ROS.

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

Python packages can be installed from `requirements.txt`:

```bash
cd ~/catkin_ws/src/SimART
python3 -m pip install --user -r requirements.txt
```

If you prefer a virtual environment for Sionna or beam-selection tools, create
it with access to ROS Python packages:

```bash
python3 -m venv --system-site-packages ~/.venvs/simart
source ~/.venvs/simart/bin/activate
python3 -m pip install --upgrade pip
python3 -m pip install -r ~/catkin_ws/src/SimART/requirements.txt
```

`requirements.txt` includes sionna. You can also refer to [Sionna](https://github.com/NVlabs/sionna.git) to install it.

### 3. Build the Default GUI

The default build does not require the AirSim C++ SDK.

Use one catkin build tool consistently in the same workspace. Do not switch
between `catkin_make` and `catkin build` without cleaning `build/` and `devel/`,
because their build spaces are not compatible.

#### Recommended: `catkin build`

`catkin build` is convenient for this repository because it can build the GUI package and its local message-package dependencies together. For the first time you build, run:

```bash
cd ~/catkin_ws
source /opt/ros/noetic/setup.bash
catkin init
catkin config --extend /opt/ros/noetic --cmake-args -DCMAKE_BUILD_TYPE=RelWithDebInfo
catkin build airsim_gui_UErealtime
source devel/setup.bash
```

catkin_make is also fine to build.

After a successful build, run SimART with:

```bash
rosrun airsim_gui_UErealtime airsim_gui_UErealtime
```

### 4. Optional: Enable AirSim C++ Live View

The normal build can load local meshes and use ROS topics without the AirSim
C++ SDK. Enable AirSim C++ support only when you need UE live-view integration(The UE Live View panel).

First make sure AirSim has been cloned and built, including its RPC library.
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
---
### Try SimART

4 sample maps and 1 sample rosbag for one of them are provided. The map BigCitySample can be used for a quick start while the rest can be used for further exploration.

### 5. Download the Sample Maps and Rosbags

In the root directory of the repository you just cloned, run:

```bash
chmod +x download_sample_maps.sh
./download_sample_maps.sh
```

and:

```bash
chmod +x download_sample_rosbags.sh
./download_sample_rosbags.sh
```

The folder SimART_sample_maps contains 4 sample maps. The folder SimART_sample_rosbags contains 1 rosbag for the map BigCitySample.

### 6. Load the Config File in SimART and Start the Simulation

Run SimART by using the command:

```bash
rosrun airsim_gui_UErealtime airsim_gui_UErealtime
```

- In SimART, click "Open Existing Config", select BigCitySample->BigCitySample.agcfg in the downloaded sample maps.

- Then click "Simulation Settings", and select a python environment with sionna installed in python environment box.

- Click "Open Rosbag Tools", select the downloaded sample rosbag in Replay panel. Click "Start Playback".

- Click "Start Simulation" to start sionna simulation. Rf, sys and beam simulation will all be started. The data can be viewd at Wireless Data and Sionna SYS panels. The raw data can be obtained in rostopics.

### 7. Further Exploration

The rest of the maps should work with a UAV simulation software, e.g., AirSim(Recommended), Gazebo, etc, and a matched map (Take AirSim for instance, a corresponding Unreal Engine project is required). The reqiured output of the UAV simulation software is a rostopic containing the pose of the UAV(data type is nav_msgs/Odometry or geometry_msgs/PoseStamped). If you decide to use AirSim, please follow [Create your own maps](Tutorials/CreateMap/CreateMap.md).