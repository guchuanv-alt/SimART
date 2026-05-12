# Build and Compile Method

This project is a ROS1 catkin workspace package set. The repository should be
placed under a catkin workspace, for example:

```bash
mkdir -p ~/catkin_ws/src
cd ~/catkin_ws/src
git clone <this-repository-url> SimART
```

The repository contains these catkin packages:

- `airsim_gui_UErealtime`: the C++/Qt/VTK GUI package
- `rf_msgs`: RF observation message definitions
- `sionna_sys_msgs`: Sionna SYS message definitions
- `sionna_beam_msgs`: Sionna beam/codebook message definitions

## 1. System and ROS Dependencies

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

## 2. Python Dependencies

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

Note: `requirements.txt` only installs Python packages. ROS, Qt, VTK, Assimp,
and compiler dependencies must be installed with `apt` and `rosdep`.

## 3. Build the Default GUI

The default build does not require the AirSim C++ SDK.

```bash
cd ~/catkin_ws
source /opt/ros/noetic/setup.bash
catkin_make
source devel/setup.bash
```

After a successful build, run the GUI with:

```bash
rosrun airsim_gui_UErealtime airsim_gui_UErealtime
```

If the GUI needs ROS topics at runtime, start `roscore` in another terminal
before running the application:

```bash
roscore
```

## 4. Optional: Enable AirSim C++ Live View

The normal build can load local meshes and use ROS topics without the AirSim
C++ SDK. Enable AirSim C++ support only when you need UE live-view integration.

First make sure AirSim has been cloned and built, including its RPC library.
Then rebuild the catkin workspace with:

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
catkin_make -DAIRSIM_GUI_ENABLE_AIRSIM=ON
```

## 5. Clean Rebuild

If CMake cache settings changed, remove the previous catkin build products and
compile again:

```bash
cd ~/catkin_ws
rm -rf build devel
source /opt/ros/noetic/setup.bash
catkin_make
source devel/setup.bash
```

## 6. Quick Build Checklist

```bash
source /opt/ros/noetic/setup.bash
cd ~/catkin_ws
rosdep install --from-paths src/SimART --ignore-src -r -y
python3 -m pip install --user -r src/SimART/requirements.txt
catkin_make
source devel/setup.bash
rosrun airsim_gui_UErealtime airsim_gui_UErealtime
```
