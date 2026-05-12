# Usage of Sample Maps(Quick Start for SimART)

4 sample maps and 1 sample rosbag for one of them are provided. The map BigCitySample can be used for a quick start while the rest can be used for further exploration.

## 1. Download the sample maps and rosbags

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

## 2. Load the config file in SimART and start the simulation

Run SimART by using the command:

```bash
rosrun airsim_gui_UErealtime airsim_gui_UErealtime
```

- In SimART, click "Open Existing Config", select BigCitySample->BigCitySample.agcfg in the downloaded sample maps.

- Then click "Simulation Settings", and select a python environment with sionna installed in python environment box.

- Click "Open Rosbag Tools", select the downloaded sample rosbag in Replay panel. Click "Start Playback".

- Click "Start Simulation" to start sionna simulation. Rf, sys and beam simulation will all be started. The data can be viewd at Wireless Data and Sionna SYS panels. The raw data can be obtained in rostopics.

## 3. Further exploration

The rest of the maps should work with a UAV simulation software, e.g., AirSim(Recommended), Gazebo, etc. The reqiured output of the UAV simulation software is a rostopic containing the pose of the UAV(data type is nav_msgs/Odometry or geometry_msgs/PoseStamped). If you decide to use AirSim, please follow xxx.