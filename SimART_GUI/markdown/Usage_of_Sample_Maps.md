# Usage of Sample Maps(Quick Start for SimART)

4 sample maps are provided. They can be used for a quick start. 1 of them is a virtual map while 3 of them are real maps.

## 1. Download the sample maps

In the root directory of the repository you just cloned, run:

```bash
chmod +x download_sample_maps.sh
./download_sample_maps.sh
```

The folder SimART_sample_maps contains the 4 sample maps.

## 2. Load the config file in SimART

Run SimART by using the command:

```bash
rosrun xxxx xxxx
```

In SimART, click "Open Existing Config", select one of the config files in the sample maps folder. For example, BigCitySample->BigCitySample.agcfg. Then click "Simulation Settings", and select a python environment with sionna in python environment box.

Now run the simulation with the same map in AirSim or other simulation softwares, make sure that there is a rostopic indicating the UAV pose.

Click "Start Simulation" to start sionna simulation. Rf, sys and beam simulation will all be started. The data can be previewd at Wireless Data and Sionna SYS panels. The raw data can be obtained in rostopics.