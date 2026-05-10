# airsim_gui_UErealtime

3D desktop GUI demo for AirSim / ROS1 / beam-path visualization using C++, Qt5, VTK, and Assimp.

## What is new in this version

This version adds a unified scene import layer with material-aware rendering:

- **Load Scene Mesh**: import a local `OBJ`, `FBX`, or `STL` file as the 3D map.
- **Material-aware rendering**: load diffuse/base-color textures when present, otherwise use the imported material color.
- **Opaque scene rendering**: imported buildings are rendered fully opaque to avoid the washed-out translucent look.
- **Scene Manager abstraction**: the main window keeps the same scene-loading pipeline for local meshes while UE live view continues to come directly from AirSim.

## Dependencies

On Ubuntu, install Assimp before building:

```bash
sudo apt update
sudo apt install libassimp-dev
```

## Default build (no AirSim SDK required)

```bash
cd ~/catkin_ws
catkin_make
source devel/setup.bash
rosrun airsim_gui_UErealtime airsim_gui_UErealtime
```

In the default build:

- local `OBJ` / `FBX` / `STL` scene loading works
- ROS1 pose/path visualization works
- demo source works
- AirSim UE live view remains disabled until the project is rebuilt with AirSim C++ support

## Enable AirSim C++ live-view support

This requires an AirSim checkout with the C++ client headers available. In this version, UE live-view camera/image fetching uses the AirSim C++ RPC client directly.

```bash
cd ~/catkin_ws
catkin_make -DAIRSIM_GUI_ENABLE_AIRSIM=ON -DAIRSIM_CLIENT_ROOT=/path/to/AirSim
source devel/setup.bash
```

You can also set `AIRSIM_CLIENT_ROOT=/path/to/AirSim` in the environment before running `catkin_make`.

After that, use **Start Live View** in the right-side AirSim panel to connect the GUI to AirSim.

## Mock ROS publishers

```bash
roscore
python ~/catkin_ws/src/airsim_gui_UErealtime/scripts/mock_publishers.py
```

Then run the GUI and click **Connect ROS1**.


## UE Live View (C++ RPC)

The UE Live View tab now uses a native C++ AirSim RPC polling loop instead of the previous Python helper.
It continuously calls simSetCameraPose and simGetImages from the Qt application, so no Python airsim package is required.


## External preview camera for UE Live View
For a preview camera that does not follow the UAV, define the GUI preview camera as an external camera in AirSim settings instead of placing it under Vehicles/PX4/Cameras. Example:

```json
"ExternalCameras": {
  "GuiPreview": {
    "CaptureSettings": [
      {
        "ImageType": 0,
        "Width": 1280,
        "Height": 720,
        "FOV_Degrees": 90
      }
    ],
    "X": -8.0, "Y": 0.0, "Z": -2.0,
    "Pitch": 10, "Roll": 0, "Yaw": 0
  }
}
```

Then use `Camera = GuiPreview` in the GUI. `Vehicle` can still be set to `PX4` only for initial focus placement, but the live preview itself will be fetched and controlled as an external camera in world coordinates.

## Base-station camera previews

This version also supports a per-base-station UE preview window:

- Selecting a base station in the 3D editor or UE Live View shows that station's camera window on the left side.
- Each station preview is a separate dock window. It can be pinned, floated, dragged back to the left dock area, closed, and reset independently.
- Unpinned station windows hide automatically when the station is deselected.
- Each station preview now uses its own dedicated AirSim camera name (for example `bs_alpha_camera`) and the GUI keeps that camera fixed at the base-station position, so it does not follow the UAV or reuse `GuiPreview`.

### Optional base-station JSON fields

Each base station can now include these optional fields:

```json
{
  "id": "bs_1",
  "name": "bs_1",
  "position": [0.0, 0.0, 27.0],
  "color": [0.95, 0.55, 0.20],
  "preview_camera_name": "bs_1_camera",
  "preview_ros_topic": "/airsim_gui/station_cameras/bs_1/image_raw",
  "preview_offset_z": 0.0,
  "preview_fps": 4.0
}
```

- `preview_camera_name`: AirSim camera name used for that station preview. If omitted, the GUI derives a dedicated default like `<station_id>_camera`. Do not reuse `GuiPreview` for station previews.
- `preview_ros_topic`: ROS topic to publish the preview image. If omitted, the GUI publishes to `/airsim_gui/station_cameras/<station_id>/image_raw`.
- `preview_offset_z`: optional vertical offset added to the base-station position when placing the preview camera.
- `preview_fps`: preview refresh rate for that station window.

### Resolution note

Station preview resolution still comes from AirSim `settings.json`, not from the GUI. Create one dedicated external camera per base station in `ExternalCameras`, and increase the `CaptureSettings` resolution for each station camera if the preview looks blurry.


### Recommended AirSim settings pattern for station cameras

Define one external camera per base station in `settings.json`, for example `bs_alpha_camera`, `bs_bravo_camera`, and `bs_charlie_camera`. The GUI uses those names for the station preview windows and publishes their images to ROS topics such as `/airsim_gui/station_cameras/bs_alpha/image_raw`.

The station preview implementation no longer expects `GuiPreview` to be reused for base stations. `GuiPreview` should remain reserved for the main overview/live-map camera.
