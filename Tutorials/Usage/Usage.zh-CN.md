中文 | [English](Usage.md)

# 使用说明

## 1. 加载可视化场景地图

点击 `Load Scene Mesh` 加载场景地图，随后可以在 3D 视图窗口中预览该场景。

## 2. Sionna 仿真设置

点击 `Simulation Settings` 打开仿真设置窗口。该窗口包含若干仿真配置项。

- ### Environment

Python 环境是运行 Sionna 仿真时使用的 Python 环境。请确保你选择的环境中已经安装了 `sionna` 包。

仿真场景 XML 是从 Blender 导出的 XML 文件，它是用于 Sionna 仿真的简化场景。RF 信息实际上是基于这个场景文件计算得到的。第 1 步中加载的场景文件仅用于可视化显示。

- ### SYS over RT

如果希望使用 OFDM 信号进行仿真，并获取服务基站和信道信息，请勾选 `Enable Sionna SYS link adaptation on top of RT`，然后填写参数，或直接使用默认值。此时会出现名为 `sys_observations` 的 rostopic。

- ### Beamforming_Beam selection

如果希望进行波束赋形，请勾选 `Enable TX beam codebook generation and selection`。默认模式会使每个 TX 天线从 Sionna 生成的码本中选择 8 个波束之一进行发送，并在每个时刻提供 oracle beam。此时会出现名为 `beam_observations` 和 `codebook` 的 rostopic。

## 3. 坐标系设置

点击 `Coordinate Frames`，加载用于描述 ROS 坐标系、3D 场景坐标系和 AirSim 坐标系之间变换关系的 JSON 文件。

GUI 中使用 3 个坐标系：

- ROS frame：UAV 位姿 topic 使用的坐标系。UAV 在该坐标系中的起点为 `(0,0,0)`。通常它是一个 NED 坐标系。

- 3D frame：3D 场景使用的坐标系。该坐标系可以在 Blender 加载 3D 场景时获得，也可以在 Unreal Engine 中获得。在 Unreal Engine 中，查看 `PlayerStart` 对象的坐标。Unreal Engine 中显示的坐标属于 3D frame，而它在 ROS frame 中对应的坐标为 `(0,0,0)`。由此可以得到 ROS frame 与 3D frame 之间的平移关系。3D 视图窗口中显示的坐标轴表示 3D frame 的方向，也可以由此得到 ROS frame 与 3D frame 之间的旋转关系。

- AirSim frame：Unreal Engine 中显示的坐标通常也处于 AirSim frame。因此在大多数情况下，3D frame 和 AirSim frame 是相同的。

## 4. 添加基站

在 `Controls/Live info` 面板的 `Bse Stations` 窗口中，点击 `Add`，然后在场景中希望放置基站的位置点击一次，基站就会被放置到点击的位置。当某个基站被选中时，也可以使用键盘编辑它的坐标。

如果要删除基站，请选中它并点击 `Deleted selected`。

如果要保存基站配置，可以点击 `Save JSON`，将场景中所有基站的配置保存到 JSON 文件。要加载 JSON 文件，请点击 `Load JOSM` 加载基站。

## 5. 选择 Sionna 仿真所需的必要 topic

Sionna 仿真的必要输入 topic 是 UAV 位姿 topic。启动外部仿真软件之后，例如 AirSim（推荐）、Gazebo 或其他软件，找到 UAV 位姿 topic。在 ROS1 输入 topic 窗口中，在 `Pose topic` 框里选择该 UAV 位姿 topic。

## 6. 启动 Sionna 仿真

点击 `Connect ROS1`，仿真将会启动。你可以在 3D 视图窗口中查看仿真结果。

## 7. UE Live view

如果不想只在 3D mesh 中查看仿真，Unreal Engine 可以提供更好的可视化体验。该功能要求 UAV 运行在 AirSim 中。

在 Unreal Engine 和 AirSim 中运行 UAV 仿真，然后在 GUI 中切换到 `UE Live View` 窗口。点击底部的 `Start Live View` 启动 UE Live View，之后无人机、轨迹、基站和射线都可以显示在 UE 场景中。

在 UE Live View 窗口中选择基站后，所选基站对应摄像机的图像会被显示出来。

## 8. Rosbag 工具

GUI 中也可以录制和回放 rosbag。在 `Rosbag Recording` 窗口中，点击 `Start Recording` 可以开始录制 rosbag，并将其输出到指定路径。

打开 Rosbag Tools 面板后，可以回放 rosbag。需要填写 rosbag 中 RF、SYS 和 beam topic 的名称。可以指定回放 topic 的类型，推荐选择 clock topic。点击 `Start Playbak` 回放 rosbag。

如果 rosbag 中包含 RF、SYS、beam 和 UAV 位姿 topic 中的任意一种，rosbag 中的 RF、SYS 或 beam 数据会自动播放。如果 rosbag 不包含这 3 类 topic，但包含 UAV 位姿 topic，则会播放 UAV 位姿 topic，此时可以点击 `Connect ROS1`，基于 rosbag 中的 UAV 位置启动仿真。

re-sim wireless 功能提供了一种方式，可以向缺少 RF、SYS 或 beam topic 的 rosbag 中补充这些 topic，也可以提高这些 topic 中消息的频率。

## 9. 保存配置文件

GUI 中设置的参数可以保存到配置文件中。点击 `Save Config` 可以将所有参数保存到配置文件。加载配置文件时，保存的所有参数都会被加载回 GUI。

## 10. RF 数据、SYS 数据和 beam 数据预览

这些数据可以在对应的 rostopic 中获取。要预览数据，可以打开 `Wireless Data` 和 `Sionna SYS` 面板。
