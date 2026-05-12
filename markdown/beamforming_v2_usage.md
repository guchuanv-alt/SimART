# Beamforming V2 使用说明

## 目标
- 码本生成与 oracle 穷举：使用 Sionna
- 波束预测：使用 beam-selection MLP

## GUI 入口
Simulation Settings -> Beamforming & Beam Selection

## 模式
- `off`：关闭 beamforming
- `exhaustive_sweep`：Sionna 码本全部扫一遍，直接选 oracle beam
- `manual_index`：固定选某个 beam index
- `learned_selector`：先跑学习式 beam selector 预测，再与 oracle beam 对比

## learned selector 需要什么
1. 一个 PyTorch checkpoint
2. checkpoint 的输出维度必须等于当前 Sionna codebook 的 beam 数
3. 当前实现默认使用 13 维输入特征 `geom_vel_path13`，同时保留旧的 4 维模式兼容

### 当前支持的特征模式
- `geom_vel_path13` = `[dx_m, dy_m, dz_m, distance_3d_m, azimuth_rad, elevation_rad, rx_vx_mps, rx_vy_mps, rx_vz_mps, num_paths, strongest_path_power_db, tau_std_s, doppler_hz]`
- `pos_bbox4` = `[pos_distance, pos_angle, bbox_distance, bbox_angle]`
  说明：其中 `bbox_distance/bbox_angle` 不是跑真实图像检测，而是把 UAV 的仿真位置投影到基站代理相机平面后生成的 2 维视觉代理特征
- `geom4` = `[distance_xy_m, distance_3d_m, azimuth_rad, elevation_rad]`
- `rel_xyzd` = `[dx_m, dy_m, dz_m, distance_3d_m]`
- `rel_xyazel` = `[dx_m, dy_m, azimuth_rad, elevation_rad]`

## 在线效果在哪里看
启动内部仿真后，打开 SYS 数据窗口：
- Serving Link 卡片
- Candidate Base Stations 表格
- Raw SYS Snapshot (JSON)

其中会看到：
- Predictor status
- Pred Beam / Pred Conf
- Oracle in TopK / Hit
- TopK
- Source

## 重要说明
如果 checkpoint 缺失、加载失败，或者输出 beam 数与当前 Sionna 码本不一致，
系统会自动 fallback 到 oracle beam，并在 Predictor status / Source 中显示原因。
