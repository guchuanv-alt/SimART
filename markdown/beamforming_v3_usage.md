# Beamforming & Beam Selection 使用说明

当前 beam selection 流程包含两块能力：

1. **导出你自己场景里的 beam 训练样本**
   - 仍然由 Sionna 码本 + oracle sweep 给标签
   - 导出为 CSV
2. **用导出的 CSV 训练 beam-selection MLP**
   - 训练完成后生成新的 checkpoint
   - GUI 会自动把 checkpoint 路径填回 predictor 配置

## 推荐流程

### 第一步：导出你自己场景的数据

在 Simulation Settings 里：

- Enable TX beam codebook generation and selection = 打开
- Selection mode = `exhaustive_sweep`
- 配置好你的真实 TX 阵列、码本类型、oversampling
- 勾选 `Export oracle beam samples for training`
- 设置 `Training dataset CSV`
- 如需重新采样，勾选 `Reset beam dataset file when the simulator starts`

然后启动内部仿真，让无人机在你的场景里飞行一段时间。

导出的 CSV 每一行都包含：

- 特征向量 `feature_0..feature_N`
- `oracle_beam_index`
- `num_beams`
- `codebook_type`
- TX/RX 坐标
- BS 索引与名称

### 第二步：训练 beam-selection 模型

在同一个设置面板里：

- 填 `Checkpoint output`
- 设置训练超参数：epochs / batch size / hidden dim / learning rate / validation split
- 点击 `Train beam-selection model from dataset now...`

训练成功后：

- 会生成一个新的 checkpoint
- 还会生成一个同名 `.json` 训练摘要
- GUI 会自动：
  - 把 `Beam selection checkpoint` 更新成新的输出路径
  - 把 `Selection mode` 切换成 `learned_selector`

### 第三步：应用训练好的模型

重新启动内部仿真，保持：

- Selection mode = `learned_selector`
- Beam selection checkpoint = 你刚训练出来的 checkpoint

这时 GUI 会在 SYS Data Window / Raw JSON / beam-log 中显示：

- `Pred Beam`
- `Oracle Beam`
- `Hit`
- `Oracle in TopK`
- `Predictor status`

## 训练数据注意事项

- 一个训练数据集最好只对应**一种固定码本配置**
  - 同一个 `num_beams`
  - 同一个 `codebook_type`
  - 同一个阵列规模/oversampling
- 如果你改了 TX 阵列或 oversampling，建议重新导出新数据集
- 想正式部署时，建议使用你自己场景导出的数据训练得到的 checkpoint

## 最推荐的第一版做法

- 先用 `geom_vel_path13`
- 如果你想做 4 维输入的对照实验，可以改用 `pos_bbox4`
- 先固定一种阵列和一种码本
- 先跑 `exhaustive_sweep`
- 采一批自己的场景数据
- 训练一个自己的 checkpoint
- 再切到 `learned_selector`

这样最稳。
