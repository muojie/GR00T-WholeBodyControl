# Sony mocopi / SMPL POSE v3 路线

本文只记录更接近 SONIC 原生人体 pose encoder 的路线。它和 [BVH-G1 POSE v1 路线](mocopi_pose_bvh_g1_v1_route.md) 是两条不同方案，不应互相覆盖。

## 定位

这条路线希望让非 PICO 输入源输出 SMPL-like full-body reference，再由 deploy 侧按 SMPL encoder 语义处理。

```text
BVH / mocopi skeleton
  -> normalized SMPL-like joints / local pose
  -> pose topic, protocol v3, encoder_mode=smpl
  -> deploy SMPL / motion reference observation
  -> policy
```

和 BVH-G1 POSE v1 的根本区别：

| 项目 | SMPL POSE v3 | BVH-G1 POSE v1 |
|------|--------------|----------------|
| 语义 | 人体 SMPL-like pose | G1 机器人关节参考 |
| retarget 位置 | 主要希望 deploy/encoder 侧理解人体 pose | manager 侧提前算好 G1 `joint_pos/joint_vel` |
| 优点 | 更接近 PICO full-body / SONIC 原生设计 | 工程可控、当前效果更稳定 |
| 难点 | 骨架比例、rest pose、坐标轴、observation 契约更敏感 | 不再让 deploy 自己理解人体语义 |

## 历史实现

早期 `--source bvh` `--control-mode pose` 会生成 SMPL-like full-body reference，并按 protocol v3 发送：

```text
smpl_joints
smpl_pose
body_pos
body_quat_w
joint_pos
joint_vel
frame_index
```

当时补过几个关键问题：

- protocol v2 缺少 `joint_pos/joint_vel`，release SMPL mode 里 wrist joint observation 不完整；
- 29 维 `joint_pos` 不能用全 0，至少要用 G1 默认站姿作为稳定基线；
- streamed motion 需要 `body_pos`，否则 root 高度可能退化到 `z=0`；
- POSE 模式应等第一条 window 发出后再发送 `start=True`。

## 当前实现

当前保留 BVH-G1 POSE v1 主线，同时新增 Sony/BVH SMPL POSE v3 实验线：

- `mocopi_source.py`：官方 mocopi binary 27 bone、只带 `joints/bones` 的 mocopi JSON、以及已带 `smpl_*` 的 JSON 都可以进入 `FullBodyReference`。
- `bvh_source.py`：新增统一的命名 skeleton frame -> SMPL-like `FullBodyReference` 构造入口。
- `bvh_g1_source.py`：继续输出原来的 G1 `joint_pos/joint_vel/body_pos/body_quat_w`，同时为 v3 构造非零 `smpl_joints/smpl_pose`。当前默认 `--bvh-g1-smpl-joints-source g1_fk`，即把 v1 已验证的 G1 FK 14 个关键点投影到 24 个 SMPL 槽位；`skeleton` 只作为旧 V3 原始 BVH skeleton 的 A/B 回退。
- `bvh_stream_source.py`：每个 UDP `bvh_stream_v1` frame 既做 BVH-to-G1 retarget，也按同一 `--bvh-g1-smpl-joints-source` 规则构造 SMPL-like `smpl_joints/smpl_pose`。
- `mocap_manager_server.py`：`pkl` 仍只允许 v1/g1；`bvh_g1` 和 `bvh_stream` 支持两条显式线：
  - v1 主线：`--pose-protocol-version 1 --pose-encoder-mode g1`
  - v3 实验线：`--pose-protocol-version 3 --pose-encoder-mode smpl --allow-sony-pose-v3`

这样 v1/G1 的稳定路径不被覆盖；v3 需要显式选择，避免误把主验证命令切到 SMPL encoder。

## 评估方法

不要直接进闭环盲调 v3。先离线用同一个 BVH 同时生成：

- v1 参考：BVH-G1 retarget 后的 G1 `joint_pos`，再用 G1 MJCF FK 算 14 个 body keypoints；
- v3 skeleton 旧线：BVH skeleton 直接填到 `smpl_joints`；
- v3 g1_fk 新线：把 v1 G1 FK keypoints 投影到 SMPL 24 joint 槽位。

评估脚本：

```bash
.venv_teleop/bin/python -u gear_sonic/scripts/evaluate_sony_pose_v3.py \
  --bvh-file /home/nolo/RAYNOS_Motion1.bvh \
  --json-out /tmp/sony_pose_v3_eval.json
```

脚本会输出关键点 RMSE、最佳 yaw/scale 对齐误差、肢段方向角、左右侧符号错位率、root 高度/姿态统计。`RAYNOS_Motion1.bvh` 全量 926 帧的当前结果：

| 方案 | direct RMSE | yaw+scale RMSE | 肢段角均值 | 左右侧错位 |
|------|-------------|----------------|------------|------------|
| `skeleton` | 0.195 m | 0.092 m | 45.2 deg | 98.5% |
| `g1_fk` | 0.000 m | 0.000 m | 0.006 deg | 0.0% |

这说明旧 v3 的主要问题不是几个 gain，而是原始 BVH skeleton 相对 v1 G1 参考有约 90 deg yaw/轴差和左右侧几何错位。默认改成 `g1_fk` 后，v3 的 `smpl_joints` 至少在 deploy 当前 release 实际使用的关键点几何上和 v1 对齐；如果要验证旧行为，显式加：

```bash
--bvh-g1-smpl-joints-source skeleton
```

### release V3 observation check

当前 deploy release 的 SMPL mode 实际消费：

```text
smpl_joints_10frame_step1
smpl_anchor_orientation_10frame_step1
motion_joint_positions_wrists_10frame_step1
```

`evaluate_sony_pose_v3.py` 现在会额外输出 `release_observations`，对这三项做离线近似评估：

- `smpl_joints_10frame_step1`：10 帧窗口内 keypoint 速度和 horizon delta；
- `smpl_anchor_orientation_10frame_step1`：按 deploy 的 first-frame heading removal 近似生成 6D anchor，并统计 root/anchor 角速度、10 帧 horizon 和 filter lag；
- `motion_joint_positions_wrists_10frame_step1`：统计 Sony 实际发出的 wrist `joint_pos[23..28]`，并和 SMPL pose 投影出的 PICO-like wrist joint 做差；
- `raw` / `filtered`：同一段 reference 在进入 `PoseStreamPublisher` filter 前后的对比。

离线 anchor 评估假设 reset 时机器人 base 为 identity；真实闭环还会叠加 deploy 当前 `base_quat`，因此它用于排查 reference 方向和时序风险，不等价于闭环稳定性结论。

`RAYNOS_Motion1.bvh` 926 帧结果显示：

- root tilt p95/max 为 `0/0 deg`，这段动作的 root/anchor 风险主要不是 roll/pitch，而是 yaw 速度和 filter lag。
- raw anchor 速度 p95/max 为 `401.5/581.7 deg/s`，10 帧 horizon p95/max 为 `72.7/100.0 deg`。
- `stable` profile 会把 anchor 速度压到 `50.1/50.2 deg/s`，10 帧 horizon 压到 `9.0/9.0 deg`，但 p95 anchor lag 达 `71.97 deg`。
- `responsive` profile 的 SMPL/wrist lag 很小（SMPL p95 `0.009 m`，wrist p95 `0.040 rad`），但快速 yaw 段仍会出现约 `72 deg` 的 p95 anchor lag。
- `off` 没有 filter lag，但 raw anchor/wrist 速度直接进入 deploy。
- Sony 当前 wrist `joint_pos` 与 SMPL pose 投影 wrist 的差异很大：总体 p95/max `2.254/2.692 rad`，roll 维 p95 约 `2.69 rad`，yaw 维 p95 约 `2.13 rad`。这是 Sony V3 与 PICO 之间仍需重点 A/B 的字段语义差异。

对比 filter profile：

```bash
.venv_teleop/bin/python -u gear_sonic/scripts/evaluate_sony_pose_v3.py \
  --bvh-file /home/nolo/RAYNOS_Motion1.bvh \
  --pose-filter-profile responsive \
  --json-out /tmp/sony_pose_v3_release_eval_responsive.json

.venv_teleop/bin/python -u gear_sonic/scripts/evaluate_sony_pose_v3.py \
  --bvh-file /home/nolo/RAYNOS_Motion1.bvh \
  --pose-filter-profile off \
  --json-out /tmp/sony_pose_v3_release_eval_off.json
```

当前建议：闭环 A/B 时不要只用默认 `stable`；至少并行测试 `responsive` 和 `off`。腕部差异需要等 PICO pose window 抓取后再决定是否让 Sony V3 runtime 使用 SMPL pose 投影 wrist 来替代 BVH->G1 retarget wrist。

## 为什么暂停为独立研究线

这条路线不是无效，而是待解决问题更多：

- BVH / mocopi 骨架比例和 SMPL 模型比例不一致；
- BVH local rotation channel 与 SMPL rest pose / axis convention 不一致；
- `Y-up -> Z-up`、root base rotation、heading 归一化必须和 PICO 路严格对齐；
- deploy 当前 release 配置里实际启用的 SMPL observation 需要重新确认；
- 下肢是否来自 `smpl_joints`、`smpl_pose` 还是 `joint_pos`，必须用日志和 observation 配置分开验证。

因此不要用 BVH-G1 的调参结论覆盖 SMPL v3，也不要把 SMPL v3 的字段契约要求反推到 G1 v1。

## 研究命令模板

`--source bvh` 是本地 SMPL debug 入口：

```bash
.venv_teleop/bin/python -u gear_sonic/scripts/mocap_manager_server.py \
  --source bvh \
  --bvh-file /home/nolo/RAYNOS_Motion1.bvh \
  --bvh-loop \
  --control-mode pose \
  --pose-window-size 80 \
  --pose-encoder-mode smpl \
  --pose-protocol-version 3 \
  --zmq-port 5556
```

`bvh_stream` 的 v3 实验入口：

```bash
.venv_teleop/bin/python -u gear_sonic/scripts/mocap_manager_server.py \
  --source bvh_stream \
  --bvh-stream-port 12352 \
  --control-mode pose \
  --pose-window-size 80 \
  --pose-encoder-mode smpl \
  --pose-protocol-version 3 \
  --allow-sony-pose-v3 \
  --bvh-g1-smpl-joints-source g1_fk \
  --zmq-port 5556

.venv_teleop/bin/python -u gear_sonic/scripts/bvh_stream_sender.py \
  --bvh-file /home/nolo/RAYNOS_Motion1.bvh \
  --host 127.0.0.1 \
  --port 12352 \
  --loop
```

`bvh_g1` 的 v3 实验入口：

```bash
.venv_teleop/bin/python -u gear_sonic/scripts/mocap_manager_server.py \
  --source bvh_g1 \
  --bvh-file /home/nolo/RAYNOS_Motion1.bvh \
  --bvh-loop \
  --control-mode pose \
  --pose-window-size 80 \
  --pose-encoder-mode smpl \
  --pose-protocol-version 3 \
  --allow-sony-pose-v3 \
  --bvh-g1-smpl-joints-source g1_fk \
  --zmq-port 5556
```

外置 tmux 启动工具等价写法：

```bash
~/tools/sony-isaaclab-sonic-launcher/launch_sony_isaaclab_closed_loop.py \
  --backend isaaclab \
  --input-source sony \
  --bvh-source bvh_stream \
  --sony-pose-line v3
```

使用前需要确认当前 deploy 的 protocol v3 字段契约和 observation 配置完全一致；如果只是验证当前效果，优先使用 BVH-G1 POSE v1 路线。

## 成功标准

SMPL v3 路线重新推进时，应单独验收：

1. `smpl_joints/smpl_pose/body_quat_w/body_pos` 的 shape、dtype、坐标系与 deploy 解析一致。
2. deploy 当前 `observation_config.yaml` 的 SMPL mode 实际消费项明确。
3. `evaluate_sony_pose_v3.py` 对同一段 BVH 的 skeleton/g1_fk 指标可复现，且解释任何进入闭环前的姿态相似度差异。
4. root/anchor、wrist `joint_pos` 和时序平滑能用 `release_observations` 独立量化。
5. 同一段 BVH 与 BVH-G1 POSE v1、PICO V3 做 A/B，对比动作保真度、稳定性和延迟。
