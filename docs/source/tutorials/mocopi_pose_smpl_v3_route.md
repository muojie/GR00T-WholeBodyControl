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
- `bvh_g1_source.py`：继续输出原来的 G1 `joint_pos/joint_vel/body_pos/body_quat_w`，同时用同一 BVH skeleton 填充非零 `smpl_joints/smpl_pose`。
- `bvh_stream_source.py`：每个 UDP `bvh_stream_v1` frame 既做 BVH-to-G1 retarget，也同步构造 SMPL-like `smpl_joints/smpl_pose`。
- `mocap_manager_server.py`：`pkl` 仍只允许 v1/g1；`bvh_g1` 和 `bvh_stream` 支持两条显式线：
  - v1 主线：`--pose-protocol-version 1 --pose-encoder-mode g1`
  - v3 实验线：`--pose-protocol-version 3 --pose-encoder-mode smpl --allow-sony-pose-v3`

这样 v1/G1 的稳定路径不被覆盖；v3 需要显式选择，避免误把主验证命令切到 SMPL encoder。

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
3. root 高度、root heading、下肢 local pose 和 wrist joint observation 都能独立量化。
4. 同一段 BVH 与 BVH-G1 POSE v1 做 A/B，对比动作保真度、稳定性和延迟。
