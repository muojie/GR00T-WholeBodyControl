# Sony mocopi / BVH POSE 流支持追踪

本文档专门追踪 Sony mocopi / BVH 输入源对齐 PICO `pose` topic 的工作。短期策略是：`PLANNER_VR_3PT` 继续作为三点实时验证链路，先不继续扩展腿部 IK 或额外 planner 目标；下一步重点转向 full-body `POSE` 流，让非 PICO 输入源也能向 deploy 提供完整身体参考。

## 结论

PICO VR 侧有腿部 tracker，下肢数据不是没有进入系统。它主要通过 full-body / SMPL 数据进入 `pose` topic，而不是在 PICO manager 里被手写映射成 G1 hip / knee / ankle 关节。

当前应区分两条链路：

```text
PICO full-body / SMPL
  -> pose topic
  -> smpl_pose / smpl_joints / body_quat_w / joint_pos / joint_vel
  -> deploy motion reference / policy observation
```

```text
PICO or mocap VR3PT
  -> planner topic
  -> vr_3pt_position / vr_3pt_orientation
  -> PLANNER_VR_3PT
```

第一条链路可以携带下肢 tracker 融合后的 full-body 信息。第二条链路只表达左腕、右腕、头/颈三点，以及手部和 locomotion command，不应该被当作下肢复原路径。

## PICO 侧实际做了什么

PICO manager 通过 `xrt.get_body_joints_pose()` 读取 24 个 body joints。`compute_from_body_poses()` 会把这些 body joints 转成 SMPL local pose、SMPL joints 和 root orientation。

POSE stream 发送的数据包括：

```text
smpl_pose
smpl_joints
body_quat_w
joint_pos
joint_vel
vr_position
vr_orientation
frame_index
left_hand_joints
right_hand_joints
```

其中 `smpl_pose`、`smpl_joints`、`body_quat_w` 是 full-body 信息，包含 PICO 腿部 tracker 融合后的下肢信号。`joint_pos` 当前由 PICO manager 初始化为 29 维零向量后，只显式填了左右 wrist 的 6 个 G1 wrist 关节；它不是完整 G1 下肢 retarget 结果。

## deploy 侧为什么能用下肢信息

deploy 侧的 motion/reference 代码有 full-body 和 lower-body observation 项，例如：

```text
motion_joint_positions_lowerbody_10frame_step5
motion_joint_velocities_lowerbody_10frame_step5
motion_joint_positions_lowerbody_10frame_step1
motion_joint_velocities_lowerbody_10frame_step1
smpl_joints_lower_10frame_step1
smpl_pose_10frame_step1
```

因此 PICO 的下肢 tracker 数据更可能通过 `smpl_joints` / `smpl_pose` / reference observation 被 policy 使用，而不是通过 manager 手写 G1 下肢 IK 使用。

## 与当前 mocap manager 的差距

当前 `mocap_manager_server.py` 的主线是 `planner` topic：

```text
mocopi / BVH
  -> MocapFrame
  -> VR3PointRetargeter
  -> planner topic
  -> vr_3pt_position / vr_3pt_orientation
```

这条链路适合快速验证实时三点遥操作，但它天然不会携带完整下肢 tracker 信息。BVH source 已经能保留 pelvis、spine、head、shoulder、elbow、wrist 等关节，后续需要扩展为 full-body `pose` stream，才能更接近 PICO 的 full-body 数据路径。

## 当前实现

已新增非 PICO 输入源的第一版 `pose` topic 发布能力，重点用于 BVH 回放验证。

实现范围：

- `MocapFrame` 新增 `full_body` 字段，统一携带 `smpl_joints=(24,3)`、`smpl_pose=(21,3)`、`body_quat_w=(4,)`。
- `FullBodyReference` 同时携带 `joint_pos=(29,)`、`joint_vel=(29,)`。如果输入源没有显式提供 `joint_pos`，会按 PICO manager 的同一套 SMPL elbow/wrist 映射补齐左右 wrist 的 6 个 G1 wrist 关节，其余关节为 0，`joint_vel` 默认为 0。
- `BvhPlaybackSource` 会基于 BVH FK 结果生成 SMPL-like full-body reference。`smpl_joints` 会减去 root/pelvis 平移，并用 root 四元数逆旋回本地坐标，以接近 PICO 的 `smpl_joints_local`。
- `mocap_manager_server.py` 新增 `pose` topic publisher，默认按 deploy 支持的 protocol v3 发送：`smpl_joints`、`smpl_pose`、`body_quat_w`、`joint_pos`、`joint_vel`、`frame_index`。
- 新增 `--control-mode pose`，通过 `command` topic 发送 `planner=false`，让 deploy 的 `ZMQManager` 切到 streamed-motion / POSE 模式。
- 新增 `--enable-pose-stream`，可以在保持 `planner` 控制模式时额外发布 `pose` topic，便于抓包和数据流调试。
- JSON bridge 如果直接提供 `smpl_joints`、`smpl_pose`、`body_quat_w`，也会填入 `full_body` 并可进入 `pose` topic；如果额外提供 `joint_pos` / `joint_vel`，manager 会直接透传。

BVH POSE 回放示例：

```bash
.venv_teleop/bin/python gear_sonic/scripts/mocap_manager_server.py \
  --source bvh \
  --bvh-file /home/nolo/RAYNOS_Motion1.bvh \
  --bvh-loop \
  --bvh-fps 50 \
  --target-fps 50 \
  --control-mode pose \
  --pose-window-size 80 \
  --zmq-port 5556
```

MuJoCo release policy 的 SMPL mode 会读取未来帧 observation，`--pose-window-size 5` 太短，deploy 会频繁进入 `Motion streamed completed and waiting following motion`。当前 `--control-mode pose` 不显式设置窗口时默认使用 80 帧；BVH POSE 验证仍建议把 `--pose-window-size 80` 写在命令里，deploy 控制端应能看到 `Processing 80 frames` 和 `Merged streamed data: 80+ current-rate frames`。

BVH source 的 `frame_index` 是播放流单调编号，不再直接使用源 BVH 帧号。源文件帧号会进入日志的 `source_frame` 字段；当 `--bvh-loop` 回到文件开头时，`frame` 继续递增，`source_frame` 回到小值，从而避免 deploy merger 把 loop 当成旧数据或新会话。

如果只想在 planner/VR3PT 控制模式下同时发布 `pose` topic 做调试：

```bash
.venv_teleop/bin/python gear_sonic/scripts/mocap_manager_server.py \
  --source bvh \
  --bvh-file /home/nolo/RAYNOS_Motion1.bvh \
  --bvh-loop \
  --bvh-fps 50 \
  --target-fps 50 \
  --control-mode planner \
  --enable-pose-stream \
  --zmq-port 5556
```

## MuJoCo 无动作问题定位

第一版 BVH POSE publisher 使用了 protocol v2，只发送 `smpl_joints`、`smpl_pose`、`body_quat_w`、`frame_index`。这会被 deploy 解析为 SMPL encoder mode，但是 release 版 `observation_config.yaml` 的 `smpl` mode 还要求：

```text
motion_joint_positions_wrists_10frame_step1
```

这个 observation 来自 streamed motion 的 `joint_pos`。如果 POSE 包里没有 `joint_pos/joint_vel`，deploy 侧虽然能收到 SMPL 字段，但 wrist joint observation 缺失，策略输入不完整，表现就是 MuJoCo 没有明显动作，甚至启动控制后机器人倒下。

当前修正：

- POSE 默认协议改为 v3。
- v3 消息包含 `joint_pos=(N,29)` 和 `joint_vel=(N,29)`。
- `joint_pos` 的 wrist 6 维按 PICO manager 的 SMPL elbow/wrist 映射生成。
- POSE 模式下 manager 会等到第一条 pose 窗口实际发出后，才在 `command` topic 中发送 `start=True`。

因此重新测试时，建议同时重启终端 2 的 deploy 和终端 3 的 manager，避免 deploy 侧保留旧 protocol 状态。

当前没有把官方 mocopi UDP 27 bone 强行映射成 POSE。官方二进制包仍优先走 VR3PT；mocopi 要进入 POSE，需要后续补稳定的 mocopi 27 bone -> SMPL 24/21 映射，或由上游 bridge 直接输出 SMPL-like 字段。

## 下一步目标

短期不继续推进 `--enable-upper-body-ik` 的第二阶段，也不做腿部 IK。下一步目标是新增非 PICO 输入源的 `pose` topic 发布能力：

```text
BVH / mocopi full-body source
  -> normalized full-body skeleton
  -> smpl_pose / smpl_joints / body_quat_w
  -> joint_pos / joint_vel
  -> pose topic
  -> deploy streamed motion / policy observation
```

已完成：

- 梳理 PICO `pose` topic 的最小必需字段、shape、dtype 和帧窗口。
- 为 BVH source 输出可复用的 full-body reference 数据，而不仅是 VR3PT 三点。
- 新增 `pose` topic publisher，对齐 deploy protocol v3 的字段名和 dtype。
- 用 BVH 文件做离线回放 smoke test，确认 manager 能持续发布 POSE 窗口。
- 修正 MuJoCo 无动作问题：补齐 deploy SMPL mode 需要的 `joint_pos/joint_vel`，并延迟 POSE 模式的 `start=True`。

下一步：

1. 联合 MuJoCo deploy 验证 `ZMQManager` 侧能解码 protocol v3，并进入 streamed-motion。
2. 对 BVH -> SMPL-like 的关节映射做可视化和误差检查，尤其是 shoulder/collar、foot/toe、root heading。
3. 再决定是否需要把 mocopi 官方 27 bone 映射到 SMPL 24 joints，或先要求 bridge 输出中间格式。
4. POSE 路径稳定后，再评估是否回到上肢 IK、手腕细节或腿部 retarget。

## 暂不做的事

- 不把腿部 tracker 数据塞进 `planner` topic。
- 不在 manager 里手写 G1 下肢 IK。
- 不把 `PLANNER_VR_3PT` 当作 full-body motion replay。
- 不继续扩大 `--enable-upper-body-ik` 的职责，直到 POSE 流打通后再评估。

## 成功判断

第一阶段成功标准：

- 非 PICO 输入源可以发布 `pose` topic。已完成 BVH / JSON bridge 路径。
- deploy 侧能解码 `smpl_pose`、`smpl_joints`、`body_quat_w`、`joint_pos`、`joint_vel`，日志不报 shape / field 缺失。待 MuJoCo 联调确认。
- BVH / mocopi 的 full-body 数据能进入 motion/reference observation，而不是只剩 VR3PT 三点。
- `planner` topic 仍可作为三点实时验证链路独立运行。
