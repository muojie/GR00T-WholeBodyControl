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

## 下一步目标

短期不继续推进 `--enable-upper-body-ik` 的第二阶段，也不做腿部 IK。下一步目标是新增非 PICO 输入源的 `pose` topic 发布能力：

```text
BVH / mocopi full-body source
  -> normalized full-body skeleton
  -> smpl_pose / smpl_joints / body_quat_w
  -> optional joint_pos / joint_vel
  -> pose topic
  -> deploy streamed motion / policy observation
```

优先实现顺序：

1. 梳理 PICO `pose` topic 的最小必需字段、shape、dtype 和帧窗口。
2. 为 BVH source 输出可复用的 full-body reference 数据，而不仅是 VR3PT 三点。
3. 新增 `pose` topic publisher，先对齐 PICO 当前字段名和 dtype。
4. 用 BVH 文件做离线回放，确认 deploy 能解码 `smpl_pose`、`smpl_joints`、`body_quat_w`。
5. 再决定是否需要把 mocopi 官方 27 bone 映射到 SMPL 24 joints，或先用 bridge 输出中间格式。

## 暂不做的事

- 不把腿部 tracker 数据塞进 `planner` topic。
- 不在 manager 里手写 G1 下肢 IK。
- 不把 `PLANNER_VR_3PT` 当作 full-body motion replay。
- 不继续扩大 `--enable-upper-body-ik` 的职责，直到 POSE 流打通后再评估。

## 成功判断

第一阶段成功标准：

- 非 PICO 输入源可以发布 `pose` topic。
- deploy 侧能解码 `smpl_pose`、`smpl_joints`、`body_quat_w`，日志不报 shape / field 缺失。
- BVH / mocopi 的 full-body 数据能进入 motion/reference observation，而不是只剩 VR3PT 三点。
- `planner` topic 仍可作为三点实时验证链路独立运行。
