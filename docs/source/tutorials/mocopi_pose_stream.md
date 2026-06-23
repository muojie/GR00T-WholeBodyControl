# Sony mocopi / BVH POSE 流支持追踪

本文档专门追踪 Sony mocopi / BVH 输入源对齐 PICO `pose` topic 的工作。当前已经从早期 `SMPL / protocol v3` 试验转到稳定性更好的 `BVH -> G1 joint reference -> POSE v1` 路线；`PLANNER_VR_3PT` 仍保留为三点实时验证链路。

## 结论

PICO VR 侧有腿部 tracker，下肢数据不是没有进入系统。它主要通过 full-body / SMPL 数据进入 `pose` topic，而不是在 PICO manager 里被手写映射成 G1 hip / knee / ankle 关节。

当前应区分三条链路：

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

```text
BVH / mocopi full-body source
  -> manager 侧 G1 retarget
  -> pose topic, protocol v1, encoder_mode=g1
  -> streamed G1 joint reference / policy observation
```

第一条链路可以携带下肢 tracker 融合后的 full-body 信息。第二条链路只表达左腕、右腕、头/颈三点，以及手部和 locomotion command，不应该被当作下肢复原路径。第三条是当前 BVH 验证主线：先在 Python manager 侧完成 BVH-to-G1 retarget，再把 G1 关节参考窗口交给 deploy。

## PICO 侧实际做了什么

PICO manager 通过 `xrt.get_body_joints_pose()` 读取 24 个 body joints。`compute_from_body_poses()` 会把这些 body joints 转成 SMPL local pose、SMPL joints 和 root orientation。

POSE stream 发送的数据包括：

```text
smpl_pose
smpl_joints
body_pos
body_quat_w
joint_pos
joint_vel
vr_position
vr_orientation
frame_index
left_hand_joints
right_hand_joints
```

其中 `smpl_pose`、`smpl_joints`、`body_quat_w` 是 full-body 信息，包含 PICO 腿部 tracker 融合后的下肢信号。PICO manager 的 `joint_pos` 路径主要显式填左右 wrist 的 G1 wrist 关节；它不是完整 G1 下肢 retarget 结果。

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

## 当前有效实现：BVH-G1 POSE v1

当前推荐使用 `bvh_stream` 做在线验证。它把“输入源实时流”和“manager 到 deploy 的 POSE v1”拆开：

```text
BVH file
  -> bvh_stream_sender.py
  -> UDP bvh_stream_v1
  -> mocap_manager_server.py --source bvh_stream
  -> BvhStreamUdpSource
  -> BVH skeleton frame -> G1 29DOF retarget
  -> PoseStreamPublisher protocol v1, encoder_mode=g1
  -> deploy StreamedMotionMerger
```

实现范围：

- `MocapFrame` 新增 `full_body` 字段，统一携带 `smpl_joints=(24,3)`、`smpl_pose=(21,3)`、`body_quat_w=(4,)`，以及 root `body_pos_w=(3,)`。
- `BvhG1PlaybackSource` 和 `BvhStreamUdpSource` 会在 manager 侧把 BVH skeleton frame 重定向成 G1 29 维 `joint_pos/joint_vel`。
- `PoseStreamPublisher` 以 protocol v1 发送 `joint_pos`、`joint_vel`、`body_pos`、`body_quat_w`、`frame_index`、`catch_up`、`encoder_mode=g1`。
- 新增 `--control-mode pose`，通过 `command` topic 发送 `planner=false`，让 deploy 的 `ZMQManager` 切到 streamed-motion / POSE 模式。
- POSE 首包 bootstrap 默认开启，第一帧到来后立即发送一个 80 帧 hold window，让 deploy 不必等真实窗口累计满。

推荐 manager 命令：

```bash
.venv_teleop/bin/python -u gear_sonic/scripts/mocap_manager_server.py \
  --source bvh_stream \
  --bvh-stream-port 12352 \
  --control-mode pose \
  --pose-window-size 80 \
  --pose-encoder-mode g1 \
  --pose-protocol-version 1 \
  --zmq-port 5556
```

推荐 sender 命令：

```bash
.venv_teleop/bin/python -u gear_sonic/scripts/bvh_stream_sender.py \
  --bvh-file /home/nolo/RAYNOS_Motion1.bvh \
  --host 127.0.0.1 \
  --port 12352 \
  --loop
```

验证通过的运行指纹：

```text
control_mode=pose pose_stream=1 pose_protocol=v1 pose_window=80 pose_encoder=g1
listening for BVH stream UDP on 0.0.0.0:12352
pose=sent:1
recv_fps≈50 recv=N dropped=0
```

换 BVH 时只重启 `bvh_stream_sender.py`，MuJoCo、deploy 和 manager 都可以常驻。新的 sender 从 `frame_index=0` 开始时，deploy 的 streamed-motion merger 会触发 catch-up reset 到新窗口。MuJoCo 里需要清空机器人状态时，直接在 viewer 中按 `Backspace`。

如果只做单文件验证，也可以用 `--source bvh_g1 --bvh-file ...`。这个模式仍然是在线逐帧 retarget，但 manager 绑定具体 BVH 文件，换文件需要重启 manager。

## 历史 POSE / SMPL 试验

早期 `--source bvh --control-mode pose` 使用的是 SMPL-like full-body reference 和 protocol v3：`smpl_joints`、`smpl_pose`、`body_pos`、`body_quat_w`、`joint_pos`、`joint_vel`、`frame_index`。这条路线更接近 SONIC 原生 SMPL encoder 语义，但 BVH/SMPL 骨架比例、rest pose、root base rotation、坐标轴和下肢稳定性问题更多。当前不再把它作为主验证路径。

## MuJoCo 无动作问题定位

第一版 BVH POSE publisher 使用了 protocol v2，只发送 `smpl_joints`、`smpl_pose`、`body_quat_w`、`frame_index`。这会被 deploy 解析为 SMPL encoder mode，但是 release 版 `observation_config.yaml` 的 `smpl` mode 还要求：

```text
motion_joint_positions_wrists_10frame_step1
```

这个 observation 来自 streamed motion 的 `joint_pos`。如果 POSE 包里没有 `joint_pos/joint_vel`，deploy 侧虽然能收到 SMPL 字段，但 wrist joint observation 缺失，策略输入不完整，表现就是 MuJoCo 没有明显动作，甚至启动控制后机器人倒下。

历史修正：

- POSE SMPL 试验阶段默认协议改为 v3。
- v3 消息补齐 `joint_pos=(N,29)` 和 `joint_vel=(N,29)`。
- `joint_pos` 不再以 29 维全 0 作为默认值，而是以 G1 默认站姿作为稳定基线；已有 wrist 6 维仍按 PICO manager 的 SMPL elbow/wrist 映射生成。
- POSE 模式下 manager 会等到第一条 pose 窗口实际发出后，才在 `command` topic 中发送 `start=True`。

运行时日志会输出 POSE 诊断字段：

```text
pose=sent:24 q=[-1.15,0.98] dq_abs=0.00 lower_dq=0.00 smpl_lz=[-0.76,0.00] smpl_lspan=0.90m smpl_lpose=0.53rad root_z=0.793m root_tilt=0.53rad
```

字段含义：

| 字段 | 含义 |
|------|------|
| `q=[min,max]` | 当前发送的 29 维 `joint_pos` 范围 |
| `dq_abs` | 当前发送的 29 维 `joint_vel` 最大绝对值 |
| `lower_dq` | 下肢 12 个 G1 关节相对默认站姿的最大偏差；当前 BVH-G1 主线会显式生成下肢参考，不应长期接近 `0` |
| `smpl_lz` | SMPL 下肢 joints 在 root-local 坐标中的 z 范围 |
| `smpl_lspan` | SMPL 下肢 joints 的包围盒对角线长度，用于观察输入尺度 |
| `smpl_lpose` | SMPL 下肢 local pose rotvec 最大范数 |
| `root_z` | POSE `body_pos` 的 root 高度；默认 BVH 路径应为 `0.793m` |
| `root_tilt` | root quaternion 相对竖直方向的倾斜角 |

这组日志用于区分两类现象：历史 SMPL 路线里腿部跟着动，通常来自 `smpl_joints/smpl_pose` 被 policy 使用；而 `lower_dq=0.00` 表示当时并没有把 BVH 下肢显式重定向为 G1 hip/knee/ankle 关节角。当前 v1/G1 路线里，`lower_dq` 会直接反映 manager 侧生成的 G1 下肢关节参考偏离默认站姿的幅度。

当前没有把官方 mocopi UDP 27 bone 强行映射成 POSE。官方二进制包仍优先走 VR3PT；mocopi 要进入 POSE，需要后续补稳定的 mocopi 27 bone -> SMPL 24/21 映射，或由上游 bridge 直接输出 SMPL-like 字段。

## 下一步目标

当前阶段已经完成非 PICO 输入源的 `pose` topic 发布和在线 BVH stream 验证。下一步目标是把这条路径从 BVH 文件发送器推广到 mocopi 实时骨架输入：

```text
BVH stream / mocopi full-body source
  -> normalized skeleton frame
  -> manager 侧 G1 retarget
  -> joint_pos / joint_vel / body_pos / body_quat_w
  -> pose topic
  -> deploy streamed motion / policy observation
```

已完成：

- 梳理 PICO `pose` topic 的最小必需字段、shape、dtype 和帧窗口。
- 为 BVH source 输出可复用的 full-body reference 数据，而不仅是 VR3PT 三点。
- 新增 `pose` topic publisher；早期对齐 deploy protocol v3，当前 BVH-G1 主线使用 protocol v1 + `encoder_mode=g1`。
- 用 BVH 文件做离线回放 smoke test，确认 manager 能持续发布 POSE 窗口。
- 修正 MuJoCo 无动作问题：补齐 deploy SMPL mode 需要的 `joint_pos/joint_vel`，并延迟 POSE 模式的 `start=True`。
- 修正 POSE `joint_pos` 默认值：非 PICO 输入源没有显式 `joint_pos` 时，使用 G1 默认站姿作为稳定基线，而不是 29 维全 0。
- 修正 POSE root 高度：发送端新增 `body_pos`，默认 root 高度为 `0.793m`；deploy merger 对旧消息也用 `{0,0,0.793}` 保底，避免 streamed motion 的 `BodyPositions` 仍为 `z=0`。
- 新增 POSE 运行时诊断：记录下肢 SMPL 范围、下肢 joint_pos 相对默认站姿偏差、整体关节目标范围和 root 倾斜。
- 新增 `bvh_stream_sender.py` 和 `--source bvh_stream`，验证 BVH UDP stream 在线输入；manager、deploy、MuJoCo 可常驻，只重启 sender 即可换动作。
- 当前有效主线改为 `POSE v1 + encoder_mode=g1`，发送 G1 29 维关节参考窗口，而不是让 deploy 自己理解 BVH/SMPL。

下一步：

1. 把 `bvh_stream_v1` 的 canonical skeleton packet 抽象成 mocopi 实时输入也能复用的中间格式。
2. 为 mocopi 官方 27 bone 补完整 FK / 标定层，或要求上游 bridge 直接输出 normalized skeleton frame。
3. 增加 sender/manager 端延迟、丢包、source frame gap 的日志，量化实时输入稳定性。
4. 在当前 90% 效果基础上继续调 wrist、root heading、脚踝/脚尖段的细节保真度。
5. 稳定后再评估是否回到 SONIC 原生 `pose_protocol v3 + pose_encoder smpl` 路线做 A/B。

## 暂不做的事

- 不把腿部 tracker 数据塞进 `planner` topic。
- 不在 planner topic 里手写 G1 下肢 IK。
- 不把 `PLANNER_VR_3PT` 当作 full-body motion replay。
- 不把官方 mocopi 原始 UDP 包直接传给 deploy；必须先转成 canonical skeleton 或 G1 reference。

## 成功判断

第一阶段成功标准：

- 非 PICO 输入源可以发布 `pose` topic。已完成 BVH / JSON bridge 路径。
- deploy 侧能解码 `smpl_pose`、`smpl_joints`、`body_quat_w`、`joint_pos`、`joint_vel`，日志不报 shape / field 缺失。待 MuJoCo 联调确认。
- BVH / mocopi 的 full-body 数据能进入 motion/reference observation，而不是只剩 VR3PT 三点。
- `planner` topic 仍可作为三点实时验证链路独立运行。
