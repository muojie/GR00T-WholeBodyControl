# Sony mocopi / BVH POSE 路线总览

本文只做路线入口和对比，不承载具体部署步骤。两条路线是不同设计，不应互相覆盖。

## 路线区分

| 路线 | 文档 | 当前状态 | 核心含义 |
|------|------|----------|----------|
| BVH-G1 POSE v1 | [Sony mocopi / BVH-G1 POSE v1 路线](mocopi_pose_bvh_g1_v1_route.md) | 当前主验证路线 | manager 侧先把 BVH / canonical skeleton 重定向成 G1 `joint_pos/joint_vel`，deploy 只消费 G1 reference |
| SMPL POSE v3 | [Sony mocopi / SMPL POSE v3 路线](mocopi_pose_smpl_v3_route.md) | 已有实验入口，需显式选择 | manager 侧输出 SMPL-like `smpl_joints/smpl_pose`，deploy 侧按 SMPL encoder 语义消费 |
| PLANNER_VR_3PT | [Sony mocopi 动捕管理器](mocopi_mocap_manager.md) | 三点实时验证链路 | 只表达左右腕和头/颈三点，不是 full-body 复原路线 |

当前 MuJoCo 里效果较好的路径仍是 BVH-G1 POSE v1。SMPL POSE v3 不是被删除，也不覆盖 v1；现在它作为显式实验线存在，因为它解决的问题不同：它更接近 SONIC 原生人体 pose encoder，但需要处理 BVH/mocopi 到 SMPL 的骨架、rest pose、坐标轴和 observation 契约。

## PICO 下肢信息的含义

PICO VR 侧有腿部 tracker，下肢数据不是没有进入系统。它主要通过 full-body / SMPL 数据进入 `pose` topic，而不是在 PICO manager 里被手写映射成 G1 hip / knee / ankle 关节。

PICO full-body 路线可以概括为：

```text
PICO full-body / SMPL
  -> pose topic
  -> smpl_pose / smpl_joints / body_quat_w / joint_pos / joint_vel
  -> deploy motion reference / policy observation
```

三点 planner 路线是另一条链路：

```text
PICO or mocap VR3PT
  -> planner topic
  -> vr_3pt_position / vr_3pt_orientation
  -> PLANNER_VR_3PT
```

因此后续接 mocopi / BVH full-body 时，不应把下肢 tracker 数据塞进 `planner` topic；应该选择一条 POSE 路线。

## 当前建议

短期继续使用 [BVH-G1 POSE v1 路线](mocopi_pose_bvh_g1_v1_route.md) 做实时性和稳定性优化，因为它已经能在 MuJoCo 中稳定复原大部分动作，并支持 `bvh_stream_sender.py` 热切换 BVH。

SMPL POSE v3 作为独立实验线推进，不把它的结论覆盖到 v1/G1 文档中。`bvh_g1` / `bvh_stream` 若要走 v3，需要显式传 `--pose-protocol-version 3 --pose-encoder-mode smpl --allow-sony-pose-v3`；外置 tmux 工具对应 `--sony-pose-line v3`。只有当 mocopi/BVH 到 SMPL 的骨架语义和 deploy observation 契约重新验证清楚后，再做 A/B 对比。
