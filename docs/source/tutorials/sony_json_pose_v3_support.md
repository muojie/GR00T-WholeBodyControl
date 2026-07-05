# Sony JSON 一键脚本 POSE v3 协议支持

本文档说明如何在 Sony BoneData JSON 一键启动脚本中使用 POSE v3 协议（PICO SMPL 栈）。

## 概述

从 2026-07-05 开始，`launch_sonic_json_mujoco_closed_loop.sh` 和 `launch_sonic_json_isaaclab_closed_loop.sh` 两个一键启动脚本支持通过环境变量切换 POSE 协议版本：

- **v1 协议**（默认）：BVH-G1 路线，使用 `--pose-encoder-mode g1`
- **v3 协议**：Sony PICO SMPL 路线，使用 `--pose-encoder-mode smpl --allow-sony-pose-v3`

## 协议版本对比

| 特性 | POSE v1 | POSE v3 |
|------|---------|---------|
| 编码模式 | `g1` | `smpl` |
| 数据源 | BVH skeleton retarget | Sony BoneData → SMPL |
| 腕关节投影 | G1 原生 IK | PICO SMPL 腕投影 |
| 额外标志 | 无 | `--allow-sony-pose-v3` |
| 典型用途 | 通用 BVH 动作 | Sony mocopi/PICO VR 数据 |

## 基本使用

### MuJoCo 后端

```bash
cd /home/nolo/GR00T-WholeBodyControl-sony-json-stream-20260702

# 默认 v1 协议（向后兼容）
./scripts/launch_sonic_json_mujoco_closed_loop.sh ~/saveBoneData_Yup20260702.json

# 使用 v3 协议
POSE_PROTOCOL_VERSION=3 ./scripts/launch_sonic_json_mujoco_closed_loop.sh ~/saveBoneData_Yup20260702.json
```

### IsaacLab 后端

```bash
# 默认 v1 协议
./scripts/launch_sonic_json_isaaclab_closed_loop.sh ~/saveBoneData_Yup20260702.json

# 使用 v3 协议
POSE_PROTOCOL_VERSION=3 ./scripts/launch_sonic_json_isaaclab_closed_loop.sh ~/saveBoneData_Yup20260702.json

# 连接远程 Windows IsaacLab + v3 协议
POSE_PROTOCOL_VERSION=3 ./scripts/launch_sonic_json_isaaclab_closed_loop.sh \
  --no-isaaclab \
  --windows-ip 192.168.50.100 \
  ~/saveBoneData_Yup20260702.json
```

## 实现细节

### 自动参数切换

脚本会根据 `POSE_PROTOCOL_VERSION` 环境变量自动调整 `mocap_manager_server.py` 的参数：

**v1 协议（默认）：**
```bash
--source bvh_stream \
--bvh-stream-host 0.0.0.0 \
--bvh-stream-port 12362 \
--bvh-stream-bonedata-coordinate-frame left_handed_yup \
--bvh-stream-bonedata-position-scale 1.0 \
--bvh-stream-bonedata-input-quat-order xyzw \
--bvh-stream-bonedata-rotation-mode input \
--pose-encoder-mode g1 \
--pose-protocol-version 1
```

**v3 协议：**
```bash
--source sony_pico \
--bvh-stream-port 12362 \
--bvh-stream-bonedata-position-scale 1.0 \
--bvh-stream-bonedata-input-quat-order xyzw \
--pose-encoder-mode smpl \
--pose-protocol-version 3
```

**关键区别：**
- v3 使用 `--source sony_pico`（Sony → PICO SMPL 直通线）
- v3 **不使用** `coordinate-frame` 和 `rotation-mode`（PICO 栈内部处理）
- v3 直接调用 PICO 转换栈，与 PICO 头显同构

### 验证方法

### 检查 manager 日志

启动后，检查 manager 窗口或日志，确认使用了正确的 source 和 encoder：

```bash
# 查看实时输出
tmux attach-session -t sonic_json_yup_mujoco
# 然后切换到 manager 窗口（Ctrl+B 然后按 1）

# 或直接查看日志
tail -f logs/sony_json_mujoco_sonic_json_yup_mujoco/manager.log
```

**v1 协议日志特征：**
```
[MocapManager] ... encoder=g1 ... body_n=14 ...
```

**v3 协议日志特征：**
```
[MocapManager] ... encoder=smpl ... body_n=1 ... smpl_lspan=0.92m ...
```

**关键区别：**
- `body_n=14`：bvh_stream 使用 14 个 G1 FK 关键点
- `body_n=1`：sony_pico 使用 PICO 转换栈
- `smpl_lspan`：v1 约 0.66m，v3 约 0.92m（不同的转换路径）

### 检查 deploy 输出

deploy 窗口应该显示 SMPL 相关字段：

**v3 协议特征：**
```
Frame[10] (idx=1014) ... smpl_joints: [(0.000000, 0.000000, 0.000000)], 
smpl_pose: [(0.003524, -0.059697, -0.062969)]
```

## 常见场景

### Sony mocopi 数据验证

```bash
# Sony mocopi saveBoneData 通常使用 y-up 坐标系 + v3 协议
POSE_PROTOCOL_VERSION=3 \
COORDINATE_FRAME=left_handed_yup \
  ./scripts/launch_sonic_json_mujoco_closed_loop.sh ~/saveBoneData_Yup20260702.json
```

### PICO VR 实时流

```bash
# PICO VR 头显实时数据（通过 sony_bonedata_json_v1 格式发送）
POSE_PROTOCOL_VERSION=3 \
COORDINATE_FRAME=left_handed_yup \
  ./scripts/launch_sonic_json_mujoco_closed_loop.sh --receiver-only

# 在另一个终端手动启动 PICO sender
```

### 调试 v1 vs v3 差异

```bash
# 先用 v1 运行一遍
SESSION=test_v1 ./scripts/launch_sonic_json_mujoco_closed_loop.sh test.json

# 再用 v3 运行
SESSION=test_v3 POSE_PROTOCOL_VERSION=3 \
  ./scripts/launch_sonic_json_mujoco_closed_loop.sh test.json

# 对比两个会话的表现
tmux attach-session -t test_v1
tmux attach-session -t test_v3
```

## 兼容性说明

### 向后兼容

- 默认行为（不设置 `POSE_PROTOCOL_VERSION`）保持 v1 协议，与之前版本完全兼容
- 所有现有脚本和工作流无需修改即可继续使用

### 跨分支兼容

- `main` 分支暂不支持 v3 协议参数，需使用 `sony-pico-smpl-route` 分支
- v1 协议在所有分支通用

### 数据格式要求

无论 v1 还是 v3，输入 JSON 文件格式要求相同：

- `format`: `"sony_bonedata_json_v1"`
- `position`: 对象数组 `[{x, y, z}, ...]` 或嵌套列表 `[[x,y,z], ...]`
- `rotation`: 对象数组 `[{x, y, z, w}, ...]` 或嵌套列表 `[[x,y,z,w], ...]`
- 每帧 27 个关节（`joints_per_frame=27`）

## 相关文档

- [Sony BoneData JSON raw stream sender](sony_bonedata_json_stream_sender.md) - 输入文件格式详细说明
- [mocopi POSE SMPL v3 route](mocopi_pose_smpl_v3_route.md) - v3 协议技术细节
- [一键启动脚本使用指南](launch_scripts_usage_guide.md) - 通用参数和调试方法

## 2026-07-05 验证记录

### 测试环境

- 分支：`sony-pico-smpl-route`
- 文件：`/home/nolo/saveBoneData_Yup20260702.json`（7305 帧，27 关节）
- 后端：MuJoCo
- 协议：v3
- Source：`sony_pico`

### 验证结果

| 指标 | 结果 |
|------|------|
| json_sender 发送帧率 | 50.0 fps |
| manager 接收帧率 | 50.0 fps |
| manager source 模式 | `sony_pico` ✓ |
| manager encoder 模式 | `smpl` ✓ |
| manager body_n | `1`（PICO 栈）✓ |
| deploy protocol version | `3` ✓ |
| deploy encode mode | `2` (SMPL) ✓ |
| deploy SMPL 字段 | 正常输出 ✓ |
| 丢帧率 | 0 ✓ |
| 日志错误 | 无 ✓ |

### 日志片段

**manager.log:**
```
[MocapManager] recv_fps=50.0 recv=775 vr_3pt=no pose=sent:693 encoder=smpl 
q=[-0.79,0.67] dq_abs=0.07 wrist_abs=0.79 wrist_dq=0.07 wrist_margin=1.18 
body_n=1 lower_dq=0.00 smpl_lz=[-0.92,-0.08] smpl_lspan=0.92m 
smpl_lpose=0.12rad root_z=0.793m body0_z=0.793m root_tilt=0.01rad 
smpl_lag=0.002m pose_lag=0.00rad q_lag=0.00rad wrist_lag=0.00rad 
root_raw=0.01rad dropped=0 frame=774
```

**关键特征：**
- `body_n=1`：使用 sony_pico PICO 转换栈（区别于 bvh_stream 的 `body_n=14`）
- `smpl_lspan=0.92m`：SMPL 肢段长度（区别于 bvh_stream v3 的 0.66m）

**deploy.log:**
```
[ZMQEndpointInterface] active_protocol_version_=3
[ZMQEndpointInterface] result.motion->GetEncodeMode()=2
Frame[17] (idx=1545) joint_pos: [-0.312000, -0.312000], 
smpl_joints: [(-0.009844, -0.351365, 0.009359)], 
smpl_pose: [(-0.040017, -0.063219, 0.073330)]
```

所有组件正常运行，数据流畅传输，v3 协议（sony_pico 路线）验证通过 ✅
