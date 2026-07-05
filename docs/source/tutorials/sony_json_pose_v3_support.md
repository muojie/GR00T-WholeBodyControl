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
--pose-encoder-mode g1 \
--pose-protocol-version 1
```

**v3 协议：**
```bash
--pose-encoder-mode smpl \
--pose-protocol-version 3 \
--allow-sony-pose-v3
```

### 错误处理

如果手动指定了不兼容的参数组合，manager 会报错：

```
error: --source bvh_stream supports either 
  --pose-protocol-version 1 --pose-encoder-mode g1, or 
  --pose-protocol-version 3 --pose-encoder-mode smpl --allow-sony-pose-v3
```

一键脚本会自动避免这个错误，无需手动指定 encoder mode 和额外标志。

## 验证方法

### 检查 manager 日志

启动后，检查 manager 窗口或日志，确认使用了正确的 encoder：

```bash
# 查看实时输出
tmux attach-session -t sonic_json_yup_mujoco
# 然后切换到 manager 窗口（Ctrl+B 然后按 1）

# 或直接查看日志
tail -f logs/sony_json_mujoco_sonic_json_yup_mujoco/manager.log
```

**v1 协议日志特征：**
```
[MocapManager] ... encoder=g1 ...
```

**v3 协议日志特征：**
```
[MocapManager] ... encoder=smpl ... smpl_lz=[-0.75,-0.13] smpl_lspan=0.66m ...
```

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

### 验证结果

| 指标 | 结果 |
|------|------|
| json_sender 发送帧率 | 50.0 fps |
| manager 接收帧率 | 50.0 fps |
| manager encoder 模式 | `smpl` ✓ |
| deploy SMPL 字段 | 正常输出 ✓ |
| 丢帧率 | 0 ✓ |
| 日志错误 | 无 ✓ |

### 日志片段

**manager.log:**
```
[MocapManager] recv_fps=50.0 recv=1998 vr_3pt=no pose=sent:1912 encoder=smpl 
q=[-1.00,1.00] dq_abs=0.11 wrist_abs=0.72 wrist_dq=0.11 wrist_margin=0.97 
body_n=14 lower_dq=0.37 smpl_lz=[-0.75,-0.13] smpl_lspan=0.66m 
smpl_lpose=0.15rad root_z=0.953m body0_z=0.953m root_tilt=0.00rad 
smpl_lag=0.001m pose_lag=0.00rad q_lag=0.00rad wrist_lag=0.00rad 
root_raw=0.00rad dropped=0 frame=1997
```

**deploy.log:**
```
Frame[10] (idx=1014) joint_pos: [-0.053581, -0.077066], 
joint_vel: [-0.008915, -0.005217], 
body_quat: [(0.999933, 0.000000, 0.000000, 0.011603)], 
smpl_joints: [(0.000000, 0.000000, 0.000000)], 
smpl_pose: [(0.003524, -0.059697, -0.062969)]
```

所有组件正常运行，数据流畅传输，v3 协议验证通过 ✅
