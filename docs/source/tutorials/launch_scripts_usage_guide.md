# Sony JSON 一键启动脚本使用指南

本文档汇总 `launch_sonic_json_mujoco_closed_loop.sh` 和 `launch_sonic_json_isaaclab_closed_loop.sh` 两个一键启动脚本的常用命令和调试技巧。

## 快速开始

### MuJoCo 后端（最快，适合快速验证）

```bash
cd /home/nolo/GR00T-WholeBodyControl-sony-json-stream-20260702

# 基础启动
./scripts/launch_sonic_json_mujoco_closed_loop.sh ~/saveBoneData_Yup20260702.json

# 使用 v3 协议
POSE_PROTOCOL_VERSION=3 ./scripts/launch_sonic_json_mujoco_closed_loop.sh ~/saveBoneData_Yup20260702.json
```

### IsaacLab 后端（完整物理仿真）

```bash
# 本地 IsaacLab
./scripts/launch_sonic_json_isaaclab_closed_loop.sh ~/saveBoneData_Yup20260702.json

# 连接远程 Windows IsaacLab
./scripts/launch_sonic_json_isaaclab_closed_loop.sh \
  --no-isaaclab \
  --windows-ip 192.168.50.100 \
  ~/saveBoneData_Yup20260702.json
```

## 常用调试命令

### 1. 分步调试模式

#### 只启动接收端（receiver），手动控制发送端

```bash
# 启动 receiver（manager + deploy + mujoco/isaaclab）
./scripts/launch_sonic_json_mujoco_closed_loop.sh --receiver-only

# 在另一个终端手动启动 JSON sender
./scripts/launch_sonic_json_mujoco_closed_loop.sh --sender-only ~/saveBoneData_Yup20260702.json

# 或打印 sender 命令自己调整
./scripts/launch_sonic_json_mujoco_closed_loop.sh --print-sender-command ~/saveBoneData_Yup20260702.json
```

**适用场景：**
- 调试 JSON 数据格式
- 测试不同 JSON 文件
- 调整 sender 参数（FPS、循环播放等）

#### 连接外部 IsaacLab（跨机器）

```bash
# 本机运行 input/proxy/deploy，连接远程 IsaacLab
./scripts/launch_sonic_json_isaaclab_closed_loop.sh \
  --no-isaaclab \
  --windows-ip 192.168.50.100 \
  ~/saveBoneData_Yup20260702.json
```

**适用场景：**
- 连接 Windows GPU 机器的 IsaacLab
- 分离仿真和控制逻辑调试

### 2. 坐标系/协议调试

```bash
# Sony PICO 原生坐标系（y-up）
COORDINATE_FRAME=left_handed_yup ./scripts/launch_sonic_json_mujoco_closed_loop.sh file.json

# SONIC 内部坐标系（z-up）
COORDINATE_FRAME=sonic_zup ./scripts/launch_sonic_json_mujoco_closed_loop.sh file.json

# 使用 POSE v3 协议（PICO SMPL 栈）
POSE_PROTOCOL_VERSION=3 ./scripts/launch_sonic_json_mujoco_closed_loop.sh file.json

# 组合：v3 + y-up 坐标系
POSE_PROTOCOL_VERSION=3 COORDINATE_FRAME=left_handed_yup \
  ./scripts/launch_sonic_json_mujoco_closed_loop.sh file.json
```

### 3. 端口冲突解决

```bash
# 自定义端口（避免冲突）
BVH_STREAM_PORT=22362 MOCAP_ZMQ_PORT=6656 DEBUG_PORT=6657 \
  ./scripts/launch_sonic_json_mujoco_closed_loop.sh file.json

# 或修改 session 名（多实例并行）
SESSION=my_test ./scripts/launch_sonic_json_mujoco_closed_loop.sh file.json
```

### 4. 后端切换

```bash
# 统一入口脚本，通过 --backend 切换
./scripts/launch_sonic_json_mujoco_closed_loop.sh --backend mujoco file.json
./scripts/launch_sonic_json_mujoco_closed_loop.sh --backend isaaclab file.json

# 或使用快捷别名
./scripts/launch_sonic_json_mujoco_closed_loop.sh --mujoco file.json
./scripts/launch_sonic_json_mujoco_closed_loop.sh --isaaclab file.json
```

## Tmux 会话管理

### 查看和附加会话

```bash
# 查看当前 tmux 会话
tmux ls

# 附加到 MuJoCo 会话
tmux attach-session -t sonic_json_yup_mujoco

# 附加到 IsaacLab 会话
tmux attach-session -t sonic_json_isaaclab
```

### Tmux 快捷键

在 tmux 会话内：
- **切换窗口**：`Ctrl+B` 然后按 `0/1/2/3`（窗口编号）
- **下一个窗口**：`Ctrl+B` 然后按 `n`
- **上一个窗口**：`Ctrl+B` 然后按 `p`
- **列出所有窗口**：`Ctrl+B` 然后按 `w`
- **分离会话**（不停止）：`Ctrl+B` 然后按 `d`

### 停止会话

```bash
# 停止 MuJoCo 会话
tmux kill-session -t sonic_json_yup_mujoco

# 停止 IsaacLab 会话
tmux kill-session -t sonic_json_isaaclab

# 强制替换已存在会话（脚本默认已开启 REPLACE=1）
REPLACE=1 ./scripts/launch_sonic_json_mujoco_closed_loop.sh file.json
```

## 日志查看

### 实时日志

```bash
# MuJoCo 后端日志目录
cd ~/GR00T-WholeBodyControl-sony-json-stream-20260702/logs/sony_json_mujoco_sonic_json_yup_mujoco

# 实时查看 manager 日志
tail -f manager.log

# 实时查看 deploy 日志
tail -f deploy.log

# 实时查看 json_sender 日志
tail -f json_sender.log

# IsaacLab 后端日志（临时目录）
tail -f /tmp/sonic_local_*.log
```

### 日志搜索

```bash
# 搜索错误
grep -i error logs/sony_json_mujoco_*//*.log

# 搜索特定帧号
grep "frame=100" logs/sony_json_mujoco_*/manager.log

# 检查丢帧
grep "dropped=" logs/sony_json_mujoco_*/manager.log | grep -v "dropped=0"
```

## 调试工作流推荐

### 新数据验证流程

```bash
# 1. 先用 MuJoCo 快速验证数据格式正确性
./scripts/launch_sonic_json_mujoco_closed_loop.sh new_data.json

# 2. 切换到 v3 协议测试（如果是 Sony/PICO 数据）
POSE_PROTOCOL_VERSION=3 ./scripts/launch_sonic_json_mujoco_closed_loop.sh new_data.json

# 3. 验证通过后，用 IsaacLab 完整测试
POSE_PROTOCOL_VERSION=3 ./scripts/launch_sonic_json_isaaclab_closed_loop.sh new_data.json
```

### 分步排查问题

```bash
# 1. 只启动 receiver
./scripts/launch_sonic_json_mujoco_closed_loop.sh --receiver-only

# 2. 另一个终端测试单帧发送
./scripts/launch_sonic_json_mujoco_closed_loop.sh --print-sender-command test.json
# 复制输出的命令，修改参数后手动执行

# 3. 观察 manager/deploy 日志判断问题
tmux attach-session -t sonic_json_yup_mujoco
# 切换到 manager 窗口查看实时输出
```

### 对比 v1 vs v3 协议

```bash
# 先用 v1 运行一遍
SESSION=test_v1 ./scripts/launch_sonic_json_mujoco_closed_loop.sh test.json

# 再用 v3 运行
SESSION=test_v3 POSE_PROTOCOL_VERSION=3 \
  ./scripts/launch_sonic_json_mujoco_closed_loop.sh test.json

# 对比两个会话的表现
tmux attach-session -t test_v1   # 在一个终端
tmux attach-session -t test_v3   # 在另一个终端
```

## 常用环境变量组合

### Sony PICO v3 标准配置

```bash
POSE_PROTOCOL_VERSION=3 \
COORDINATE_FRAME=left_handed_yup \
BONEDATA_INPUT_QUAT_ORDER=xyzw \
  ./scripts/launch_sonic_json_mujoco_closed_loop.sh file.json
```

### 调试模式（慢速播放 + 更频繁日志）

```bash
FPS=30 \
  ./scripts/launch_sonic_json_mujoco_closed_loop.sh file.json
```

### 跨机器调试（本机 deploy，远程 IsaacLab）

```bash
NO_ISAACLAB=1 \
ISAAC_STATE_HOST=192.168.50.100 \
  ./scripts/launch_sonic_json_isaaclab_closed_loop.sh file.json
```

## 环境变量完整列表

### MuJoCo 后端

| 环境变量 | 默认值 | 说明 |
|---------|--------|------|
| `SESSION` | `sonic_json_yup_mujoco` | tmux 会话名 |
| `BACKEND` | `mujoco` | 后端类型（mujoco/isaaclab）|
| `MODE` | `all` | 运行模式（all/receiver/sender）|
| `JSON_FILE` | `/home/nolo/saveBoneData_Yup20260702.json` | JSON 文件路径 |
| `BVH_STREAM_PORT` | `12362` | UDP 流端口 |
| `MOCAP_ZMQ_PORT` | `5656` | manager ZMQ 端口 |
| `DEBUG_PORT` | `5657` | deploy debug ZMQ 端口 |
| `FPS` | `50` | 播放帧率 |
| `COORDINATE_FRAME` | `left_handed_yup` | 坐标系转换模式 |
| `BONEDATA_POSITION_SCALE` | `1.0` | 位置缩放 |
| `BONEDATA_INPUT_QUAT_ORDER` | `xyzw` | 四元数顺序 |
| `BONEDATA_ROTATION_MODE` | `input` | 旋转处理模式 |
| `POSE_PROTOCOL_VERSION` | `1` | POSE 协议版本（1/3）|
| `REPLACE` | `1` | 自动替换已存在的会话 |

### IsaacLab 后端（额外参数）

| 环境变量 | 默认值 | 说明 |
|---------|--------|------|
| `SESSION` | `sonic_json_isaaclab` | tmux 会话名 |
| `BVH_STREAM_PORT` | `12352` | UDP 流端口 |
| `MOCAP_ZMQ_PORT` | `5556` | manager ZMQ 端口 |
| `DEBUG_PORT` | `5557` | deploy debug ZMQ 端口 |
| `STATE_PORT` | `5560` | IsaacLab state ZMQ 端口 |
| `NO_ISAACLAB` | `0` | 禁用本地 IsaacLab 窗口 |
| `ISAAC_STATE_HOST` | `127.0.0.1` | 外部 IsaacLab 地址 |

## 故障排查

### 端口已被占用

**症状：**
```
ERROR: one or more requested ports are already in use:
tcp  LISTEN  0  128  *:12362  *:*  users:(("python",pid=12345))
```

**解决方法：**
```bash
# 方法 1：停止旧会话
tmux kill-session -t sonic_json_yup_mujoco

# 方法 2：使用不同端口
BVH_STREAM_PORT=22362 MOCAP_ZMQ_PORT=6656 DEBUG_PORT=6657 \
  ./scripts/launch_sonic_json_mujoco_closed_loop.sh file.json
```

### 机器人定格在第一帧

**症状：**
- UDP JSON 帧持续收到（`recv` 持续增长、`recv_fps` 正常）
- 机器人姿势正确但一直不动
- 日志显示 `frame=0` 恒定不变，`pose=buf:0/80`

**根因：**
外部发送端未携带递增 `frame_index`

**解决方法：**
使用仓库自带的 `sony_bonedata_json_stream_sender.py`（自动生成递增帧号）

### manager 报协议版本错误

**症状：**
```
error: --source bvh_stream supports either 
  --pose-protocol-version 1 --pose-encoder-mode g1, or 
  --pose-protocol-version 3 --pose-encoder-mode smpl --allow-sony-pose-v3
```

**根因：**
v3 协议需要配合 `smpl` encoder 和 `--allow-sony-pose-v3` 标志

**解决方法：**
使用一键脚本的环境变量（自动处理参数组合）：
```bash
POSE_PROTOCOL_VERSION=3 ./scripts/launch_sonic_json_mujoco_closed_loop.sh file.json
```

### JSON 文件格式错误

**症状：**
```
ValueError: position and rotation must have the same length; got 81, 108
```

**根因：**
文件格式可能是平铺字符串数组，而非嵌套对象/列表数组

**诊断方法：**
```bash
python3 scripts/diagnose_json_format.py ~/your_file.json
```

**正确格式：**
- `position`: `[{x, y, z}, ...]` 或 `[[x,y,z], ...]`
- `rotation`: `[{x, y, z, w}, ...]` 或 `[[x,y,z,w], ...]`
- 每帧 27 个关节

## 相关文档

- [Sony BoneData JSON raw stream sender](sony_bonedata_json_stream_sender.md) - sender 详细说明
- [Sony JSON POSE v3 支持](sony_json_pose_v3_support.md) - v3 协议使用文档
- [mocopi POSE SMPL v3 route](mocopi_pose_smpl_v3_route.md) - v3 协议技术细节
- [三层验证体系](../../reference/eval_deploy_layers.md) - Isaac Eval / Sim2Sim / Sim2Real

## 快速参考卡片

```bash
# 最常用命令（复制粘贴即可）

# MuJoCo + v1
./scripts/launch_sonic_json_mujoco_closed_loop.sh ~/file.json

# MuJoCo + v3
POSE_PROTOCOL_VERSION=3 ./scripts/launch_sonic_json_mujoco_closed_loop.sh ~/file.json

# IsaacLab + v3
POSE_PROTOCOL_VERSION=3 ./scripts/launch_sonic_json_isaaclab_closed_loop.sh ~/file.json

# 只启动接收端
./scripts/launch_sonic_json_mujoco_closed_loop.sh --receiver-only

# 附加会话
tmux attach-session -t sonic_json_yup_mujoco

# 停止会话
tmux kill-session -t sonic_json_yup_mujoco

# 查看日志
tail -f logs/sony_json_mujoco_sonic_json_yup_mujoco/manager.log
```
