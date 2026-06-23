# Sony mocopi / BVH 输入源：`--source bvh` 与 `--source bvh_stream`

本文统一记录 `mocap_manager_server.py` 中两个 BVH 输入源。它们处理的都是 BVH 动作数据，但架构定位不同：

- `--source bvh`：manager 直接读取本地 BVH 文件，适合离线回放、VR3PT、SMPL v3 协议研究。
- `--source bvh_stream`：manager 监听在线 UDP skeleton stream，BVH 文件由 sender 进程读取，适合 BVH-G1 POSE v1 主验证和热切换。

两种输入源可以放在同一个知识页面里，但命令不要混用：`--source bvh` 不会启动 UDP stream receiver，`--source bvh_stream` 也不会直接读取 `--bvh-file`。

(mocopi-bvh-source-file)=
## `--source bvh`：本地 BVH 文件回放

`--source bvh` 由 manager 进程自己读取本地 BVH 文件：

```text
BVH file on disk
  -> BvhPlaybackSource
  -> BVH parser / FK / frame stepping
  -> MocapFrame
  -> planner VR3PT 或可选 POSE 调试
```

特点：

- manager 绑定一个具体 `--bvh-file`。
- 换 BVH 文件需要重启 manager。
- 适合离线文件回放、三点 retarget 调试、BVH parser/FK 验证。
- 不适合模拟实时外部动捕输入的热切换流程。

### Planner / VR3PT 回放

默认用法是三点 planner 验证：

```bash
.venv_teleop/bin/python gear_sonic/scripts/mocap_manager_server.py \
  --source bvh \
  --bvh-file /path/to/motion.bvh \
  --bvh-loop \
  --bvh-fps 30 \
  --zmq-port 5556
```

本地看三点目标：

```bash
.venv_teleop/bin/python gear_sonic/scripts/mocap_manager_server.py \
  --source bvh \
  --bvh-file /path/to/motion.bvh \
  --bvh-loop \
  --bvh-fps 30 \
  --zmq-port 5556 \
  --visualize-vr3pt
```

日志指纹：

```text
playing BVH /path/to/motion.bvh at 25.0 Hz (source_fps=50.0, stride=2, loop=True)
vr_3pt=yes
```

这里的 `vr_3pt=yes` 表示 BVH 已被解析成左右腕和头/颈三点，并进入 `planner` topic。

### 可选 POSE / SMPL 调试

`--source bvh` 也可以用于 [SMPL POSE v3 路线](mocopi_pose_smpl_v3_route.md) 的协议研究：

```bash
.venv_teleop/bin/python -u gear_sonic/scripts/mocap_manager_server.py \
  --source bvh \
  --bvh-file /path/to/motion.bvh \
  --bvh-loop \
  --control-mode pose \
  --pose-window-size 80 \
  --pose-encoder-mode smpl \
  --pose-protocol-version 3 \
  --zmq-port 5556
```

这不是当前 MuJoCo 主验证命令。当前主验证路线使用 `--source bvh_stream` 或 `--source bvh_g1` 生成 G1 joint reference。

### 频率说明

如果源 BVH 是 50 Hz，而指定 `--bvh-fps 30`，当前实现会用整数 stride 降采样，可能得到 25 Hz：

```text
source_fps=50.0, stride=2
```

评估动作自然度时，优先让 BVH 回放频率和 manager 发布频率一致：

```bash
--bvh-fps 50 --target-fps 50
```

不要用 `BVH 实际 25 Hz + manager 20 Hz` 这类不一致组合判断复原质量。

(mocopi-bvh-source-stream)=
## `--source bvh_stream`：在线 BVH UDP 流

`--source bvh_stream` 让 manager 监听 UDP skeleton frame。BVH 文件由另一个 sender 进程读取和发送：

```text
BVH file
  -> bvh_stream_sender.py
  -> UDP bvh_stream_v1
  -> BvhStreamUdpSource
  -> manager 侧 G1 retarget
  -> POSE v1 + encoder_mode=g1
```

特点：

- manager 不绑定具体 BVH 文件。
- MuJoCo、deploy、manager 可以常驻。
- 换 BVH 文件只需要重启 `bvh_stream_sender.py`。
- 当前主要用于 [BVH-G1 POSE v1 路线](mocopi_pose_bvh_g1_v1_route.md)。

### Manager 命令

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

预期日志：

```text
control_mode=pose pose_stream=1 pose_protocol=v1 pose_window=80 pose_encoder=g1
listening for BVH stream UDP on 0.0.0.0:12352
```

### Sender 命令

```bash
.venv_teleop/bin/python -u gear_sonic/scripts/bvh_stream_sender.py \
  --bvh-file /home/nolo/RAYNOS_Motion1.bvh \
  --host 127.0.0.1 \
  --port 12352 \
  --loop
```

预期日志：

```text
[BvhStreamSender] streaming /home/nolo/RAYNOS_Motion1.bvh to udp://127.0.0.1:12352 format=msgpack fps=50.0
```

### 热切换

换 BVH 文件时：

1. 保持 MuJoCo、deploy、manager 不动。
2. 停止当前 `bvh_stream_sender.py`。
3. 用新的 `--bvh-file` 重启 sender。
4. 必要时在 MuJoCo viewer 中按 `Backspace` reset 机器人状态。

新的 sender 从 `frame_index=0` 开始时，deploy 的 streamed-motion merger 会通过 catch-up reset 切到新 motion window。

### 接收指纹

manager 应看到：

```text
recv_fps≈50
dropped=0
pose=sent:N
```

如果 sender 先于 manager 启动，UDP 前几帧可能丢失。实际操作时先等 manager 打印 `listening for BVH stream UDP...`，再启动 sender。

## 对比与选择

| 项目 | `--source bvh` | `--source bvh_stream` |
|------|----------------|------------------------|
| 输入 | manager 直接读本地 BVH 文件 | manager 监听 UDP skeleton frame |
| 文件读取者 | manager 进程 | `bvh_stream_sender.py` |
| 换文件 | 重启 manager | 只重启 sender |
| 主要用途 | 离线回放、VR3PT、SMPL v3 研究 | 在线输入模拟、BVH-G1 POSE v1 主验证 |
| 是否模拟实时动捕 | 否 | 是 |
| 推荐 POSE 主线 | 否 | 是 |
