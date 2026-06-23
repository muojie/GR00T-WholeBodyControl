# Sony mocopi / BVH-G1 POSE v1 路线

本文只记录当前跑通的工程路线：输入源先在 Python manager 侧重定向成 G1 关节参考，再通过 POSE protocol v1 发给 deploy。

## 定位

这条路线使用 SONIC / GR00T 的 deploy、MuJoCo、ZMQ stream、MotionSequence 和 policy runtime，但不走 SONIC 原生 SMPL encoder 语义。

```text
BVH / canonical skeleton frame
  -> manager 侧 BVH/mocopi-to-G1 retarget
  -> G1 29 维 joint_pos / joint_vel
  -> pose topic, protocol v1, encoder_mode=g1
  -> deploy StreamedMotionMerger
  -> policy 跟随 G1 joint reference
```

BVH 或 mocopi 原始数据不会直接交给 deploy；deploy 收到的是已经由 manager 侧算好的 G1 reference 窗口。

## BVH 输入源

这条路线下的 BVH 入口统一归在本文维护，但两种 `--source` 不能混用：

- `--source bvh_stream`：当前 BVH-G1 POSE v1 主路径。manager 监听 UDP skeleton stream，BVH 文件由 sender 进程读取，换动作只重启 sender。
- `--source bvh`：manager 直接读取本地 BVH 文件，主要用于离线回放、VR3PT、SMPL v3 协议研究；它不是当前 MuJoCo 主验证命令。

(mocopi-bvh-source-stream)=
### `--source bvh_stream`：BVH-G1 POSE v1 主路径

当前推荐把输入源和 manager 拆成两个进程：

```text
BVH file
  -> bvh_stream_sender.py
  -> UDP bvh_stream_v1, 默认 12352
  -> mocap_manager_server.py --source bvh_stream
  -> BvhStreamUdpSource
  -> G1 retarget
  -> ZMQ pose protocol v1, 默认 5556
```

特点：

- manager 不绑定具体 BVH 文件。
- MuJoCo、deploy、manager 可以常驻。
- 换 BVH 文件只需要重启 `bvh_stream_sender.py`。
- 适合在线输入模拟、BVH-G1 POSE v1 主验证和热切换。

manager 命令：

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

sender 命令：

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

换 BVH 时只重启 sender。MuJoCo、deploy、manager 可以常驻；MuJoCo 中需要清空状态时，在 viewer 里按 `Backspace`。新的 sender 从 `frame_index=0` 开始时，deploy 侧通过 catch-up reset 切到新 motion window。

如果 sender 先于 manager 启动，UDP 前几帧可能丢失。实际操作时先等 manager 打印 `listening for BVH stream UDP...`，再启动 sender。

(mocopi-bvh-source-file)=
### `--source bvh`：本地 BVH 文件回放

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

### 对比与选择

| 项目 | `--source bvh_stream` | `--source bvh` |
|------|------------------------|----------------|
| 输入 | UDP skeleton stream | manager 直接读本地 BVH 文件 |
| 文件读取者 | `bvh_stream_sender.py` | manager 进程 |
| 换文件 | 只重启 sender | 重启 manager |
| 主要用途 | 在线输入模拟、BVH-G1 POSE v1 主验证 | 离线回放、VR3PT、SMPL v3 研究 |
| 是否模拟实时动捕 | 是 | 否 |
| 推荐 POSE 主线 | 是 | 否 |

## POSE v1 字段

当前主要发送：

```text
joint_pos
joint_vel
body_pos
body_quat_w
frame_index
catch_up
encoder_mode
```

关键约束：

- `joint_pos` / `joint_vel` 是 G1 29 维关节参考。
- `encoder_mode=g1`。
- `pose_window_size` 建议 80，避免 future observation 不够。
- 新 sender 从 `frame_index=0` 开始时，deploy 侧通过 catch-up reset 切到新 motion window。

## 单进程回归路径

如果不需要热切换，可以用 `bvh_g1`：

```bash
.venv_teleop/bin/python -u gear_sonic/scripts/mocap_manager_server.py \
  --source bvh_g1 \
  --bvh-file /home/nolo/RAYNOS_Motion1.bvh \
  --bvh-loop \
  --control-mode pose \
  --pose-window-size 80 \
  --pose-encoder-mode g1 \
  --pose-protocol-version 1 \
  --zmq-port 5556
```

这个模式仍是在线逐帧 retarget，但 manager 绑定具体 BVH 文件，换文件需要重启 manager。

## 运行指纹

manager 侧应看到：

```text
control_mode=pose pose_stream=1 pose_protocol=v1 pose_window=80 pose_encoder=g1
listening for BVH stream UDP on 0.0.0.0:12352
pose=sent:1
recv_fps≈50 dropped=0
```

诊断字段重点：

| 字段 | 判断 |
|------|------|
| `q=[min,max]` | 29 维 G1 关节目标范围 |
| `dq_abs` | 关节速度是否有尖峰 |
| `lower_dq` | 下肢 G1 关节相对默认站姿的偏差；当前路线不应长期为 0 |
| `root_z` | root 高度是否合理 |
| `root_tilt` | 切入时 root 倾斜是否过大 |

## 后续工作

这条路线下一步应把 `bvh_stream_v1` 的 skeleton packet 抽象成 mocopi 实时输入也能复用的 canonical skeleton frame，并继续优化 wrist、root heading、脚踝/脚尖和速度限幅。
