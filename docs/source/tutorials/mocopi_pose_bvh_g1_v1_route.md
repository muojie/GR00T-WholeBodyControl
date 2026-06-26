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

这条路线下的 BVH 入口统一归在本文维护，但三种 `--source` 不能混用：

- `--source bvh_stream`：当前 BVH-G1 POSE v1 主路径。manager 监听 UDP skeleton stream，BVH 文件由 sender 进程读取，换动作只重启 sender。
- `--source bvh_g1`：单进程 BVH-G1 POSE v1 回归路径。manager 直接读取本地 BVH 文件，但使用和 `bvh_stream` 同一套 BVH-to-G1 retarget。
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

如果要让外部同事开发自己的 sender app，按 [BVH stream sender app 协议](bvh_stream_sender_app_protocol.md) 实现 UDP `bvh_stream_v1` 包即可。app 侧只发 skeleton frame；manager 侧继续负责 BVH/canonical skeleton 到 G1 29 维 joint reference 的 retarget，并通过 POSE v1 发给 deploy。

换 BVH 时只重启 sender。MuJoCo、deploy、manager 可以常驻；MuJoCo 中需要清空状态时，在 viewer 里按 `Backspace`。新的 sender 从 `frame_index=0` 开始时，deploy 侧通过 catch-up reset 切到新 motion window。

如果 sender 先于 manager 启动，UDP 前几帧可能丢失。实际操作时先等 manager 打印 `listening for BVH stream UDP...`，再启动 sender。

(mocopi-bvh-source-g1)=
### `--source bvh_g1`：单进程 BVH-G1 POSE v1 回归路径

`--source bvh_g1` 由 manager 进程直接读取本地 BVH 文件，但不走老的 `BvhPlaybackSource` 三点/SMPL 路径：

```text
BVH file on disk
  -> mocap_manager_server.py --source bvh_g1
  -> BvhG1PlaybackSource
  -> bvh_g1_source.py
  -> G1 retarget
  -> ZMQ pose protocol v1, 默认 5556
```

特点：

- manager 绑定一个具体 `--bvh-file`。
- 换 BVH 文件需要重启 manager。
- 不需要 UDP sender，适合单进程回归、调参和与 `bvh_stream` 做同算法对照。
- 输出和 `bvh_stream` 一样，是 POSE protocol v1 + `encoder_mode=g1` + G1 `joint_pos/joint_vel/body_pos`。
- 它不模拟外部实时动捕输入，也不支持只重启 sender 热切换。

命令：

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

日志指纹：

```text
retargeting BVH /path/to/motion.bvh to G1 joint references
pose_protocol=v1
pose_encoder=g1
pose=sent:N
```

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

| 项目 | `--source bvh_stream` | `--source bvh_g1` | `--source bvh` |
|------|------------------------|---------------------|----------------|
| 输入 | UDP skeleton stream | manager 直接读本地 BVH 文件 | manager 直接读本地 BVH 文件 |
| 文件读取者 | `bvh_stream_sender.py` | manager 进程 | manager 进程 |
| manager 是否绑定文件 | 否 | 是 | 是 |
| 换文件 | 只重启 sender | 重启 manager | 重启 manager |
| 主要代码路径 | `BvhStreamUdpSource` -> `bvh_g1_source.py` | `BvhG1PlaybackSource` -> `bvh_g1_source.py` | `BvhPlaybackSource` |
| 主要输出 | G1 `joint_pos/joint_vel/body_pos` | G1 `joint_pos/joint_vel/body_pos` | VR3PT / SMPL-like full body 辅助字段 |
| deploy 契约 | POSE v1 + `encoder_mode=g1` | POSE v1 + `encoder_mode=g1` | planner/VR3PT 或 SMPL POSE v3 调试 |
| 是否模拟实时动捕 | 是 | 否 | 否 |
| 推荐 POSE 主线 | 是，主验证路径 | 是，单进程回归路径 | 否 |

### `bvh_stream` 效果好于早期 `bvh` 的原因

最近 `--source bvh_stream` 明显好于早期 `--source bvh`，根因不是 UDP 传输本身更好，而是两者走的不是同一条控制链路。

`bvh_stream` 和 `bvh_g1` 先在 manager 侧把 BVH skeleton frame 重定向成 G1 29 维关节参考，再通过 POSE v1 交给 deploy；`bvh` 则是老的本地 BVH playback 入口，主要服务 VR3PT、BVH parser/FK 验证和 SMPL v3 协议研究。

关键差异：

- 控制对象不同：`bvh_stream` / `bvh_g1` 直接生成机器人 G1 `joint_pos/joint_vel`；`bvh` 先生成人体/三点语义，再由后续路径近似映射。
- deploy encoder 不同：`bvh_stream` / `bvh_g1` 强制 `--pose-protocol-version 1 --pose-encoder-mode g1`；`bvh` 既可以走 planner 三点，也可以走 SMPL POSE v3 调试。
- retargeter 不同：`bvh_stream` / `bvh_g1` 使用 `bvh_g1_source.py`，包含 skeleton segment direction、`bvh_y_forward` 轴映射、左右符号修正、IK/refine、关节限位、速度限幅、滤波、root yaw/height 稳定和 G1 FK `body_pos`；`bvh` 只是在 `BvhPlaybackSource` 上补了 wrist 和下肢启发式，不是完整 G1 retarget。
- deploy 稳定性修复不同：当前 G1 主线已经补齐 `body_pos`、root 默认高度、`encoder_mode` 和 streamed-motion window/catch-up 处理；早期 `bvh` 调试时这些修复还没完整闭环。

如果要判断“网络流式输入是否影响效果”，不要拿 `bvh_stream` 和 `bvh` 直接比；应拿同一套 G1 retarget 下的 `bvh_stream` 与 `bvh_g1` 对比。两者目标应接近，差异主要来自 UDP 丢包、FPS、启动时机和换动作窗口。

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

## 运行指纹

`bvh_stream` manager 侧应看到：

```text
control_mode=pose pose_stream=1 pose_protocol=v1 pose_window=80 pose_encoder=g1
listening for BVH stream UDP on 0.0.0.0:12352
pose=sent:1
recv_fps≈50 dropped=0
```

`bvh_g1` manager 侧应看到：

```text
control_mode=pose pose_stream=1 pose_protocol=v1 pose_window=80 pose_encoder=g1
retargeting BVH /path/to/motion.bvh to G1 joint references
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
