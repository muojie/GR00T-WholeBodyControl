# Sony mocopi 动捕管理器

本文档说明如何使用 Sony mocopi、基于 mocopi 的桥接程序，或者 BVH 文件回放，作为 SONIC 现有 ZMQ 部署链路的输入源。

当前有两条稳定验证链路：`planner` topic 的 VR3PT 三点验证，以及 `pose` topic 的 BVH-G1 joint reference 验证。BVH-G1 主线推荐使用 `bvh_stream_sender.py -> --source bvh_stream -> POSE v1 + encoder_mode=g1`，这样 MuJoCo、deploy、manager 可以常驻，只重启 sender 就能换 BVH。

POSE 路线不要混写：当前主线见 [Sony mocopi / BVH-G1 POSE v1 路线](mocopi_pose_bvh_g1_v1_route.md)；更接近 SONIC 原生人体 pose encoder 的研究线见 [Sony mocopi / SMPL POSE v3 路线](mocopi_pose_smpl_v3_route.md)。

BVH 输入源也不要混写：[`--source bvh`](mocopi_source_bvh_file.md) 是 manager 直接读本地 BVH 文件；[`--source bvh_stream`](mocopi_source_bvh_stream.md) 是 manager 监听 UDP skeleton stream，由 `bvh_stream_sender.py` 负责读文件和发送。

当前实现刻意独立于 `pico_manager_thread_server.py`。这样可以先验证非 PICO 输入链路，而不影响已有 PICO/XR 遥操作流程。

```{admonition} 当前状态
:class: warning
这是非 PICO 输入源集成层。它可以接收 mocopi UDP 数据、回放 BVH 文件、监听 BVH UDP stream，并发布现有 `command`、`planner`、`pose`、`manager_state` ZMQ topic。真正直接使用官方 mocopi 27 bone 的完整 FK / 标定层仍是后续工作；当前效果最好的 full-body 验证路径是 BVH stream 到 G1 joint reference。
```

```{admonition} 默认不会弹出窗口
:class: note
`mocap_manager_server.py` 默认只是 ZMQ publisher，不会自动打开可视化窗口。要看本地三点目标窗口，需要显式添加 `--visualize-vr3pt`。要看机器人实际运动，还需要另开 sim/deploy 进程订阅 `5556`。
```

## 代码位置

mocopi 输入链路由以下文件实现：

```text
gear_sonic/scripts/mocap_manager_server.py
gear_sonic/utils/teleop/sources/base.py
gear_sonic/utils/teleop/sources/bvh_source.py
gear_sonic/utils/teleop/sources/bvh_g1_source.py
gear_sonic/utils/teleop/sources/bvh_stream_source.py
gear_sonic/utils/teleop/sources/mocopi_source.py
gear_sonic/utils/teleop/sources/g1_body_fk.py
gear_sonic/utils/teleop/sources/robot_pkl_source.py
gear_sonic/utils/teleop/sources/joint_probe_source.py
gear_sonic/utils/teleop/sources/__init__.py
gear_sonic/scripts/bvh_stream_sender.py
gear_sonic/utils/teleop/controls/keyboard_control.py
gear_sonic/utils/teleop/controls/__init__.py
gear_sonic/utils/teleop/retarget/vr3pt_retargeter.py
gear_sonic/utils/teleop/retarget/upper_body_ik.py
gear_sonic/utils/teleop/retarget/__init__.py
gear_sonic/utils/teleop/zmq/zmq_pose_sender.py
```

职责划分：

- `mocap_manager_server.py`：接收标准化后的动捕帧，并发布 deploy 侧兼容的 ZMQ 消息。
- `base.py`：定义 `Pose7D`、`MocapFrame`、`MocapSource` 等通用数据结构。
- `mocopi_source.py`：实现 UDP 接收、官方 mocopi 二进制包解析、JSON bridge 包解析。
- `bvh_source.py`：实现 BVH hierarchy/motion 解析、FK、循环回放，并输出 `MocapFrame`。
- `bvh_g1_source.py`：把 BVH skeleton frame 重定向成 G1 29 维 `joint_pos/joint_vel`，支持 `online` 与 `precompute`。
- `bvh_stream_source.py`：监听 `bvh_stream_v1` UDP packet，缓存最新骨架帧，并在 manager 主循环中执行 BVH-to-G1 retarget。
- `bvh_stream_sender.py`：把本地 BVH 文件按 FPS 逐帧发送成 UDP skeleton stream，用于模拟实时动捕输入。
- `g1_body_fk.py`：根据 G1 MJCF 计算 POSE v1 需要的 14 个 body position reference。
- `keyboard_control.py`：提供基于 stdin 的行命令控制，用来替代 PICO 手柄按键。
- `vr3pt_retargeter.py`：把标准化后的动捕帧转换为 deploy 侧需要的 `vr_position` / `vr_orientation`。
- `upper_body_ik.py`：实验性可选模块，把 VR3PT wrist 目标求解成 deploy 侧 17 维 `upper_body_position` / `upper_body_velocity`。
- `zmq_pose_sender.py`：维护 POSE 滑动窗口，并按 deploy protocol v1/v3 发布 `pose` topic；当前 BVH-G1 主线使用 v1。

## 数据流

```text
Sony mocopi app / mocopi bridge / BVH file
  -> MocopiUdpSource 或 BvhPlaybackSource
  -> MocapFrame
  -> VR3PointRetargeter
  -> mocap_manager_server.py
  -> ZMQ PUB，默认端口 5556
      - command
      - planner
      - manager_state
  -> deploy 侧 ZMQ subscriber
  -> PLANNER_VR_3PT / locomotion policy
```

deploy 侧继续消费现有 ZMQ schema。第一版集成不需要在 deploy 侧新增 topic。

注意：这张图描述的是 `planner` topic 的三点实时验证链路，不是 PICO full-body `pose` topic。PICO 的下肢 tracker 数据会融入 full-body / SMPL 数据；mocopi / BVH 要对齐这条能力，需要选择单独的 POSE 路线，而不是继续扩展 `PLANNER_VR_3PT`。路线总览见 [Sony mocopi / BVH POSE 路线总览](mocopi_pose_stream.md)。

BVH-G1 full-body 验证走 `pose` topic，详细设计见 [Sony mocopi / BVH-G1 POSE v1 路线](mocopi_pose_bvh_g1_v1_route.md)：

```text
BVH file
  -> bvh_stream_sender.py
  -> UDP bvh_stream_v1
  -> BvhStreamUdpSource
  -> manager 侧 BVH-to-G1 retarget
  -> MocapFrame.full_body(joint_pos/joint_vel/body_pos/body_quat)
  -> PoseStreamPublisher
  -> ZMQ PUB，默认端口 5556
      - command，planner=false 时切到 streamed-motion
      - pose，protocol v1: joint_pos / joint_vel / body_pos / body_quat_w / frame_index / encoder_mode=g1
      - manager_state
  -> deploy 侧 ZMQManager
  -> streamed motion / POSE reference
```

## 启动 mocopi UDP 输入

在仓库根目录使用 `.venv_teleop` 启动：

```bash
.venv_teleop/bin/python gear_sonic/scripts/mocap_manager_server.py \
  --source mocopi \
  --mocopi-format auto \
  --mocopi-port 12351 \
  --zmq-port 5556
```

如果上游发送方明确是 JSON bridge：

```bash
.venv_teleop/bin/python gear_sonic/scripts/mocap_manager_server.py \
  --source mocopi \
  --mocopi-format json \
  --mocopi-port 12351 \
  --zmq-port 5556
```

## 启动 BVH 文件回放：`--source bvh`

`--source bvh` 是本地文件回放输入源。manager 直接读取 `--bvh-file`，换文件需要重启 manager。完整说明见 [Sony mocopi / `--source bvh` 本地 BVH 文件回放](mocopi_source_bvh_file.md)。

它适合在没有 mocopi 硬件时验证后续链路，也适合调试三点 retargeting：

```bash
.venv_teleop/bin/python gear_sonic/scripts/mocap_manager_server.py \
  --source bvh \
  --bvh-file /path/to/motion.bvh \
  --bvh-loop \
  --bvh-fps 30 \
  --zmq-port 5556
```

如果只想先看 BVH 生成的三点目标，不连接 deploy，可以打开本地可视化窗口：

```bash
.venv_teleop/bin/python gear_sonic/scripts/mocap_manager_server.py \
  --source bvh \
  --bvh-file /path/to/motion.bvh \
  --bvh-loop \
  --bvh-fps 30 \
  --zmq-port 5556 \
  --visualize-vr3pt
```

## 启动 BVH 在线流：`--source bvh_stream`

`--source bvh_stream` 是在线 UDP skeleton stream 输入源。manager 只监听 UDP，不绑定具体 BVH 文件；换动作只重启 sender。完整说明见 [Sony mocopi / `--source bvh_stream` 在线 BVH UDP 流](mocopi_source_bvh_stream.md)。

如果要验证 BVH-G1 full-body POSE v1，推荐使用 `bvh_stream`。先启动 manager：

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

再启动 sender：

```bash
.venv_teleop/bin/python -u gear_sonic/scripts/bvh_stream_sender.py \
  --bvh-file /path/to/motion.bvh \
  --host 127.0.0.1 \
  --port 12352 \
  --loop
```

当前推荐路径使用 `--pose-protocol-version 1 --pose-encoder-mode g1`。manager 侧先把 BVH skeleton frame 重定向成 G1 29 维 `joint_pos/joint_vel`，再交给 deploy 的 streamed-motion merger。它不是把 BVH 或 mocopi 原始数据直接交给 deploy。

MuJoCo release policy 会读取未来帧 observation。不要用早期 `--pose-window-size 5` 做主验证，当前 `--control-mode pose` 不显式设置窗口时默认使用 80 帧；推荐命令里仍保留 `--pose-window-size 80`，否则旧脚本或手工命令容易复现 `Motion streamed completed and waiting following motion`。

换 BVH 文件时只需要重启 sender；MuJoCo、deploy、manager 都可以常驻。新的 sender 从 `frame_index=0` 开始时，deploy 会通过 catch-up reset 切到新 streamed motion 窗口。

POSE 日志会额外输出参考姿态诊断，例如：

```text
pose=sent:24 q=[-1.15,0.98] dq_abs=0.00 lower_dq=0.00 smpl_lz=[-0.76,0.00] smpl_lspan=0.90m smpl_lpose=0.53rad root_tilt=0.53rad
```

其中 `lower_dq` 是下肢 12 个 G1 关节相对默认站姿的最大偏差。当前 `bvh_stream` / `bvh_g1` 主线会显式生成 G1 下肢关节参考，所以它不应长期接近 `0`；如果仍接近 `0`，通常说明没有进入 BVH-G1 retarget 主线，或输入骨架没有足够下肢信息。`smpl_lspan`、`smpl_lpose`、`root_tilt` 用来判断 BVH 下肢尺度、姿态幅度和根姿态是否异常。

如果只想在原来的 planner/VR3PT 模式下同时发布 `pose` topic 做抓包或数据检查，可以保持 `--control-mode planner` 并添加：

```bash
--enable-pose-stream
```

BVH source 会解析 hierarchy 和 motion 数据，执行 FK，并尽量保留 torso、neck、head、shoulder、elbow、wrist、pelvis 等有效关节。默认 planner 仍只消费 `left_wrist`、`right_wrist`、`head` 三点；如果显式开启 `--enable-upper-body-ik`，manager 会额外发布 deploy 侧已有的 17 维上肢关节目标。

如果用于评估动作自然度，建议让 BVH 回放频率和 manager 发布频率一致。例如 BVH 原始文件是 `50 Hz` 时，先用：

```bash
.venv_teleop/bin/python gear_sonic/scripts/mocap_manager_server.py \
  --source bvh \
  --bvh-file /path/to/motion.bvh \
  --bvh-loop \
  --bvh-fps 50 \
  --target-fps 50 \
  --zmq-port 5556 \
  --visualize-vr3pt \
  --visualize-g1
```

如果机器负载较高，再统一降到 `25 Hz`：

```bash
--bvh-fps 25 --target-fps 25
```

不要让 BVH 实际输出频率和 `--target-fps` 长期不一致，否则会引入额外采样抖动，影响 MuJoCo 里看到的顺滑度。

常用参数：

| 参数 | 默认值 | 作用 |
|------|--------|------|
| `--source` | `mocopi` | 输入源，可选 `mocopi`、`bvh`、`bvh_g1`、`bvh_stream`、`pkl`、`joint_probe` |
| `--mocopi-host` | `0.0.0.0` | UDP 绑定地址 |
| `--mocopi-port` | `12351` | UDP 绑定端口 |
| `--mocopi-format` | `auto` | 输入包格式，可选 `auto`、`binary`、`json` |
| `--bvh-stream-host` | `0.0.0.0` | `--source bvh_stream` 的 UDP 绑定地址 |
| `--bvh-stream-port` | `12352` | `--source bvh_stream` 的 UDP 绑定端口 |
| `--bvh-stream-format` | `auto` | BVH stream 输入包格式，可选 `auto`、`msgpack`、`json` |
| `--bvh-file` | 无 | `--source bvh` 时要回放的 BVH 文件 |
| `--bvh-loop` | 关闭 | BVH 播放到末尾后循环 |
| `--bvh-fps` | BVH 原始 FPS | BVH 目标回放 FPS；低于原始 FPS 时按 stride 跳帧 |
| `--bvh-unit-scale` | `0.01` | BVH 坐标单位转米，厘米制 BVH 使用 `0.01` |
| `--bvh-no-y-up-to-z-up` | 关闭 | 禁用 BVH Y-up 到 SONIC Z-up 的坐标转换 |
| `--bvh-world-frame` | 关闭 | 保留 BVH root 全局平移，不做 body-local 化 |
| `--zmq-port` | `5556` | deploy 侧订阅的 ZMQ PUB 端口 |
| `--target-fps` | `20` | planner 发布循环频率 |
| `--control-mode` | `planner` | 通过 command topic 选择 deploy 控制模式；`planner` 使用 PLANNER/VR3PT，`pose` 使用 streamed-motion / POSE |
| `--enable-pose-stream` | 关闭 | 当输入源有 `full_body` 时额外发布 `pose` topic；`--control-mode pose` 会自动开启 |
| `--pose-window-size` | `pose` 模式为 `80`；planner/debug 为 `5` | 每条 POSE 消息包含的 full-body 帧数；MuJoCo release policy 验证建议显式设为 `80` |
| `--pose-protocol-version` | `3` | POSE 协议版本；BVH-G1 / PKL / bvh_stream 主线必须显式使用 `1` |
| `--pose-encoder-mode` | `smpl` | deploy encoder 模式；BVH-G1 / PKL / bvh_stream 主线必须显式使用 `g1` |
| `--bvh-g1-runtime-mode` | `online` | `--source bvh_g1` 的逐帧 retarget 模式；`precompute` 用作慢速参考 |
| `--bvh-g1-ik-mode` | `auto` | `online` 下解析为 `analytic`，避免启动时整段 numeric IK |
| `--mocap-timeout-s` | `0.5` | 最新动捕帧超过该时间后停止发布 VR 三点目标 |
| `--start-paused` | 关闭 | 启动时不立即启用控制 |
| `--allow-bone-translation-vr` | 关闭 | 调试用途：把 bone translation 当作 VR 三点位置 |
| `--no-vr3pt-calibration` | 关闭 | 禁用命名关节输入的首帧位置标定 |
| `--vr3pt-scale` | `1.0` | 命名关节位置进入首帧标定前的缩放系数 |
| `--no-vr3pt-fk-calibration` | 关闭 | 禁用 G1 FK 参考标定，回退到旧的首帧位置平移 |
| `--require-vr3pt-fk-calibration` | 关闭 | 如果 G1 FK 标定依赖加载失败，直接退出而不是回退 |
| `--no-vr3pt-filter` | 关闭 | 禁用三点滤波和速度限制 |
| `--vr3pt-position-alpha` | `0.45` | 位置低通系数，越大越跟手，越小越平滑 |
| `--vr3pt-orientation-alpha` | `0.45` | 四元数 slerp 平滑系数，越大越跟手 |
| `--vr3pt-max-speed` | `3.0` | 三点最大平移速度，单位 m/s，`<=0` 表示关闭 |
| `--vr3pt-max-accel` | `25.0` | 三点最大平移加速度，单位 m/s^2，`<=0` 表示关闭 |
| `--vr3pt-max-angular-speed` | `8.0` | 三点最大角速度，单位 rad/s，`<=0` 表示关闭 |
| `--enable-upper-body-ik` | 关闭 | 实验开关：从 VR3PT wrist 目标求解并发布 `upper_body_position` / `upper_body_velocity` |
| `--upper-body-ik-iterations` | `8` | 每帧上肢 IK 迭代次数 |
| `--upper-body-ik-damping` | `0.08` | damped least-squares 阻尼，越大越稳但误差可能更大 |
| `--upper-body-ik-position-weight` | `1.0` | wrist 位置误差权重 |
| `--upper-body-ik-orientation-weight` | `0.15` | wrist 姿态误差权重 |
| `--upper-body-ik-posture-weight` | `0.03` | 回到默认姿态的正则权重 |
| `--upper-body-ik-step-size` | `0.7` | IK 单次更新步长 |
| `--upper-body-ik-max-joint-step` | `0.08` | 每次迭代单关节最大变化，单位 rad |
| `--visualize-vr3pt` | 关闭 | 打开 PyVista 窗口，实时显示生成的 VR 三点目标 |
| `--visualize-g1` | 关闭 | 在三点可视化窗口中同时显示 G1 模型 |
| `--visualize-width` / `--visualize-height` | `1400` / `900` | 可视化窗口尺寸 |

## 运行时命令

manager 会轮询 stdin，支持以下行命令：

```text
start
pause
stop
mode N
move x y z
face x y z
speed v
height h
dc
abort
```

这些命令在非 PICO 路径中替代 PICO 手柄按键。当前实现偏基础，后续生产使用应接入更明确的控制源。

## 推荐调试顺序

建议按下面顺序排查和验证：

1. 先启动 mocopi UDP 输入，确认日志中的 `recv` 从 `0` 增长。如果一直是 `recv=0`，先排查 mocopi app 的目标 IP、端口 `12351`、手机/电脑是否在同一局域网，以及本机防火墙。
2. 用 JSON bridge 发送一帧 `vr_position` / `vr_orientation`，确认 manager 日志出现 `vr_3pt=yes`。这一步验证 ZMQ 发布和 deploy 侧三点字段，不依赖 mocopi 骨架 FK。
3. 用 `--source bvh` 回放 BVH 文件，确认离线动捕文件能进入同一套 `MocapFrame -> VR3PointRetargeter -> planner` 链路。
4. 再接真实 mocopi 二进制包，做 27 bone 可视化、FK、首帧标定和坐标系校正。

如果 manager 日志已经出现 `vr_3pt=yes`，但屏幕没有窗口，通常是因为没有加 `--visualize-vr3pt`。如果加了参数仍没有窗口，检查当前环境是否有可用桌面显示，例如 `echo $DISPLAY`。

## 输入格式

### 官方 mocopi 二进制 UDP

mocopi app 默认通过端口 `12351` 发送二进制 UDP 包。当前解析器支持以下结构：

```text
head / ftyp / vrsn
sndf / ipad / rcvp
可选 fram / fnum / time / uttm / tmcd
btrs
27 x btdt
  - bnid
  - tran
```

每个 `tran` 包含 7 个 `float32`：

```text
qx, qy, qz, qw, px, py, pz
```

解析器会将其转换为仓库内统一约定：

```text
position: [x, y, z]
quat_wxyz: [w, x, y, z]
```

同时会把已知 mocopi bone id 映射为 `root`、`head`、`left_wrist`、`right_wrist` 等名称。

```{admonition} 重要限制
:class: warning
官方 mocopi 数据包还不是机器人可直接使用的完整 POSE 数据。它主要提供 mocopi 骨架 transform。当前官方二进制包仍优先用于 VR3PT；要稳定进入 POSE，需要补 mocopi 27 bone 到 SMPL 24/21 的映射，或由上游 bridge 直接输出 SMPL-like 字段。
```

### JSON Bridge

最快的闭环路径是由上游 bridge 直接发送机器人需要的 VR 三点数组：

```json
{
  "source": "sony_mocopi_bridge",
  "frame_index": 1,
  "vr_position": [
    0.1, 0.2, 0.3,
    0.4, -0.2, 0.3,
    0.0, 0.0, 0.5
  ],
  "vr_orientation": [
    1, 0, 0, 0,
    1, 0, 0, 0,
    1, 0, 0, 0
  ]
}
```

JSON bridge 也可以直接发送 full-body POSE 所需字段。实际发送时数组必须是完整长度：

```text
source: sony_mocopi_bridge
frame_index: int
smpl_joints: [24, 3] 或 [1, 24, 3]
smpl_pose: [21, 3]、[1, 21, 3] 或展平后的 [63]
body_quat_w: [4]
joint_pos: 可选，[29] 或 [1, 29]
joint_vel: 可选，[29] 或 [1, 29]
```

`vr_position` 顺序：

```text
left_wrist xyz, right_wrist xyz, head xyz
```

`vr_orientation` 顺序：

```text
left_wrist quat_wxyz, right_wrist quat_wxyz, head quat_wxyz
```

bridge 也可以发送 `vr_pose`，形状为 `[3, 7]`：

```text
第 0 行：left_wrist [x, y, z, qw, qx, qy, qz]
第 1 行：right_wrist [x, y, z, qw, qx, qy, qz]
第 2 行：head [x, y, z, qw, qx, qy, qz]
```

### BVH 文件

BVH source 支持常见 BVH hierarchy/motion 文件。默认约定：

- BVH 是 Y-up。
- BVH 坐标单位是厘米，因此默认 `--bvh-unit-scale 0.01`。
- 输出前会减去 root / hips 平移，形成 body-local 位置。
- 默认把 BVH Y-up 转成 SONIC / MuJoCo 使用的 Z-up：`(x, y, z) -> (x, -z, y)`。
- 默认查找 `Hips`、`LeftHand`、`RightHand`、`Head`。

当前内置别名已经覆盖常见 BVH / Mixamo 命名，也覆盖了 `RAYNOS_Motion1.bvh` 中的 `torso_*`、`l_shoulder`、`l_low_arm`、`r_shoulder`、`r_low_arm`、`l_hand`、`r_hand` 等命名。如果 BVH 文件使用其他关节命名，需要继续扩展 `BVH_DEFAULT_JOINT_ALIASES`，或者先在上游转换成兼容命名。

BVH source 输出的是标准 `MocapFrame`，其中包含：

```text
root
pelvis
spine
chest
neck
head
left_shoulder
left_elbow
left_wrist
right_shoulder
right_elbow
right_wrist
```

随后 `VR3PointRetargeter` 会进行首帧位置标定，让第一帧对齐 deploy 侧默认 VR 三点姿态。这样回放 BVH 时不会因为 BVH 原始站位离机器人默认参考姿态太远而产生明显跳变。

运行日志中的 `joints=N` 表示当前输入帧携带了多少个有效命名关节。以 `/home/nolo/RAYNOS_Motion1.bvh` 为例，当前应能看到 `joints=12`。

## VR3PT 质量指标

当 `vr_3pt=yes` 时，manager 会在日志中附带一组质量指标：

```text
span=0.371m head_z=0.398m max_v=1.350m/s lag=0.169m fk=1
```

含义：

| 字段 | 含义 | 用途 |
|------|------|------|
| `span` | 左右腕三点目标距离 | 判断动作尺度是否过大或过小 |
| `head_z` | head 三点目标高度 | 判断坐标系、身高和 body-local 处理是否合理 |
| `max_v` | 三点中最大的滤波后速度 | 判断是否过冲、抽动或限速过紧 |
| `lag` | 原始三点和滤波后三点的最大距离 | 判断滤波带来的滞后 |
| `fk` | 是否使用 G1 FK 参考标定 | `1` 表示使用 FK 标定，`0` 表示回退到旧的首帧平移标定 |

这些指标不会改变 planner 消息格式，只用于调试。后续如果 MuJoCo 里动作不自然，先看三点窗口和这组指标，再决定是调 `VR3PointRetargeter`、planner，还是补完整 IK。

## 可选上肢 IK

默认路径只发送 VR3PT 三点。为了验证 shoulder / elbow / wrist 方向的下一步优化空间，当前新增了一个默认关闭的上肢 IK 验证链路：

```bash
.venv_teleop/bin/python gear_sonic/scripts/mocap_manager_server.py \
  --source bvh \
  --bvh-file /home/nolo/RAYNOS_Motion1.bvh \
  --bvh-loop \
  --bvh-fps 25 \
  --target-fps 25 \
  --zmq-port 5556 \
  --enable-upper-body-ik
```

启用后，manager 会在普通 `vr_position` / `vr_orientation` 之外，额外发送：

```text
upper_body_position: 17 floats
upper_body_velocity: 17 floats
```

17 维顺序和 deploy 侧一致：

```text
waist_yaw, waist_roll, waist_pitch,
left_shoulder_pitch, left_shoulder_roll, left_shoulder_yaw, left_elbow,
left_wrist_roll, left_wrist_pitch, left_wrist_yaw,
right_shoulder_pitch, right_shoulder_roll, right_shoulder_yaw, right_elbow,
right_wrist_roll, right_wrist_pitch, right_wrist_yaw
```

实现要点：

- IK 使用 G1 Pinocchio robot model。
- wrist 目标来自已经标定和滤波后的 VR3PT 左右腕。
- wrist 目标点使用与 `get_g1_key_frame_poses()` 相同的本地 offset，不直接使用 wrist link 原点。
- 求解器使用 damped least-squares、关节限位裁剪、默认姿态正则和单步限幅。

启用后日志会多出：

```text
ik=1 ik_err=0.057m ik_dq=2.955rad ik_active=17 ik_v=3.81rad/s ik_margin=0.000rad
```

含义：

| 字段 | 含义 |
|------|------|
| `ik` | `1` 表示本帧发送了 `upper_body_position`，`0` 表示没有发送 |
| `ik_err` | 左右 wrist 中较大的 IK 位置误差 |
| `ik_dq` | 17 维上肢目标相对 G1 默认上肢姿态的最大关节偏移，单位 rad |
| `ik_active` | 相对默认姿态偏移超过 `0.02rad` 的上肢关节数量 |
| `ik_v` | 17 维上肢目标里的最大关节速度，单位 rad/s |
| `ik_margin` | 当前上肢关节离最近限位的最小余量 |

这几个值可以用来量化“加不加 `--enable-upper-body-ik` 有没有区别”。如果 `ik_dq` 接近 `0` 或 `ik_active=0`，说明 manager 侧基本没有生成新的上肢目标；如果 `ik_dq` 明显大于 `0` 且 `ik_active` 有多个关节，就说明 manager 侧已经产生了不同目标。deploy 侧没有 PICO 专用逻辑，只按通用 `planner` 消息消费字段，因此这里不把 deploy 作为 PICO / mocopi / BVH 输入源差异的量化对象。`ik_v` 用来判断目标是否过快，过大时容易造成抖动或被下游限幅。

如果 `ik_margin` 长期接近 `0`，说明目标容易把 G1 上肢推到关节限位，应先降低 wrist 目标尺度、增加正则，或继续补 elbow pole vector 约束。这个功能仍是实验路径，不建议直接替代默认 VR3PT 稳定链路。

## 与 PICO 遥操作的关系

PICO 遥操作当前链路：

```text
PICO/XR body pose
  -> PicoReader
  -> ThreePointPose
  -> PlannerStreamer
  -> planner topic
```

mocopi 输入链路：

```text
mocopi UDP
  -> MocopiUdpSource
  -> MocapFrame
  -> VR3PointRetargeter
  -> planner topic
```

BVH 回放链路：

```text
BVH file
  -> BvhPlaybackSource
  -> MocapFrame
  -> VR3PointRetargeter
  -> planner topic
```

两条链路最终都发布 deploy 侧已有字段：

- `vr_position`
- `vr_orientation`
- 可选手部关节字段
- planner movement / facing 字段

主要区别在于模块边界。PICO 采样、手柄输入、stream 切换和标定目前仍集中在原 PICO manager 中；mocopi 路径先拆成较小的 source/control 结构，便于后续继续接其他动捕设备。

## 当前限制

当前 mocopi manager 尚未实现：

- mocopi 骨架 FK，用于求 wrist/head 全局姿态。
- 官方 mocopi 骨架到机器人坐标系的完整标定。
- 身高和臂长比例归一化。
- 惯性动捕漂移补偿。
- 等价于 PICO trigger/grip 的手部控制。
- 与 PICO 共用的统一 manager 抽象。
- BVH 关节命名的外部配置文件，目前只内置了一组常见别名。
- BVH 三点路径虽已补 G1 FK 标定和滤波，但不等价于完整机器人重定向。

`--allow-bone-translation-vr` 只应作为调试选项使用，或者用于上游 bridge 已经写入有意义全局 translation 的情况。它不应作为最终 mocopi 重定向方案。

## 自然度与复原度优化路线

当前 mocopi / BVH 路径能让 MuJoCo 中的 G1 跟随动作，但动作自然度和 PICO 路径仍有差距。最初的根本原因是实现停留在轻量三点 retarget：

- 只使用 `left_wrist`、`right_wrist`、`head` 三个点。
- planner 消息里的 `left_hand_joints` / `right_hand_joints` 当前仍发送零值。
- 默认路径下 BVH 的肩、肘、躯干信息尚未直接参与 G1 上肢 IK。
- 已提供实验性 `--enable-upper-body-ik`，可把 VR3PT wrist 目标求解成 deploy 侧 17 维上肢关节目标，但还没有把 BVH elbow pole vector 作为约束。

现在 `VR3PointRetargeter` 已补齐第一阶段优化：

- 默认尝试使用 G1 FK 参考姿态做三点标定。
- 首帧会捕获 head / neck 初始朝向，并把后续 wrist/head 目标转到归一化后的 body frame。
- 左右腕会按 G1 FK 参考姿态计算位置 offset 和姿态 offset。
- FK 依赖不可用时默认回退到旧的首帧位置平移标定。
- 默认启用三点位置低通、四元数 slerp、最大速度和最大加速度限制。

对应工程提交：

```text
5e0ea54 feat: improve mocopi vr3pt retargeting
```

因此，“能动”不等于“能完整复原 BVH / mocopi 动作”。`PLANNER_VR_3PT` 会把外部三点目标交给 planner 和 policy 做稳定控制，下游本身也会把动作改造成 G1 可执行的形式。

优先级建议：

1. 已完成：把 PICO 的三点标定策略移植到 `VR3PointRetargeter`，包括 neck 朝向归一化、G1 FK 参考、左右腕位置 offset、左右腕姿态 offset。
2. 已完成：给三点目标增加滤波和限速，包括位置低通、四元数 slerp、最大速度和最大加速度限制。
3. 已完成：扩展 `MocapFrame` 的有效关节，至少保留 spine / chest / neck / head / shoulder / elbow / wrist / pelvis，不再只保留三点。
4. 已完成：在 manager 日志中增加 VR3PT 质量指标，包括腕距、head 高度、最大速度、滤波滞后和 FK 标定状态。
5. 已完成：新增默认关闭的上肢 IK 验证链路，能从 VR3PT wrist 目标发布 17 维 `upper_body_position` / `upper_body_velocity`。
6. 下一步：把 BVH shoulder-elbow-wrist 的 elbow pole vector 加入 IK 目标函数，减少 `ik_margin=0` 和肘部折叠方向不稳定。
7. 下一步：给 BVH 关节别名和坐标系 offset 做外部配置，避免不同 BVH 文件反复改代码。
8. 下一步：为 mocopi 官方二进制包补完整 FK / 标定层，把 27 bone 转成机器人 body frame 下的稳定三点和可选上肢目标。
9. 如果目标是“尽量复原离线 BVH”，应增加 pose/reference streaming 路径，直接输出 G1 `joint_pos`；如果目标是“实时遥操作稳定控制”，继续优先优化 planner VR 三点路径。

推荐先用默认优化参数跑。如果动作仍然滞后，可以提高：

```bash
--vr3pt-position-alpha 0.65 --vr3pt-orientation-alpha 0.65
```

如果动作仍然抖动，可以降低 alpha 或收紧速度：

```bash
--vr3pt-position-alpha 0.30 --vr3pt-orientation-alpha 0.30 --vr3pt-max-speed 2.0
```

## 验证方法

编译新增模块：

```bash
.venv_teleop/bin/python -m compileall \
  gear_sonic/utils/teleop/sources \
  gear_sonic/utils/teleop/controls \
  gear_sonic/utils/teleop/retarget \
  gear_sonic/scripts/mocap_manager_server.py
```

查看 CLI：

```bash
.venv_teleop/bin/python gear_sonic/scripts/mocap_manager_server.py --help
```

最小 mocopi JSON 运行检查：

1. 用 `--mocopi-format json` 启动 manager。
2. 向配置的 UDP 端口发送一帧 JSON bridge 数据。
3. 确认 manager 日志中出现 `vr_3pt=yes`。

最小 BVH 运行检查：

```bash
.venv_teleop/bin/python gear_sonic/scripts/mocap_manager_server.py \
  --source bvh \
  --bvh-file /path/to/motion.bvh \
  --bvh-loop \
  --bvh-fps 30 \
  --zmq-port 5556
```

如果 BVH 中能找到左右手和头部关节，日志应出现：

```text
vr_3pt=yes
```

## 下一步

推荐下一阶段补 mocopi retargeting 层：

```text
mocopi bones
  -> mocopi skeleton FK
  -> operator calibration
  -> robot-frame VR 3-point pose
  -> planner topic
```

这层稳定后，再把 PICO 和 mocopi 路径收敛成共享的 `TeleopManager`，并通过可插拔的 `MotionSource` 和 `ControlSource` 选择不同输入设备。
