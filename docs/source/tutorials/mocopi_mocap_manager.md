# Sony mocopi 动捕管理器

本文档说明如何使用 Sony mocopi、基于 mocopi 的桥接程序，或者 BVH 文件回放，作为 SONIC 现有 ZMQ 部署链路的输入源。

当前实现刻意独立于 `pico_manager_thread_server.py`。这样可以先验证非 PICO 输入链路，而不影响已有 PICO/XR 遥操作流程。

```{admonition} 当前状态
:class: warning
这是第一版输入源集成层。它可以接收 mocopi UDP 数据、回放 BVH 文件，并发布现有 `command`、`planner`、`manager_state` ZMQ topic。真正可直接闭环控制机器人的 mocopi 路径目前依赖上游桥接程序提供 `vr_position` 和 `vr_orientation`；完整的 mocopi 骨架 FK 与标定层仍是后续工作。
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
gear_sonic/utils/teleop/sources/mocopi_source.py
gear_sonic/utils/teleop/sources/__init__.py
gear_sonic/utils/teleop/controls/keyboard_control.py
gear_sonic/utils/teleop/controls/__init__.py
gear_sonic/utils/teleop/retarget/vr3pt_retargeter.py
gear_sonic/utils/teleop/retarget/__init__.py
```

职责划分：

- `mocap_manager_server.py`：接收标准化后的动捕帧，并发布 deploy 侧兼容的 ZMQ 消息。
- `base.py`：定义 `Pose7D`、`MocapFrame`、`MocapSource` 等通用数据结构。
- `mocopi_source.py`：实现 UDP 接收、官方 mocopi 二进制包解析、JSON bridge 包解析。
- `bvh_source.py`：实现 BVH hierarchy/motion 解析、FK、循环回放，并输出 `MocapFrame`。
- `keyboard_control.py`：提供基于 stdin 的行命令控制，用来替代 PICO 手柄按键。
- `vr3pt_retargeter.py`：把标准化后的动捕帧转换为 deploy 侧需要的 `vr_position` / `vr_orientation`。

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

## 启动 BVH 文件回放

BVH 回放适合在没有 mocopi 硬件时验证后续链路，也适合调试三点 retargeting：

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

BVH source 会解析 hierarchy 和 motion 数据，执行 FK，提取 `LeftHand`、`RightHand`、`Head` 等关节，再通过 `VR3PointRetargeter` 转成 deploy 侧需要的 VR 三点目标。

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
| `--source` | `mocopi` | 输入源，可选 `mocopi` 或 `bvh` |
| `--mocopi-host` | `0.0.0.0` | UDP 绑定地址 |
| `--mocopi-port` | `12351` | UDP 绑定端口 |
| `--mocopi-format` | `auto` | 输入包格式，可选 `auto`、`binary`、`json` |
| `--bvh-file` | 无 | `--source bvh` 时要回放的 BVH 文件 |
| `--bvh-loop` | 关闭 | BVH 播放到末尾后循环 |
| `--bvh-fps` | BVH 原始 FPS | BVH 目标回放 FPS；低于原始 FPS 时按 stride 跳帧 |
| `--bvh-unit-scale` | `0.01` | BVH 坐标单位转米，厘米制 BVH 使用 `0.01` |
| `--bvh-no-y-up-to-z-up` | 关闭 | 禁用 BVH Y-up 到 SONIC Z-up 的坐标转换 |
| `--bvh-world-frame` | 关闭 | 保留 BVH root 全局平移，不做 body-local 化 |
| `--zmq-port` | `5556` | deploy 侧订阅的 ZMQ PUB 端口 |
| `--target-fps` | `20` | planner 发布循环频率 |
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
官方 mocopi 数据包还不是机器人可直接使用的 VR 三点目标。它主要提供 mocopi 骨架 transform。要稳定得到机器人坐标系下的 `left_wrist`、`right_wrist`、`head` 全局目标，还需要补骨架 FK 和标定层。
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

如果 BVH 文件使用不同关节命名，需要后续扩展 `BVH_DEFAULT_JOINT_ALIASES`，或者先在上游转换成兼容命名。

BVH source 输出的是标准 `MocapFrame`，其中包含：

```text
left_wrist
right_wrist
head
root
```

随后 `VR3PointRetargeter` 会进行首帧位置标定，让第一帧对齐 deploy 侧默认 VR 三点姿态。这样回放 BVH 时不会因为 BVH 原始站位离机器人默认参考姿态太远而产生明显跳变。

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
- BVH 的肩、肘、躯干等信息尚未参与 G1 上肢 IK。

现在 `VR3PointRetargeter` 已补齐第一阶段优化：

- 默认尝试使用 G1 FK 参考姿态做三点标定。
- 首帧会捕获 head / neck 初始朝向，并把后续 wrist/head 目标转到归一化后的 body frame。
- 左右腕会按 G1 FK 参考姿态计算位置 offset 和姿态 offset。
- FK 依赖不可用时默认回退到旧的首帧位置平移标定。
- 默认启用三点位置低通、四元数 slerp、最大速度和最大加速度限制。

因此，“能动”不等于“能完整复原 BVH / mocopi 动作”。`PLANNER_VR_3PT` 会把外部三点目标交给 planner 和 policy 做稳定控制，下游本身也会把动作改造成 G1 可执行的形式。

优先级建议：

1. 已完成：把 PICO 的三点标定策略移植到 `VR3PointRetargeter`，包括 neck 朝向归一化、G1 FK 参考、左右腕位置 offset、左右腕姿态 offset。
2. 已完成：给三点目标增加滤波和限速，包括位置低通、四元数 slerp、最大速度和最大加速度限制。
3. 下一步：扩展 `MocapFrame` 的有效关节，至少保留 shoulder / elbow / wrist / head / pelvis，不要只取三点。
4. 下一步：基于 shoulder-elbow-wrist 做 G1 上肢 IK，目标函数同时考虑 wrist 位置、wrist 朝向、肘部方向、关节限位和上一帧平滑项。
5. 下一步：给 BVH 关节别名和坐标系 offset 做外部配置，避免不同 BVH 文件反复改代码。
6. 下一步：为 mocopi 官方二进制包补完整 FK / 标定层，把 27 bone 转成机器人 body frame 下的稳定三点和可选上肢目标。
7. 如果目标是“尽量复原离线 BVH”，应增加 pose/reference streaming 路径，直接输出 G1 `joint_pos`；如果目标是“实时遥操作稳定控制”，继续优先优化 planner VR 三点路径。

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
