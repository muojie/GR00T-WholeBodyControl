# Sony mocopi 动捕管理器

本文档说明如何使用 Sony mocopi，或者基于 mocopi 的桥接程序，作为 SONIC 现有 ZMQ 部署链路的输入源。

当前实现刻意独立于 `pico_manager_thread_server.py`。这样可以先验证非 PICO 输入链路，而不影响已有 PICO/XR 遥操作流程。

```{admonition} 当前状态
:class: warning
这是第一版输入源集成层。它可以接收 mocopi UDP 数据，并发布现有 `command`、`planner`、`manager_state` ZMQ topic。真正可直接闭环控制机器人的路径目前依赖上游桥接程序提供 `vr_position` 和 `vr_orientation`；完整的 mocopi 骨架 FK 与标定层仍是后续工作。
```

## 代码位置

mocopi 输入链路由以下文件实现：

```text
gear_sonic/scripts/mocap_manager_server.py
gear_sonic/utils/teleop/sources/base.py
gear_sonic/utils/teleop/sources/mocopi_source.py
gear_sonic/utils/teleop/sources/__init__.py
gear_sonic/utils/teleop/controls/keyboard_control.py
gear_sonic/utils/teleop/controls/__init__.py
```

职责划分：

- `mocap_manager_server.py`：接收标准化后的动捕帧，并发布 deploy 侧兼容的 ZMQ 消息。
- `base.py`：定义 `Pose7D`、`MocapFrame`、`MocapSource` 等通用数据结构。
- `mocopi_source.py`：实现 UDP 接收、官方 mocopi 二进制包解析、JSON bridge 包解析。
- `keyboard_control.py`：提供基于 stdin 的行命令控制，用来替代 PICO 手柄按键。

## 数据流

```text
Sony mocopi app 或 mocopi bridge
  -> UDP，默认端口 12351
  -> MocopiUdpSource
  -> MocapFrame
  -> mocap_manager_server.py
  -> ZMQ PUB，默认端口 5556
      - command
      - planner
      - manager_state
  -> deploy 侧 ZMQ subscriber
  -> PLANNER_VR_3PT / locomotion policy
```

deploy 侧继续消费现有 ZMQ schema。第一版集成不需要在 deploy 侧新增 topic。

## 启动方式

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

常用参数：

| 参数 | 默认值 | 作用 |
|------|--------|------|
| `--mocopi-host` | `0.0.0.0` | UDP 绑定地址 |
| `--mocopi-port` | `12351` | UDP 绑定端口 |
| `--mocopi-format` | `auto` | 输入包格式，可选 `auto`、`binary`、`json` |
| `--zmq-port` | `5556` | deploy 侧订阅的 ZMQ PUB 端口 |
| `--target-fps` | `20` | planner 发布循环频率 |
| `--mocap-timeout-s` | `0.5` | 最新动捕帧超过该时间后停止发布 VR 三点目标 |
| `--start-paused` | 关闭 | 启动时不立即启用控制 |
| `--allow-bone-translation-vr` | 关闭 | 调试用途：把 bone translation 当作 VR 三点位置 |

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
  -> build_vr_3pt_from_frame
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
- 操作者到机器人坐标系的标定。
- 身高和臂长比例归一化。
- 惯性动捕漂移补偿。
- 等价于 PICO trigger/grip 的手部控制。
- 与 PICO 共用的统一 manager 抽象。

`--allow-bone-translation-vr` 只应作为调试选项使用，或者用于上游 bridge 已经写入有意义全局 translation 的情况。它不应作为最终 mocopi 重定向方案。

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

最小运行检查：

1. 用 `--mocopi-format json` 启动 manager。
2. 向配置的 UDP 端口发送一帧 JSON bridge 数据。
3. 确认 manager 日志中出现 `vr_3pt=yes`。

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
