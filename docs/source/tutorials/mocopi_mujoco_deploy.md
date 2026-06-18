# Sony mocopi / BVH 的 MuJoCo 验证部署流程

本文记录如何把 `mocap_manager_server.py` 的输入链路接到 MuJoCo sim2sim 中验证。适用场景：

- BVH 文件回放验证。
- Sony mocopi bridge 已输出 `vr_position` / `vr_orientation` 后的闭环验证。
- 后续 mocopi FK / retargeting 完成后的仿真侧安全验证。

当前验证链路：

```text
BVH / mocopi source
  -> MocapFrame
  -> VR3PointRetargeter
  -> planner topic, ZMQ 5556
  -> deploy zmq_manager
  -> MuJoCo sim loop
```

```{admonition} 关键点
:class: note
`mocap_manager_server.py` 默认只是 ZMQ publisher，不会自动显示机器人窗口。MuJoCo 机器人窗口由 `gear_sonic/scripts/run_sim_loop.py` 打开；VR 三点调试窗口由 `mocap_manager_server.py --visualize-vr3pt` 打开。
```

## 终端 1：启动 MuJoCo 仿真

在仓库根目录运行：

```bash
cd /home/nolo/GR00T-WholeBodyControl

source .venv_sim/bin/activate
python gear_sonic/scripts/run_sim_loop.py
```

预期现象：

- MuJoCo viewer 窗口出现。
- 终端保持运行。

## 终端 2：启动 deploy 并订阅 ZMQ

在 `gear_sonic_deploy/` 下运行：

```bash
cd /home/nolo/GR00T-WholeBodyControl/gear_sonic_deploy

bash deploy.sh \
  --input-type zmq_manager \
  --zmq-host localhost \
  --zmq-port 5556 \
  sim
```

这里必须使用：

```text
--input-type zmq_manager
```

因为 `mocap_manager_server.py` 发布的是：

```text
command
planner
manager_state
```

普通 `--input-type zmq --zmq-topic pose` 是 pose/reference streaming 路径，不是 planner 三点控制路径。

启动后操作：

1. 在 deploy 终端按 `]` 启动控制。
2. 回到 MuJoCo 窗口按 `9`，把机器人放到地面。
3. 保持 deploy 终端运行，等待终端 3 的 planner 数据。

## 终端 3：启动 BVH 回放输入

在仓库根目录运行：

```bash
cd /home/nolo/GR00T-WholeBodyControl

.venv_teleop/bin/python gear_sonic/scripts/mocap_manager_server.py \
  --source bvh \
  --bvh-file /home/nolo/RAYNOS_Motion1.bvh \
  --bvh-loop \
  --bvh-fps 30 \
  --zmq-port 5556
```

如果需要本地三点可视化窗口，加：

```bash
--visualize-vr3pt
```

完整命令：

```bash
.venv_teleop/bin/python gear_sonic/scripts/mocap_manager_server.py \
  --source bvh \
  --bvh-file /home/nolo/RAYNOS_Motion1.bvh \
  --bvh-loop \
  --bvh-fps 30 \
  --zmq-port 5556 \
  --visualize-vr3pt
```

如果还想在三点窗口中显示 G1 模型，加：

```bash
--visualize-g1
```

## 预期日志

终端 3 应出现类似：

```text
[MocapManager] publishing planner data on tcp://*:5556; playing BVH ...
[MocapManager] recv_fps=25.0 recv=56 vr_3pt=yes dropped=0 frame=110 ...
```

重点看：

```text
vr_3pt=yes
```

含义：

- BVH 已被解析。
- 左腕、右腕、头部三点已提取。
- `VR3PointRetargeter` 已生成 `vr_position` / `vr_orientation`。
- manager 正在向 `planner` topic 发布三点目标。

## 当前样例解释

如果运行：

```bash
.venv_teleop/bin/python gear_sonic/scripts/mocap_manager_server.py \
  --source bvh \
  --bvh-file /home/nolo/RAYNOS_Motion1.bvh \
  --bvh-loop \
  --bvh-fps 30 \
  --zmq-port 5556
```

日志中出现：

```text
playing BVH /home/nolo/RAYNOS_Motion1.bvh at 25.0 Hz (source_fps=50.0, stride=2, loop=True)
```

解释：

- BVH 原始 FPS 是 `50`。
- 目标 `--bvh-fps 30` 低于源 FPS。
- 当前实现按整数 stride 降采样，`stride=2`。
- 实际播放 FPS 因此是 `50 / 2 = 25 Hz`。

这不是错误。如果需要更接近 30 Hz，后续要实现插值重采样，而不是整数 stride 跳帧。

做动作自然度评估时，建议先让回放频率和 manager 发布频率一致：

```bash
.venv_teleop/bin/python gear_sonic/scripts/mocap_manager_server.py \
  --source bvh \
  --bvh-file /home/nolo/RAYNOS_Motion1.bvh \
  --bvh-loop \
  --bvh-fps 50 \
  --target-fps 50 \
  --zmq-port 5556 \
  --visualize-vr3pt \
  --visualize-g1
```

如果 50 Hz 负载较高，可以统一降到：

```bash
--bvh-fps 25 --target-fps 25
```

不要使用 `BVH 实际 25 Hz + manager 20 Hz` 这类不一致组合来判断复原质量，因为它会叠加采样抖动。

## 成功判断

基础成功：

- 终端 1 MuJoCo 窗口正常。
- 终端 2 deploy 正常启动，并按过 `]`。
- MuJoCo 窗口按过 `9`。
- 终端 3 显示 `vr_3pt=yes`。

进一步成功：

- MuJoCo 中机器人进入 policy 控制。
- 机器人上半身、手腕或 head target 对 BVH 三点输入有响应。
- 如果开启 `--visualize-vr3pt`，本地三点窗口能看到三点坐标系在更新。

## 常见问题

### 只有 `vr_3pt=yes`，但没有窗口

原因：默认不打开窗口。

解决：

```bash
--visualize-vr3pt
```

如果加了参数仍没有窗口，检查：

```bash
echo $DISPLAY
```

还要确认 `.venv_teleop` 中有 PyVista/VTK。

### MuJoCo 机器人不动

优先检查：

1. 终端 2 是否使用了 `--input-type zmq_manager`。
2. 终端 2 是否按过 `]`。
3. MuJoCo 窗口是否按过 `9`。
4. 终端 3 是否显示 `vr_3pt=yes`。
5. 端口 `5556` 是否被旧进程占用。

### 不要用普通 ZMQ pose 模式

不要用：

```bash
bash deploy.sh --input-type zmq --zmq-topic pose sim
```

除非输入源发布的是 `pose` topic。

当前 `mocap_manager_server.py` 发布 planner 控制消息，应使用：

```bash
bash deploy.sh --input-type zmq_manager --zmq-port 5556 sim
```

### BVH 目标姿态不自然

当前 BVH 路径仍是三点 retarget，但已经加入第一阶段优化：

- 默认查找 `LeftHand`、`RightHand`、`Head`。
- 默认 Y-up 转 Z-up。
- 默认使用 head / neck 初始朝向做 body frame 归一化。
- 默认使用 G1 FK 参考姿态计算左右腕位置和姿态 offset。
- 默认启用三点位置滤波、四元数 slerp、速度和加速度限制。
- 如果 G1 FK 依赖不可用，默认回退到旧的首帧位置平移标定。
- 没有使用 shoulder / elbow / torso 信息做上肢 IK。
- 手部关节当前仍发送零值。

对应工程提交：

```text
5e0ea54 feat: improve mocopi vr3pt retargeting
```

它还不是完整的人体到 G1 重定向。后续需要：

- BVH 关节名配置化。
- 更精细的手腕/head 姿态坐标系配置。
- 身高、臂长、肩宽比例处理。
- 基于肩肘腕的 G1 上肢 IK。

判断问题位置时按顺序看：

1. `--visualize-vr3pt --visualize-g1` 中三点本身是否顺滑、方向是否合理。
2. 三点窗口合理但 MuJoCo 不自然，优先调 deploy / planner / compliance。
3. 三点窗口本身就飘、抖或左右手方向明显不对，优先改 `VR3PointRetargeter`。
4. 如果目标是完整复原 BVH，而不是实时稳定遥操作，需要新增 pose/reference streaming，把 BVH retarget 成 G1 `joint_pos`，不要只依赖 `PLANNER_VR_3PT`。

调参建议：

```bash
# 更跟手，但可能更抖
--vr3pt-position-alpha 0.65 --vr3pt-orientation-alpha 0.65

# 更稳，但滞后更明显
--vr3pt-position-alpha 0.30 --vr3pt-orientation-alpha 0.30 --vr3pt-max-speed 2.0

# 对比旧行为
--no-vr3pt-fk-calibration --no-vr3pt-filter
```

## 推荐验证顺序

1. 先只运行终端 3，并加 `--visualize-vr3pt`，确认 BVH 三点窗口能动。
2. 再运行终端 1 + 终端 2，确认 MuJoCo sim2sim 闭环。
3. 再把 BVH 输入换成 JSON bridge 输入，验证 `vr_position` / `vr_orientation`。
4. 最后换真实 mocopi UDP 输入，补 mocopi FK 和标定。
