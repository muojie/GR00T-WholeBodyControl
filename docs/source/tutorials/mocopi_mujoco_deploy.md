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

BVH POSE 验证链路：

```text
BVH source
  -> MocapFrame.full_body
  -> pose topic, protocol v3, ZMQ 5556
  -> deploy zmq_manager，command planner=false
  -> streamed motion / POSE reference
  -> MuJoCo sim loop
```

```{admonition} 关键点
:class: note
`mocap_manager_server.py` 默认只是 ZMQ publisher，不会自动显示机器人窗口。MuJoCo 机器人窗口由 `gear_sonic/scripts/run_sim_loop.py` 打开；VR 三点调试窗口由 `mocap_manager_server.py --visualize-vr3pt` 打开。
```

## 日志目录

调试时统一把三端日志保存到：

```text
/home/nolo/GR00T-WholeBodyControl/logs/mocopi_pose/latest/
```

开始新一轮测试前，在任意终端运行：

```bash
cd /home/nolo/GR00T-WholeBodyControl
mkdir -p logs/mocopi_pose/latest
rm -f logs/mocopi_pose/latest/*.log
```

下面的命令都使用 `2>&1 | tee ...` 保存日志，不要再用 `script -c`。`script` 的最后一个参数是输出文件，写错路径会覆盖源码文件；`tee` 更直观，适合当前三端调试。

## 终端 1：启动 MuJoCo 仿真

在仓库根目录运行：

```bash
cd /home/nolo/GR00T-WholeBodyControl

bash -lc 'cd /home/nolo/GR00T-WholeBodyControl && source .venv_sim/bin/activate && PYTHONUNBUFFERED=1 python -u gear_sonic/scripts/run_sim_loop.py' \
  2>&1 | tee /home/nolo/GR00T-WholeBodyControl/logs/mocopi_pose/latest/terminal1_mujoco.log
```

预期现象：

- MuJoCo viewer 窗口出现。
- 终端保持运行。

## 终端 2：启动 deploy 并订阅 ZMQ

在 `gear_sonic_deploy/` 下运行：

```bash
bash -lc 'cd /home/nolo/GR00T-WholeBodyControl/gear_sonic_deploy && stdbuf -oL -eL bash deploy.sh --input-type zmq_manager --zmq-host localhost --zmq-port 5556 sim' \
  2>&1 | tee /home/nolo/GR00T-WholeBodyControl/logs/mocopi_pose/latest/terminal2_deploy.log
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

默认 planner / VR3PT 链路启动后操作：

1. 在 deploy 终端按 `]` 启动控制。
2. 回到 MuJoCo 窗口按 `9`，把机器人放到地面。
3. 保持 deploy 终端运行，等待终端 3 的 planner 数据。

BVH POSE 链路不要在终端 3 启动前提前按 `]`。POSE 模式由终端 3 的 `command.start=true` 自动触发 streamed-motion 控制；如果需要手动按键，也等终端 3 出现 `pose=sent:N` 之后再按。

## 终端 3：启动 BVH 回放输入

在仓库根目录运行：

```bash
cd /home/nolo/GR00T-WholeBodyControl

bash -lc 'cd /home/nolo/GR00T-WholeBodyControl && PYTHONUNBUFFERED=1 .venv_teleop/bin/python -u gear_sonic/scripts/mocap_manager_server.py --source bvh --bvh-file /home/nolo/RAYNOS_Motion1.bvh --bvh-loop --bvh-fps 30 --zmq-port 5556' \
  2>&1 | tee /home/nolo/GR00T-WholeBodyControl/logs/mocopi_pose/latest/terminal3_mocap_manager.log
```

如果需要本地三点可视化窗口，加：

```bash
--visualize-vr3pt
```

完整命令：

```bash
bash -lc 'cd /home/nolo/GR00T-WholeBodyControl && PYTHONUNBUFFERED=1 .venv_teleop/bin/python -u gear_sonic/scripts/mocap_manager_server.py --source bvh --bvh-file /home/nolo/RAYNOS_Motion1.bvh --bvh-loop --bvh-fps 30 --zmq-port 5556 --visualize-vr3pt' \
  2>&1 | tee /home/nolo/GR00T-WholeBodyControl/logs/mocopi_pose/latest/terminal3_mocap_manager.log
```

如果还想在三点窗口中显示 G1 模型，加：

```bash
--visualize-g1
```

如果要验证实验性上肢 IK，再加：

```bash
--enable-upper-body-ik
```

### BVH POSE 模式

如果目标是验证 full-body `pose` topic，而不是默认三点 planner 链路，终端 3 使用：

```bash
bash -lc 'cd /home/nolo/GR00T-WholeBodyControl && PYTHONUNBUFFERED=1 .venv_teleop/bin/python -u gear_sonic/scripts/mocap_manager_server.py --source bvh --bvh-file /home/nolo/RAYNOS_Motion1.bvh --bvh-loop --bvh-fps 50 --target-fps 50 --control-mode pose --pose-window-size 80 --zmq-port 5556' \
  2>&1 | tee /home/nolo/GR00T-WholeBodyControl/logs/mocopi_pose/latest/terminal3_mocap_manager.log
```

这个命令会发送 `command.planner=false`，让 `zmq_manager` 进入 streamed-motion / POSE。日志中应能看到：

```text
pose_protocol=v3
pose=sent:N
```

POSE 模式下 manager 会等第一条 pose 窗口发出后再发送 `start=True`，避免 deploy 先进入控制但还没有 reference motion。当前默认 v3 消息包含 `smpl_joints`、`smpl_pose`、`body_quat_w`、`joint_pos`、`joint_vel`、`frame_index`；release policy 的 SMPL mode 需要 `joint_pos` 里的 wrist observation，因此不要把 `--pose-protocol-version 2` 当作 MuJoCo 主验证路径。

`--pose-window-size` 不要沿用早期的 `5`。release policy 的 SMPL mode 会读取类似 `10frame_step5` 的未来 observation，5 帧窗口太短，deploy 会频繁打印 `Motion streamed completed and waiting following motion`。当前 `--control-mode pose` 在不显式设置窗口时默认使用 80 帧；命令里保留 `--pose-window-size 80` 是为了让验证参数更清楚。日志中应出现 `Processing 80 frames`、`Merged streamed data: 80+ current-rate frames`，表示 streamed-motion 窗口足够长。

BVH 循环回放时，manager 发给 deploy 的 `frame_index` 必须保持单调递增。当前实现已经把 ZMQ `frame_index` 改成播放流编号，并在日志里额外打印原始 BVH 帧号：

```text
frame=1013 ... source_frame=87
```

这表示 BVH 文件已经 loop 回源帧 87，但 deploy 看到的全局帧仍是 1013。正常情况下 loop 边界后仍应继续看到 `pose=sent:N`，不应重新进入 `pose=buf:x/80`。

如果之前跑过 protocol v2，建议重启终端 2 的 deploy，再重启终端 3 的 manager。deploy 侧会记录 active protocol version，混用旧 v2 和新 v3 容易触发协议状态不一致。

### POSE 模式没有动作或机器人倒下

优先检查：

1. 终端 3 是否显示 `pose_protocol=v3` 和持续增长的 `pose=sent:N`。
2. 终端 2 是否使用 `--input-type zmq_manager`，而不是普通 `--input-type zmq --zmq-topic pose`。
3. 是否重启过 deploy，避免残留旧 protocol v2 状态。
4. deploy 日志是否出现 `Version 3 missing required field 'joint_pos'`、`motion has no joints`、`Failed to gather encoder observations` 等错误。

第一版 v2 只发 `smpl_*` 和 `body_quat`，没有 `joint_pos/joint_vel`，会导致 release SMPL mode 的 `motion_joint_positions_wrists_10frame_step1` 缺失。这是“manager 有 pose 日志但 MuJoCo 没动作”的主要原因。

第二个会导致“完全没反应”的问题是 streamed-motion 的启动转发。`zmq_manager` 初始在 planner mode，如果在终端 3 启动前就按 `]`，这个按键不会传给内部 POSE 接口；同时第一版 `ZMQManager` 没有把 `command.start=true` 转发到 streamed-motion 控制启动逻辑。当前已在 `ZMQManager` 中补齐：切到 streamed-motion 后收到 `command.start=true` 会设置 `operator_state.start=true`。修改后需要重新编译 deploy，并重启终端 2。

重新编译：

```bash
cd /home/nolo/GR00T-WholeBodyControl
cmake --build gear_sonic_deploy/build --target g1_deploy_onnx_ref -j2
```

### ChannelFactory / LowState 日志判断

`ChannelFactory create domain error` 需要结合上下文判断。当前 MuJoCo 仿真端已经去掉了 `run_sim_loop.py` 里的提前初始化，只保留 `BaseSimulator.__init__()` 中带异常保护的 `ChannelFactoryInitialize(...)`。如果初始化失败，会打印成：

```text
Note: Channel factory initialization attempt: ...
```

因此，如果 MuJoCo 仿真端没有直接退出，deploy 控制端日志后续又出现 `Init Done`，说明 MuJoCo 和 deploy 之间的 `rt/lowstate` 至少已经建立过，不要直接把这条日志当作 POSE 链路失败。

真正需要关注的是 deploy 是否长期收不到 LowState：

```text
LowState is not available, waiting for robot to be ready
```

如果这类日志一直持续且没有 `Init Done`，优先排查 MuJoCo 仿真端 / deploy 控制端的 Unitree DDS 接口是否一致。MuJoCo 端发布 `rt/lowstate`、订阅 `rt/lowcmd`；deploy 端订阅 `rt/lowstate`、发布 `rt/lowcmd`。这条 DDS 闭环不通时，ZMQ POSE 即使正常收到也无法驱动 MuJoCo。

`Lost LowState data connection from robot` 如果出现在手动停止 MuJoCo 仿真端或 deploy 控制端之后，通常只是进程停止后的副作用；如果出现在运行中，则表示 deploy 的 LowState 时间戳超过安全阈值，会触发 `Safety check failed` 并停止控制。

完整验证命令：

```bash
.venv_teleop/bin/python gear_sonic/scripts/mocap_manager_server.py \
  --source bvh \
  --bvh-file /home/nolo/RAYNOS_Motion1.bvh \
  --bvh-loop \
  --bvh-fps 25 \
  --target-fps 25 \
  --zmq-port 5556 \
  --visualize-vr3pt \
  --visualize-g1 \
  --enable-upper-body-ik
```

这个开关会额外发布 deploy 侧已有的 `upper_body_position` / `upper_body_velocity`。它不改变腿部控制；腿部仍由 locomotion planner 根据 `mode/movement/facing/speed/height` 生成。

## 预期日志

测试结束后，日志文件应包含：

```text
logs/mocopi_pose/latest/terminal1_mujoco.log
logs/mocopi_pose/latest/terminal2_deploy.log
logs/mocopi_pose/latest/terminal3_mocap_manager.log
```

快速定位命令：

```bash
cd /home/nolo/GR00T-WholeBodyControl
rg -n "ERROR|Error|missing|required|Failed|Protocol|STREAMED|ZMQ STREAMING|motion name|pose=|start requested|operator_state|ChannelFactory|LowState|倒|nan|NaN" \
  logs/mocopi_pose/latest/*.log
```

如果需要把最近一轮日志整体打包：

```bash
cd /home/nolo/GR00T-WholeBodyControl
tar -czf logs/mocopi_pose/latest.tar.gz -C logs/mocopi_pose latest
```

终端 3 应出现类似：

```text
[MocapManager] publishing planner data on tcp://*:5556; playing BVH ...
[MocapManager] recv_fps=25.0 recv=56 vr_3pt=yes span=0.371m head_z=0.398m max_v=1.350m/s lag=0.169m fk=1 dropped=0 frame=110 ... joints=12 ...
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

同一行里的质量指标用于判断动作是否合理：

| 字段 | 含义 | 判断方式 |
|------|------|----------|
| `span` | 左右腕目标距离 | 长期过小会像夹臂，长期过大会让上肢目标不可达 |
| `head_z` | head 目标高度 | 明显异常时优先检查坐标系、单位和 body-local 处理 |
| `max_v` | 三点最大速度 | 抽动时看它是否频繁接近 `--vr3pt-max-speed` |
| `lag` | 滤波前后三点最大偏差 | 越大越稳但越滞后，越小越跟手但可能抖 |
| `fk` | G1 FK 参考标定状态 | `1` 是正常优化路径，`0` 说明回退到了旧标定 |

如果启用了 `--enable-upper-body-ik`，还会出现：

```text
ik=1 ik_err=0.057m ik_dq=2.955rad ik_active=17 ik_v=3.81rad/s ik_margin=0.000rad
```

含义：

| 字段 | 含义 | 判断方式 |
|------|------|----------|
| `ik` | 是否发送了 17 维上肢关节目标 | `1` 表示本帧包含 `upper_body_position` |
| `ik_err` | 左右 wrist 中较大的 IK 位置误差 | 越小表示 wrist 目标越能被 G1 上肢达到 |
| `ik_dq` | 17 维上肢目标相对默认姿态的最大关节偏移 | 接近 `0` 表示 IK 目标几乎没改动；明显大于 `0` 表示 manager 侧已经生成不同上肢目标 |
| `ik_active` | 相对默认姿态偏移超过 `0.02rad` 的上肢关节数量 | `0` 表示 manager 侧没有实际上肢变化；多个关节变化表示 manager 侧已经生成不同目标 |
| `ik_v` | 17 维上肢目标中的最大关节速度 | 过大时容易被下游限幅，或表现成抖动、不自然 |
| `ik_margin` | 上肢关节离最近限位的最小余量 | 长期接近 `0` 表示目标贴近关节限位，需要调尺度或加肘部约束 |

这些是 mocap manager 侧的目标量化指标，能回答“加不加 `--enable-upper-body-ik` 是否生成了不同目标”。deploy 侧没有 PICO 专用逻辑，只按通用 `planner` 消息字段消费 `upper_body_position` / `upper_body_velocity`，因此不把 deploy 作为 PICO / mocopi / BVH 输入源差异的量化对象。如果 `ik_dq`、`ik_active` 已经明显变化但 MuJoCo 里看不出来，优先按通用 planner 消费链路和配置排查，不把它记作输入源量化指标。

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

### ZMQ input 类型选择

默认 VR3PT planner 链路不要用：

```bash
bash deploy.sh --input-type zmq --zmq-topic pose sim
```

因为默认 `mocap_manager_server.py` 发布的是 `command` + `planner`，需要 `zmq_manager` 同时订阅多个 topic。

默认三点 planner 和新增 BVH POSE 都建议使用：

```bash
bash deploy.sh --input-type zmq_manager --zmq-port 5556 sim
```

差别在终端 3 的 `--control-mode`：`planner` 会让 deploy 使用 planner topic；`pose` 会让 deploy 使用 pose topic。

### BVH 目标姿态不自然

BVH 默认路径仍是三点 retarget，但已经加入第一阶段优化：

- 默认查找 `LeftHand`、`RightHand`、`Head`。
- 默认尽量保留 `spine`、`chest`、`neck`、`head`、`shoulder`、`elbow`、`wrist`、`pelvis` 等有效关节；`RAYNOS_Motion1.bvh` 当前可匹配到 `joints=12`。
- 默认 Y-up 转 Z-up。
- 默认使用 head / neck 初始朝向做 body frame 归一化。
- 默认使用 G1 FK 参考姿态计算左右腕位置和姿态 offset。
- 默认启用三点位置滤波、四元数 slerp、速度和加速度限制。
- 默认在日志中输出 `span/head_z/max_v/lag/fk`，用于判断输入尺度、滤波滞后和 FK 标定状态。
- 可选 `--enable-upper-body-ik`，从 VR3PT wrist 目标求解并发布 17 维上肢关节目标。
- 如果 G1 FK 依赖不可用，默认回退到旧的首帧位置平移标定。
- shoulder / elbow / torso 信息已经进入 `MocapFrame`；当前 IK 先用 wrist 目标，尚未把 BVH elbow pole vector 纳入目标函数。
- 手部关节当前仍发送零值。
- 当前 `PLANNER_VR_3PT` 不携带完整下肢 tracker 信息。PICO 的腿部 tracker 数据主要通过 full-body `pose` topic / SMPL reference 路径进入 deploy；BVH 已新增第一版 `pose` topic 支持，mocopi 官方二进制包还需要补 27 bone -> SMPL 映射或由 bridge 直接输出 SMPL-like 字段。

对应工程提交：

```text
5e0ea54 feat: improve mocopi vr3pt retargeting
```

它还不是完整的人体到 G1 重定向。后续需要：

- 新增 `pose` topic / full-body reference stream，字段和 shape 对齐 PICO POSE 流。
- BVH 关节名配置化，避免不同 BVH 文件反复改代码。
- 更精细的手腕/head 姿态坐标系配置。
- 身高、臂长、肩宽比例处理。
- POSE 流打通后，再评估是否继续做上肢 IK 的 elbow pole vector、关节限位余量和目标尺度调优。

判断问题位置时按顺序看：

1. `--visualize-vr3pt --visualize-g1` 中三点本身是否顺滑、方向是否合理。
2. 日志里的 `span/head_z/max_v/lag/fk` 是否稳定、尺度是否合理。
3. 如果启用 IK，先看 `ik_dq/ik_active/ik_v` 判断 manager 是否生成了不同上肢目标，再看 `ik_err` 和 `ik_margin`。`ik_margin` 长期为 `0` 时，不要继续加大动作幅度，先调目标尺度或补 elbow pole vector。
4. 三点窗口合理但 MuJoCo 不自然，优先调 deploy / planner / compliance。
5. 三点窗口本身就飘、抖或左右手方向明显不对，优先改 `VR3PointRetargeter`。
6. 如果目标是完整复原 BVH 或利用腿部 tracker/full-body 信息，不要只依赖 `PLANNER_VR_3PT`，应使用 `--control-mode pose` 验证 `pose` topic / reference streaming。后续用 [Sony mocopi / BVH POSE 流支持追踪](mocopi_pose_stream.md) 单独记录。

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
