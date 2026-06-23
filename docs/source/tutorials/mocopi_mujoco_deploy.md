# Sony mocopi / BVH-G1 POSE v1 的 MuJoCo 验证部署流程

本文记录如何把 `mocap_manager_server.py` 的输入链路接到 MuJoCo sim2sim 中验证。适用场景：

- BVH 文件回放验证。
- Sony mocopi bridge 已输出 `vr_position` / `vr_orientation` 后的闭环验证。
- 后续 mocopi FK / retargeting 完成后的仿真侧安全验证。

本文只记录当前跑通的 BVH-G1 POSE v1 部署流程。SMPL / protocol v3 是另一条研究路线，见 [Sony mocopi / SMPL POSE v3 路线](mocopi_pose_smpl_v3_route.md)；两条路线不要互相覆盖。

本文使用 [BVH-G1 POSE v1 路线](mocopi_pose_bvh_g1_v1_route.md) 中的 `--source bvh_stream`。如果要看 manager 直接读取本地 BVH 文件的 `--source bvh`，见同一路线页中的 BVH 输入源章节；不要把两个输入源的命令混在同一流程里。

当前推荐验证链路：

```text
BVH file
  -> bvh_stream_sender.py
  -> UDP bvh_stream_v1, 默认端口 12352
  -> mocap_manager_server.py --source bvh_stream
  -> BVH skeleton -> G1 29DOF retarget
  -> pose topic, protocol v1, encoder_mode=g1, ZMQ 5556
  -> deploy zmq_manager / streamed motion
  -> MuJoCo sim loop
```

保留的三点 planner 验证链路：

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

```{admonition} 热切换建议
:class: tip
当前 BVH-G1 POSE v1 路径不需要每次换动作都重启 MuJoCo 和 deploy。推荐让 MuJoCo、deploy、`mocap_manager_server.py --source bvh_stream` 三个进程常驻；换 BVH 时只停止并重启 `bvh_stream_sender.py`。MuJoCo 中机器人状态需要清空时，在 viewer 里按 `Backspace` reset。
```

## 日志目录

调试时统一把多端日志保存到：

```text
/home/nolo/GR00T-WholeBodyControl/logs/mocopi_pose/latest/
```

开始新一轮测试前，在任意终端运行：

```bash
cd /home/nolo/GR00T-WholeBodyControl
mkdir -p logs/mocopi_pose/latest
rm -f logs/mocopi_pose/latest/*.log
```

下面的命令都使用 `2>&1 | tee ...` 保存日志，不要再用 `script -c`。`script` 的最后一个参数是输出文件，写错路径会覆盖源码文件；`tee` 更直观，适合当前多端调试。

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

## 终端 3：启动 BVH stream manager

在仓库根目录运行：

```bash
cd /home/nolo/GR00T-WholeBodyControl

bash -lc 'cd /home/nolo/GR00T-WholeBodyControl && PYTHONUNBUFFERED=1 .venv_teleop/bin/python -u gear_sonic/scripts/mocap_manager_server.py --source bvh_stream --bvh-stream-port 12352 --control-mode pose --pose-window-size 80 --pose-encoder-mode g1 --pose-protocol-version 1 --zmq-port 5556 --log-interval-s 1.0' \
  2>&1 | tee /home/nolo/GR00T-WholeBodyControl/logs/mocopi_pose/latest/terminal3_mocap_manager.log
```

预期日志：

```text
control_mode=pose pose_stream=1 pose_protocol=v1 pose_window=80 pose_encoder=g1
listening for BVH stream UDP on 0.0.0.0:12352
pose=sent:1
recv_fps≈50 dropped=0
```

这个 manager 进程可以常驻。它只监听 BVH stream UDP 并向 deploy 发布 G1 joint reference，不绑定某一个 BVH 文件。

## 终端 4：发送 BVH stream

另开一个终端发送 BVH：

```bash
cd /home/nolo/GR00T-WholeBodyControl

bash -lc 'cd /home/nolo/GR00T-WholeBodyControl && PYTHONUNBUFFERED=1 .venv_teleop/bin/python -u gear_sonic/scripts/bvh_stream_sender.py --bvh-file /home/nolo/RAYNOS_Motion1.bvh --host 127.0.0.1 --port 12352 --loop' \
  2>&1 | tee /home/nolo/GR00T-WholeBodyControl/logs/mocopi_pose/latest/terminal4_bvh_stream_sender.log
```

预期日志：

```text
[BvhStreamSender] streaming /home/nolo/RAYNOS_Motion1.bvh to udp://127.0.0.1:12352 format=msgpack fps=50.0
```

换 BVH 文件时，只停止终端 4，然后用新的 `--bvh-file` 重新启动 sender。终端 1/2/3 不需要重启。新的 sender 从 `frame_index=0` 开始时，deploy 的 `StreamedMotionMerger` 会触发 catch-up reset，把 streamed motion 窗口切到新动作。

如果终端 4 启动得太早，UDP 前几帧可能在 manager 完成 bind 前丢失。实际操作时先等终端 3 出现 `listening for BVH stream UDP...`，再启动 sender。

### 旧的单进程 BVH POSE 模式

如果不需要热切换，也可以让 manager 自己读 BVH 文件并逐帧在线 retarget：

```bash
bash -lc 'cd /home/nolo/GR00T-WholeBodyControl && PYTHONUNBUFFERED=1 .venv_teleop/bin/python -u gear_sonic/scripts/mocap_manager_server.py --source bvh_g1 --bvh-file /home/nolo/RAYNOS_Motion1.bvh --bvh-loop --control-mode pose --pose-window-size 80 --pose-encoder-mode g1 --pose-protocol-version 1 --zmq-port 5556' \
  2>&1 | tee /home/nolo/GR00T-WholeBodyControl/logs/mocopi_pose/latest/terminal3_mocap_manager.log
```

这个路径仍然需要换文件时重启 manager，因此现在只作为回归对照或单文件验证使用。日志中应能看到：

```text
runtime=online, ik=analytic
pose_protocol=v1
pose_encoder=g1
pose=sent:N
```

POSE 模式下 manager 会等第一条 pose 窗口发出后再发送 `start=True`，避免 deploy 先进入控制但还没有 reference motion。当前推荐路径发送的是 protocol v1 + `encoder_mode=g1`，主要字段是 `joint_pos`、`joint_vel`、`body_pos`、`body_quat_w`、`frame_index`，不是把 BVH 或 mocopi 原始数据直接交给 deploy。

`--pose-window-size` 不要沿用早期的 `5`。release policy 会读取未来帧 observation，5 帧窗口太短，deploy 会频繁打印 `Motion streamed completed and waiting following motion`。当前命令保留 `--pose-window-size 80`，deploy 日志中应出现 `Processing 80 frames`、`Merged streamed data: 80+ current-rate frames`。

BVH 循环或新 sender 重启时，manager 发给 deploy 的 `frame_index` 应保持当前会话内单调递增；源 BVH 帧号写入日志的 `source_frame`：

```text
frame=1013 ... source_frame=87
```

这表示 BVH 文件已经 loop 回源帧 87，但 deploy 看到的全局帧仍是 1013。正常情况下 loop 边界后仍应继续看到 `pose=sent:N`，不应长时间回到 `pose=buf:x/80`。

### POSE 模式没有动作或机器人倒下

优先检查：

1. 终端 3 是否显示 `pose_protocol=v1`、`pose_encoder=g1` 和持续增长的 `pose=sent:N`。
2. 终端 2 是否使用 `--input-type zmq_manager`，而不是普通 `--input-type zmq --zmq-topic pose`。
3. 终端 4 sender 是否持续输出，终端 3 是否 `recv_fps≈50` 且 `dropped=0`。
4. deploy 日志是否出现 `missing required field 'joint_pos'`、`motion has no joints`、`Failed to gather encoder observations` 等错误。

历史 v2/v3 SMPL 路线只发或主要依赖 `smpl_*` 和 `body_quat`，容易遇到字段契约、骨架比例、root base rotation 和下肢稳定性问题。当前跑通的路径是 v1/G1 joint reference，先在 manager 侧把 BVH retarget 成 G1 `joint_pos/joint_vel`，再交给 deploy。

第二个会导致“完全没反应”的问题是 streamed-motion 的启动转发。`zmq_manager` 初始在 planner mode，如果在终端 3 启动前就按 `]`，这个按键不会传给内部 POSE 接口；同时第一版 `ZMQManager` 没有把 `command.start=true` 转发到 streamed-motion 控制启动逻辑。当前已在 `ZMQManager` 中补齐：切到 streamed-motion 后收到 `command.start=true` 会设置 `operator_state.start=true`。修改后需要重新编译 deploy，并重启终端 2。

如果已经有动作但仍会倒，优先看终端 3 的 POSE 诊断：

| 字段 | 判断重点 |
|------|----------|
| `q=[min,max]` | 29 维 `joint_pos` 范围是否异常放大 |
| `dq_abs` | `joint_vel` 是否有速度尖峰；若频繁顶到限幅，需要优先看输入帧跳变或 retarget 速度限幅 |
| `lower_dq` | 下肢 12 个 G1 关节相对默认站姿的偏差；当前 BVH-G1 主线会显式生成下肢参考，不应长期接近 `0` |
| `smpl_lz` | SMPL 下肢在 root-local 坐标里的 z 范围是否明显反向或尺度异常 |
| `smpl_lspan` | 下肢包围盒尺度是否接近人体比例 |
| `smpl_lpose` | SMPL 下肢 local pose 幅度是否过大 |
| `root_z` | POSE `body_pos` 的 root 高度；默认 BVH 路径应为 `0.793m`，不能是 `0.000m` |
| `root_tilt` | root 姿态是否大幅倾斜，过大时很容易切入后失稳 |

示例：

```text
pose=sent:24 q=[-1.15,0.98] dq_abs=0.00 lower_dq=0.00 smpl_lz=[-0.76,0.00] smpl_lspan=0.90m smpl_lpose=0.53rad root_z=0.793m root_tilt=0.53rad
```

如果当前跑的是 `bvh_stream` / `bvh_g1`，这类日志不应长期保持 `lower_dq=0.00`，否则说明没有进入 G1 retarget 主线。此前 streamed motion 未携带 `BodyPositions`，deploy 侧的 `motion_root_z_position` / `base_trans_target` 可能读到 `z=0`；当前已补 `body_pos` 字段，并在 deploy merger 中为旧消息加入 `{0,0,0.793}` 的 root 高度保底。如果仍会倒，下一步优先看 root 倾斜、G1 下肢关节幅度/速度是否过激，以及启动切入瞬间是否和当前机器人状态差太大。

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

旧 planner/VR3PT 的 `--source bvh` 调试命令不放在本文，见 [BVH-G1 POSE v1 路线](mocopi_pose_bvh_g1_v1_route.md) 中的 BVH 输入源章节。

## 预期日志

测试结束后，日志文件应包含：

```text
logs/mocopi_pose/latest/terminal1_mujoco.log
logs/mocopi_pose/latest/terminal2_deploy.log
logs/mocopi_pose/latest/terminal3_mocap_manager.log
logs/mocopi_pose/latest/terminal4_bvh_stream_sender.log
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
[MocapManager] publishing on tcp://*:5556; control_mode=pose pose_stream=1 pose_protocol=v1 pose_window=80 pose_encoder=g1 ...
[MocapManager] listening for BVH stream UDP on 0.0.0.0:12352
[MocapManager] recv_fps=50.0 recv=120 pose=sent:N dropped=0 ...
```

重点看：

```text
pose_protocol=v1
pose_encoder=g1
pose=sent:N
recv_fps≈50 dropped=0
```

含义：

- manager 已经进入 POSE v1 + G1 encoder 主线。
- BVH stream UDP 已被接收并转为 G1 reference。
- manager 正在向 `pose` topic 发布 streamed-motion 窗口。

如果切回旧 planner/VR3PT 调试路径，同一行里的质量指标用于判断三点目标是否合理：

| 字段 | 含义 | 判断方式 |
|------|------|----------|
| `span` | 左右腕目标距离 | 长期过小会像夹臂，长期过大会让上肢目标不可达 |
| `head_z` | head 目标高度 | 明显异常时优先检查坐标系、单位和 body-local 处理 |
| `max_v` | 三点最大速度 | 抽动时看它是否频繁接近 `--vr3pt-max-speed` |
| `lag` | 滤波前后三点最大偏差 | 越大越稳但越滞后，越小越跟手但可能抖 |
| `fk` | G1 FK 参考标定状态 | `1` 是正常优化路径，`0` 说明回退到了旧标定 |

如果旧 planner/VR3PT 路径启用了 `--enable-upper-body-ik`，还会出现：

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

做动作自然度评估时，当前优先使用 `bvh_stream` 路径：

```bash
.venv_teleop/bin/python gear_sonic/scripts/mocap_manager_server.py \
  --source bvh_stream \
  --bvh-stream-port 12352 \
  --control-mode pose \
  --pose-window-size 80 \
  --pose-encoder-mode g1 \
  --pose-protocol-version 1 \
  --zmq-port 5556
```

另开 sender：

```bash
.venv_teleop/bin/python gear_sonic/scripts/bvh_stream_sender.py \
  --bvh-file /home/nolo/RAYNOS_Motion1.bvh \
  --host 127.0.0.1 \
  --port 12352 \
  --loop
```

不要使用 `BVH 实际 25 Hz + manager 20 Hz` 这类不一致组合来判断复原质量，因为它会叠加采样抖动。

## 成功判断

基础成功：

- 终端 1 MuJoCo 窗口正常。
- 终端 2 deploy 正常启动，使用 `--input-type zmq_manager`。
- MuJoCo 窗口按过 `9`。
- 终端 3 显示 `pose_protocol=v1`、`pose_encoder=g1`、`pose=sent:N`。
- 终端 4 sender 持续发送，终端 3 显示 `recv_fps≈50` 且 `dropped=0`。

进一步成功：

- MuJoCo 中机器人进入 policy 控制。
- 机器人全身动作对 BVH stream 有响应，且不需要每次换 BVH 重启 MuJoCo / deploy / manager。
- 如果开启 planner/VR3PT 调试路径，本地三点窗口能看到三点坐标系在更新。

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
2. 终端 3 是否显示 `pose=sent:N`，而不是一直停在 `pose=buf:x/80`。
3. 终端 4 是否在向 `127.0.0.1:12352` 发送 BVH stream。
4. MuJoCo 窗口是否按过 `9`。
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
- 当前 `PLANNER_VR_3PT` 不携带完整下肢 tracker 信息。PICO 的腿部 tracker 数据主要通过 full-body `pose` topic / SMPL reference 路径进入 deploy；BVH-G1 已新增稳定的 `POSE v1 + encoder_mode=g1` 主线，mocopi 官方二进制包还需要补 27 bone -> canonical skeleton / G1 retarget，或由 bridge 直接输出可重定向的骨架帧。

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
6. 如果目标是完整复原 BVH 或利用腿部 tracker/full-body 信息，不要只依赖 `PLANNER_VR_3PT`，应先选择 POSE 路线。路线对比见 [Sony mocopi / BVH POSE 路线总览](mocopi_pose_stream.md)。

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
