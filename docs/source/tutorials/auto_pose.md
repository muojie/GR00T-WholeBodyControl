# Auto-POSE 免按键启动（知识库）

本文档说明 `pico_manager_thread_server.py` 的 **auto_pose 自动进 POSE 模式**功能：为什么不再需要按 A+B+X+Y 和 A+X、标定语义发生了什么变化、以及"不在零位标定姿势时启动"的行为分析。

## 背景：原按键启动流程

manager（`gear_sonic/scripts/pico_manager_thread_server.py` 中的 `run_pico_manager`）维护一个流模式状态机，原始启动流程为：

```
OFF ──(A+B+X+Y，同时做零位标定)──► PLANNER ──(A+X)──► POSE
```

- **A+B+X+Y**：启动策略，并用"按下瞬间的身体追踪帧"调用 `three_point.calibrate_now()` 做零位标定（要求操作员站在零位参考姿势）。
- **A+X**：从 PLANNER 切入 POSE（全身模仿模式），机器人开始跟随流入的 SMPL 姿态。

## auto_pose 功能

**默认开启**。manager 启动后处于 OFF 状态，一旦 `PicoReader` 收到第一帧身体追踪数据：

1. 用这一帧调用 `three_point.calibrate_now()` 做零位标定；
2. 标定成功则直接切入 **POSE** 模式（跳过 PLANNER），失败则留在 OFF、下一帧重试；
3. 进 POSE 时自动执行 `pose_streamer.reset_yaw()`，并向机器人发送 `command` 消息（`start=True, stop=False, planner=False`）；
4. 之后在所有非 OFF 模式下，**以 1 Hz 周期性重发当前模式的 command 消息**（保活），确保晚连接或中途重启的机器人端也能收到 start（见下文 FAQ）。

无需按任何组合键。终端会打印：

```
[Manager] auto_pose enabled: will enter POSE automatically on first data
[Manager] Auto-start: data received, entering POSE mode
```

### 为什么直接 OFF→POSE 需要配套的 C++ 修复

机器人 C++ 端（`zmq_manager.hpp` 的 `OnCommandReceived`）在**接收层**对 command 消息的处理是：`start`/`stop` 用 OR 逻辑累积，`planner` 标志取最新值——这一层直接发 `(start=True, planner=False)` 与手动流程等价。

但在**消费层**原本不等价：`start` 标志只有 PLANNER 模式的 `handlePlannerInput` 会消费；STREAMED_MOTION 模式把输入委托给 pose 接口，而 pose 接口的 start 只能由 C++ 终端键盘 `]` 键触发。手动流程恰好先经过 PLANNER（A+B+X+Y 发 `planner=True`），start 在那里被消费掉；直接 OFF→POSE 则没人消费 start。**本仓库已在 `ZMQManager::handle_input` 的 STREAMED_MOTION 分支补上 start 消费逻辑**（语义与 `]` 键一致），此后两条路径才真正等价。详见下文 FAQ 根因二。

### 保留的按键功能

- **A+B+X+Y**：任意模式下仍是急停（→ OFF 并退出进程）。
- **A+X / B+Y / 摇杆按压**：进入 POSE 后仍可正常切换 PLANNER、PLANNER_FROZEN_UPPER_BODY、VR_3PT 等模式。

### 关闭自动模式

需要恢复原按键流程时，加 `--no_auto_pose`：

```bash
python gear_sonic/scripts/pico_manager_thread_server.py --manager --no_auto_pose
```

## FAQ：不在零位标定姿势时启动，会失败吗？

**不会启动失败。** `calibrate_now` 无法检测操作员是否真的处于零位姿势——它只是把"收到的第一帧"无条件当作零位参考：拿当前帧的头+双腕位姿，与"机器人全零关节 FK"做差，把差值存为偏移量。只有发生**异常**（机器人模型未加载、数据形状错误等）才返回 False。只要数据格式正常，标定必然成功，状态机照常进 POSE，机器人照常启动。

但"能启动"不等于"没影响"，后果分两层：

### 1. 三点（头+双腕）映射带固定偏移

标定的语义是"你此刻的姿势 = 机器人的全零姿势"。若标定瞬间不在零位姿势（例如手臂下垂），之后整段会话中 pose 消息里的 `vr_position` / `vr_orientation` 三点数据都带系统性偏差。**影响范围**：VR_3PT 模式及依赖三点数据的功能。表现为"整个会话机器人手腕位置总是偏一截"，解决办法是重新标定（停止后摆好姿势重启）。

### 2. POSE 模式的主体跟随不受标定影响

`PoseStreamer.run_once` 发给机器人的主体数据——`smpl_pose`、`smpl_joints`、`joint_pos`（腕关节角直接由 SMPL 肘/腕轴角解算）——是从收到的 body poses **直接计算**的，完全不经过 `three_point` 的标定；yaw 也在进 POSE 时由 `reset_yaw()` 独立归零。所以纯 POSE 全身模仿不会因标定姿势不对而错乱。

### 真正的风险：进 POSE 瞬间的姿势跳变

auto_pose 一收到数据就开始跟随。若此刻人的姿势与机器人当前姿势（通常为站立位）差异很大，参考轨迹第一帧就是大跳变，机器人可能猛动。手动流程中文档要求"先把手臂对齐机器人再按 A+X"，就是为了规避这一点。

## FAQ：进了 POSE，但机器人不动？

**症状**：manager 日志正常（`StreamMode switch: OFF -> POSE`、PoseLoop 50 FPS 持续发送），但机器人保持静止。这个问题有**两层叠加的根因**，都已修复。

**根因一：ZMQ slow joiner 丢掉一次性 start 命令（manager 端已修复）**

ZMQ PUB 对"尚未连接的订阅者"发送的消息**直接丢弃**。`start` 命令原本只在模式切换瞬间发送一次；auto_pose 收到首帧数据就立刻切 POSE，距 socket bind 可能不到 1 秒，机器人端 SUB 往往还没连上，命令就丢了。修复：manager 在所有非 OFF 模式下以 1 Hz 重发当前模式的 command 消息（保活）。附带收益：机器人端程序中途重启后约 1 秒自动重新接入。

**根因二：start 命令在 STREAMED_MOTION 模式下无人消费（C++ 端已修复）**

即使命令送达，`ZMQManager` 原本**只在 PLANNER 模式**（`handlePlannerInput`）消费 start 标志；STREAMED_MOTION 模式把输入委托给 pose 接口，后者的 start 只能靠 C++ 终端 `]` 键。auto_pose 直接发 `planner=False`，机器人立刻切到 STREAMED_MOTION——start 从未被消费，`WAIT_FOR_CONTROL` 永远等待。手动流程不踩这个坑纯粹因为它先经过 PLANNER 模式。修复：在 `ZMQManager::handle_input` 的 STREAMED_MOTION 分支补上 start 消费（与 `]` 键语义一致）。**该修复是 C++ 改动，需要重新编译——`deploy.sh` 每次运行会自动 build，正常重跑即可。**

保活重发在机器人端的安全性验证：

| C++ 端行为 | 位置 | 结论 |
|-----------|------|------|
| `if (start_control_ && !operator_state.start)` | `zmq_manager.hpp` | start 幂等，已启动时重复收到是空操作 |
| 模式切换仅在 planner 标志变化时触发 | `zmq_manager.hpp` | 重发相同模式不会触发 safety reset |
| `Input()` 在 stop 后直接早退 | `g1_deploy_onnx_ref.cpp` | 已急停的机器人不再消费命令，不会被重新拉起 |

**验证方法**：机器人端 C++ 终端按顺序出现以下日志即为链路健康：

```
[ZMQManager] Switched to: STREAMED MOTION mode (safety reset)   ← 命令送达（根因一排除）
[ZMQManager] ZMQ streaming enabled
[ZMQManager] Start command consumed in streamed-motion mode     ← start 被消费（根因二排除）
[Control] DEBUG: operator_state.start=true, transitioning to CONTROL state
```

**临时手动兜底**：在机器人 C++ 终端按 `]` 键 = 直接触发 start（pose 接口的启动键），可用于现场快速验证是否卡在 start 消费环节。

**如果第一行都不出现**，说明命令根本到不了机器人，按顺序检查：

1. 机器人端启动参数是否使用 ZMQ 输入（如 `--input-type zmq_manager`）；
2. 机器人端连接的 host/port 是否指向 manager 所在机器的发布端口（默认 5556；跨机器时 `--zmq-host` 不能是 localhost）；
3. 机器人终端是否有任何 `[ZMQManager]` 收包日志——一条都没有即网络/地址问题。

## 操作建议

1. **数据源可控时（回放/合成数据/自建 UDP 源）**：让数据流第一帧从接近零位/站立的姿势开始，偏移和跳变问题都不存在。
2. **真人驱动时**：保持原操作习惯——先摆好零位标定姿势，再开始发送数据。
3. **启动顺序**：不再有硬性要求。command 消息以 1 Hz 保活重发（见 FAQ），机器人端无论先启动、后启动还是中途重启，都会在约 1 秒内收到 start 并进入 CONTROL。
4. **急停**：A+B+X+Y 始终可用；机器人侧 C++ 终端的 `O`/`o` 键急停也不受影响（急停后保活重发不会重新拉起机器人）。

## 相关代码位置

| 位置 | 内容 |
|------|------|
| `gear_sonic/scripts/pico_manager_thread_server.py` → `run_pico_manager` | 状态机与 auto_pose 分支（OFF 状态下收到首帧即标定+进 POSE）、1 Hz command 保活重发 |
| 同文件 → `ThreePointPose.calibrate_now` | 零位标定实现（仅异常返回 False，不校验姿势） |
| 同文件 → `PoseStreamer.run_once` | POSE 主体数据计算（不依赖标定）与 `vr_position/vr_orientation`（依赖标定） |
| `gear_sonic_deploy/.../input_interface/zmq_manager.hpp` → `OnCommandReceived` / `update` | command 消息 start/stop OR 累积、planner 取最新、start 幂等守卫 |
| 同文件 → `handle_input` STREAMED_MOTION 分支 | start 命令在流式模式下的消费（根因二修复处，语义同 `]` 键） |
| `gear_sonic_deploy/.../src/g1_deploy_onnx_ref.cpp` → `Control()` | `WAIT_FOR_CONTROL` 等待 `operator_state.start`，送达时打印 DEBUG 转换日志 |
