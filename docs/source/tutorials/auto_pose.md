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
3. 进 POSE 时自动执行 `pose_streamer.reset_yaw()`，并向机器人发送 `command` 消息（`start=True, stop=False, planner=False`）。

无需按任何组合键。终端会打印：

```
[Manager] auto_pose enabled: will enter POSE automatically on first data
[Manager] Auto-start: data received, entering POSE mode
```

### 为什么直接 OFF→POSE 与手动流程等价

机器人 C++ 端（`zmq_manager.hpp` 的 `OnCommandReceived`）对 command 消息的处理是：`start`/`stop` 用 **OR 逻辑累积**，`planner` 标志**取最新值**。手动流程先发 `(start=True, planner=True)` 再发 `(start=True, planner=False)`，最终状态与直接发一次 `(start=True, planner=False)` 完全相同。

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

## 操作建议

1. **数据源可控时（回放/合成数据/自建 UDP 源）**：让数据流第一帧从接近零位/站立的姿势开始，偏移和跳变问题都不存在。
2. **真人驱动时**：保持原操作习惯——先摆好零位标定姿势，再开始发送数据。
3. **启动顺序**：先启动机器人侧 C++ deploy 程序，再启动 manager。`start` 命令只在自动进 POSE 那一刻**发送一次**（ZMQ PUB 对未连接的订阅者直接丢消息，即 slow joiner 问题）；手动流程靠人按键天然有延迟，auto_pose 没有，必须靠启动顺序保证。
4. **急停**：A+B+X+Y 始终可用；机器人侧 C++ 终端的 `O`/`o` 键急停也不受影响。

## 相关代码位置

| 位置 | 内容 |
|------|------|
| `gear_sonic/scripts/pico_manager_thread_server.py` → `run_pico_manager` | 状态机与 auto_pose 分支（OFF 状态下收到首帧即标定+进 POSE） |
| 同文件 → `ThreePointPose.calibrate_now` | 零位标定实现（仅异常返回 False，不校验姿势） |
| 同文件 → `PoseStreamer.run_once` | POSE 主体数据计算（不依赖标定）与 `vr_position/vr_orientation`（依赖标定） |
| `gear_sonic_deploy/.../input_interface/zmq_manager.hpp` → `OnCommandReceived` | command 消息 start/stop OR 累积、planner 取最新的语义 |
