# A+B+X+Y 组合键无响应排查（知识库）

本文档说明 manager（`gear_sonic/scripts/pico_manager_thread_server.py`）中**按 A+B+X+Y（或 A+X / B+Y）组合键没有反应**时的排查方法，以及为此内置的四层跟踪日志。

## 组合键的工作原理

manager 主循环每次迭代通过 XRoboToolkit（xrt）轮询 PICO 手柄按键，组合键判定是**电平 AND + 上升沿触发**：

```python
start_combo = a_pressed and b_pressed and x_pressed and y_pressed  # 电平 AND
if start_combo and not prev_start_combo:                           # 上升沿才触发
    ...
```

要点：

- 四个键必须**在同一次轮询时刻同时处于按下状态**（循环 ≥50 Hz，正常按压窗口足够）；
- **按住不放只触发一次**，松开后再按才会再次触发；
- PICO 上 **A/B 在右手柄、X/Y 在左手柄**——任何一只手柄休眠或未连接，组合键必然无法成立。

故障链路共四段：`xrt 读取 → 组合判定 → 状态机分支 → 模式切换`，日志按层覆盖。

## 四层跟踪日志

### 1. 底层读取失败告警（`get_abxy_buttons`）

xrt 读按键抛异常时，原实现会静默返回全 False（最隐蔽的故障点）。现在会限频打印（每秒最多一条）：

```
[Buttons] WARNING: A/B/X/Y read failed, treating as not pressed: <异常>
```

### 2. 按键状态变化日志

任何键按下/松开都会打印一行原始状态：

```
[Manager] Buttons: A=1 B=1 X=1 Y=0 | A+X=1 B+Y=0 A+B+X+Y=0 | mode=POSE
```

组合键"没反应"最常见的原因就是某个键从未到过 1，这一行能直接看出缺席的是哪个键。

### 3. 组合键边沿检测日志

组合被识别到（上升沿）时打印：

```
[Manager] Combo A+B+X+Y detected (mode=POSE)
```

特殊情况：**auto_pose 开启时，OFF 状态下的 A+B+X+Y 会被有意忽略**（自动等首帧数据，见 [Auto-POSE 免按键启动](auto_pose.md)），此时会明确提示：

```
[Manager] A+B+X+Y pressed in OFF, but auto_pose is active: combo ignored, ...
```

### 4. 5 秒心跳

```
[Manager] Poll alive: mode=POSE A=0 B=0 X=0 Y=0
```

用于区分"主循环卡死没在轮询"和"循环正常但按键读数全为 False"。

## 诊断决策表

按住 A+B+X+Y 观察日志，对照下表：

| 日志现象 | 结论 | 处理 |
|---------|------|------|
| 连 `Poll alive` 心跳都没有 | 主循环卡死 | 查看是否阻塞在 planner 初始化等待等处，抓栈定位 |
| 有心跳 + `[Buttons] WARNING` | xrt 读取路径故障 | 检查 XRoboToolkit 服务/手柄连接；自建 UDP 数据源场景 PICO 手柄可能根本不在线 |
| 有心跳、按键恒为 0、无 WARNING | xrt 正常返回"未按下" | 手柄休眠/未配对/不在追踪范围，唤醒或重连手柄 |
| 某一侧两个键恒为 0（如 X/Y） | 对应手柄（左）休眠或断连 | 唤醒该手柄 |
| `Combo A+B+X+Y detected` 出现但无 `StreamMode switch` | 状态机分支未接受该组合 | 对照状态机：OFF+auto_pose 下组合被忽略属预期；其余情况检查代码 |
| `Combo ... detected` + `StreamMode switch: XXX -> OFF` 后进程退出 | **一切正常** | POSE 下按 A+B+X+Y 的设计行为就是发 stop 命令并 `exit()` 退出 manager |

## 常见根因备忘

1. **数据源已改为自建 UDP 流、PICO 手柄不在线**：身体数据照常流入（走 UDP），但按键仍从 xrt 读取——手柄不在线时按键永远是 False。此场景下按键功能整体失效，急停请用机器人侧 C++ 终端的 `O`/`o` 键。
2. **单侧手柄休眠**：PICO 手柄静置会休眠，A/B（右）或 X/Y（左）整组读 0。
3. **auto_pose 忽略 OFF 状态的启动组合**：这是设计行为，用 `--no_auto_pose` 恢复按键启动流程。
4. **对"成功"的预期不符**：POSE 下 A+B+X+Y = 急停并**退出进程**，不是"暂停后继续运行"。暂停/恢复请用 A+X（回 PLANNER）或按住左 menu 键（POSE_PAUSE）。

## 相关代码位置

| 位置 | 内容 |
|------|------|
| `pico_manager_thread_server.py` → `get_abxy_buttons` | 按键读取 + 失败告警（限频 1 条/秒） |
| 同文件 → `run_pico_manager` 主循环开头 | 按键变化日志、组合边沿日志、5 秒心跳 |
| 同文件 → 状态机 OFF 分支 | auto_pose 下忽略 A+B+X+Y 的提示 |
| `docs/source/tutorials/auto_pose.md` | auto_pose 行为与启动顺序注意事项 |
