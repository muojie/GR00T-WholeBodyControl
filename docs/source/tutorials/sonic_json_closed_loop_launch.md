# SONIC JSON 闭环一键启动（pose / planner-follow）

本文说明如何用一键脚本把 Sony mocopi `saveBoneData*.json` 跑成 SONIC 闭环，重点是
`CONTROL_MODE=pose|planner` 的切换，以及 planner 模式下的 **root 轨迹跟随**
（planner-follow，移植自 BVH `--follow-trajectory`）。两个后端共用同一套环境变量与
命令行参数：

- `scripts/launch_sonic_json_mujoco_closed_loop.sh` —— 统一前门，带 `--backend`，
  默认 `mujoco` 本地闭环，也能 `--backend isaaclab` 切到 IsaacLab。
- `scripts/launch_sonic_json_isaaclab_closed_loop.sh` —— IsaacLab 后端脚本，可单独用。

数据链路参见 [Sony BoneData JSON raw stream sender](sony_bonedata_json_stream_sender.md)；
外接 Windows IsaacLab 的完整桥接见
[Windows IsaacLab deploy bridge](windows_isaaclab_deploy_bridge.md)。

## TL;DR：你想跑的命令

**mujoco 本地闭环 + planner-follow（root_yaw 朝向）：**

```bash
REPLACE=1 CONTROL_MODE=planner \
  MANAGER_EXTRA_ARGS="--planner-follow-facing-source root_yaw" \
  scripts/launch_sonic_json_mujoco_closed_loop.sh \
  --backend mujoco /home/nolo/saveBoneData_Yup20260702.json
```

**IsaacLab 外接（Windows 上跑 IsaacLab，本机只跑接收端）+ planner-follow：**

```bash
REPLACE=1 CONTROL_MODE=planner \
  MANAGER_EXTRA_ARGS="--planner-follow-facing-source root_yaw" \
  scripts/launch_sonic_json_mujoco_closed_loop.sh \
  --backend isaaclab --no-isaaclab --no-json-sender --windows-ip 192.168.1.137 \
  /home/nolo/saveBoneData_Yup20260702.json
```

这两条命令等价效果：`CONTROL_MODE=planner` 且 `PLANNER_FOLLOW`（默认 1）会自动带上
`--planner-follow-stream-root`，`MANAGER_EXTRA_ARGS` 里的 `--planner-follow-facing-source
root_yaw` 会原样追加给 `mocap_manager_server.py`。两个后端都支持
`--no-isaaclab / --no-json-sender / --windows-ip`（这三个仅对 IsaacLab 后端有意义）。

> 说明：`--windows-ip` 说明 IsaacLab 跑在别的机器（Windows）上，因此要配
> `--backend isaaclab --no-isaaclab`。上面第一条示例里我保留了你原来的
> `--backend mujoco`，那是**本机 MuJoCo 闭环**，此时 `--no-isaaclab / --windows-ip`
> 不生效。请按目标后端二选一。

## 三种模式（MODE）

| 参数 | MODE | 起哪些窗口 |
|---|---|---|
| （默认） | `all` | 接收端全套 + JSON sender |
| `--receiver-only` / `--no-json-sender` | `receiver` | 接收端全套，不起 sender（之后用 `--sender-only` 手动起） |
| `--sender-only` | `sender` | 只起 JSON sender |
| `--print-sender-command` | `print_sender` | 只打印 sender 命令后退出 |

- **mujoco 后端**接收端窗口：`mujoco` / `manager` / `deploy`（+ `json_sender`）。
- **isaaclab 后端**接收端窗口：由 Python 启动器
  `launch_sonic_local_isaaclab_closed_loop.py` 拉起 `input`(mocap manager) / `proxy`
  (C++ lowstate) / `deploy`，本机 IsaacLab 走 `isaaclab` 窗口（`--no-isaaclab` 时不起，
  改用外部 `sonic_state` 端点），再加 `sony_json_sender`。

planner-follow 需要**持续的骨骼流**来推算 root 轨迹，所以 sender 必须在跑：要么用默认
`all`，要么先 `--no-json-sender` 起接收端、等 IsaacLab 就绪后再 `--sender-only` 补起。

## 核心环境变量

| 变量 | 默认 | 作用 |
|---|---|---|
| `CONTROL_MODE` | `pose` | `pose`=流式动作直传；`planner`=走 deploy planner |
| `PLANNER_FOLLOW` | `1` | 仅 `planner` 模式生效：`1` 自动加 `--planner-follow-stream-root`（用流式 root 轨迹生成 move/face/speed），`0` 回退 stdin 手动指令 |
| `MANAGER_EXTRA_ARGS` | 空 | 原样追加给 `mocap_manager_server.py` 的额外参数（见下表微调项） |
| `REPLACE` | `1` | 先杀掉同名 tmux session |
| `MODE` | `all` | 见上（也可用命令行参数覆盖） |
| `COORDINATE_FRAME` | `left_handed_yup` | 接收侧 BoneData 坐标系转换 |
| `FPS` | `50` | sender 发送帧率 |
| `JSON_FILE` | 见脚本 | JSON 路径（也可用位置参数或 `--json-file`） |

**端口（默认随后端不同）：**

| 变量 | mujoco 后端 | isaaclab 后端 |
|---|---|---|
| `BVH_STREAM_PORT` | `12362` | `12352` |
| `MOCAP_ZMQ_PORT` | `5656` | `5556` |
| `DEBUG_PORT` | `5657` | `5557` |
| `STATE_PORT` | `5560` | `5560`（IsaacLab `sonic_state`） |

> 通过 `--backend isaaclab` 从 mujoco 前门切过去时，若未显式设置端口，会自动采用
> isaaclab 后端的默认端口（12352/5556/5557）。

## IsaacLab 外接相关参数

| 参数 | 作用 |
|---|---|
| `--no-isaaclab` | 不在本机起 IsaacLab，改用外部 `sonic_state` 端点 |
| `--windows-ip HOST` / `--isaac-state-host HOST` | 外部 IsaacLab 发布 `sonic_state` 的主机/IP，端点为 `tcp://HOST:STATE_PORT` |
| `--isaac-state-endpoint tcp://HOST:PORT` | 直接给完整端点，覆盖上面两个 |

例如 `--no-isaaclab --windows-ip 192.168.1.137` → 本机接收端会去
`tcp://192.168.1.137:5560` 订阅 IsaacLab 状态。

## planner-follow 微调（放进 `MANAGER_EXTRA_ARGS`）

以下是 `mocap_manager_server.py` 在 planner-follow 模式下的可调项，两个后端都能透传
（写进 `MANAGER_EXTRA_ARGS`，多个参数用空格分隔并整体加引号）：

| 参数 | 默认 | 作用 |
|---|---|---|
| `--planner-follow-facing-source travel\|root_yaw` | `travel` | 朝向来源：`travel` 跟行进方向（稳，不做原地转身）；`root_yaw` 跟参考 root 偏航（能复现原地整圈旋转，实验性） |
| `--planner-follow-speed-scale FLOAT` | `1.0` | 把流式 root 速度乘以该系数后作为 planner speed 指令 |
| `--planner-follow-speed-max FLOAT` | `0.6` | planner speed 指令上限 |
| `--planner-follow-upper-body-from-stream` | 默认开 | 上肢直传流式重定向的 17 关节角（保真度同 pose 模式） |
| `--no-planner-follow-upper-body-from-stream` | — | 关闭上肢直传，回退 3 点 VR 手臂路径 |

`--planner-follow-stream-root` 本身**不用**手动写进 `MANAGER_EXTRA_ARGS`——
`CONTROL_MODE=planner` + `PLANNER_FOLLOW=1` 会自动带上。

## 常用配方

```bash
# 1) pose 模式（默认），mujoco 本地闭环
REPLACE=1 scripts/launch_sonic_json_mujoco_closed_loop.sh \
  --backend mujoco /home/nolo/saveBoneData_Yup20260702.json

# 2) planner-follow（默认 travel 朝向），mujoco 本地闭环
REPLACE=1 CONTROL_MODE=planner scripts/launch_sonic_json_mujoco_closed_loop.sh \
  --backend mujoco /home/nolo/saveBoneData_Yup20260702.json

# 3) planner-follow + root_yaw 朝向 + 限速微调，mujoco 本地闭环
REPLACE=1 CONTROL_MODE=planner \
  MANAGER_EXTRA_ARGS="--planner-follow-facing-source root_yaw --planner-follow-speed-max 0.8" \
  scripts/launch_sonic_json_mujoco_closed_loop.sh \
  --backend mujoco /home/nolo/saveBoneData_Yup20260702.json

# 4) IsaacLab 外接（Windows）+ planner-follow root_yaw，先只起接收端
REPLACE=1 CONTROL_MODE=planner \
  MANAGER_EXTRA_ARGS="--planner-follow-facing-source root_yaw" \
  scripts/launch_sonic_json_mujoco_closed_loop.sh \
  --backend isaaclab --no-isaaclab --no-json-sender --windows-ip 192.168.1.137 \
  /home/nolo/saveBoneData_Yup20260702.json
# 等 Windows IsaacLab 就绪后，再补起 JSON sender：
scripts/launch_sonic_json_mujoco_closed_loop.sh --backend isaaclab \
  --sender-only /home/nolo/saveBoneData_Yup20260702.json

# 5) 只打印 sender 命令（复制到别的终端手动跑）
scripts/launch_sonic_json_mujoco_closed_loop.sh --backend isaaclab \
  --print-sender-command /home/nolo/saveBoneData_Yup20260702.json

# 6) planner 模式但关掉自动跟随（用 stdin 手动 move/face 指令）
REPLACE=1 CONTROL_MODE=planner PLANNER_FOLLOW=0 \
  scripts/launch_sonic_json_mujoco_closed_loop.sh \
  --backend mujoco /home/nolo/saveBoneData_Yup20260702.json
```

也可以直接用 IsaacLab 脚本（不经 mujoco 前门），参数完全一致：

```bash
REPLACE=1 CONTROL_MODE=planner \
  MANAGER_EXTRA_ARGS="--planner-follow-facing-source root_yaw" \
  scripts/launch_sonic_json_isaaclab_closed_loop.sh \
  --no-isaaclab --no-json-sender --windows-ip 192.168.1.137 \
  /home/nolo/saveBoneData_Yup20260702.json
```

## 实现说明：IsaacLab 后端如何透传这些参数

mujoco 后端脚本直接生成 `manager.sh` 拼 `mocap_manager_server.py`，`MANAGER_EXTRA_ARGS`
天然可用。IsaacLab 后端多一层委托，链路是：

```text
launch_sonic_json_mujoco_closed_loop.sh --backend isaaclab
  -> launch_sonic_json_isaaclab_closed_loop.sh
  -> launch_sonic_local_isaaclab_closed_loop.py   (Python 启动器)
  -> mocap_manager_server.py
```

Python 启动器接收 `--control-mode` / `--planner-follow-stream-root` /
`--manager-extra-args` 三个参数并转发给 mocap manager。其中 `MANAGER_EXTRA_ARGS`
作为**单个字符串**经 `--manager-extra-args` 传入，启动器内部用 `shlex.split()` 还原成
多个参数，所以像 `"--planner-follow-facing-source root_yaw"` 这种带空格的组合会被正确
拆开。`CONTROL_MODE` / `PLANNER_FOLLOW` / `MANAGER_EXTRA_ARGS` 在 mujoco 前门
`--backend isaaclab` 委托时会一并转发，两条入口行为一致。

## 排障

- **端口占用**：脚本启动前会检查 `BVH_STREAM_PORT / MOCAP_ZMQ_PORT / DEBUG_PORT`，
  被占用会直接报错退出。改端口或 `tmux kill-session -t <session>` 后重来。
- **planner 模式没走位/原地不动**：确认 sender 在跑（planner-follow 依赖持续骨骼流），
  且 `CONTROL_MODE=planner`（启动日志会打印 `control_mode=... planner_follow=...`）。
- **IsaacLab 外接收不到状态**：检查 `--windows-ip` 对应机器上 IsaacLab 是否在
  `STATE_PORT`(默认 5560) 发布 `sonic_state`，以及防火墙/网段。
- **想确认到底传了什么**：`--control-mode` 非法值会被 `choices` 拦下；启动日志里的
  `control_mode` 一行可快速核对模式与跟随开关。
