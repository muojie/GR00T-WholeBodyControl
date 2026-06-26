# Windows IsaacLab 作为 SONIC deploy 仿真端

本文记录当前推荐的双机闭环：

```text
Sony / BVH / mocap
  -> Ubuntu mocap_manager_server.py
  -> Ubuntu deploy
  -> ZMQ g1_debug target, tcp://*:5557
  -> Windows IsaacLab sonic_robot
  -> ZMQ sonic_state, tcp://*:5560
  -> Ubuntu sonic_unitree_lowstate_cpp_proxy
  -> DDS rt/lowstate / rt/secondary_imu
  -> Ubuntu deploy
```

这条路线的目标是让 IsaacLab 保持在 Windows 侧，以便使用 Isaac Sim / IsaacLab streaming；同时把 Unitree DDS 留在 Ubuntu 本机，避免 Windows 原生 CycloneDDS 网卡发现问题。

## 是否需要改 IsaacLab 代码

当前不需要改核心代码。现有 IsaacLab 仓库已经有三部分：

- `SonicDeployTargetAction`：Windows IsaacLab 订阅 deploy 的 ZMQ `g1_debug` 目标。
- `SonicRobotStatePublisherAction`：Windows IsaacLab 发布 `sonic_robot` 的真实仿真状态到 ZMQ `sonic_state`。
- `sonic_unitree_lowstate_cpp_proxy.cpp`：Ubuntu 侧把 Windows 发来的 `sonic_state` 转成 Unitree DDS `rt/lowstate` / `rt/secondary_imu`。

需要做的是按下面的端口和环境变量启动。

## 端口

| 端口 | 方向 | 用途 |
| --- | --- | --- |
| `5556` | mocap manager -> deploy | POSE / planner 输入 |
| `5557` | deploy -> Windows IsaacLab | `g1_debug` 目标流 |
| `5560` | Windows IsaacLab -> Ubuntu proxy | `sonic_state` 真实仿真状态 |

Ubuntu 上的 deploy 和 lowstate proxy 使用本机 DDS loopback，即 `sim` / `lo`。这会把测试和真实机器人网卡隔离开。

## Ubuntu：编译 lowstate C++ proxy

依赖：

```bash
sudo apt-get install libzmq3-dev libmsgpack-dev
```

编译：

```bash
cd /home/nolo/GR00T-WholeBodyControl

mkdir -p gear_sonic_deploy/build/tools

g++ -std=c++20 -O2 \
  /home/nolo/xiaoyang_IssacLab/IsaacLab/scripts/tools/sonic_unitree_lowstate_cpp_proxy.cpp \
  -I/home/nolo/GR00T-WholeBodyControl/gear_sonic_deploy/thirdparty/unitree_sdk2/include \
  -L/home/nolo/GR00T-WholeBodyControl/gear_sonic_deploy/thirdparty/unitree_sdk2/lib/$(uname -m) \
  -lunitree_sdk2 -lzmq -lmsgpackc -pthread \
  -o /home/nolo/GR00T-WholeBodyControl/gear_sonic_deploy/build/tools/sonic_unitree_lowstate_cpp_proxy
```

如果链接器提示缺少 CycloneDDS 相关库，说明本机 Unitree SDK2 静态库还需要额外链接系统 DDS 库；优先改成通过 `gear_sonic_deploy` 的 CMake 接入该 proxy，而不是在 Windows 上绕回 DDS。

## Ubuntu：启动 lowstate proxy

把 `<windows_ip>` 换成 Windows IsaacLab 机器的 IP：

```bash
cd /home/nolo/GR00T-WholeBodyControl

gear_sonic_deploy/build/tools/sonic_unitree_lowstate_cpp_proxy \
  --interface lo \
  --domain-id 0 \
  --lowcmd-topic rt/lowcmd \
  --lowstate-topic rt/lowstate \
  --secondary-imu-topic rt/secondary_imu \
  --isaac-state-endpoint tcp://<windows_ip>:5560 \
  --isaac-state-topic sonic_state \
  --isaac-state-timeout 0.25
```

日志里 `src=isaac` 表示 proxy 正在使用 Windows IsaacLab 回传的真实状态；`src=synthetic` 表示还没收到 Windows 状态，正在用内部一阶跟随状态兜底。

## Ubuntu：启动 deploy

这里 deploy 仍用 `sim`，让 DDS 走 loopback，并保持 simulator 兼容逻辑。

```bash
cd /home/nolo/GR00T-WholeBodyControl/gear_sonic_deploy

bash deploy.sh \
  --input-type zmq_manager \
  --zmq-host localhost \
  --zmq-port 5556 \
  --output-type zmq \
  --zmq-out-port 5557 \
  --zmq-out-topic g1_debug \
  sim
```

## Windows：启动 IsaacLab

PowerShell 网络环境变量：

```powershell
$env:ISAACLAB_MACHINE_A_IP="<ubuntu_ip>"
$env:ISAACLAB_MACHINE_B_IP="<windows_ip>"
$env:ISAACLAB_LOCAL_MACHINE_IP="<windows_ip>"
$env:ISAACLAB_TRACKING_HUB_IP="<ubuntu_ip>"

$env:SONIC_DEPLOY_TRANSPORT="zmq"
$env:SONIC_DEPLOY_ENDPOINT="tcp://<ubuntu_ip>:5557"
$env:SONIC_DEPLOY_TOPIC="g1_debug"
$env:SONIC_DEPLOY_TARGET_FIELD="last_action"
$env:SONIC_DEPLOY_REFERENCE_TARGET_FIELD="body_q_target"

$env:SONIC_PUBLISH_STATE_ZMQ="1"
$env:SONIC_STATE_ZMQ_BIND="tcp://*:5560"
$env:SONIC_STATE_ZMQ_TOPIC="sonic_state"
```

网络参数含义：

- `ISAACLAB_MACHINE_A_IP`：Ubuntu / deploy 机器 IP。
- `ISAACLAB_MACHINE_B_IP`：Windows / IsaacLab 机器 IP。
- `ISAACLAB_LOCAL_MACHINE_IP`：Windows 本机实际 IP；IsaacLab 会用它 bind 本机同步端口。
- `ISAACLAB_TRACKING_HUB_IP`：tracking hub / Ubuntu 侧 IP。
- `SONIC_DEPLOY_ENDPOINT`：Windows IsaacLab 订阅 Ubuntu deploy 的目标流，默认端口 `5557`。
- `SONIC_STATE_ZMQ_BIND`：Windows IsaacLab 发布仿真状态，Ubuntu proxy 连接这个地址，默认端口 `5560`。

Windows 防火墙需要允许 IsaacLab 入站 TCP `5560`；Ubuntu 侧需要能连到 `tcp://<windows_ip>:5560`。

PowerShell IsaacLab 参数，与一键启动默认保持一致：

```powershell

$env:SONIC_G1_PHYSICS_MODE="1"
$env:SONIC_G1_VISUAL_SERVO_MODE="0"
$env:SONIC_G1_SELF_COLLISIONS="0"
$env:SONIC_DEPLOY_STABILIZE_ROOT="1"
$env:SONIC_DEPLOY_TARGET_RATE_LIMIT="0.04"
```

这组值与一键启动默认保持一致：`--physics-mode 1`、`--visual-servo-mode 0`、`--self-collisions 0`、`--stabilize-root 1`、`--target-rate-limit 0.04`。

启动任务：

```powershell
cd D:\path\to\IsaacLab
.\isaaclab.bat -p scripts\environments\teleoperation\teleop_se3_agent.py `
  --task Isaac-SonicSolo-Locomanipulation-G1-v0 `
  --device cpu `
  --kit_args "--/app/vsync=false --/app/runLoops/main/rateLimitEnabled=false"
```

`Isaac-SonicSolo-Locomanipulation-G1-v0` 对应一键启动里的 `--isaac-scene solo`，是 Windows 侧先验证 streaming / deploy target / state publisher 的最小空场景。

`ISAACLAB_LOCAL_MACHINE_IP` 必须是 Windows 本机真实 IP。否则 IsaacLab 的双机同步 action 会尝试在默认地址 `192.168.50.68:15565` 上 bind，并报：

```text
Address not available (addr='tcp://192.168.50.68:15565')
```

如果需要临时排查固定 root 行为，可以单独覆盖：

```powershell
$env:SONIC_G1_PHYSICS_MODE="0"
$env:SONIC_DEPLOY_STABILIZE_ROOT="1"
```

这不是标准 Windows 启动路径；标准路径应继续和一键启动保持一致。

## 当前场景完整手动启动命令

本节对应一键启动器的 `--backend isaaclab --input-source sony --bvh-source bvh_stream --isaac-scene solo`，但把 IsaacLab pane 移到 Windows PowerShell，其余端仍在 Ubuntu。

推荐启动顺序与 launcher 保持一致：Windows IsaacLab -> Ubuntu C++ proxy -> Ubuntu deploy -> Ubuntu mocap manager -> BVH sender。

### Windows 端：IsaacLab / SonicSolo

#### 推荐：启动脚本

```powershell
powershell -ExecutionPolicy Bypass -File "<GR00T_ROOT>\scripts\start_windows_isaaclab_sonic.ps1" `
  -UbuntuIp "<ubuntu_ip>" `
  -WindowsIp "<windows_ip>" `
  -IsaacLabRoot "D:\path\to\IsaacLab"
```

脚本参数说明：

- `-UbuntuIp`：Ubuntu / deploy 机器 IP（必填）
- `-WindowsIp`：Windows 本机 IP（建议显式传）
- `-IsaacLabRoot`：IsaacLab 安装目录（默认当前目录）
- `-Task`：任务名，默认 `Isaac-SonicSolo-Locomanipulation-G1-v0`
- `-Device`：设备，默认 `cpu`
- `-DebugPort`：`SONIC_DEPLOY_ENDPOINT` 中使用的端口，默认 `5557`
- `-StatePort`：`SONIC_STATE_ZMQ_BIND` 中使用的端口，默认 `5560`
- `-Headless`：是否 headless
- `-EnablePinocchio`：是否启用 `--enable_pinocchio`

#### 手工启动（等价）

```powershell
cd D:\path\to\IsaacLab

$env:ISAACLAB_MACHINE_A_IP="<ubuntu_ip>"
$env:ISAACLAB_MACHINE_B_IP="<windows_ip>"
$env:ISAACLAB_LOCAL_MACHINE_IP="<windows_ip>"
$env:ISAACLAB_TRACKING_HUB_IP="<ubuntu_ip>"

$env:SONIC_DEPLOY_TRANSPORT="zmq"
$env:SONIC_DEPLOY_ENDPOINT="tcp://<ubuntu_ip>:5557"
$env:SONIC_DEPLOY_TOPIC="g1_debug"
$env:SONIC_DEPLOY_TARGET_FIELD="last_action"
$env:SONIC_DEPLOY_REFERENCE_TARGET_FIELD="body_q_target"

$env:SONIC_PUBLISH_STATE_ZMQ="1"
$env:SONIC_STATE_ZMQ_BIND="tcp://*:5560"
$env:SONIC_STATE_ZMQ_TOPIC="sonic_state"

$env:SONIC_G1_PHYSICS_MODE="1"
$env:SONIC_G1_VISUAL_SERVO_MODE="0"
$env:SONIC_G1_SELF_COLLISIONS="0"
$env:SONIC_DEPLOY_STABILIZE_ROOT="1"
$env:SONIC_DEPLOY_TARGET_RATE_LIMIT="0.04"

.\isaaclab.bat -p scripts\environments\teleoperation\teleop_se3_agent.py `
  --task Isaac-SonicSolo-Locomanipulation-G1-v0 `
  --device cpu `
  --kit_args "--/app/vsync=false --/app/runLoops/main/rateLimitEnabled=false"
```

### Ubuntu 端 1：C++ LowState proxy

```bash
export GROOT_REPO_ROOT=/home/nolo/GR00T-WholeBodyControl
export WINDOWS_IP=<windows_ip>
export DDS_INTERFACE=enp4s0
export SDK="$GROOT_REPO_ROOT/gear_sonic_deploy/thirdparty/unitree_sdk2"
export LD_LIBRARY_PATH="$SDK/thirdparty/lib/$(uname -m):$SDK/lib/$(uname -m):${LD_LIBRARY_PATH:-}"

cd "$GROOT_REPO_ROOT"
gear_sonic_deploy/build/tools/sonic_unitree_lowstate_cpp_proxy \
  --interface "$DDS_INTERFACE" \
  --domain-id 0 \
  --lowstate-hz 500 \
  --follow-alpha 0.35 \
  --isaac-state-endpoint "tcp://$WINDOWS_IP:5560" \
  --isaac-state-topic sonic_state
```

### Ubuntu 端 2：SONIC deploy

```bash
export GROOT_REPO_ROOT=/home/nolo/GR00T-WholeBodyControl
export DDS_INTERFACE=enp4s0

cd "$GROOT_REPO_ROOT/gear_sonic_deploy"
source scripts/setup_env.sh

just run g1_deploy_onnx_ref \
  "$DDS_INTERFACE" \
  policy/release/model_decoder.onnx \
  reference/example \
  --obs-config policy/release/observation_config.yaml \
  --encoder-file policy/release/model_encoder.onnx \
  --planner-file planner/target_vel/V2/planner_sonic.onnx \
  --input-type zmq_manager \
  --zmq-host localhost \
  --zmq-port 5556 \
  --zmq-topic pose \
  --output-type all \
  --disable-crc-check
```

deploy 的 realtime debug 默认发布到 `tcp://*:5557` / `g1_debug`；Windows 的 `SONIC_DEPLOY_ENDPOINT` 必须与它一致。如果改 deploy 输出端口，也要同步改 Windows 侧 endpoint。

### Ubuntu 端 3：Sony/BVH stream manager

```bash
export GROOT_REPO_ROOT=/home/nolo/GR00T-WholeBodyControl

cd "$GROOT_REPO_ROOT"
PYTHONUNBUFFERED=1 .venv_teleop/bin/python -u gear_sonic/scripts/mocap_manager_server.py \
  --source bvh_stream \
  --bvh-stream-host 0.0.0.0 \
  --bvh-stream-port 12352 \
  --control-mode pose \
  --pose-window-size 80 \
  --pose-encoder-mode g1 \
  --pose-protocol-version 1 \
  --zmq-port 5556 \
  --log-interval-s 1
```

### Ubuntu 端 4：本机 BVH sender

```bash
export GROOT_REPO_ROOT=/home/nolo/GR00T-WholeBodyControl
export BVH_FILE="${BVH_FILE:-$HOME/RAYNOS_Motion1.bvh}"

cd "$GROOT_REPO_ROOT"
PYTHONUNBUFFERED=1 .venv_teleop/bin/python -u gear_sonic/scripts/bvh_stream_sender.py \
  --bvh-file "$BVH_FILE" \
  --host 127.0.0.1 \
  --port 12352 \
  --loop \
  --log-interval-s 1
```

如果 BVH sender 在另一台机器上运行，Ubuntu 端 4 不启动；外部 sender 发送到 Ubuntu 机器的 `<ubuntu_ip>:12352`。

## 一键部署脚本里的 IsaacLab 启动方式

本地 tmux launcher 在 IsaacLab backend 下会按这个顺序启动 pane：

```text
1. IsaacLab
2. C++ LowState proxy
3. SONIC deploy
4+. mocap / bvh sender / pico input
```

入口：

```bash
~/tools/sony-isaaclab-sonic-launcher/launch_sony_isaaclab_closed_loop.py \
  --backend isaaclab \
  --input-source sony \
  --bvh-source bvh_stream
```

launcher 的 IsaacLab pane 等价于在 IsaacLab 根目录执行：

```bash
cd <isaaclab_root>
source <conda.sh>
conda activate env_isaaclab
export PYTHONUNBUFFERED=1
export UNITREE_DDS_INTERFACE=<dds_interface>
export UNITREE_DDS_DOMAIN_ID=0
export SONIC_G1_PHYSICS_MODE=<0|1>
export SONIC_G1_VISUAL_SERVO_MODE=<0|1>
export SONIC_G1_SELF_COLLISIONS=<0|1>
export SONIC_DEPLOY_STABILIZE_ROOT=<0|1>
export SONIC_DEPLOY_TARGET_RATE_LIMIT=<value>
export SONIC_PUBLISH_STATE_ZMQ=1

./isaaclab.sh -p scripts/environments/teleoperation/teleop_se3_agent.py \
  --task <isaac_task> \
  --kit_args "--/app/vsync=false --/app/runLoops/main/rateLimitEnabled=false"
```

一键启动默认值是：`SONIC_G1_PHYSICS_MODE=1`、`SONIC_G1_VISUAL_SERVO_MODE=0`、`SONIC_G1_SELF_COLLISIONS=0`、`SONIC_DEPLOY_STABILIZE_ROOT=1`、`SONIC_DEPLOY_TARGET_RATE_LIMIT=0.04`。

注意：这个 launcher 的 IsaacLab pane 是 Linux 本机启动方式，默认 `--isaac-state-endpoint tcp://127.0.0.1:5560`，适合 IsaacLab 和 proxy 在同一台 Linux 机器上。Windows IsaacLab 分机运行时，不能让 launcher 同时启动本机 IsaacLab pane；Ubuntu 侧只保留 proxy / deploy / input，proxy 的 `--isaac-state-endpoint` 要改成：

```bash
--isaac-state-endpoint tcp://<windows_ip>:5560
```

Windows PowerShell 侧则使用本节前面的 `isaaclab.bat` 命令，并额外设置网络变量：`ISAACLAB_MACHINE_A_IP`、`ISAACLAB_MACHINE_B_IP`、`ISAACLAB_LOCAL_MACHINE_IP`、`ISAACLAB_TRACKING_HUB_IP`、`SONIC_DEPLOY_ENDPOINT`、`SONIC_STATE_ZMQ_BIND`。

## Ubuntu：启动 mocap manager

Sony / BVH 输入仍走现有 POSE v1 / G1 路线：

```bash
cd /home/nolo/GR00T-WholeBodyControl

PYTHONUNBUFFERED=1 .venv_teleop/bin/python -u gear_sonic/scripts/mocap_manager_server.py \
  --source bvh_stream \
  --bvh-stream-port 12352 \
  --control-mode pose \
  --pose-window-size 80 \
  --pose-encoder-mode g1 \
  --pose-protocol-version 1 \
  --zmq-port 5556 \
  --log-interval-s 1.0
```

发送 BVH 或 Sony bridge 时，把目标发到 Ubuntu：

```bash
PYTHONUNBUFFERED=1 .venv_teleop/bin/python -u gear_sonic/scripts/bvh_stream_sender.py \
  --bvh-file /home/nolo/RAYNOS_Motion1.bvh \
  --host <ubuntu_ip> \
  --port 12352 \
  --loop
```

## 成功标准

- lowstate proxy 日志出现 `src=isaac`。
- deploy 不再长期打印 `Waiting for LowState` 或 `Lost LowState data connection from robot`。
- Windows IsaacLab 日志出现 `SonicDeployTarget` first packet。
- Windows IsaacLab 日志出现 `SonicRobotStatePublisher` packets 增长。
- deploy 启动控制后，Windows IsaacLab 里的 `sonic_robot` 对 Sony / BVH 动作有响应。

## 关键边界

- 不要让 deploy 或 proxy 绑定真实 G1 网卡做这个实验；先用 `sim` / `lo`。
- Windows 原生不需要安装 Unitree DDS；只需要 `pyzmq` 和 `msgpack`。
- Windows 手动启动 IsaacLab 时，默认参数应与一键启动保持一致；固定 root 只作为临时排错覆盖，不作为标准启动参数。
- 如果只跑 `sonic_unitree_dds_proxy.py`，那是 synthetic lowstate 兜底，不是 IsaacLab 真实状态闭环。
