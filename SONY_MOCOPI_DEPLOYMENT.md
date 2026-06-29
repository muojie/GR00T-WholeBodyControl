# Sony mocopi 最小部署文档

目标：同事拿到当前工程后，让另一台机器把 Sony mocopi 数据流发过来，本机接收并驱动 MuJoCo 或 G1。

当前仓库：

```bash
git clone https://github.com/muojie/GR00T-WholeBodyControl.git
cd GR00T-WholeBodyControl
git checkout feature/realtime-bvh-g1-retarget
git lfs pull
```

必须切到 `feature/realtime-bvh-g1-retarget`。不要直接用默认 `main` 跑这套 Sony mocopi 接入流程。

## 1. 一次性安装

在接收机上执行：

```bash
# 下载 deploy 用 ONNX 模型
python3 -m pip install -U pip huggingface_hub
python3 download_from_hf.py

# mocopi / teleop manager 环境
bash install_scripts/install_pico.sh

# 如果要先跑 MuJoCo 仿真
bash install_scripts/install_mujoco_sim.sh

# 编译 C++ deploy
cd gear_sonic_deploy
export TensorRT_ROOT=$HOME/TensorRT
bash scripts/install_deps.sh
source scripts/setup_env.sh
just build
```

环境分工不要混：

```text
run_sim_loop.py                  用 .venv_sim
mocap_manager_server.py          用 .venv_teleop
gear_sonic_deploy/deploy.sh      C++ 程序，不用 Python venv
```

## 2. 发送机设置

发送机是 Sony mocopi app、mocopi bridge，或其他上游程序。

把发送目标设成接收机：

```text
协议: UDP
IP: 接收机 IPv4 地址
端口: 12351
格式: mocopi UDP；如果是自研 bridge，就用 JSON
```

接收机查 IP：

```bash
ip -4 addr
```

mocopi UDP 是单向数据流，发送机和接收机要在同一个可信局域网里。

## 3. 先跑 MuJoCo 仿真

开 3 个终端。

### 终端 1：MuJoCo

```bash
cd ~/GR00T-WholeBodyControl
source .venv_sim/bin/activate
python -u gear_sonic/scripts/run_sim_loop.py
```

窗口出来后按 `9`，让机器人落地。

### 终端 2：deploy

```bash
cd ~/GR00T-WholeBodyControl/gear_sonic_deploy
./deploy.sh --input-type zmq_manager --zmq-host localhost --zmq-port 5556 sim
```

启动后在 deploy 终端按 `]`。

### 终端 3：接收 Sony 数据

官方 mocopi UDP：

```bash
cd ~/GR00T-WholeBodyControl
.venv_teleop/bin/python -u gear_sonic/scripts/mocap_manager_server.py \
  --source mocopi \
  --mocopi-format auto \
  --mocopi-host 0.0.0.0 \
  --mocopi-port 12351 \
  --mocopi-g1-retarget on \
  --mocopi-skeleton-mode fk \
  --control-mode pose \
  --pose-window-size 80 \
  --pose-encoder-mode g1 \
  --pose-protocol-version 1 \
  --target-fps 50 \
  --zmq-port 5556
```

如果发送机发的是 JSON bridge，把 `--mocopi-format auto` 改成：

```bash
--mocopi-format json
```

成功标志：

```text
manager 日志里 recv 增长
manager 日志里出现 pose=sent
MuJoCo 里的机器人有响应
```

## 4. 真机运行

仿真确认能跑后再上真机。真机只需要 2 个终端，不跑 MuJoCo。

### 终端 1：deploy

manager 和 deploy 在同一台机器：

```bash
cd ~/GR00T-WholeBodyControl/gear_sonic_deploy
./deploy.sh --input-type zmq_manager --zmq-host localhost --zmq-port 5556 real
```

如果 manager 在另一台接收 PC，deploy 在 G1 onboard：

```bash
./deploy.sh --input-type zmq_manager --zmq-host <接收PC的IP> --zmq-port 5556 real
```

### 终端 2：接收 Sony 数据

```bash
cd ~/GR00T-WholeBodyControl
.venv_teleop/bin/python -u gear_sonic/scripts/mocap_manager_server.py \
  --source mocopi \
  --mocopi-format auto \
  --mocopi-host 0.0.0.0 \
  --mocopi-port 12351 \
  --mocopi-g1-retarget on \
  --mocopi-skeleton-mode fk \
  --control-mode pose \
  --pose-window-size 80 \
  --pose-encoder-mode g1 \
  --pose-protocol-version 1 \
  --target-fps 50 \
  --zmq-port 5556
```

真机注意：

```text
先小幅动作测试
deploy 终端按 O 可停止
不要同时开 MuJoCo 和真机 deploy
```

## 5. 最常见问题

`recv=0`：

发送机没打到接收机。查 IP、端口 `12351`、防火墙、是否同网段。接收机可用：

```bash
sudo tcpdump -ni any udp port 12351
```

有 `recv` 但没有 `vr_3pt=yes`：

格式不对。官方 mocopi 用 `auto`；自研 JSON bridge 用 `json`。

MuJoCo 不动：

deploy 必须用：

```bash
--input-type zmq_manager
```

只有日志没有窗口：

正常。manager 默认不弹窗口。需要看三点目标时，在 manager 命令后加：

```bash
--visualize-vr3pt --visualize-g1
```

真机 deploy 一直等 `LowState`：

机器人网络或 DDS 没通。先停掉 MuJoCo，再确认 `real` 模式找到了 `192.168.123.x` 网卡，必要时直接传网卡名或 IP。
