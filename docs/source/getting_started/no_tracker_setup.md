# 无追踪器模式（仅手柄）搭建指南

本文档记录在**只有 PICO 头显 + 手柄、没有脚部追踪器**的情况下，搭建 GEAR-SONIC 全身控制仿真环境的完整流程。

## 前置条件（一次性安装）

### 1. 安装 teleop 虚拟环境
```bash
bash install_scripts/install_pico.sh
```
验证：`ls .venv_teleop/` 存在。包含 Python 3.10、gear_sonic[teleop+sim]、xrobotoolkit_sdk 1.0.2、mujoco 3.8.1、unitree_sdk2py。

### 2. 安装 XRoboToolkit PC Service
通过 deb 包安装，安装后验证：
```bash
ls /opt/apps/roboticsservice/runService.sh
```

### 3. 下载 ONNX 模型
```bash
# 在 base conda 环境下执行
python download_from_hf.py
```
从 `nvidia/GEAR-SONIC` 下载以下三个文件：
- `gear_sonic_deploy/policy/release/model_encoder.onnx`（48 MB）
- `gear_sonic_deploy/policy/release/model_decoder.onnx`（40 MB）
- `gear_sonic_deploy/planner/target_vel/V2/planner_sonic.onnx`（739 MB）

---

## 每次启动流程

### 启动 XRoboToolkit 服务

> ⚠️ 必须用 `bash` 显式执行，否则 `${BASH_SOURCE[0]}` 语法报错。

```bash
sudo bash /opt/apps/roboticsservice/runService.sh
```
服务在后台运行（`&`），看到三行路径输出即成功。

### 连接 PICO 头显
1. PICO 和本机连接**同一 Wi-Fi**
2. 打开 PICO 上的 **XRoboToolKit app**
3. 填入本机 IP：`192.168.50.68`，确认连接
4. Terminal 3 启动后会显示 `device found PA9410MGKA220055G`

### 开三个终端

**Terminal 1 — MuJoCo 模拟器**
```bash
source .venv_teleop/bin/activate
python gear_sonic/scripts/run_sim_loop.py
```

**Terminal 2 — C++ 推理部署**
```bash
cd gear_sonic_deploy
source scripts/setup_env.sh
./deploy.sh --input-type zmq_manager sim
```
等待出现 `Init done` 后再操作。

**Terminal 3 — PICO 数据流（无追踪器模式）**
```bash
source .venv_teleop/bin/activate
python gear_sonic/scripts/pico_manager_thread_server.py --manager --no_body
```
启动成功后显示：
```
device found PA9410MGKA220055G
[PoseLoop] Robot model loaded for FK calibration
Manager controls: A+X=toggle mode, A+B+X+Y=start/stop policy
```

---

## 校准与操作

| 操作 | 按键 |
|------|------|
| 启动 policy + 校准 | A + B + X + Y（同时按，保持校准姿势） |
| 切换到全身遥操（Pose 模式） | A + X |
| 切换回 Planner 模式 | A + X |
| 紧急停止（手柄） | A + B + X + Y |
| 紧急停止（键盘） | `O`（在 Terminal 2 聚焦时按） |

### 校准姿势
1. 站直，直视前方
2. 双脚并拢，脚平行无间隙
3. 上臂自然垂下，手臂垂直挂在身侧

---

## `--no_body` 模式原理

标准模式需要 PICO 脚部追踪器提供全身 SMPL 关节数据。无追踪器模式直接使用：
- `get_left_controller_pose()` → 左腕
- `get_right_controller_pose()` → 右腕
- `get_headset_pose()` → 颈部（方向用于校准，位置由运动学链重新计算）

C++ deploy 只消费 `vr_position`（9 floats）和 `vr_orientation`（12 floats），`smpl_pose` 等字段在此模式下填零，不影响推理。

代码修改位于 `gear_sonic/scripts/pico_manager_thread_server.py`，关键新增：
- `_process_3pt_pose_from_controllers()` — 从控制器构建 3-point pose
- `ThreePointPose.process_controller_pose()` — 校准 + 输出
- `PicoReader(no_body=True)` — 跳过 body tracking 等待
- `--no_body` argparse 参数

---

## 键盘控制模式（备用）

如果不需要 PICO 遥操，可以单独用键盘控制行走：

```bash
# 只需 Terminal 1 + Terminal 2
./deploy.sh --input-type keyboard sim
```

Terminal 2 聚焦后按键：
- `L` — 站立
- `W/A/S/D` — 前/左/后/右移动
- `O` — 紧急停止

---

## 已知问题

| 问题 | 原因 | 影响 |
|------|------|------|
| 退出时 `terminate called without an active exception` + core dump | XRoboToolkit SDK C++ 析构阶段 bug | 仅退出时发生，不影响运行功能 |
| 手臂方向映射可能偏差 | OFFSETS 按 SMPL 关节帧设计，控制器帧可能有差异 | 首次使用观察手臂响应，必要时调整 `pico_manager_thread_server.py` 里的 `OFFSETS[1]`/`OFFSETS[2]` |
