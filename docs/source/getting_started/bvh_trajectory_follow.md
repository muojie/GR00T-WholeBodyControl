# BVH 轨迹跟随（让机器人走出 BVH 的根部路径）

本文档介绍如何用离线 BVH 动作文件驱动 G1：在 planner 模式下逐帧跟随 BVH 的**根部轨迹**（转弯、曲线、加减速），让机器人走出动作捕捉里的行走路径，而不是原地复现或恒定直线前进。

## 背景：两种控制模式

`gear_sonic/scripts/bvh_fullbody_pose_sender.py` 把 BVH 重定向到 G1 后，可用两种 `--control-mode` 发送：

| 模式 | 命令什么 | 适用 |
|------|----------|------|
| `streamed` | IK 后的 29DOF 关节角（pose 协议：`joint_pos/joint_vel/body_quat_w`）。**协议里没有 root 平移字段**，机器人原地复现关节动作。 | 原地动作、上半身动作 |
| `planner` | BVH 提供上半身 VR3PT 目标，root 移动交给 SONIC locomotion planner。 | 含行走/位移的片段 |

G1 采用**解耦全身控制**：上半身开环插值，下半身（腿+腰）由 RL locomotion 策略吃速度级命令、自己迈步保平衡。因此：

- 走路片段用 `streamed` 会因为没有 root 命令而"像被拖着走"，运行时会打印 WARNING。
- 腿**无法**开环逐关节照 BVH 播放（会摔倒）；行走必须经由 planner 的速度命令 + RL 平衡。

## `--follow-trajectory`：逐帧跟随根部路径

planner 模式默认把整段片段估算成一个**标量平均速度**，恒定朝正前方直线行走——会丢掉 BVH 里所有的转弯和变速。

加上 `--follow-trajectory` 后，发送端逐帧从 BVH 根部解算出世界系的：

- `movement` —— 速度方向（单位向量）
- `facing` —— 朝向（跟随行进方向）
- `speed` —— 速度大小（m/s）

机器人就会跟随 BVH 的转弯、曲线与加减速；腿部迈步仍由 RL 策略保平衡。

实现要点：

- `movement`/`facing` 是**世界系绝对方向**，deploy 侧再换算成机体速度命令。
- 坐标系（经 `_axis_basis` 转换后）：robot **X=前、Y=左、Z=上**，地面为 XY 平面。
- **朝向对齐**：整段路径会旋转到"首个运动帧朝机器人 +X"，否则机器人启动时会冲向 BVH 任意的初始世界朝向。
- 速度做 5 帧滑动平均，避免 `facing` 抖动影响平衡；当某帧速度 < 0.03 m/s 时自动切到 IDLE。

### 已知局限

- `facing` 跟随行进方向，**不表示侧步 / 倒走**（侧移会被当成转身朝该方向走）。
- 发送端是**开环**的（不读机器人实际状态），对短片段没问题；长片段 RL 跟踪误差会让朝向逐渐漂移。

## 用法

```bash
source .venv_teleop/bin/activate

# 跟随 BVH 根部轨迹
python -m gear_sonic.scripts.bvh_fullbody_pose_sender \
    --control-mode planner --follow-trajectory <your.bvh>

# 跑前预览速度曲线、不发布
python -m gear_sonic.scripts.bvh_fullbody_pose_sender \
    --control-mode planner --follow-trajectory --dry-run <your.bvh>
```

常用选项：

| 选项 | 说明 |
|------|------|
| `--speed-scale 1.5` | 整体放大/缩小估算速度（嫌太慢就调大） |
| `--planner-speed <m/s>` | 用固定速度覆盖逐帧估算 |
| `--loop` | 循环播放该片段 |
| `--axis-map {mcp,soma,unity}` | BVH 轴向约定（默认 `mcp`） |
| `--start-frame / --end-frame / --stride` | 截取 / 抽帧 |

## 三终端 Sim 运行流程

用 BVH 发送端替代 [无追踪器模式](no_tracker_setup.md) 里的 PICO 数据流终端：

```bash
# Terminal 1 — MuJoCo 仿真
source .venv_teleop/bin/activate
python gear_sonic/scripts/run_sim_loop.py

# Terminal 2 — C++ deploy（等待日志出现 "Init Done"）
cd gear_sonic_deploy
source scripts/setup_env.sh
./deploy.sh --input-type zmq_manager sim
```

```{admonition} deploy.sh 的交互确认
:class: warning
`deploy.sh` 启动前会询问 `Proceed with deployment? [Y/n]`。前台运行时直接回车即可；若以脚本/后台方式启动，需要把 `Y` 喂进 stdin（见下方命令）。
```

```bash
# 后台/脚本启动 deploy 时，喂 Y 跳过交互确认
printf "Y\n" | ./deploy.sh --input-type zmq_manager sim
```

```bash
# Terminal 3 — BVH 轨迹跟随发送端（ZMQ PUB，默认端口 5556）
source .venv_teleop/bin/activate
python -m gear_sonic.scripts.bvh_fullbody_pose_sender \
    --control-mode planner --follow-trajectory <your.bvh>
```

deploy 收到命令后会持续打印 `Replanning with mode: WALK, ..., movement: [...]`，其中 `movement` 方向随轨迹变化即为跟随生效。
