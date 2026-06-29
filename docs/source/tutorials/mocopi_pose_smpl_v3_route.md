# Sony mocopi / SMPL POSE v3 路线

本文只记录更接近 SONIC 原生人体 pose encoder 的路线。它和 [BVH-G1 POSE v1 路线](mocopi_pose_bvh_g1_v1_route.md) 是两条不同方案，不应互相覆盖。

## 2026-06-29 调优计划跟踪

本轮 SMPL POSE v3 调优在独立 worktree 中推进，避免污染当前 BVH-G1 POSE v1 稳定线。

| 项目 | 当前值 |
|------|--------|
| worktree | `/home/nolo/GR00T-WholeBodyControl-v3-tuning` |
| branch | `optimization/v3-smpl-g1-tuning-20260629` |
| base commit | `72c899e` |
| 调优对象 | `protocol_version=3` + `encoder_mode=smpl` + Sony/BVH `bvh_stream` / `bvh_g1` |
| 不动边界 | BVH-G1 POSE v1 主线、deploy wire format、当前 v1 IsaacLab 稳定性修复 |

### 本轮工作分层

1. 先验证数据契约：确认 `smpl_joints`、`smpl_pose`、`body_pos`、`body_quat_w`、`joint_pos`、`joint_vel` 的 shape、dtype、坐标系和 deploy parser 一致。
2. 再验证 release observation：把 `smpl_joints_10frame_step1`、`smpl_anchor_orientation_10frame_step1`、`motion_joint_positions_wrists_10frame_step1` 单独量化，先解释离线差异。
3. 再做闭环 A/B：同一 BVH 分别跑 v1/G1、v3 `stable`、v3 `responsive`、v3 `off`，优先用 MuJoCo 快速验证，再记录 IsaacLab 稳定性、动作保真度、root/yaw/height、wrist 差异和延迟。
4. 最后再调参数或代码：只有当前三层证据能说明瓶颈时，才改 filter profile、wrist 生成、anchor 处理或 sender/parser 逻辑。

### 验收门槛

- `.venv_teleop` 下的 compile/import/smoke 检查通过。
- `evaluate_sony_pose_v3.py` 对固定 BVH 的 release observation 指标可复现。
- deploy 日志明确显示 `protocol_version: 3`、`encoder_mode=smpl`，且没有在同一 session 内发生 protocol 切换。
- MuJoCo / IsaacLab 指标至少包含启动成功、持续时长、fall/reset、root height/yaw、body keypoint tracking、wrist observation 差异。
- 每次阶段性结论同步更新本工程页和本地 robotics KB 的 `Sony-mocopi-SMPL-POSE-v3路线` 页面。

### 2026-06-29 首轮基线

运行环境：

```bash
.venv_teleop/bin/python -m compileall \
  gear_sonic/scripts/mocap_manager_server.py \
  gear_sonic/scripts/evaluate_sony_pose_v3.py \
  gear_sonic/utils/teleop
```

结果：compile/import 通过；`.venv_teleop` 中 `scipy=1.15.3`、`pyzmq=27.1.0`、`numpy=1.26.4`。

离线基线使用 `/home/nolo/RAYNOS_Motion1.bvh` 全量 926 帧，输出到 `/tmp/sony_pose_v3_baseline_20260629/`：

| profile | anchor speed p95/max | anchor horizon p95/max | filter lag p95 | wrist projection error p95/max |
|---------|----------------------|-------------------------|----------------|--------------------------------|
| `stable` | `50.1/50.2 deg/s` | `9.0/9.0 deg` | `smpl=0.122 m, anchor=71.97 deg, wrist=0.714 rad` | `1.996/2.686 rad` after filtering |
| `responsive` | `300.8/300.8 deg/s` | `54.1/54.1 deg` | `smpl=0.009 m, anchor=72.14 deg, wrist=0.040 rad` | `2.252/2.692 rad` after filtering |
| `off` | `401.5/581.7 deg/s` | `72.7/100.0 deg` | `smpl=0.000 m, anchor=0.00 deg, wrist=0.000 rad` | `2.254/2.692 rad` |

共同结论：

- `g1_fk` 投影仍和 v1 G1 FK body keypoints 几何对齐：direct RMSE、yaw+scale RMSE 和 point error 都为 `0.000 m`；旧 `skeleton` 路线仍有 `0.195 m` mean direct RMSE 和 `98.5%` side mismatch。
- `stable` 把 anchor 速度压得最稳，但代价是约 `72 deg` 的 p95 anchor lag。
- `responsive` 保留较小 SMPL/wrist lag，但快速 yaw 仍很激进。
- `off` 只适合作 raw reference 对照，不适合作第一条闭环稳定候选。

### 2026-06-29 launcher 入口修正

repo-local `scripts/launch_sonic_local_isaaclab_closed_loop.py` 已补显式 v3 入口：

```bash
.venv_teleop/bin/python scripts/launch_sonic_local_isaaclab_closed_loop.py \
  --sony-pose-line v3 \
  --pose-filter-profile stable \
  --pose-root-yaw-only \
  --bvh-file /home/nolo/RAYNOS_Motion1.bvh
```

dry-run 已确认：

- 默认仍是 v1：`--pose-encoder-mode g1 --pose-protocol-version 1`。
- `--sony-pose-line v3` 会切为 `--pose-encoder-mode smpl --pose-protocol-version 3 --allow-sony-pose-v3 --bvh-g1-smpl-joints-source g1_fk`。
- launcher 默认 `repo_root` 改为脚本所在 checkout，避免从 v3 worktree 误启动旧 worktree。
- deploy debug 输出端口已补透传：`--debug-port` 现在会传给 C++ deploy 的 `--zmq-out-port`，`--debug-topic` 会传给 `--zmq-out-topic`，避免多 session 时仍硬绑定 `5557`。

### 2026-06-29 v3 专用启动脚本

为避免和 v1/MCPM 或默认 `sonic_local_isaaclab` 栈混用，v3 调优后续统一从新 worktree 的专用脚本启动：

```bash
cd /home/nolo/GR00T-WholeBodyControl-v3-tuning
scripts/launch_sonic_v3_tuning_closed_loop.py --no-attach --headless
```

该脚本固定 v3 语义并使用独立端口：

默认 profile 已改为 `stable`。`responsive` 只作为显式 A/B 对照使用：

```bash
scripts/launch_sonic_v3_mujoco_closed_loop.py --pose-filter-profile responsive --no-attach
```

| 通道 | 端口 |
|------|------|
| mocap manager -> deploy | `6056` |
| deploy `g1_debug` | `6057` |
| IsaacLab `sonic_state` | `6060` |
| BVH UDP stream | `12403` |

dry-run 已确认 input、bvh sender、metrics 的工作目录均为 `/home/nolo/GR00T-WholeBodyControl-v3-tuning`，并且 input 命令带有 `--pose-encoder-mode smpl --pose-protocol-version 3 --allow-sony-pose-v3`。Python 窗口显式设置 `PYTHONPATH=/home/nolo/GR00T-WholeBodyControl-v3-tuning`，避免复用软链虚拟环境时导入旧 worktree 包。

DDS 隔离：`g1_deploy_onnx_ref` 已支持 `--domain-id`，默认 v3 调优 domain 为 `4`；v3 worktree 的 `gear_sonic_deploy/build` 和 `gear_sonic_deploy/target` 已切成本地目录，并已在该 worktree 单独 `just build`。

### 2026-06-29 MuJoCo 专用验证入口

为快速验证 v3 闭环，新增 MuJoCo 专用脚本：

```bash
cd /home/nolo/GR00T-WholeBodyControl-v3-tuning
scripts/launch_sonic_v3_mujoco_closed_loop.py --no-attach
```

该脚本启动 `mujoco/input/deploy/bvh_sender/metrics` 五个 tmux 窗口，不启动 IsaacLab/proxy。默认隔离参数：

| 通道 | 值 |
|------|----|
| Unitree DDS domain | `4` |
| mocap manager -> deploy | `6156` |
| deploy `g1_debug` | `6157` |
| BVH UDP stream | `12413` |
| MuJoCo viewer | 默认 `--no-enable-onscreen` |

deploy 侧同样传入 `--domain-id 4`，MuJoCo 侧通过 `run_sim_loop.py --domain-id 4` 覆盖 WBC YAML 中的 `DOMAIN_ID`。

### 2026-06-29 MuJoCo 20s 验证结果

命令：

```bash
scripts/launch_sonic_v3_mujoco_closed_loop.py \
  --replace \
  --no-attach \
  --metrics-duration-s 20 \
  --metrics-startup-timeout-s 120
```

证据文件：

| 类型 | 路径 |
|------|------|
| metrics summary | `/tmp/sony_pose_v3_mujoco_20260629_115617_summary.json` |
| metrics samples | `/tmp/sony_pose_v3_mujoco_20260629_115617_samples.jsonl` |
| deploy log | `/tmp/sonic_v3_mujoco_deploy_20260629_115617.log` |
| input log | `/tmp/sonic_v3_mujoco_input_20260629_115617.log` |
| bvh sender log | `/tmp/sonic_v3_mujoco_bvh_sender_20260629_115617.log` |

关键结果：

| 指标 | 结果 |
|------|------|
| samples / elapsed | `386 / 19.96 s` |
| deploy FPS | `50.00` |
| protocol / encoder | deploy log 出现 `active_protocol_version_=3`、`GetEncodeMode()=2` |
| LowState freshness | loop timing 多数约 `0.8-8.3 ms` |
| nonfinite / missing fields | `0 / 0` |
| fall frames | `0` |
| root tilt max | `0.405 rad` |
| joint RMSE mean / p95 / max | `0.522 / 0.984 / 1.223 rad` |
| target step max | `1.178 rad` |
| joint velocity max | `44.19 rad/s` |
| pass | `false`，仅 `joint_velocity_peak` 未过阈值 `35 rad/s` |

结论：MuJoCo 闭环已经跑通且端口/DDS domain 与当前 v1 会话隔离；v3 `responsive + root_yaw_only + g1_fk` 没有 fall、没有 NaN、没有字段缺失，但会出现关节速度峰值过高。下一步调优优先压低 v3 输出速度峰值，而不是继续排查启动链路。

### 2026-06-29 MuJoCo 指标增强与 stable A/B

`collect_sonic_mujoco_metrics.py` 已从全局 max 扩展为可定位指标：

- 默认采样频率从 `20 Hz` 提到 `60 Hz`，降低 target step 因跳采样被放大的概率。
- 新增 `warmup_s=2.0`，pass/fail 统计排除启动初始窗口跳变，但 samples 仍保留全量数据。
- summary 记录 `joint_names_order`、`top_joints`、`deploy_index_delta`、`target_velocity_absmax_radps`。
- samples JSONL 默认保留 per-joint 的 tracking / velocity / target-step / target-velocity 数组，便于事后定位。

对比命令：

```bash
scripts/launch_sonic_v3_mujoco_closed_loop.py \
  --replace \
  --no-attach \
  --pose-filter-profile stable \
  --metrics-duration-s 12 \
  --metrics-summary-json /tmp/sony_pose_v3_mujoco_stable_enhanced_summary.json \
  --metrics-samples-jsonl /tmp/sony_pose_v3_mujoco_stable_enhanced_samples.jsonl

scripts/launch_sonic_v3_mujoco_closed_loop.py \
  --replace \
  --no-attach \
  --pose-filter-profile responsive \
  --metrics-duration-s 12 \
  --metrics-summary-json /tmp/sony_pose_v3_mujoco_responsive_warmup_summary.json \
  --metrics-samples-jsonl /tmp/sony_pose_v3_mujoco_responsive_warmup_samples.jsonl
```

| profile | pass | samples / eval | joint velocity max / p95 | target step max | target velocity max | joint RMSE mean | root tilt max | 速度峰值 top joint |
|---------|------|----------------|---------------------------|-----------------|---------------------|-----------------|---------------|--------------------|
| `stable` | `true` | `571 / 475` | `28.77 / 15.52 rad/s` | `0.057 rad` | `2.71 rad/s` | `0.490 rad` | `0.174 rad` | `left_ankle_pitch_joint` |
| `responsive` | `false` | `563 / 468` | `48.39 / 22.70 rad/s` | `0.880 rad` | `42.22 rad/s` | `0.660 rad` | `0.377 rad` | `right_shoulder_pitch_joint` |

结论：`responsive` 在 warmup 后 target 侧不再触发阈值，但机器人实测速度仍由右肩 pitch 打穿 `35 rad/s`；`stable` 在同样 MuJoCo 窗口内所有 checks 通过，并且 joint RMSE / root tilt 也更低。因此 v3 专用启动脚本默认切到 `stable`，`responsive` 保留为显式对照。

### 2026-06-29 MCPM BVH 120s MuJoCo 验证

按用户指定动作文件验证优化后的 v3 默认线：

```bash
scripts/launch_sonic_v3_mujoco_closed_loop.py \
  --replace \
  --no-attach \
  --onscreen \
  --bvh-file /home/nolo/MCPM_20260526_190029.BVH \
  --metrics-duration-s 120 \
  --metrics-startup-timeout-s 120 \
  --metrics-summary-json /tmp/sony_pose_v3_mujoco_mcpm_20260526_190029_summary.json \
  --metrics-samples-jsonl /tmp/sony_pose_v3_mujoco_mcpm_20260526_190029_samples.jsonl
```

| 指标 | 结果 |
|------|------|
| profile | `stable + root_yaw_only + g1_fk` |
| BVH | `/home/nolo/MCPM_20260526_190029.BVH`，`2561` frames，sender `50 FPS` loop |
| samples / elapsed | `5473 / 119.96 s` |
| eval samples | `5380`，`warmup_s=2.0` |
| deploy FPS | `50.01` |
| nonfinite / missing fields | `0 / 0` |
| fall frames | `0` |
| root tilt max | `0.279 rad` |
| joint RMSE mean / p95 / max | `0.441 / 1.030 / 1.194 rad` |
| target step max | `0.408 rad` |
| target velocity max | `9.78 rad/s` |
| joint velocity max / p95 | `35.26 / 16.41 rad/s` |
| pass | `false`，仅 `joint_velocity_peak` 略高于 `35 rad/s` |

top velocity joints：

| joint | max |
|-------|-----|
| `right_knee_joint` | `35.26 rad/s` |
| `left_knee_joint` | `30.50 rad/s` |
| `left_ankle_pitch_joint` | `29.34 rad/s` |

结论：指定 MCPM BVH 在 120s MuJoCo 中无 fall、无 NaN、无字段缺失，整体观感较上一轮明显更稳；剩余问题从上肢大峰值收敛为右膝单点窄峰值，超阈值约 `0.26 rad/s`。下一步可围绕 lower-body 速度峰值做小幅压制，而不是回退到 `responsive`。

### 2026-06-29 闭环启动记录

以下是专用脚本落地前的手动启动记录，仅保留为排障证据；后续 v3 调优以 `scripts/launch_sonic_v3_tuning_closed_loop.py` 和 `6056/6057/6060/12403` 端口组为准。

本轮曾尝试用独立端口和 domain 启动 v3：

```bash
.venv_teleop/bin/python scripts/launch_sonic_local_isaaclab_closed_loop.py \
  --session sonic_v3_tuning_20260629 \
  --headless \
  --domain-id 2 \
  --zmq-port 5756 \
  --debug-port 5757 \
  --state-port 5760 \
  --bvh-stream-port 12372 \
  --sony-pose-line v3 \
  --pose-filter-profile responsive \
  --pose-root-yaw-only \
  --bvh-file /home/nolo/RAYNOS_Motion1.bvh \
  --decoder /home/nolo/GR00T-WholeBodyControl/gear_sonic_deploy/policy/release/model_decoder.onnx \
  --encoder /home/nolo/GR00T-WholeBodyControl/gear_sonic_deploy/policy/release/model_encoder.onnx \
  --planner-file /home/nolo/GR00T-WholeBodyControl/gear_sonic_deploy/planner/target_vel/V2/planner_sonic.onnx
```

启动依赖处理：

- 新 worktree 本地 symlink 复用原 worktree 的 `.venv_teleop`、`.venv_sim`；`gear_sonic_deploy/build`、`gear_sonic_deploy/target` 已切成本地目录；这些运行辅助只写入本地 `.git/info/exclude`，不进入分支。
- 新 worktree 未带 release ONNX 大文件，闭环命令显式传原 worktree 的 decoder/encoder/planner ONNX 路径。
- IsaacLab 当前 `locomanipulation_g1_env_cfg.py` 已被瘦身，导致 `SonicSolo` 任务导入旧接口失败；已在 IsaacLab 侧做最小兼容：`SonicSolo` 自包含 SONIC robot/action/state publisher，Sonic 系列任务注册改为字符串 entrypoint，主配置只补回 `_env_flag`。

已验证到的状态：

- `mocap_manager_server.py` 进入 v3：`pose_protocol=v3 pose_encoder=smpl pose_filter=responsive root_yaw_only=1`。
- deploy 识别 SMPL encoder observation：`smpl_joints_10frame_step1`、`smpl_anchor_orientation_10frame_step1`、`motion_joint_positions_wrists_10frame_step1` 维度匹配，总 encoder dim `1762`。
- IsaacLab `SonicSolo` 能完成环境 setup，并以约 `50 Hz` 发布 `sonic_state`。

已知验证缺口：

- v3 MuJoCo 已得到 final metrics JSON；IsaacLab v3 final metrics 仍未重跑。
- `gear_sonic_deploy/target/release/run_tests` 当前会查找硬编码目录 `reference/bones_072925_test/`，v3 worktree 无该测试数据，测试进程退出 `139`；本轮未为测试补数据目录。

## 定位

这条路线希望让非 PICO 输入源输出 SMPL-like full-body reference，再由 deploy 侧按 SMPL encoder 语义处理。

```text
BVH / mocopi skeleton
  -> normalized SMPL-like joints / local pose
  -> pose topic, protocol v3, encoder_mode=smpl
  -> deploy SMPL / motion reference observation
  -> policy
```

和 BVH-G1 POSE v1 的根本区别：

| 项目 | SMPL POSE v3 | BVH-G1 POSE v1 |
|------|--------------|----------------|
| 语义 | 人体 SMPL-like pose | G1 机器人关节参考 |
| retarget 位置 | 主要希望 deploy/encoder 侧理解人体 pose | manager 侧提前算好 G1 `joint_pos/joint_vel` |
| 优点 | 更接近 PICO full-body / SONIC 原生设计 | 工程可控、当前效果更稳定 |
| 难点 | 骨架比例、rest pose、坐标轴、observation 契约更敏感 | 不再让 deploy 自己理解人体语义 |

## 历史实现

早期 `--source bvh` `--control-mode pose` 会生成 SMPL-like full-body reference，并按 protocol v3 发送：

```text
smpl_joints
smpl_pose
body_pos
body_quat_w
joint_pos
joint_vel
frame_index
```

当时补过几个关键问题：

- protocol v2 缺少 `joint_pos/joint_vel`，release SMPL mode 里 wrist joint observation 不完整；
- 29 维 `joint_pos` 不能用全 0，至少要用 G1 默认站姿作为稳定基线；
- streamed motion 需要 `body_pos`，否则 root 高度可能退化到 `z=0`；
- POSE 模式应等第一条 window 发出后再发送 `start=True`。

## 当前实现

当前保留 BVH-G1 POSE v1 主线，同时新增 Sony/BVH SMPL POSE v3 实验线：

- `mocopi_source.py`：官方 mocopi binary 27 bone、只带 `joints/bones` 的 mocopi JSON、以及已带 `smpl_*` 的 JSON 都可以进入 `FullBodyReference`。
- `bvh_source.py`：新增统一的命名 skeleton frame -> SMPL-like `FullBodyReference` 构造入口。
- `bvh_g1_source.py`：继续输出原来的 G1 `joint_pos/joint_vel/body_pos/body_quat_w`，同时为 v3 构造非零 `smpl_joints/smpl_pose`。当前默认 `--bvh-g1-smpl-joints-source g1_fk`，即把 v1 已验证的 G1 FK 14 个关键点投影到 24 个 SMPL 槽位；`skeleton` 只作为旧 V3 原始 BVH skeleton 的 A/B 回退。
- `bvh_stream_source.py`：每个 UDP `bvh_stream_v1` frame 既做 BVH-to-G1 retarget，也按同一 `--bvh-g1-smpl-joints-source` 规则构造 SMPL-like `smpl_joints/smpl_pose`。
- `mocap_manager_server.py`：`pkl` 仍只允许 v1/g1；`bvh_g1` 和 `bvh_stream` 支持两条显式线：
  - v1 主线：`--pose-protocol-version 1 --pose-encoder-mode g1`
  - v3 实验线：`--pose-protocol-version 3 --pose-encoder-mode smpl --allow-sony-pose-v3`

这样 v1/G1 的稳定路径不被覆盖；v3 需要显式选择，避免误把主验证命令切到 SMPL encoder。

## 评估方法

不要直接进闭环盲调 v3。先离线用同一个 BVH 同时生成：

- v1 参考：BVH-G1 retarget 后的 G1 `joint_pos`，再用 G1 MJCF FK 算 14 个 body keypoints；
- v3 skeleton 旧线：BVH skeleton 直接填到 `smpl_joints`；
- v3 g1_fk 新线：把 v1 G1 FK keypoints 投影到 SMPL 24 joint 槽位。

评估脚本：

```bash
.venv_teleop/bin/python -u gear_sonic/scripts/evaluate_sony_pose_v3.py \
  --bvh-file /home/nolo/RAYNOS_Motion1.bvh \
  --json-out /tmp/sony_pose_v3_eval.json
```

脚本会输出关键点 RMSE、最佳 yaw/scale 对齐误差、肢段方向角、左右侧符号错位率、root 高度/姿态统计。`RAYNOS_Motion1.bvh` 全量 926 帧的当前结果：

| 方案 | direct RMSE | yaw+scale RMSE | 肢段角均值 | 左右侧错位 |
|------|-------------|----------------|------------|------------|
| `skeleton` | 0.195 m | 0.092 m | 45.2 deg | 98.5% |
| `g1_fk` | 0.000 m | 0.000 m | 0.006 deg | 0.0% |

这说明旧 v3 的主要问题不是几个 gain，而是原始 BVH skeleton 相对 v1 G1 参考有约 90 deg yaw/轴差和左右侧几何错位。默认改成 `g1_fk` 后，v3 的 `smpl_joints` 至少在 deploy 当前 release 实际使用的关键点几何上和 v1 对齐；如果要验证旧行为，显式加：

```bash
--bvh-g1-smpl-joints-source skeleton
```

### release V3 observation check

当前 deploy release 的 SMPL mode 实际消费：

```text
smpl_joints_10frame_step1
smpl_anchor_orientation_10frame_step1
motion_joint_positions_wrists_10frame_step1
```

`evaluate_sony_pose_v3.py` 现在会额外输出 `release_observations`，对这三项做离线近似评估：

- `smpl_joints_10frame_step1`：10 帧窗口内 keypoint 速度和 horizon delta；
- `smpl_anchor_orientation_10frame_step1`：按 deploy 的 first-frame heading removal 近似生成 6D anchor，并统计 root/anchor 角速度、10 帧 horizon 和 filter lag；
- `motion_joint_positions_wrists_10frame_step1`：统计 Sony 实际发出的 wrist `joint_pos[23..28]`，并和 SMPL pose 投影出的 PICO-like wrist joint 做差；
- `raw` / `filtered`：同一段 reference 在进入 `PoseStreamPublisher` filter 前后的对比。

离线 anchor 评估假设 reset 时机器人 base 为 identity；真实闭环还会叠加 deploy 当前 `base_quat`，因此它用于排查 reference 方向和时序风险，不等价于闭环稳定性结论。

`RAYNOS_Motion1.bvh` 926 帧结果显示：

- root tilt p95/max 为 `0/0 deg`，这段动作的 root/anchor 风险主要不是 roll/pitch，而是 yaw 速度和 filter lag。
- raw anchor 速度 p95/max 为 `401.5/581.7 deg/s`，10 帧 horizon p95/max 为 `72.7/100.0 deg`。
- `stable` profile 会把 anchor 速度压到 `50.1/50.2 deg/s`，10 帧 horizon 压到 `9.0/9.0 deg`，但 p95 anchor lag 达 `71.97 deg`。
- `responsive` profile 的 SMPL/wrist lag 很小（SMPL p95 `0.009 m`，wrist p95 `0.040 rad`），但快速 yaw 段仍会出现约 `72 deg` 的 p95 anchor lag。
- `off` 没有 filter lag，但 raw anchor/wrist 速度直接进入 deploy。
- Sony 当前 wrist `joint_pos` 与 SMPL pose 投影 wrist 的差异很大：总体 p95/max `2.254/2.692 rad`，roll 维 p95 约 `2.69 rad`，yaw 维 p95 约 `2.13 rad`。这是 Sony V3 与 PICO 之间仍需重点 A/B 的字段语义差异。

对比 filter profile：

```bash
.venv_teleop/bin/python -u gear_sonic/scripts/evaluate_sony_pose_v3.py \
  --bvh-file /home/nolo/RAYNOS_Motion1.bvh \
  --pose-filter-profile responsive \
  --json-out /tmp/sony_pose_v3_release_eval_responsive.json

.venv_teleop/bin/python -u gear_sonic/scripts/evaluate_sony_pose_v3.py \
  --bvh-file /home/nolo/RAYNOS_Motion1.bvh \
  --pose-filter-profile off \
  --json-out /tmp/sony_pose_v3_release_eval_off.json
```

当前建议：闭环默认使用 `stable`，`responsive` 只在需要动作响应性对照时显式启用，`off` 仅保留为 raw reference 风险对照。腕部差异需要等 PICO pose window 抓取后再决定是否让 Sony V3 runtime 使用 SMPL pose 投影 wrist 来替代 BVH->G1 retarget wrist。

## 为什么暂停为独立研究线

这条路线不是无效，而是待解决问题更多：

- BVH / mocopi 骨架比例和 SMPL 模型比例不一致；
- BVH local rotation channel 与 SMPL rest pose / axis convention 不一致；
- `Y-up -> Z-up`、root base rotation、heading 归一化必须和 PICO 路严格对齐；
- deploy 当前 release 配置里实际启用的 SMPL observation 需要重新确认；
- 下肢是否来自 `smpl_joints`、`smpl_pose` 还是 `joint_pos`，必须用日志和 observation 配置分开验证。

因此不要用 BVH-G1 的调参结论覆盖 SMPL v3，也不要把 SMPL v3 的字段契约要求反推到 G1 v1。

## 研究命令模板

`--source bvh` 是本地 SMPL debug 入口：

```bash
.venv_teleop/bin/python -u gear_sonic/scripts/mocap_manager_server.py \
  --source bvh \
  --bvh-file /home/nolo/RAYNOS_Motion1.bvh \
  --bvh-loop \
  --control-mode pose \
  --pose-window-size 80 \
  --pose-encoder-mode smpl \
  --pose-protocol-version 3 \
  --zmq-port 5556
```

`bvh_stream` 的 v3 实验入口：

```bash
.venv_teleop/bin/python -u gear_sonic/scripts/mocap_manager_server.py \
  --source bvh_stream \
  --bvh-stream-port 12352 \
  --control-mode pose \
  --pose-window-size 80 \
  --pose-encoder-mode smpl \
  --pose-protocol-version 3 \
  --allow-sony-pose-v3 \
  --bvh-g1-smpl-joints-source g1_fk \
  --zmq-port 5556

.venv_teleop/bin/python -u gear_sonic/scripts/bvh_stream_sender.py \
  --bvh-file /home/nolo/RAYNOS_Motion1.bvh \
  --host 127.0.0.1 \
  --port 12352 \
  --loop
```

`bvh_g1` 的 v3 实验入口：

```bash
.venv_teleop/bin/python -u gear_sonic/scripts/mocap_manager_server.py \
  --source bvh_g1 \
  --bvh-file /home/nolo/RAYNOS_Motion1.bvh \
  --bvh-loop \
  --control-mode pose \
  --pose-window-size 80 \
  --pose-encoder-mode smpl \
  --pose-protocol-version 3 \
  --allow-sony-pose-v3 \
  --bvh-g1-smpl-joints-source g1_fk \
  --zmq-port 5556
```

外置 tmux 启动工具等价写法：

```bash
~/tools/sony-isaaclab-sonic-launcher/launch_sony_isaaclab_closed_loop.py \
  --backend isaaclab \
  --input-source sony \
  --bvh-source bvh_stream \
  --sony-pose-line v3
```

使用前需要确认当前 deploy 的 protocol v3 字段契约和 observation 配置完全一致；如果只是验证当前效果，优先使用 BVH-G1 POSE v1 路线。

## 成功标准

SMPL v3 路线重新推进时，应单独验收：

1. `smpl_joints/smpl_pose/body_quat_w/body_pos` 的 shape、dtype、坐标系与 deploy 解析一致。
2. deploy 当前 `observation_config.yaml` 的 SMPL mode 实际消费项明确。
3. `evaluate_sony_pose_v3.py` 对同一段 BVH 的 skeleton/g1_fk 指标可复现，且解释任何进入闭环前的姿态相似度差异。
4. root/anchor、wrist `joint_pos` 和时序平滑能用 `release_observations` 独立量化。
5. 同一段 BVH 与 BVH-G1 POSE v1、PICO V3 做 A/B，对比动作保真度、稳定性和延迟。
