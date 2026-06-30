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
3. 再做闭环 A/B：同一 BVH 分别跑 v1/G1、v3 `stable`、v3 `balanced`、v3 `responsive`、v3 `off`，优先用 MuJoCo 快速验证，再记录 IsaacLab 稳定性、动作保真度、root/yaw/height、wrist 差异和延迟。
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

### 2026-06-29 MCPM lower-body 速度峰值压制

对前一节 `right_knee_joint` 窄峰值做第一项优化：只把 v3 默认 `--bvh-g1-max-joint-velocity` 从 `6.0` 下调到 `5.5`，其余参数保持 `stable + root_yaw_only + g1_fk`。

验证命令：

```bash
scripts/launch_sonic_v3_mujoco_closed_loop.py \
  --replace \
  --no-attach \
  --onscreen \
  --bvh-file /home/nolo/MCPM_20260526_190029.BVH \
  --bvh-g1-max-joint-velocity 5.5 \
  --metrics-duration-s 120 \
  --metrics-startup-timeout-s 120 \
  --metrics-no-per-joint-jsonl \
  --metrics-summary-json /tmp/sony_pose_v3_mujoco_mcpm_v55_120s_summary.json \
  --metrics-samples-jsonl /tmp/sony_pose_v3_mujoco_mcpm_v55_120s_samples.jsonl
```

| 指标 | `6.0` baseline | `5.5` optimized |
|------|----------------|-----------------|
| pass | `false` | `true` |
| samples / elapsed | `5473 / 119.96 s` | `5641 / 120.00 s` |
| deploy FPS | `50.01` | `50.00` |
| fall frames | `0` | `0` |
| nonfinite / missing fields | `0 / 0` | `0 / 0` |
| root tilt max | `0.279 rad` | `0.135 rad` |
| joint RMSE mean / p95 / max | `0.441 / 1.030 / 1.194 rad` | `0.303 / 0.364 / 0.489 rad` |
| target step max | `0.408 rad` | `0.480 rad` |
| target velocity max | `9.78 rad/s` | `23.22 rad/s` |
| joint velocity max / p95 | `35.26 / 16.41 rad/s` | `14.21 / 2.72 rad/s` |
| top velocity joint | `right_knee_joint` `35.26 rad/s` | `left_ankle_pitch_joint` `14.21 rad/s` |

补充定位：`6.0` baseline 的右膝峰值帧中，右膝 target velocity 只有约 `0.144 rad/s`、target step 约 `0.003 rad`，因此它更像闭环响应尖峰，而不是输入 target 的单帧跳变。`5.5` 把该尖峰压掉后，右膝峰值降到 `4.60 rad/s`。

结论：v3 专用默认值改为 `--bvh-g1-max-joint-velocity 5.5`；这一步完成了第一项 lower-body 速度峰值优化。

### 2026-06-29 balanced profile 验证

第二项优化新增 `balanced` profile，作为 `stable` 和 `responsive` 之间的可选响应档；默认仍保持 `stable`。第一版 balanced 过于靠近响应侧，MCPM 90s 虽然通过，但右膝峰值回到 `33.61 rad/s`、root tilt max 到 `0.289 rad`，因此收紧为：

| 参数 | `stable` | `balanced` | `responsive` |
|------|----------|------------|--------------|
| `reference_alpha` | `0.35` | `0.45` | `0.75` |
| `max_smpl_joint_speed_mps` | `1.2` | `1.6` | `3.5` |
| `max_smpl_pose_speed_radps` | `3.0` | `4.0` | `10.0` |
| `max_root_angular_speed_radps` | `2.5` | `3.2` | `7.0` |
| `max_joint_speed_radps` | `4.0` | `5.5` | `14.0` |
| `root_tilt_limit_rad` | `0.45` | `0.50` | `0.65` |

MCPM 120s 最终验证：

```bash
scripts/launch_sonic_v3_mujoco_closed_loop.py \
  --replace \
  --no-attach \
  --onscreen \
  --bvh-file /home/nolo/MCPM_20260526_190029.BVH \
  --pose-filter-profile balanced \
  --metrics-duration-s 120 \
  --metrics-startup-timeout-s 120 \
  --metrics-no-per-joint-jsonl \
  --metrics-summary-json /tmp/sony_pose_v3_mujoco_mcpm_balanced_v2_120s_summary.json \
  --metrics-samples-jsonl /tmp/sony_pose_v3_mujoco_mcpm_balanced_v2_120s_samples.jsonl
```

RAYNOS 12s 响应性对照：

```bash
scripts/launch_sonic_v3_mujoco_closed_loop.py \
  --replace \
  --no-attach \
  --onscreen \
  --bvh-file /home/nolo/RAYNOS_Motion1.bvh \
  --pose-filter-profile balanced \
  --metrics-duration-s 12 \
  --metrics-startup-timeout-s 120 \
  --metrics-no-per-joint-jsonl \
  --metrics-summary-json /tmp/sony_pose_v3_mujoco_raynos_balanced_summary.json \
  --metrics-samples-jsonl /tmp/sony_pose_v3_mujoco_raynos_balanced_samples.jsonl
```

| case | pass | samples / elapsed | joint velocity max / p95 | target step max | target velocity max | joint RMSE mean / p95 | root tilt max | top velocity joint |
|------|------|-------------------|--------------------------|-----------------|---------------------|-----------------------|---------------|--------------------|
| MCPM `stable` 120s | `true` | `5641 / 120.00 s` | `14.21 / 2.72 rad/s` | `0.480 rad` | `23.22 rad/s` | `0.303 / 0.364 rad` | `0.135 rad` | `left_ankle_pitch_joint` `14.21 rad/s` |
| MCPM `balanced` 120s | `true` | `5646 / 120.00 s` | `34.23 / 16.76 rad/s` | `0.272 rad` | `12.93 rad/s` | `0.456 / 1.036 rad` | `0.295 rad` | `left_ankle_pitch_joint` `34.23 rad/s` |
| RAYNOS `stable` 12s | `true` | `571 / 12.00 s` | `28.77 / 15.52 rad/s` | `0.057 rad` | `2.71 rad/s` | `0.490 / 0.857 rad` | `0.174 rad` | `left_ankle_pitch_joint` `28.77 rad/s` |
| RAYNOS `balanced` 12s | `true` | `575 / 11.99 s` | `26.44 / 16.94 rad/s` | `0.539 rad` | `25.93 rad/s` | `0.554 / 0.949 rad` | `0.290 rad` | `waist_yaw_joint` `26.44 rad/s` |
| RAYNOS `responsive` 12s | `false` | `563 / 11.97 s` | `48.39 / 22.70 rad/s` | `0.880 rad` | `42.22 rad/s` | `0.660 / 0.991 rad` | `0.377 rad` | `right_shoulder_pitch_joint` `48.39 rad/s` |

结论：`balanced` 可作为动态动作响应性对照档，RAYNOS 的 target step / target velocity 位于 `stable` 和 `responsive` 之间且不触发失败；但 MCPM 长跑的 RMSE、root tilt 和关节速度余量明显弱于 `stable`，因此不替代默认稳定档。后续多 BVH 回归中需要继续观察 `left_ankle_pitch_joint` 和腰 yaw 的单点峰值。

### 2026-06-29 MuJoCo foot metrics 扩展

第三项优化先扩展闭环量化指标，而不是直接调 foot 参数。MuJoCo sim 进程新增可选 `mujoco_metrics` ZMQ topic，v3 专用 launcher 默认打开：

```bash
gear_sonic/scripts/run_sim_loop.py \
  --mujoco-metrics-zmq-bind tcp://*:6158 \
  --mujoco-metrics-zmq-topic mujoco_metrics \
  --mujoco-metrics-hz 50
```

`collect_sonic_mujoco_metrics.py` 会同时订阅 deploy `g1_debug` 和 MuJoCo `mujoco_metrics`：

```bash
gear_sonic/scripts/collect_sonic_mujoco_metrics.py \
  --deploy-endpoint tcp://127.0.0.1:6157 \
  --deploy-topic g1_debug \
  --sim-endpoint tcp://127.0.0.1:6158 \
  --sim-topic mujoco_metrics
```

新增指标：

- `sim_sample_age_s`：metrics 采样时 MuJoCo diagnostic sample 的年龄，默认要求 `<= 0.5 s`。
- `left/right_foot_floor_contact`：MuJoCo contact pair 中足端 body 是否直接碰到 floor。
- `left/right_foot_support_contact`：当前模型里 physical floor contact 始终为 `0`，因此另用“左右足端最低高度 + 0.04 m”推断支撑脚。
- `foot_slip_speed_mps`：支撑脚水平速度的左右最大值，默认 smoke gate `<= 8.0 m/s`。
- `support_foot_drift_m`：一次支撑段内足端水平位移，默认 smoke gate `<= 1.0 m`。
- `foot_contact_ratios`：any contact、double support、no contact、左右 floor/support contact 比例。

验证命令均使用默认 `stable + root_yaw_only + g1_fk + bvh_g1_max_joint_velocity=5.5`，且 `--metrics-no-per-joint-jsonl` 避免 `/tmp` 被 per-joint samples 写满。

| case | pass | samples / FPS | joint velocity max / p95 | RMSE mean | root tilt max | any / double support | floor contact L/R | foot slip max / p95 | support drift max / p95 |
|------|------|---------------|--------------------------|-----------|---------------|----------------------|-------------------|---------------------|--------------------------|
| RAYNOS 12s | `true` | `561 / 50.00` | `29.50 / 16.42 rad/s` | `0.748 rad` | `0.151 rad` | `1.00 / 0.31` | `0.00 / 0.00` | `5.18 / 3.49 m/s` | `0.557 / 0.370 m` |
| MCPM 45s | `true` | `2135 / 50.00` | `34.44 / 15.97 rad/s` | `0.423 rad` | `0.301 rad` | `1.00 / 0.49` | `0.00 / 0.00` | `7.23 / 3.79 m/s` | `0.969 / 0.350 m` |

结论：第 3 项中的 foot/contact/slip 指标扩展完成；当前 MuJoCo 模型没有直接 floor contact pair，因此短期用 support-height 推断支撑脚做 smoke 指标。新的量化结果显示 foot slip / support drift 是下一轮自然度优化目标；多 BVH 自动回归汇总仍需单独做。

### 2026-06-29 多 BVH 回归汇总脚本

新增 `scripts/run_sonic_v3_mujoco_regression.py`，用于把多个 BVH 的 v3 MuJoCo 指标跑成统一 JSON/Markdown 表。默认 case 是：

- `raynos_stable`：`/home/nolo/RAYNOS_Motion1.bvh`，`stable`，`12s`
- `mcpm_stable`：`/home/nolo/MCPM_20260526_190029.BVH`，`stable`，`45s`

真实运行：

```bash
scripts/run_sonic_v3_mujoco_regression.py \
  --output-dir /tmp/sony_pose_v3_mujoco_regression
```

复用已有 summary 做聚合：

```bash
scripts/run_sonic_v3_mujoco_regression.py \
  --from-summary raynos_stable=/tmp/sony_pose_v3_mujoco_raynos_foot_metrics_v4_summary.json \
  --from-summary mcpm_stable=/tmp/sony_pose_v3_mujoco_mcpm_foot_metrics_45s_summary.json \
  --output-dir /tmp/sony_pose_v3_mujoco_regression_existing
```

本次用已有 summary 验证聚合输出：

| case | pass | vel max/p95 | rmse mean/p95 | tilt max | any/double support | floor L/R | slip max/p95 | drift max/p95 |
|------|------|-------------|---------------|----------|--------------------|-----------|--------------|---------------|
| `raynos_stable` | `true` | `29.50 / 16.42` | `0.748 / 1.076` | `0.151` | `1.00 / 0.31` | `0.00 / 0.00` | `5.18 / 3.49` | `0.557 / 0.370` |
| `mcpm_stable` | `true` | `34.44 / 15.97` | `0.423 / 0.977` | `0.301` | `1.00 / 0.49` | `0.00 / 0.00` | `7.23 / 3.79` | `0.969 / 0.350` |

结论：第 3 项量化指标和多 BVH 回归汇总入口已完成。下一项转向 IsaacLab v3 验证，重点补 MuJoCo 不能覆盖的 IsaacLab root/body state、policy 输出和场景稳定性。

### 2026-06-29 MuJoCo regression reliability and alpha=0.35 tuning

按当前阶段安排，IsaacLab 暂停，先把 MuJoCo 默认线调稳。重新运行默认回归时，先暴露出两个 MuJoCo 工具层问题：

- regression 脚本在连续运行 case 时，上一轮 tmux session 退出后 TCP `6156/6158` 端口可能还没完全释放，导致下一轮 preflight 假失败。
- 当某个 case 启动失败时，Markdown 汇总读取缺失指标字段会 `KeyError`，从而掩盖真实失败原因。

本轮修正：

- `run_sonic_v3_mujoco_regression.py` 会在每个 case 前后等待 session 消失和 launcher 端口释放。
- 失败 case 也能写入统一 JSON/Markdown 汇总，不再因缺失指标崩溃。
- `collect_sonic_mujoco_metrics.py` 的 samples JSONL 新增 `relative_time_s`，便于直接定位 foot slip / support drift 峰值时间段。

当前默认线复跑显示，`bvh_g1_joint_filter_alpha=0.45` 在 MCPM 45s 上有窄余量失败：

| case | pass | failed checks | joint velocity max / p95 | RMSE mean / p95 | root tilt max | foot slip max / p95 | support drift max / p95 |
|------|------|---------------|--------------------------|-----------------|---------------|---------------------|--------------------------|
| RAYNOS default `0.45` | `true` | none | `33.36 / 18.58` | `0.512 / 0.866` | `0.209` | `6.13 / 4.50` | `0.663 / 0.498` |
| MCPM default `0.45` | `false` | `support_foot_drift` | `27.10 / 15.33` | `0.459 / 1.017` | `0.305` | `7.29 / 3.79` | `1.016 / 0.406` |

对 MCPM 45s 做小扫参：

| candidate | pass | failed checks | joint velocity max / p95 | RMSE mean / p95 | root tilt max | foot slip max / p95 | support drift max / p95 | 结论 |
|-----------|------|---------------|--------------------------|-----------------|---------------|---------------------|--------------------------|------|
| `--bvh-g1-max-joint-velocity 5.0` | `false` | `foot_slip_speed`, `support_foot_drift` | `29.32 / 15.99` | `0.448 / 1.030` | `0.299` | `8.37 / 4.28` | `1.041 / 0.370` | 单纯压低速度上限会变差 |
| `--bvh-g1-joint-filter-alpha 0.40` | `true` | none | `31.34 / 15.30` | `0.480 / 1.007` | `0.325` | `7.39 / 4.70` | `0.932 / 0.575` | 可过，但自然度和 RMSE 不如 `0.35` |
| `--bvh-g1-joint-filter-alpha 0.35` | `true` | none | `26.27 / 15.03` | `0.416 / 0.929` | `0.276` | `6.42 / 3.45` | `0.906 / 0.322` | 当前最佳，默认采用 |

因此 MuJoCo v3 默认 `--bvh-g1-joint-filter-alpha` 从 `0.45` 改为 `0.35`。新默认完整回归：

```bash
scripts/run_sonic_v3_mujoco_regression.py \
  --output-dir /tmp/sony_pose_v3_mujoco_regression_mu_alpha035_default_20260629 \
  --cleanup-timeout-s 30
```

| case | pass | joint velocity max / p95 | RMSE mean / p95 | root tilt max | any / double support | foot slip max / p95 | support drift max / p95 |
|------|------|--------------------------|-----------------|---------------|----------------------|---------------------|--------------------------|
| `raynos_stable` | `true` | `32.49 / 16.83` | `0.416 / 0.840` | `0.242` | `1.00 / 0.49` | `6.22 / 4.11` | `0.788 / 0.431` |
| `mcpm_stable` | `true` | `32.98 / 16.27` | `0.436 / 0.962` | `0.335` | `1.00 / 0.50` | `7.25 / 3.84` | `0.949 / 0.359` |

结论：MuJoCo 默认线恢复为两条默认 case 全通过；MCPM 的 `support_foot_drift` 从失败的 `1.016 m` 收到 `0.949 m`，且 samples 可直接用 `relative_time_s` 定位峰值。下一轮 MuJoCo 优化若继续推进，应优先扩展 BVH 覆盖和更长时长，而不是再单点压 `max_joint_velocity`。

### 2026-06-29 IsaacLab current-branch v3 smoke

用户切换外部 IsaacLab 到 `sonic-pico-closed-loop` 后，基于该 HEAD 新建调试分支：

```bash
git -C /home/nolo/xiaoyang_IssacLab/IsaacLab switch -c optimization/v3-isaaclab-debug-20260629
```

本轮只做当前分支最小调通，不混入旧的 safety-assist/free-root 改动：

- IsaacLab `SonicRobotStatePublisherAction` 发布包补 `root_pos_w`，让 `collect_sonic_isaaclab_metrics.py` 能计算 root height、tilt、body FK 和 joint tracking。
- v3 wrapper 新增 `--isaac-target-field {body_q_target,last_action}`。本小节最初把默认切到 `body_q_target`，用于对齐 root-locked reference 指标；2026-06-30 free-root 复核后已修正为默认 `last_action`，因为 `last_action` 才是 deploy policy 输出给物理控制的目标。
- `--target-rate-limit` 仍保持 `0.003`。在 `body_q_target` 下，`0.006/0.01/0.02` 虽然能略降 joint RMSE，但会抬高 body RMSE、关节速度和高度代价，综合分数更低。

对比烟测均使用 `/home/nolo/MCPM_20260526_190029.BVH`、`stable + root_yaw_only + g1_fk`、12s headless IsaacLab 闭环：

| case | pass | score | fall / missing / nonfinite | joint RMSE mean / p95 | body RMSE mean / p95 | base height min | joint velocity p95 |
|---|---|---:|---|---|---|---:|---:|
| 缺 `root_pos_w` 初始包 | `false` | `8.000` | `0 / 237 / 237` | - | - | - | - |
| `last_action + root_pos_w` | `false` | `76.664` | `0 / 0 / 0` | `0.357 / 0.474 rad` | `0.045 / 0.076 m` | `0.707 m` | `1.092 rad/s` |
| 默认 `body_q_target + root_pos_w` | `true` | `83.349` | `0 / 0 / 0` | `0.193 / 0.302 rad` | `0.055 / 0.079 m` | `0.724 m` | `1.109 rad/s` |

最终默认验证命令：

```bash
scripts/launch_sonic_v3_tuning_closed_loop.py \
  --replace --no-attach --headless \
  --bvh-file /home/nolo/MCPM_20260526_190029.BVH \
  --metrics-duration-s 12 \
  --metrics-summary-json /tmp/sony_pose_v3_isaaclab_default_after_summary.json \
  --metrics-samples-jsonl /tmp/sony_pose_v3_isaaclab_default_after_samples.jsonl
```

结果：

- summary：`/tmp/sony_pose_v3_isaaclab_default_after_summary.json`
- IsaacLab log：`/tmp/sonic_local_isaaclab_20260629_233105.log`
- 日志确认：`first target parsed field='body_q_target'`
- samples：`235`，deploy/Isaac FPS `50.01 / 200.02`
- 所有 checks 为 `true`，包括 `base_height`、`root_tilt`、`joint_tracking_rmse`、`body_keypoint_rmse`、`target_step_peak`。

结论：基于用户当前 IsaacLab 分支的新调试分支，v3 root-locked reference smoke 已从“协议字段缺失”修到可量化；但 `body_q_target` 只适合作为 root-locked reference 诊断，不应作为 free-root 物理控制默认目标。

### 2026-06-30 IsaacLab free-root target-field correction

用户指出当前 IsaacLab 分支之前可以站住，复核后确认上一轮思路有误：把 `body_q_target` 设为默认控制目标会改善 root-locked reference 指标，但 free-root 解锁时更容易失稳。真正的 IsaacLab 物理控制目标应保持为 deploy 的 `last_action`。

证据：

- 10:28 / 10:58 两次 60s `pass=true` 记录没有触发 unlock，属于 root-locked 通过；日志没有 `root pose unlock`，`root_tilt max=0`。
- 11:27 运行手动触发 `U unlock` 后，`body_q_target` 控制在 `53.15s` 首次 fall，root 高度最小 `0.071 m`，root tilt max `1.799 rad`。
- 同一当前分支用独立端口做自动 unlock A/B，`last_action` 明显更符合可站立控制目标。

自动 unlock A/B 均使用 `/home/nolo/MCPM_20260526_190029.BVH`、`stable + root_yaw_only + g1_fk`：

| case | duration | pass | fall / first fall | root tilt max | base height min | joint RMSE mean / p95 | body RMSE mean | 结论 |
|---|---:|---|---|---:|---:|---|---:|---|
| `last_action` + auto-unlock | 20s | `true` | `0 / none` | `0.000 rad` | `0.696 m` | `0.337 / - rad` | `0.044 m` | 能站住且过当前 gate |
| `body_q_target` + auto-unlock | 20s | `false` | `0 / none` | `0.000 rad` | `0.712 m` | `0.211 / - rad` | `0.058 m` | reference RMSE 好，但 gate 已失败，速度更高 |
| `last_action` + auto-unlock | 60s | `false` | `0 / none` | `0.000 rad` | `0.658 m` | `0.393 / 0.536 rad` | `0.064 m` | 60s 不摔；失败来自 reference RMSE / absolute yaw gate |
| `body_q_target` + manual unlock | 60s | `false` | `135 / 53.15s` | `1.799 rad` | `0.071 m` | `0.233 / 0.370 rad` | `0.060 m` | 指标表面更贴 reference，但 free-root 物理摔倒 |

因此 v3 wrapper 默认恢复为：

```text
--isaac-target-field last_action
```

后续指标也要拆开看：

- **控制稳定性**：free-root 下优先看 fall、root height、root tilt、joint velocity、action tracking。
- **reference 保真度**：`body_q_target` joint/body RMSE 仍保留，但不能单独决定 IsaacLab 物理控制目标。
- **root yaw**：free-root 不应继续用 deploy absolute yaw 作为唯一 gate；需要 base-relative 或单独诊断项。

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
- v3 wrapper 现在也会 local-first / original-fallback 解析 `sonic_unitree_lowstate_cpp_proxy`，避免新 worktree 复制构建产物。
- 本机 `env_isaaclab` 中 Isaac Sim 5.1 pip 包存在，但预检时缺 `pxr`；已在本地环境补 `usd-core==25.11` 后，`teleop_se3_agent.py` 可以 headless 创建 `Isaac-SonicSolo-Locomanipulation-G1-v0` 并以约 `50 Hz` 运行。
- IsaacLab 当前 `locomanipulation_g1_env_cfg.py` 已被瘦身，导致 `SonicSolo` 任务导入旧接口失败；已在 IsaacLab 侧做最小兼容：`SonicSolo` 自包含 SONIC robot/action/state publisher，Sonic 系列任务注册改为字符串 entrypoint，主配置只补回 `_env_flag`。

已验证到的状态：

- `mocap_manager_server.py` 进入 v3：`pose_protocol=v3 pose_encoder=smpl pose_filter=responsive root_yaw_only=1`。
- deploy 识别 SMPL encoder observation：`smpl_joints_10frame_step1`、`smpl_anchor_orientation_10frame_step1`、`motion_joint_positions_wrists_10frame_step1` 维度匹配，总 encoder dim `1762`。
- IsaacLab `SonicSolo` 能完成环境 setup，并以约 `50 Hz` 发布 `sonic_state`。

### 2026-06-29 IsaacLab v3 root-locked conservative 验证

MuJoCo stable 通过后，第一轮 IsaacLab 真实闭环验证暴露出自由根解锁风险：

- 默认 `auto_unlock_after_packets=100`、post-unlock target limit `0.45 rad/step` 时，MCPM 20s 中 unlock 后约 1s 内 `root_tilt` 拉到 `> 1 rad`，`joint_velocity_absmax` 撞到 `37 rad/s`，最终 `fall_frames=331`，`pass=false`。
- 关闭自动 unlock 后，MCPM 20s 无 fall，base/root tilt 稳定，但 `target_rate_limit=0.05` 的 joint RMSE 仍偏高。
- sweep 结果显示 `target_rate_limit=0.003` 是当前 conservative 档：动作更慢，但 20s gate 可过，且速度峰值明显低。

因此 v3 专用 IsaacLab wrapper 采用 root-locked conservative 默认：

| 参数 | 默认值 | 说明 |
| --- | --- | --- |
| `--auto-unlock-after-packets` | `0` | 默认不自动解锁 root；free-root 作为后续单独 A/B |
| `--target-rate-limit` | `0.003` | IsaacLab action target step clamp |
| `--post-unlock-target-rate-limit` | 默认等于 `--target-rate-limit` | 避免手动 unlock 后瞬时放宽到 `0.45` |
| `--metrics-ignore-root-yaw-error` | root-locked 时自动打开 | locked-root 模式不把 root yaw 跟随当作失败项 |

最终验证命令：

```bash
scripts/launch_sonic_v3_tuning_closed_loop.py \
  --replace --no-attach --headless \
  --zmq-port 7056 \
  --debug-port 7057 \
  --state-port 7060 \
  --bvh-stream-port 12503 \
  --bvh-file /home/nolo/MCPM_20260526_190029.BVH \
  --pose-filter-profile stable \
  --metrics-duration-s 20 \
  --metrics-summary-json /tmp/sony_pose_v3_isaaclab_mcpm_20s_conservative_v2_summary.json \
  --metrics-samples-jsonl /tmp/sony_pose_v3_isaaclab_mcpm_20s_conservative_v2_samples.jsonl
```

结果：

| 指标 | 结果 |
| --- | --- |
| pass | `true` |
| samples / elapsed | `391 / 19.95s` |
| deploy / Isaac FPS | `50.02 / 199.93` |
| fall / nonfinite / missing | `0 / 0 / 0` |
| body keypoint RMSE mean / p95 | `0.040 / 0.061 m` |
| joint tracking RMSE mean / p95 | `0.343 / 0.422 rad` |
| joint velocity max / p95 | `2.62 / 1.06 rad/s` |
| base height min / mean | `0.740 / 0.756 m` |
| root tilt max | `0.0 rad` |
| ignored checks | `root_yaw_error=true`，因为 root 被锁定 |

结论：IsaacLab v3 已得到一个可复现的 root-locked conservative 稳定档，补上了 MuJoCo 无法覆盖的 IsaacLab root/body state、policy 输出和场景稳定性验证。下一步才进入 free-root unlock：需要逐步放开 root yaw/translation、post-unlock damping 和 target rate，而不是直接恢复 `auto_unlock_after_packets=100`。

### 2026-06-29 IsaacLab v3 free-root unlock 对照

在 root-locked conservative 档稳定后，继续用 `/home/nolo/MCPM_20260526_190029.BVH` 做 free-root 解锁 A/B。v3 wrapper 新增了以下实验参数，统一透传到 IsaacLab 环境变量或 metrics：

| wrapper 参数 | IsaacLab / metrics 效果 |
| --- | --- |
| `--post-unlock-follow-base` | `SONIC_DEPLOY_POST_UNLOCK_FOLLOW_BASE=1`，并打开 `SONIC_DEPLOY_FOLLOW_BASE_YAW=1` / `SONIC_DEPLOY_FOLLOW_BASE_TRANSLATION=1`，解锁后用 deploy base target 写 root XY/yaw，用作诊断/视觉 replay |
| `--post-unlock-damping-steps` | `SONIC_DEPLOY_POST_UNLOCK_DAMPING_STEPS` |
| `--post-unlock-xy-velocity-scale` | `SONIC_DEPLOY_POST_UNLOCK_XY_VELOCITY_SCALE` |
| `--post-unlock-z-velocity-scale` | `SONIC_DEPLOY_POST_UNLOCK_Z_VELOCITY_SCALE` |
| `--post-unlock-angular-velocity-scale` | `SONIC_DEPLOY_POST_UNLOCK_ANGULAR_VELOCITY_SCALE` |
| `--base-yaw-rate-limit` | `SONIC_DEPLOY_BASE_YAW_RATE_LIMIT` |
| `--base-translation-rate-limit` | `SONIC_DEPLOY_BASE_TRANSLATION_RATE_LIMIT` |
| `--metrics-root-yaw-reference base_relative` | metrics 用“首帧 robot yaw + deploy base yaw delta”作为 root yaw gate，匹配 follow-base 诊断语义 |
| `--ignore-root-yaw-error` | 显式追加 `--metrics-ignore-root-yaw-error` |

对照结果：

| case | pass | fall frames | root tilt max / p95 | base height min | joint velocity max / p95 | joint RMSE mean / p95 | 结论 |
| --- | --- | --- | --- | --- | --- | --- | --- |
| conservative free-root，`auto_unlock_after_packets=100` | `false` | `327` | `2.209 / 2.187 rad` | `0.112 m` | `9.71 rad/s` / - | `0.487` / - | 即使 post-unlock target step 维持 `0.003`，解锁后仍倾倒，问题不是旧 `0.45 rad/step` 放宽尖峰 |
| damping 400 steps，XYZ/angular velocity scale 均 `0.0` | `false` | `291` | `1.818 / 1.745 rad` | `0.060 m` | `16.88 rad/s` / - | `0.421` / - | 速度阻尼单独不足以让 free-root 动力学站稳 |
| `--post-unlock-follow-base` | `false` | `0` | `0.016 rad` / - | `0.747 m` | `1.83 / 0.945 rad/s` | `0.337 / 0.408 rad` | 稳定性通过，只剩 root yaw gate 失败 |
| `--post-unlock-follow-base --base-yaw-rate-limit 0.30` | `false` | `0` | 稳定 | 稳定 | 稳定 | 稳定 | yaw p95 反而变差，说明 yaw metric target 与 follow-base 写入参考不是同一语义 |
| 旧 wrapper：`--post-unlock-follow-base --metrics-root-yaw-reference base_relative` | `false` | `0` | `0.0115 rad` / - | `0.747 m` | `2.48 / 1.16 rad/s` | `0.338 / 0.414 rad` | 暴露 wrapper 只打开 `POST_UNLOCK_FOLLOW_BASE`，未同时打开 `FOLLOW_BASE_YAW/TRANSLATION`；root 仍停在 anchor，base-relative yaw p95 `0.942 rad`，XY p95 `9.73 m` |
| 修正后：`--post-unlock-follow-base` | `true` | `0` | `0.0104 rad` / - | `0.747 m` | `2.59 / 1.09 rad/s` | `0.341 / 0.410 rad` | 真正跟随 base-relative root XY/yaw；root yaw p95 `0.0175 rad`，base-relative XY p95 `0.0169 m`，不再需要 ignore yaw |
| 纯 free-root：`--auto-unlock-after-packets 100 --metrics-root-yaw-reference base_relative` | `false` | `175` in 12s | `1.903 rad` / `1.680 rad` | `0.188 m` | `12.44 / 3.34 rad/s` | `0.547 / 0.822 rad` | yaw gate 通过（p95 `0.368 rad`），但 base height / root tilt / fall 失败，说明下一项是自由根物理稳定 |

最终 wrapper 验证命令：

```bash
scripts/launch_sonic_v3_tuning_closed_loop.py \
  --replace --no-attach --headless \
  --zmq-port 7756 \
  --debug-port 7757 \
  --state-port 7760 \
  --bvh-stream-port 12573 \
  --bvh-file /home/nolo/MCPM_20260526_190029.BVH \
  --pose-filter-profile stable \
  --metrics-duration-s 20 \
  --metrics-summary-json /tmp/sony_pose_v3_isaaclab_mcpm_20s_followbase_baserel_yaw_followxy_summary.json \
  --metrics-samples-jsonl /tmp/sony_pose_v3_isaaclab_mcpm_20s_followbase_baserel_yaw_followxy_samples.jsonl \
  --auto-unlock-after-packets 100 \
  --post-unlock-follow-base
```

最终 20s 结果：

| 指标 | 结果 |
| --- | --- |
| pass | `true` |
| samples / elapsed | `392 / 20.00s` |
| deploy / Isaac FPS | `50.01 / 200.13` |
| fall / nonfinite / missing | `0 / 0 / 0` |
| body keypoint RMSE mean / p95 | `0.041 / 0.063 m` |
| joint tracking RMSE mean / p95 | `0.341 / 0.410 rad` |
| joint velocity max / p95 | `2.59 / 1.09 rad/s` |
| base height min / mean | `0.747 / 0.760 m` |
| root tilt max | `0.0104 rad` |
| root yaw reference | `base_relative` |
| root yaw error p95 | `0.0175 rad` |
| base-relative root XY error mean / p95 | `0.0082 / 0.0169 m` |

结论：`post-unlock-follow-base` 已给 v3 一个可复现的 IsaacLab 诊断/视觉 replay 档，能稳定展示解锁后的 body/keypoint 跟随效果，并且 root yaw gate 现在可以用 `base_relative` 参考自然通过。但它不是纯 free-root 动力学通过，因为 root XY/yaw 在解锁后仍被 deploy base target 写入；真正 free-root 仍会摔倒，下一步要转向 root translation release、足端支撑/平衡控制和自由根姿态稳定。

#### 2026-06-29 pure free-root velocity-assist sweep

follow-base yaw gate 修正后，继续只做纯 free-root 释放诊断：不写 root pose，只调 unlock blend 和 post-unlock root velocity damping。为便于比较，IsaacLab metrics summary 新增 `first_fall_time_s`，v3 wrapper 新增 `--unlock-blend-steps`，不再需要裸传 `--isaac-env SONIC_DEPLOY_UNLOCK_BLEND_STEPS=...`。

| case | pass | first fall | fall frames | base height min | root tilt max / p95 | joint velocity max / p95 | joint RMSE mean / p95 | 结论 |
| --- | --- | --- | --- | --- | --- | --- | --- | --- |
| baseline：`target=0.003` | `false` | `3.07s` | `175 / 235` | `0.188 m` | `1.903 / 1.680 rad` | `12.44 / 3.34 rad/s` | `0.547 / 0.822 rad` | yaw gate 已过，但 root height / tilt 快速失败 |
| `post_unlock_target_rate_limit=0.03` | `false` | `3.17s` | `173 / 235` | `0.071 m` | `2.339 / 2.037 rad` | `12.00 / 7.86 rad/s` | `0.665 / 0.887 rad` | 单纯放开 target rate 变差，方向不成立 |
| `unlock_blend_steps=200` + `damping_steps=800` + velocity scale `0` | `false` | `7.62s` | `86 / 235` | `0.126 m` | `1.620 / 1.540 rad` | `8.67 / 2.07 rad/s` | `0.369 / 0.529 rad` | 根速度释放是主因之一，能显著推迟失稳 |
| `unlock_blend_steps=400` + `damping_steps=1600` + velocity scale `0`，12s | `true` | none in 12s | `0 / 235` | `0.716 m` | `0.101 / 0.076 rad` | `1.13 / 0.67 rad/s` | `0.345 / 0.387 rad` | 强 velocity-assist 可通过 12s，但不代表自由根已稳定 |
| 同上，20s | `false` | `12.79s` | `142 / 392` | `0.059 m` | `1.797 / 1.697 rad` | `12.62 / 2.38 rad/s` | `0.379 / 0.475 rad` | 辅助释放结束后仍倒，纯 free-root 还没解决 |

结论：本轮把纯 free-root 失稳从“yaw/target-rate 误判”收敛到“root velocity / attitude release 后缺少真实支撑稳定”。强 velocity-assist 只是在诊断窗口内把速度压住，20s 仍会倒，所以下一项不应继续单调加大 damping，而应转向足端支撑、CoM/hip 高度约束或策略输入/动作时延对自由根平衡的影响。

#### 2026-06-29 pure free-root safety velocity assist

继续沿纯 free-root 路线做诊断辅助：IsaacLab action 新增 post-unlock safety velocity assist，触发条件来自 root height 和 root tilt。它只写 root velocity，不写 root pose；当高度或倾角进入安全区间时，压低 root XY / angular velocity，抑制向下 Z velocity，并可配置一个小的向上速度下限。因此它不同于 `post-unlock-follow-base`，不会把 root XY/yaw 直接贴回 deploy base target。

v3 wrapper 同步暴露以下参数，便于从专用启动脚本复现实验：

| 参数 | 环境变量 |
| --- | --- |
| `--post-unlock-safety-assist` | `SONIC_DEPLOY_POST_UNLOCK_SAFETY_ASSIST=1` |
| `--post-unlock-safety-strength` | `SONIC_DEPLOY_POST_UNLOCK_SAFETY_STRENGTH` |
| `--post-unlock-safety-min-height` | `SONIC_DEPLOY_POST_UNLOCK_SAFETY_MIN_HEIGHT` |
| `--post-unlock-safety-height-margin` | `SONIC_DEPLOY_POST_UNLOCK_SAFETY_HEIGHT_MARGIN` |
| `--post-unlock-safety-tilt-start` | `SONIC_DEPLOY_POST_UNLOCK_SAFETY_TILT_START` |
| `--post-unlock-safety-tilt-full` | `SONIC_DEPLOY_POST_UNLOCK_SAFETY_TILT_FULL` |
| `--post-unlock-safety-lift-velocity` | `SONIC_DEPLOY_POST_UNLOCK_SAFETY_LIFT_VELOCITY` |

所有 case 均使用 `/home/nolo/MCPM_20260526_190029.BVH`、`auto_unlock_after_packets=100`、纯 free-root、不启用 follow-base。

| case | pass | failed checks | first fall | fall frames | base height min | root tilt max / p95 | root yaw p95 | joint RMSE mean / p95 | joint velocity max / p95 | 结论 |
| --- | --- | --- | --- | --- | --- | --- | --- | --- | --- | --- |
| `safety_blend200`：target `0.003`，blend `200`，damping `800`，min height `0.72`，tilt `0.05/0.28` | `false` | tilt / RMSE | none in 20s | `0 / 392` | `0.665 m` | `0.540 / 0.451 rad` | `0.792 rad` | `0.384 / 0.484 rad` | `7.27 / 0.93 rad/s` | 已消除摔倒，但倾角和 RMSE 仍偏高 |
| `safety_stable_006`：stable，target `0.006`，blend `200`，damping `800`，min height `0.74`，margin `0.22`，tilt `0.03/0.18`，lift `0.12` | `false` | joint tracking RMSE | none in 20s | `0 / 392` | `0.714 m` | `0.210 / 0.183 rad` | `0.671 rad` | `0.363 / 0.479 rad` | `2.78 / 1.39 rad/s` | 当前最佳纯 free-root 候选：无 fall，height/tilt/yaw 均过，只剩 RMSE mean 略高于 `0.35` |
| `safety_stable_008`：同上，target `0.008` | `false` | root yaw / RMSE | none in 20s | `0 / 392` | `0.710 m` | `0.162 / 0.144 rad` | `1.675 rad` | `0.359 / 0.480 rad` | `5.60 / - rad/s` | target rate 再放大后 yaw 明显变差 |
| `safety_balanced_006`：balanced，target `0.006` | `false` | root yaw / RMSE | none in 20s | `0 / 392` | `0.689 m` | `0.274 / 0.207 rad` | `0.986 rad` | `0.372 / 0.537 rad` | `2.30 / - rad/s` | balanced 在 IsaacLab free-root 上不如 stable |

结论：安全速度辅助有效解决了 20s 窗口内的摔倒、高度塌陷和大倾角问题，说明当前主要瓶颈已经从“站不住”推进到“站住后 joint tracking RMSE 还差一小段”。后续不宜继续盲目加强安全辅助，否则会进一步牺牲动作跟随；下一项应在 `safety_stable_006` 这条线上优化 reference/action 时延、target rate release、policy observation 对齐和下肢跟踪误差。

已知剩余缺口：

- 纯 PhysX free-root unlock 已能在 safety velocity assist 下 20s 不摔，但严格 pass 仍差 joint RMSE：当前最佳 `safety_stable_006` 的 mean RMSE 为 `0.363 rad`，阈值是 `0.35 rad`。
- follow-base 诊断档已经能用 `base_relative` yaw gate 通过；后续不能把它误当成纯 free-root 动力学通过。
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
