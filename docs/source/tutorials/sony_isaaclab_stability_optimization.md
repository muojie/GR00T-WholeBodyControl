# Sony / BVH 到 IsaacLab 闭环稳定性优化跟踪

本文跟踪当前本机闭环验证与后续自动优化任务。主线固定为 BVH stream -> G1 POSE v1 -> deploy -> IsaacLab，不切到 SMPL / POSE v3，也不修改 deploy 输入 wire format。

## 当前通路

```text
BVH sender
  -> UDP bvh_stream_v1 :12352
  -> mocap_manager_server.py --source bvh_stream
  -> ZMQ pose :5556, pose_protocol=v1, pose_encoder=g1
  -> C++ deploy, input-type=zmq_manager
  -> ZMQ g1_debug :5557
  -> IsaacLab SonicSolo, target field last_action, reference field body_q_target
  -> ZMQ sonic_state :5560
  -> sonic_unitree_lowstate_cpp_proxy
  -> DDS rt/lowstate / rt/secondary_imu on lo
  -> C++ deploy closed-loop state
```

边界：

- `mocap_manager_server.py` 必须显示 `pose_protocol=v1`、`pose_encoder=g1`。
- deploy 仍消费原有 `pose` topic，metrics 只订阅旁路状态流。
- `sonic_state` 允许携带额外只读字段，例如 `root_pos_w`、`body_pos14_w`，C++ proxy 仍只读取原有 lowstate 必需字段。

## 启动方式

先 dry-run 确认命令和日志路径：

```bash
cd /home/nolo/GR00T-WholeBodyControl
python scripts/launch_sonic_local_isaaclab_closed_loop.py \
  --dry-run \
  --session sonic_local_isaaclab_codex_20260628 \
  --no-attach
```

正式启动独立 tmux session：

```bash
python scripts/launch_sonic_local_isaaclab_closed_loop.py \
  --session sonic_local_isaaclab_codex_20260628 \
  --replace \
  --no-attach
```

窗口：

- `input`: BVH stream manager，发布 POSE v1 / G1。
- `isaaclab`: 本机 IsaacLab `Isaac-SonicSolo-Locomanipulation-G1-v0`。
- `proxy`: `sonic_unitree_lowstate_cpp_proxy`，从 `sonic_state` 转 DDS lowstate。
- `deploy`: C++ `g1_deploy_onnx_ref`。
- `bvh_sender`: `MCPM_20260526_190029.BVH` 循环发送。
- `metrics`: `collect_sonic_isaaclab_metrics.py`，生成 pass/fail 与 score。

清理：

```bash
tmux kill-session -t sonic_local_isaaclab_codex_20260628
```

## 日志位置

launcher 每次启动使用 `/tmp/sonic_local_<name>_<timestamp>.log`：

- `/tmp/sonic_local_input_*.log`
- `/tmp/sonic_local_isaaclab_*.log`
- `/tmp/sonic_local_proxy_*.log`
- `/tmp/sonic_local_deploy_*.log`
- `/tmp/sonic_local_bvh_sender_*.log`
- `/tmp/sonic_local_metrics_*.log`

metrics 额外输出：

- `/tmp/sonic_local_metrics_samples_*.jsonl`
- `/tmp/sonic_local_metrics_summary_*.json`

## 2026-06-29 回退分支自由根 baseline

在确认 `--post-unlock-follow-base` 属于 BVH 轨迹/动作回放口径、不能代表真实自由根稳定性后，基于错误方向之前的点新建分支重新验证：

- GR00T 分支：`optimization/v1-free-root-stability-20260629`
- IsaacLab worktree：`/home/nolo/xiaoyang_IssacLab/IsaacLab-v1-free-root-20260629`
- IsaacLab 分支：`optimization/v1-free-root-stability-20260629`
- BVH：`/home/nolo/MCPM_20260526_190029.BVH`
- session：`sonic_g1_v1_free_baseline_20260629`

dry-run 后执行：

```bash
scripts/launch_sonic_local_isaaclab_closed_loop.py \
  --session sonic_g1_v1_free_baseline_20260629 \
  --replace \
  --no-attach \
  --isaaclab-root /home/nolo/xiaoyang_IssacLab/IsaacLab-v1-free-root-20260629 \
  --bvh-file ~/MCPM_20260526_190029.BVH \
  --metrics-duration-s 60 \
  --metrics-startup-timeout-s 300 \
  --metrics-summary-json /tmp/sonic_g1_v1_free_baseline_60s_20260629_summary.json \
  --metrics-samples-jsonl /tmp/sonic_g1_v1_free_baseline_60s_20260629_samples.jsonl \
  --zmq-port 6356 \
  --debug-port 6357 \
  --state-port 6360 \
  --bvh-stream-port 12466
```

本次日志：

```text
/tmp/sonic_local_input_20260629_172828.log
/tmp/sonic_local_isaaclab_20260629_172828.log
/tmp/sonic_local_proxy_20260629_172828.log
/tmp/sonic_local_deploy_20260629_172828.log
/tmp/sonic_local_bvh_sender_20260629_172828.log
/tmp/sonic_local_metrics_20260629_172828.log
```

通路证据：

- input 日志显示 `pose_protocol=v1`、`pose_encoder=g1`，`recv_fps≈50`、`pose=sent`，`dropped=0`。
- deploy 日志持续显示 `Protocol version: 1`、`Requested encoder_mode: 0`、`active_protocol_version_=1`。
- proxy 从 `src=synthetic` 切到 `src=isaac`，`isaac_state` 和 `lowcmd` 持续增长。
- IsaacLab 显示 `field='last_action'`、`auto unlock after packet 100`、`root is now free`；未启用 root 回放。
- 本次 baseline 结束后已清理 tmux session。

失败证据：

```text
/tmp/sonic_g1_v1_free_baseline_60s_20260629_summary.json
pass=false
score=4.9499
samples=1174
elapsed_s=59.9856
fall_frames=1015
base_height_m.min=0.0672
root_tilt_rad.max=2.6747
root_yaw_error_rad.p95=2.5068
joint_tracking_rmse_rad.mean=0.9119
body_keypoint_rmse_m.mean=0.1615
foot_keypoint_error_m.p95=0.6458
hand_keypoint_error_m.p95=0.5834
joint_velocity_absmax_radps.p95=37.0
target_step_absmax_rad.p95=0.45
```

额外失败信号：

- deploy 初期出现一次 `Safety reset: ZMQ streaming disabled, returned to reference motion at frame 0`。
- 第 38 个 metrics sample 开始 `base_height_m < 0.55`；第 63 个 sample 开始 `root_tilt_rad > 0.8` 并触发 fall。

结论：回退后的 G1 POSE v1 自由根 baseline 通路是连通的，但稳定性不合格；下一步优化必须以自由根口径为准，优先处理解锁后根姿态快速倾倒、关节跟踪 RMSE 和关节速度峰值，不再用 root 回放分数作为收益。

## 2026-06-29 回退分支 B+C 解锁延后验证

本轮在用户确认后只做第一组低风险 A/B：不改 deploy wire format，不启用 root 回放，只把自动解锁延后并放慢 post-unlock target limiter 释放。

参数：

```text
BVH=/home/nolo/MCPM_20260526_190029.BVH
SONIC_DEPLOY_AUTO_UNLOCK_AFTER_PACKETS=180
SONIC_DEPLOY_TARGET_RATE_LIMIT=0.05
SONIC_DEPLOY_POST_UNLOCK_TARGET_RATE_LIMIT=0.45
SONIC_DEPLOY_POST_UNLOCK_RATE_LIMIT_RELEASE_STEPS=150
pose_protocol=v1
pose_encoder=g1
target_field=last_action
```

### 启动路径污染修复

第一次 A/B 运行使用了 `--isaaclab-root /home/nolo/xiaoyang_IssacLab/IsaacLab-v1-free-root-20260629`，但 IsaacLab 日志实际加载：

```text
[INFO][AppLauncher]: Loading experience file: /home/nolo/xiaoyang_IssacLab/IsaacLab/apps/isaaclab.python.kit
```

原因是 `env_isaaclab` 中 `isaaclab` / `isaaclab_tasks` editable install 仍指向原始仓库：

```text
Editable project location: /home/nolo/xiaoyang_IssacLab/IsaacLab/source/isaaclab
Editable project location: /home/nolo/xiaoyang_IssacLab/IsaacLab/source/isaaclab_tasks
```

因此只 `cd <isaaclab-root>` 并不能保证实际运行回退 worktree。已修复 `scripts/launch_sonic_local_isaaclab_closed_loop.py`：

- 在 IsaacLab 窗口中显式设置 `ISAACLAB_PATH=<--isaaclab-root>`。
- 将 `<--isaaclab-root>/source/{isaaclab,isaaclab_assets,isaaclab_contrib,isaaclab_mimic,isaaclab_rl,isaaclab_tasks}` 放到 `PYTHONPATH` 最前面。
- 启动前打印 `isaaclab.__file__`、`ISAACLAB_PATH`、`EXP_PATH`，以后从日志即可确认实际源码路径。

无效污染 run 记录：

```text
/tmp/sonic_g1_v1_bc_unlock180_release150_60s_20260629_summary.json
pass=false
score=8.4043
fall_frames=8
nonfinite_samples=212
```

该 run 在自动解锁后出现运行时异常，且路径污染到原始 IsaacLab，不能作为自由根稳定性收益。

### 修正路径后的有效重跑

命令：

```bash
scripts/launch_sonic_local_isaaclab_closed_loop.py \
  --session sonic_g1_v1_bc_unlock180_release150_fixedpath_20260629 \
  --replace \
  --no-attach \
  --isaaclab-root /home/nolo/xiaoyang_IssacLab/IsaacLab-v1-free-root-20260629 \
  --bvh-file ~/MCPM_20260526_190029.BVH \
  --auto-unlock-after-packets 180 \
  --post-unlock-rate-limit-release-steps 150 \
  --metrics-duration-s 60 \
  --metrics-startup-timeout-s 300 \
  --metrics-summary-json /tmp/sonic_g1_v1_bc_unlock180_release150_fixedpath_60s_20260629_summary.json \
  --metrics-samples-jsonl /tmp/sonic_g1_v1_bc_unlock180_release150_fixedpath_60s_20260629_samples.jsonl \
  --zmq-port 6556 \
  --debug-port 6557 \
  --state-port 6560 \
  --bvh-stream-port 12486
```

日志：

```text
/tmp/sonic_local_input_20260629_180556.log
/tmp/sonic_local_isaaclab_20260629_180556.log
/tmp/sonic_local_proxy_20260629_180556.log
/tmp/sonic_local_deploy_20260629_180556.log
/tmp/sonic_local_bvh_sender_20260629_180556.log
/tmp/sonic_local_metrics_20260629_180556.log
```

路径与参数证据：

```text
[INFO][AppLauncher]: Loading experience file: /home/nolo/xiaoyang_IssacLab/IsaacLab-v1-free-root-20260629/apps/isaaclab.python.kit
auto_unlock_after_packets=180
post_unlock_target_limit=0.450
post_unlock_release_steps=150
field='last_action'
```

60s 结果：

```text
/tmp/sonic_g1_v1_bc_unlock180_release150_fixedpath_60s_20260629_summary.json
pass=false
score=17.1812
samples=1174
deploy_fps=50.0088
isaac_fps=199.9685
fall_frames=1070
nonfinite_samples=0
base_height_m.min=0.0629
root_tilt_rad.max=1.8172
root_tilt_rad.p95=1.6756
joint_tracking_rmse_rad.mean=0.7572
body_keypoint_rmse_m.mean=0.1345
target_step_absmax_rad.max=0.4500
target_step_absmax_rad.p95=0.3651
root_yaw_error_rad.p95=1.1263
```

相对自由根 baseline：

| 指标 | baseline | B+C fixedpath | 变化 |
|---|---:|---:|---:|
| score | `4.9499` | `17.1812` | `+12.2313` |
| fall_frames | `1015` | `1070` | 变差 |
| nonfinite_samples | `0` | `0` | 持平 |
| base_height_m.min | `0.0672` | `0.0629` | 持平/略差 |
| root_tilt_rad.max | `2.6747` | `1.8172` | 改善但仍摔倒 |
| joint_tracking_rmse_rad.mean | `0.9119` | `0.7572` | 改善 |
| body_keypoint_rmse_m.mean | `0.1615` | `0.1345` | 改善 |
| target_step_absmax_rad.p95 | `0.4500` | `0.3651` | 改善 |
| root_yaw_error_rad.p95 | `2.5068` | `1.1263` | 改善 |

摔倒窗口：

```text
first_tilt>0.8: sample=100 deploy_index=255 height=0.5661 tilt=0.8394 target_step=0.1780
first_height<0.55: sample=103 deploy_index=263 height=0.4721 tilt=0.7028 target_step=0.1967
first_fall: sample=104 deploy_index=265 height=0.3976 tilt=0.8551 target_step=0.2020
```

结论：

- 路径隔离修复是必要工程修复；否则 `--isaaclab-root` 会被 conda editable install 覆盖，回退验证不可信。
- B+C 参数有局部收益：分数、tracking RMSE、body RMSE、root yaw、target step 均改善。
- 但稳定性目标未达成：解锁后约 2s 内仍快速倾倒，`fall_frames=1070`。
- 后续不应继续单纯堆 `auto_unlock_after_packets` / `post_unlock_release_steps`；需要针对解锁瞬间根姿态、支撑脚/下肢目标、自由根速度释放做新方案，并继续用自由根 metrics 做硬门槛。
- `/tmp` 中可能并行存在旧调参日志；以后定位证据必须按端口组/session 匹配，不能只按最新 timestamp 取日志。

## 稳定性指标

`collect_sonic_isaaclab_metrics.py` 订阅 `g1_debug` 和 `sonic_state`，每个样本记录：

- base height: `root_pos_w[2]`
- root tilt: 由 `root_quat_w` 计算
- fall frames: base height 低于阈值或 root tilt 超阈值
- deploy/Isaac state fps: 使用 `g1_debug.index` 和 `sonic_state.sequence`
- joint limit margin: 由 G1 MJCF joint range 与 IsaacLab 实际关节位置计算
- joint velocity peak: IsaacLab `joint_vel`
- target/action peak: deploy `body_q_target`、`last_action`
- target step peak: 相邻 `body_q_target` 最大步长
- nonfinite/missing fields: 状态流缺字段或 NaN/Inf

## 姿势还原指标

采集目标与实际：

- 目标 reference: deploy `body_q_target`、`base_trans_target`、`base_quat_target`
- IsaacLab 实际状态: `sonic_state.joint_pos`、`root_pos_w`、`root_quat_w`、`body_pos14_w`

计算：

- joint tracking RMSE / absmax
- body keypoint RMSE / absmax，14 个 SONIC body keypoint
- root yaw error / root tilt error / root height error
- hand / foot / knee / elbow group keypoint error

`body_pos14_w` 缺失时，collector 会用 `gear_sonic.utils.teleop.sources.g1_body_fk.G1BodyFk` 从实际关节和 root pose 计算 14 个 keypoint。

## 自动优化参数

第一批只优化已有启动参数和环境变量，不扩大协议面：

- `--physics-mode`
- `--visual-servo-mode`
- `--self-collisions`
- `--stabilize-root`
- `--target-rate-limit`
- `--post-unlock-target-rate-limit`
- `--post-unlock-rate-limit-release-steps`
- `--follow-alpha`
- `SONIC_DEPLOY_BASE_YAW_RATE_LIMIT`
- `SONIC_DEPLOY_BASE_TRANSLATION_RATE_LIMIT`
- `SONIC_DEPLOY_BASE_HEIGHT_RATE_LIMIT`
- `SONIC_DEPLOY_BLEND_REFERENCE_LOWER_BODY`
- `SONIC_DEPLOY_FOLLOW_BASE_YAW`
- `SONIC_DEPLOY_FOLLOW_BASE_TRANSLATION`
- `SONIC_DEPLOY_FOLLOW_BASE_HEIGHT`
- `SONIC_DEPLOY_KEEP_FEET_ON_GROUND`

score 函数由 metrics summary 给出，形式为：

```text
score = 100 - weighted normalized penalties
```

扣分项包括 fall/nonfinite/missing、joint RMSE、body RMSE、root yaw/tilt error、joint velocity peak、target step peak、joint limit margin。分数越高越好，后续自动搜索优先最大化 `score`，同时保留每个 pass/fail check 作为硬门槛。

## 验收标准

一次闭环验证通过需要同时满足：

- input/mocap 日志显示 `pose_protocol=v1`、`pose_encoder=g1`、`pose=sent`。
- BVH sender `sent` 持续增长。
- manager `recv_fps` 正常，`dropped` 和 `last packet error` 不持续增长。
- deploy 日志无 protocol mismatch、safety stop、NaN/Inf、decoder/encoder fatal error。
- proxy 日志出现 `src=isaac`，不是长期 `src=synthetic`。
- IsaacLab 日志显示 `SonicRobotStatePublisher` 正常发布。
- metrics summary `pass=true`，且 `score`、samples、deploy_fps、isaac_fps、RMSE、fall_frames 有明确数值。

## 2026-06-29 解锁后目标软限幅优化记录

本轮目标是提高 IsaacLab 内 `sonic_robot` 的稳定性和 BVH 动作还原分数，不改变 deploy wire format，仍保持：

```text
bvh_stream -> POSE v1 / encoder_mode=g1 -> deploy -> g1_debug -> IsaacLab last_action
```

### 代码改动

| 仓库 | 提交 | 改动 |
|---|---|---|
| `GR00T-WholeBodyControl` | `86762e5` | `scripts/launch_sonic_local_isaaclab_closed_loop.py` 默认 `--target-rate-limit` 从 `0.04` 调整为 `0.05`；新增 `--post-unlock-target-rate-limit 0.45` 和 `--post-unlock-rate-limit-release-steps 50`；把两个新参数导出为 IsaacLab 环境变量；修正 `--replace` 时先 kill 同名 tmux session 再做端口 preflight，避免旧 session 占用 `5556/5557/5560` 时脚本自己启动失败。 |
| `IsaacLab` | `d03824b60` | `SonicDeployTargetActionCfg` 新增 `post_unlock_target_rate_limit_rad_per_step` 和 `post_unlock_rate_limit_release_steps`；`SonicDeployTargetAction._apply_target_rate_limit()` 在 root 解锁或 unlock blend 期间从 startup limiter 线性释放到 post-unlock 上限；`locomanipulation_g1_env_cfg.py` 从 `SONIC_DEPLOY_POST_UNLOCK_TARGET_RATE_LIMIT` 和 `SONIC_DEPLOY_POST_UNLOCK_RATE_LIMIT_RELEASE_STEPS` 读取参数；初始化日志打印实际 limiter 配置。 |

### 参数结论

最终保留的组合：

```text
SONIC_DEPLOY_TARGET_RATE_LIMIT=0.05
SONIC_DEPLOY_POST_UNLOCK_TARGET_RATE_LIMIT=0.45
SONIC_DEPLOY_POST_UNLOCK_RATE_LIMIT_RELEASE_STEPS=50
SONIC_DEPLOY_AUTO_UNLOCK_AFTER_PACKETS=100
```

解释：

- root 锁定阶段仍需要限速，避免从默认站姿到 deploy 初始动作目标时出现大跳变。
- 解锁后不能完全保留 `0.04` 级别的强限速，否则平衡环响应太慢。
- 直接完全放开又会产生 `target_step` 尖峰；本轮用 `0.45 rad/step` 做 post-unlock 软上限。

### 量化收益

有效验证结果：

```text
/tmp/sonic_local_metrics_summary_20260629_103431.json
pass=true
score=80.39708534989424
samples=1174
deploy_fps=50.0116
isaac_fps=199.9965
fall_frames=0
```

相对上一轮有效 baseline `/tmp/sonic_local_metrics_summary_20260628_154348.json`：

| 指标 | 旧值 | 新值 | 变化 |
|---|---:|---:|---:|
| score | `78.5018` | `80.3971` | `+1.8952` |
| fall_frames | `0` | `0` | 持平 |
| target_step_absmax_rad.max | `0.8095` | `0.4500` | 明显下降 |
| joint_tracking_rmse_rad.mean | `0.18636` | `0.18608` | 基本持平 |
| body_keypoint_rmse_m.mean | `0.01532` | `0.01564` | 基本持平 |
| root_yaw_error_rad.p95 | `0.10049` | `0.09445` | 小幅改善 |
| root_tilt_rad.max | `0.22625` | `0.37815` | 变差但仍未触发 fall |
| joint_velocity_absmax_radps.max | `11.1824` | `13.8923` | 变差 |

主要收益来自 `target_step` 扣分下降：

```text
target_step score penalty: 5.6667 -> 3.1500
```

### 失败 A/B 反证

只保留 post-unlock 上限、但把 startup limiter 回退到 `0.04` 的验证失败：

```text
/tmp/sonic_local_metrics_summary_20260629_103732.json
pass=false
score=13.55135478285115
fall_frames=1076
base_height_min=0.0598m
root_tilt_max=2.5940rad
```

结论：`0.45` post-unlock cap 不是单独收益，启动锁根阶段需要 `0.05` 让机器人在解锁前更接近 deploy target。后续若继续优化，不能只压低 limiter，需要同时看解锁瞬间的 root tilt、joint velocity 和 policy action 响应。

## 2026-06-29 端口一致性与 MCPM 长窗复验记录

用户现场复核仍看到机器人摔倒后，复查发现有两类干扰源：

1. `scripts/launch_sonic_local_isaaclab_closed_loop.py --debug-port ...` 只传给 IsaacLab、metrics 和 BVH wait，未传给 deploy；deploy 会回落到默认 `5557`，导致自定义端口验证可能接到旧 deploy 或 stale stream。
2. 现场残留一个 `sonic_v3_tuning_20260629` tmux session，运行的是 `GR00T-WholeBodyControl-v3-tuning` 的 SMPL POSE v3 路线，并且 BVH 是 `RAYNOS_Motion1.bvh`，不是当前指定的 `~/MCPM_20260526_190029.BVH`。该 session 的 30s metrics 已经显示摔倒，不能作为当前 G1 POSE v1 路线结论。

本轮工程修复：

```text
deploy command now passes:
--zmq-out-port <debug_port>
--zmq-out-topic <debug_topic>
```

因此 launcher 中 `--debug-port` / `--debug-topic` 会同时作用于 deploy、IsaacLab、metrics 和 BVH sender wait，端口不再分裂。

本轮环境处理：

```bash
sudo -n sysctl -w fs.inotify.max_user_watches=1048576 \
  fs.inotify.max_user_instances=1024 \
  fs.inotify.max_queued_events=32768
tmux kill-session -t sonic_v3_tuning_20260629
```

原因：Isaac Sim 日志中出现大量 `Failed to create change watch ... errno=28/No space left on device`，实际磁盘仍有空间，属于 inotify watcher 上限耗尽；同时 v3 残留 session 会占用端口和 GPU/CPU 资源并输出错误 BVH 的摔倒结果。

复验命令：

```bash
python scripts/launch_sonic_local_isaaclab_closed_loop.py \
  --session sonic_g1_v1_mcpm_20260629 \
  --replace \
  --no-attach \
  --isaaclab-root /tmp/isaaclab-sonic-clean-20260629 \
  --zmq-port 5956 \
  --debug-port 5957 \
  --state-port 5960 \
  --bvh-stream-port 12392 \
  --post-unlock-target-rate-limit 0.45 \
  --post-unlock-rate-limit-release-steps 50 \
  --metrics-duration-s 180 \
  --metrics-report-interval-s 5 \
  --metrics-summary-json /tmp/sonic_g1_v1_mcpm_180s_20260629_summary.json \
  --metrics-samples-jsonl /tmp/sonic_g1_v1_mcpm_180s_20260629_samples.jsonl
```

链路证据：

- `bvh_sender` 使用 `/home/nolo/MCPM_20260526_190029.BVH`，从 `source_frame=1` 开始发送。
- deploy debug socket 绑定 `5957`，metrics 订阅 `tcp://127.0.0.1:5957/g1_debug`。
- IsaacLab 日志显示 `auto unlock after packet 100; equivalent to operator pressing U`，解锁后继续运行。

180s 结果：

```text
/tmp/sonic_g1_v1_mcpm_180s_20260629_summary.json
pass=true
score=81.35578393790246
samples=3516
deploy_fps=50.0022
isaac_fps=200.0090
fall_frames=0
base_height_m.min=0.6599
base_height_m.mean=0.7719
root_tilt_rad.max=0.2765
joint_tracking_rmse_rad.mean=0.1758
body_keypoint_rmse_m.mean=0.01382
target_step_absmax_rad.max=0.45
```

结论：在清理 v3 残留、修复 deploy debug port 传参、提高 inotify watcher 上限后，当前 G1 POSE v1 + MCPM BVH 路线 180s 长窗未复现摔倒。若后续用户仍在画面中看到摔倒，第一优先级不是继续盲目调 limiter，而是先确认实际 attach 的 tmux session、BVH 文件、端口组和 pose line 是否与上面的验证命令一致。

## 2026-06-28 本机验证记录

本次验证命令：

```bash
python scripts/launch_sonic_local_isaaclab_closed_loop.py \
  --session sonic_local_isaaclab_codex_20260628 \
  --replace \
  --no-attach \
  --metrics-duration-s 60 \
  --metrics-startup-timeout-s 300
```

结果：未通过。链路打通，但稳定性和跟踪质量没有过验收门槛。

通过证据：

- input 日志显示 `pose_protocol=v1`、`pose_encoder=g1`，`recv_fps` 约 50，`dropped=0`。
- BVH sender 持续向 `udp://127.0.0.1:12352` 发送 `RAYNOS_Motion1.bvh`。
- proxy 从 `src=synthetic` 切到 `src=isaac`，后续 `lowcmd` 和 `isaac_state` 持续增长。
- deploy 接收到 `protocol_version: 1`，`active_protocol_version_=1`，`result.motion->GetEncodeMode()=0`。
- IsaacLab 显示 `SonicDeployTarget` 收到 target packet，`SonicRobotStatePublisher` 以 `body_keypoints=14` 发布状态。
- metrics 采到 `samples=1172`，`deploy_fps=50.01`，`isaac_fps=199.96`，`missing_field_samples=0`，`nonfinite_samples=0`。

失败证据：

- metrics summary: `pass=false`，`score=22.14`。
- `fall_frames=257`，`base_height_min=0.1519m`，`base_height` 与 `no_fall_detected` check 失败。
- `joint_tracking_rmse_rad.mean=0.6362`，`p95=1.0535`，joint tracking check 失败。
- `root_yaw_error_rad.p95=1.5543`，`max=3.0720`，root yaw check 失败。
- `target_step_absmax_rad.max=1.1113`，target step check 失败。
- deploy 初期出现一次 safety reset：`Switched to: STREAMED MOTION mode (safety reset)`，随后 `Safety reset: ZMQ streaming disabled, returned to reference motion at frame 0`。

本次日志：

- `/tmp/sonic_local_input_20260628_122619.log`
- `/tmp/sonic_local_isaaclab_20260628_122619.log`
- `/tmp/sonic_local_proxy_20260628_122619.log`
- `/tmp/sonic_local_deploy_20260628_122619.log`
- `/tmp/sonic_local_bvh_sender_20260628_122619.log`
- `/tmp/sonic_local_metrics_20260628_122619.log`
- `/tmp/sonic_local_metrics_summary_20260628_122619.json`
- `/tmp/sonic_local_metrics_samples_20260628_122619.jsonl`

下一轮优化应先降低 target step、root yaw error 和 joint tracking RMSE，再看 base height / fall frames 是否随之恢复。优先尝试 `--target-rate-limit`、`--stabilize-root`、base follow 相关环境变量，以及 lower-body reference blend，不先扩大 POSE wire format。

## 待办项和风险

- 真实 IsaacLab 首次启动时间可能超过 metrics 默认等待时间，可用 `--metrics-startup-timeout-s` 调整。
- `g1_debug.last_action` 是 deploy 输出的关节目标，不是未缩放 raw action；文档和 score 里按 target/action peak 使用。
- body keypoint RMSE 当前比较 pelvis-local 形状还原，减少全局 root 偏移干扰；需要评估全局位姿时另看 root height/yaw/tilt。
- C++ proxy 忽略 `sonic_state` 的新增只读字段，但旧二进制如果使用过小 ZMQ buffer 可能需要重编译或调大接收缓冲。
- 当前 pass/fail 阈值是验证用初值，后续应根据稳定动作和失败动作各跑一组样本再收紧。
