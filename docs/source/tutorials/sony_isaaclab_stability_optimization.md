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
- `bvh_sender`: `RAYNOS_Motion1.bvh` 循环发送。
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
