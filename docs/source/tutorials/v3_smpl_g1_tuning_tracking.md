# v3 SMPL POSE 调优分支跟踪

本页是 `optimization/v3-smpl-g1-tuning-20260629` 及其后续 v3 worktree 的**高层跟踪 / 交接页**:只维护分支目标、状态看板、已固化默认值、前沿缺口和下一步。逐条实验证据(命令、日志、JSON 指标、扫参表)以详细路线文档 [SMPL POSE v3 路线](mocopi_pose_smpl_v3_route.md) 为准,本页不重复。

## 分支元信息

| 项 | 值 |
|---|---|
| 分支 | `optimization/v3-smpl-g1-tuning-20260629` |
| 工作树 | `/home/nolo/GR00T-WholeBodyControl-v3-tuning`(与主 worktree `~/GR00T-WholeBodyControl` 隔离) |
| 当前 JSON 移植分支 | `optimization/v3-old-effect-mc-bvh-20260702` / `/home/nolo/GR00T-WholeBodyControl-v3-old-effect-20260702` |
| base commit | `72c899e`(SONIC IsaacLab 软限幅优化之后) |
| 详细证据日志 | [`mocopi_pose_smpl_v3_route.md`](mocopi_pose_smpl_v3_route.md) |
| 隔离手段 | 软链复用 `.venv_teleop`/`.venv_sim`;`gear_sonic_deploy/build`+`target` 切本地目录;DDS domain **4**;独立端口组;release ONNX 显式指向原 worktree |
| JSON 来源/移植 tag | `sony-bonedata-json-stream/source-feature-20260702` -> `sony-bonedata-json-stream/v3-old-effect-port-20260702` |
| 首次跟踪快照 | 2026-07-01(见文末跟踪日志) |

## 目标与边界

在**不污染已稳定的 BVH-G1 POSE v1 主线**的前提下,独立调优 **SMPL POSE v3** 遥操路线,让它跑通 MuJoCo / IsaacLab 闭环。

| | SMPL POSE v3(本分支调优对象) | BVH-G1 POSE v1(稳定主线,不动) |
|---|---|---|
| 协议 | `protocol_version=3` + `encoder_mode=smpl` | `protocol_version=1` + `encoder_mode=g1` |
| 语义 | 发人体 SMPL-like pose,让 deploy/encoder 侧理解人体 | manager 侧提前算好 G1 `joint_pos/joint_vel` |
| 特点 | 更接近 PICO full-body / SONIC 原生设计;骨架比例、rest pose、坐标轴、observation 契约都更敏感 | 工程可控、当前更稳;不让 deploy 理解人体语义 |

**硬边界**:不能用 v1 的调参结论覆盖 v3,也不能拿 v3 的字段契约反推 v1。

## 关键入口(本分支新增)

| 工具 | 作用 | 隔离端口 / domain |
|---|---|---|
| `scripts/launch_sonic_v3_tuning_closed_loop.py` | v3 IsaacLab 闭环 wrapper(含 `--baseline-isaaclab-stack` 复用基线栈);`--input-source sony_json` 可切到 BoneData JSON sender | `6056/6057/6060/12403`,domain 4 |
| `scripts/launch_sonic_v3_mujoco_closed_loop.py` | v3 MuJoCo 快速验证(默认 `bvh_sender`;JSON 模式为 `json_sender`) | `6156/6157/6158/12413`,domain 4 |
| `scripts/run_sonic_v3_mujoco_regression.py` | 多 BVH case 回归汇总(JSON + Markdown) | 默认 RAYNOS 12s + MCPM 45s |
| `gear_sonic/scripts/sony_bonedata_json_stream_sender.py` | 从 `saveBoneData*.json` 按帧发送 raw `sony_bonedata_json_v1`;v3 一键脚本的 JSON sender pane 复用它 | 默认接收侧转换 |
| `gear_sonic/scripts/collect_sonic_mujoco_metrics.py` | MuJoCo 指标(60Hz、warmup 2s、per-joint、foot slip / support drift、`relative_time_s`) | — |
| `gear_sonic/scripts/evaluate_sony_pose_v3.py` | 离线量化 release observation(smpl_joints / anchor / wrists) | — |

## 状态看板

| 验证通道 | 状态 | 卡点 / 门槛 | 备注 |
|---|---|---|---|
| MuJoCo 默认回归(RAYNOS 12s + MCPM 45s stable) | ✅ 全过 | `alpha=0.35` 收住 MCPM support drift | 无 fall / NaN / 缺字段 |
| BoneData JSON -> v3 MuJoCo | ✅ 20s smoke 通过 | `--input-source sony_json` + `left_handed_yup` | `/tmp/sonic_v3_json_validation_summary.json`: `pass=True`, `fall_frames=0`, `deploy_fps=49.99` |
| IsaacLab root-locked conservative(MCPM 20s) | ✅ 可复现过 | `auto-unlock=0` + `target-rate=0.003` | 补上 MuJoCo 覆盖不到的 root/body state |
| IsaacLab follow-base 诊断档 | 🟡 过但**非纯 free-root** | `base_relative` yaw gate | root XY/yaw 被 deploy base target 写回,仅作视觉 replay |
| IsaacLab 纯 free-root unlock | ❌ 未过(当前前沿) | joint RMSE mean `0.363 > 0.35` | 最佳候选 `safety_stable_006`:20s 不摔,height/tilt/yaw 均过 |

## 已固化默认值

| 参数 | 默认值 | 理由 |
|---|---|---|
| `--bvh-g1-smpl-joints-source` | `g1_fk` | 把 v1 已验证 G1 FK 14 关键点投影到 SMPL 24 槽位,与 v1 几何对齐(RMSE 0.000);`skeleton` 有 ~90° yaw 差 + 98.5% 左右错位,仅留 A/B 回退 |
| `--pose-filter-profile` | `stable` | `responsive/off/balanced` 只做显式对照 |
| `--bvh-g1-max-joint-velocity` | `5.5` | 压 MCPM 下肢速度峰值 |
| `--bvh-g1-joint-filter-alpha`(MuJoCo) | `0.35` | 从 0.45 降下,过 MCPM `support_foot_drift` |
| `--input-source` | 默认 `bvh`;显式 `sony_json` | JSON 模式借鉴 `feature/sony-bonedata-json-stream` 的 sender-pane 思路,但接入当前 v3 Python launcher,不直接使用旧 shell 一键脚本 |
| JSON 接收侧坐标 | `left_handed_yup` | 对 `/home/nolo/saveBoneData_Yup20260702.json` 的 BoneData Y-up 变体做 receiver-side 转换 |
| `--isaac-target-field` | `last_action` | **6-30 修正**:`body_q_target` 只改善 root-locked reference RMSE,free-root 更易摔;`last_action` 才是 deploy policy 给物理控制的真实目标 |
| IsaacLab root | `--auto-unlock-after-packets 0` + `--target-rate-limit 0.003` | 默认锁根保守档 |

## 当前前沿 & 剩余缺口

1. **纯 free-root 就差最后一小截 joint RMSE**(0.363 → 0.35)。文档已明确:不要再盲目加强 safety assist(会牺牲跟随),应在 `safety_stable_006` 这条线上优化 reference/action 时延、target rate release、policy observation 对齐、下肢跟踪误差。
2. **Sony wrist `joint_pos` 与 SMPL pose 投影 wrist 差异大**(p95 ~2.25 rad),是与 PICO 之间仍需重点 A/B 的字段语义差异。
3. `gear_sonic_deploy/target/release/run_tests` 硬编码找 `reference/bones_072925_test/`,v3 worktree 无该测试数据 → 退出 `139`,尚未补。

## 下一步建议

- 攻纯 free-root RMSE:沿 `safety_stable_006` 优化下肢跟踪与时延对齐,而非继续加安全辅助。
- 或转向 wrist 字段语义 A/B(Sony V3 vs PICO),决定 runtime 是否改用 SMPL pose 投影 wrist。

## 工作方法(沿用路线文档约定)

分层推进:**先验数据契约 → 验 release observation → MuJoCo/IsaacLab 闭环 A/B → 最后才改参数/代码**。每次阶段性结论同步回写本工程页(证据入 [`mocopi_pose_smpl_v3_route.md`](mocopi_pose_smpl_v3_route.md))和本地 robotics KB 的 `Sony-mocopi-SMPL-POSE-v3调优分支跟踪` 页。

## 跟踪日志

### 2026-07-02 — BoneData JSON feature 分支能力移植到 v3 一键脚本

- 已给来源点和移植点打配对 tag:`sony-bonedata-json-stream/source-feature-20260702` 指向 `feature/sony-bonedata-json-stream` 的 `dba0b7d`, `sony-bonedata-json-stream/v3-old-effect-port-20260702` 指向当前 v3 移植提交 `44d9e18`。
- 当前 v3 一键脚本借鉴 feature 分支的思路是:JSON sender 独立 tmux pane、接收侧执行 BoneData 坐标/四元数转换、默认 `left_handed_yup`、`saveBoneData_Yup20260702.json`、`27` joints、`50 Hz`。没有原样合入 feature 分支的 `launch_sonic_json_*.sh`;实际入口是当前 v3 Python launcher 的 `--input-source sony_json`。
- MuJoCo 验证命令:

```bash
cd /home/nolo/GR00T-WholeBodyControl-v3-old-effect-20260702
scripts/launch_sonic_v3_mujoco_closed_loop.py \
  --session sonic_v3_mujoco_json_validation \
  --replace --no-attach \
  --input-source sony_json \
  --json-file /home/nolo/saveBoneData_Yup20260702.json \
  --metrics-duration-s 20 \
  --metrics-summary-json /tmp/sonic_v3_json_validation_summary.json \
  --metrics-samples-jsonl /tmp/sonic_v3_json_validation_samples.jsonl
```

- 结果:`pass=True`, `fall_frames=0`, `deploy_fps=49.99`, `samples=947`, `eval_samples=852`;manager 日志显示 `recv_fps≈50`, `pose=sent`, `frame` 持续递增, `dropped=0`。

### 2026-07-01 — 分支状态首次跟踪快照

新会话对分支做完整梳理后建立本跟踪页。当前结论与上表一致:

- MuJoCo 默认线(RAYNOS 12s + MCPM 45s stable)全过;IsaacLab root-locked conservative 与 follow-base 诊断档均可复现通过。
- 唯一未通过的严格门槛是 **IsaacLab 纯 free-root**:最佳候选 `safety_stable_006` 已能 20s 不摔且 height/tilt/yaw 全过,仅 joint RMSE mean `0.363 rad` 略高于 `0.35 rad` 阈值。
- 无新增代码改动,仅建立跟踪页;详细证据仍以 [`mocopi_pose_smpl_v3_route.md`](mocopi_pose_smpl_v3_route.md) 为准。
