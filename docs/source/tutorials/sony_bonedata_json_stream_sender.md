# Sony BoneData JSON raw stream sender

本文说明当前 Sony mocopi `saveBoneData*.json` 是怎么送进 SONIC 的。重点是：
sender 只负责把 JSON 按帧发出去；坐标系、四元数顺序、scale、root-local 等转换
都在 `mocap_manager_server.py --source bvh_stream` 的接收侧完成。

## 数据链路

```text
Sony BoneData JSON
  -> gear_sonic/scripts/sony_bonedata_json_stream_sender.py
  -> UDP sony_bonedata_json_v1 raw frame, 默认 127.0.0.1:12352
  -> mocap_manager_server.py --source bvh_stream
  -> BvhStreamUdpSource 接收侧转换为 bvh_stream_v1 skeleton frame
  -> skeleton retarget / BVH-G1 route
  -> ZMQ pose protocol v1, encoder_mode=g1, 默认 5556
  -> g1_deploy_onnx_ref
  -> MuJoCo / IsaacLab closed loop
```

`--source bvh_stream` 这里表示复用实时 UDP skeleton 输入源，不表示输入文件必须是
BVH。普通 BVH sender 仍可发送标准 `bvh_stream_v1`；BoneData sender 发送 raw
`sony_bonedata_json_v1`，由 receiver 自动转换成同一个下游 skeleton frame。

## 输入 JSON 结构

BoneData JSON 顶层是 dict，包含三组数组：

| 字段 | 类型 | 说明 |
|---|---|---|
| `name` | string array | bone / joint 名称 |
| `position` | object/list array | 每项为 `{x, y, z}` 或 `[x, y, z]` |
| `rotation` | object/list array | 每项为 `{x, y, z, w}` 或长度 4 的 list |

`position` 和 `rotation` 必须按帧展平成一维数组。`name` 支持两种布局：

```text
布局 A: len(name) == len(position) == len(rotation)，name 每帧重复
布局 B: len(name) == 27，position/rotation 为 frame_count * 27
```

同一个 sender 会话中，每帧 27 个节点的顺序必须一致。

除 `name/position/rotation` 外的顶层字段（如 `playbackFps`、`frameStride`）会被
加载器忽略，不会报错。实际播放/发送帧率以 sender 的 `--fps` / `--source-fps` 为准。

### 旧格式文件兼容性

转换迁到接收侧（commit `62c4afc`）只改变了 Y-up、四元数顺序等处理的位置，**输入
JSON 文件格式本身没有变化**。为旧版 sender 准备的 `saveBoneData*.json`（布局 B +
dict 形式 `{x,y,z}` / `{x,y,z,w}`）无需任何转换即可直接用当前 sender 发送；Y-up
数据改为在 manager 侧用 `--bvh-stream-bonedata-coordinate-frame left_handed_yup`
处理。2026-07-02 已用 `/home/nolo/saveBoneData_Yup20260702.json`（7305 帧 × 27
节点）验证：加载、单帧 `left_handed_yup` 转换、UDP 端到端收发解包均通过，单帧
msgpack 包约 2.7 KB。

如需逐帧 JSONL（每行一个 payload，供其他工具消费），可用
`scripts/convert_savebonedata_to_per_frame_jsonl.py`；发送链路本身不需要这一步。

## UDP raw payload

sender 每帧发送一个 UDP datagram，payload 只保留原始 BoneData 字段和帧元数据：

```json
{
  "format": "sony_bonedata_json_v1",
  "schema_version": 1,
  "path": "/home/nolo/saveBoneData_Yup20260702.json",
  "motion_name": "saveBoneData_Yup20260702",
  "joints_per_frame": 27,
  "frame_index": 0,
  "source_frame_index": 0,
  "source_fps": 50.0,
  "fps": 50.0,
  "frame_stride": 1,
  "source_time_ns": 1782791000000000000,
  "name": ["root", "torso_1", "torso_2"],
  "position": [{"x": 0.0, "y": 1.0, "z": 0.0}],
  "rotation": [{"x": 0.0, "y": 0.0, "z": 0.0, "w": 1.0}],
  "packet_format": "msgpack"
}
```

sender 不再输出 `world_positions` / `world_quat_wxyz`。这些字段由接收侧生成。

`frame_index` 必须逐帧递增（外部发送端同样适用）。POSE 发布器按 `frame_index`
去重，缺失或恒为常量会导致机器人定格在第一帧，见下文"故障排查"。

## 单独启动 sender

```bash
cd /home/nolo/GR00T-WholeBodyControl-sony-json-stream-20260702
export PYTHONPATH="$PWD:${PYTHONPATH:-}"

.venv_teleop/bin/python -u gear_sonic/scripts/sony_bonedata_json_stream_sender.py \
  --json-file /home/nolo/saveBoneData_Yup20260702.json \
  --host 127.0.0.1 \
  --port 12362 \
  --fps 50 \
  --loop
```

常用参数：

| 参数 | 默认值 | 说明 |
|---|---:|---|
| `--json-file` | 必填 | BoneData JSON 路径 |
| `--host` | `127.0.0.1` | manager 所在主机 |
| `--port` | `12352` | manager 的 `--bvh-stream-port` |
| `--format` | `msgpack` | UDP payload 编码，可选 `msgpack` / `json` |
| `--fps` | `50` | 播放和发送频率 |
| `--source-fps` | 同 `--fps` | 写入 payload 的源数据 FPS |
| `--loop` | false | 文件播完后从第一帧继续循环 |
| `--joints-per-frame` | `27` | 每帧骨骼/节点数 |
| `--max-frames` | 不限制 | 调试时只发送前 N 帧 |

坐标系相关参数不在 sender 上配置。

## 接收侧转换参数

manager 仍走 BVH-G1 POSE v1 主线。BoneData raw frame 的转换参数现在放在
receiver 侧：

```bash
PYTHONPATH="$PWD:${PYTHONPATH:-}" .venv_teleop/bin/python -u gear_sonic/scripts/mocap_manager_server.py \
  --source bvh_stream \
  --bvh-stream-port 12362 \
  --bvh-stream-bonedata-coordinate-frame left_handed_yup \
  --bvh-stream-bonedata-position-scale 1.0 \
  --bvh-stream-bonedata-input-quat-order xyzw \
  --bvh-stream-bonedata-rotation-mode input \
  --control-mode pose \
  --pose-window-size 80 \
  --pose-encoder-mode g1 \
  --pose-protocol-version 1 \
  --zmq-port 5656 \
  --log-interval-s 1.0
```

接收侧支持的坐标模式：

| 模式 | 说明 |
|---|---|
| `sonic_zup` | 输入已经是 SONIC/MuJoCo 的右手 Z-up |
| `left_handed_zup` | 输入是左手 Z-up，镜像 X |
| `left_handed_yup` | 输入是左手 Y-up，`position: (x,y,z)->(-x,-z,y)` |
| `zup_flip_xy` | 输入数值已是 Z-up，但水平 X/Y 两轴相反 |

`/home/nolo/saveBoneData_Yup20260702.json` 已按几何检查确认是 Y-up，高度主要在
`y` 轴；当前一键脚本默认使用接收侧 `left_handed_yup`。

## 一键 MuJoCo 验证

```bash
cd /home/nolo/GR00T-WholeBodyControl-sony-json-stream-20260702

# MuJoCo + manager + deploy + raw JSON sender，默认 backend
scripts/launch_sonic_json_mujoco_closed_loop.sh --backend mujoco

# IsaacLab + manager + deploy + raw JSON sender
scripts/launch_sonic_json_mujoco_closed_loop.sh --backend isaaclab

# 只启动接收端，外部单独发 raw JSON
scripts/launch_sonic_json_mujoco_closed_loop.sh --backend mujoco --receiver-only

# 只启动 raw JSON sender
scripts/launch_sonic_json_mujoco_closed_loop.sh --sender-only /home/nolo/saveBoneData_Yup20260702.json

# 打印 sender 命令
scripts/launch_sonic_json_mujoco_closed_loop.sh --print-sender-command /home/nolo/saveBoneData_Yup20260702.json
```

默认端口：

| 项 | 默认值 |
|---|---:|
| raw JSON UDP / `bvh_stream` | `12362` |
| manager ZMQ | `5656` |
| deploy debug ZMQ | `5657` |
| receiver coordinate frame | `left_handed_yup` |

选择 `--backend isaaclab` 时默认端口切到 IsaacLab 闭环常用值：

| 项 | 默认值 |
|---|---:|
| raw JSON UDP / `bvh_stream` | `12352` |
| manager ZMQ | `5556` |
| deploy debug ZMQ | `5557` |
| IsaacLab state ZMQ | `5560` |

## 故障排查：帧持续到达但机器人定格在第一帧姿势

**症状**：UDP JSON 帧持续收到（`recv` 持续增长、`recv_fps` 正常），机器人摆出的
姿势本身是对的，但之后一直保持不动。

**日志指纹**（manager.log，2026-07-02 实机定位）：

```text
recv_fps=70.6 recv=5654 ... pose=buf:0/80 ... frame=0 source_ts=None
```

三个信号同时出现即可确诊：

- `frame=0` 恒定不变——每个包解析出的 `frame_index` 都是 0；
- `pose=buf:0/80` 永远不涨——POSE 滑窗一帧都没进；
- `source_ts=None`——包不是仓库自带 sender 发的。仓库
  `sony_bonedata_json_stream_sender.py` 每帧必填递增 `frame_index` 和
  `source_time_ns`；外部 Unity/Sony 端 raw payload 若缺 `frame_index` 就会命中
  此问题。

**根因链**：

1. raw payload 缺 `frame_index`，接收侧转换函数把它默认成 **0**
   （`gear_sonic/utils/teleop/sources/sony_bonedata_json.py` 中
   `int(payload.get("frame_index", 0))`）。
2. `bvh_stream_source.py` 的 `_payload_to_frame` 本有兜底
   `payload.get("frame_index", receive_sequence)`，但转换先执行、payload 已被塞进
   `frame_index=0`，兜底永远不触发。
3. `PoseStreamPublisher.publish()`（`zmq_pose_sender.py`）按 `frame_index` 去重：
   与上一帧相同直接丢弃。首帧走 bootstrap 通道把机器人摆到正确姿势，之后每帧都被
   判为重复帧丢掉，POSE 缓冲永远 `0/window`，机器人定格。

坐标转换、retarget 链路全部正常——"姿势是对的"正说明只有去重环节卡死。

**处理结果**（2026-07-02）：在外部 Unity/Sony 发送端修复——每帧携带递增
`frame_index`；接收侧代码不改。由此固化为**协议要求**：

- 外部 raw `sony_bonedata_json_v1` 发送端必须每帧携带递增 `frame_index`
  （建议同时带 `source_time_ns`，便于丢包/乱序诊断）；
- 接收侧对缺失的 `frame_index` 仍默认 0，不做 `receive_sequence` 回填——发送端
  缺失或只带常量 `frame_index=0` 都会复现本症状，属发送端 bug；
- `joint_vel` 差分依赖 `source_frame_index` 递增，发送端带递增帧号后该差分同时
  恢复正常。

## 故障排查:上肢/转身正常但走路不动、转身角度偏差(2026-07-03 定位)

在 frame_index 问题修复后,MuJoCo 闭环仍表现为:上肢跟踪完美、原地转身基本正常
(角度略差)、走路完全不行。用 g1_debug(target vs measured)+ 仿真底座真值双通道
量化,定位出三个互相独立的根因,全部已修复:

**根因 1:MuJoCo 虚拟弹力带默认吊着机器人(仿真侧,影响最大)**

`ENABLE_ELASTIC_BAND: True` 且 `ElasticBand.enable` 默认 True,唯一手动释放方式
是 MuJoCo viewer 窗口按键 `9`。被吊着的机器人:上肢/原地转身正常,但脚不吃地,
永远走不了路——真值显示走路段 pelvis z 恒 0.949 m(悬挂)、水平位移 0.00 m。
修复:`base_sim.py` 新增 `SONIC_SIM_BAND_RELEASE_S`(仿真时间定时自动释放),
一键脚本默认 `BAND_RELEASE_S=90`(此时 deploy 已进入跟踪、参考正处站立段)。

**根因 2:catch-up 风暴(manager→deploy 链路)**

manager 主循环与 sender 各自 50Hz 不同步,拍频导致每 ~5.8s 跳过一个源帧号;
deploy 端 `StreamedMotionMerger` 只用窗口前两个帧号推断 `frame_step`,一个空洞
就把整窗误判为 stride-2,下一条消息触发强制 catch-up:回放倒带 ~1.6s + 朝向
重锚。实测 2 小时 1469 次;走路变成走-倒带循环,快速转身时重锚把瞬时 yaw 烘进
锚点(转身角度偏差的来源之一)。
修复:`zmq_pose_sender.py` 的 POSE 窗口帧号改用内部连续计数(commit `c47577a`),
修复后跨 JSON 回绕 catch-up 新增为 0。

**根因 3:deploy 流式稳态下朝向锚点被逐 tick 重置(deploy 侧)**

`UpdateHeadingState` 在 `current_frame_==0` 时重置参考朝向锚点。流式回放稳态
游标恰好停在滑窗起点(current_frame 在 0↔1 抖动),锚点逐 tick 重置 → 策略看到
的朝向误差恒为 0 → 底座永不转身。catch-up 风暴消失后该 bug 完全暴露(修复前
旋转段 target +355° / measured 恒 0°)。
修复:仅预加载动作保留 frame-0 重锚,流式动作(`name=="streamed"`)只经
`reinitialize_heading_` 锚定(`g1_deploy_onnx_ref.cpp`,commit `3f9e2ad`,
需重编译:`cmake -S . -B build_fix && cmake --build build_fix --target
g1_deploy_onnx_ref`,产物直接落 `target/release/`)。

**修复后实测**(saveBoneData_Yup20260702,`--bvh-g1-min-root-height 0.55
--bvh-g1-lower-scale 0.75`,弹力带 90s 释放):

| 指标 | 修复前 | 修复后 |
|---|---|---|
| catch-up 次数 | ~每 5.8s 一次 | 启动 1 次后为 0 |
| 参考瞬移 | 每圈多次 6.1 m 级 | 仅回绕点 1.92 m(数据固有) |
| 走路段转身 180° | 悬挂/不转 | GT +178°(目标 +182°) |
| 末段 365° 旋转 | measured 0° | GT +365°(目标 +355°) |
| 下蹲参考 z_min | 0.740(钳位) | 0.550(钳位=0.55) |
| 关节误差均值 | 0.108 rad(吊着) | 0.114 rad(落地) |

**推荐参数**:`MANAGER_EXTRA_ARGS="--bvh-g1-min-root-height 0.55
--bvh-g1-lower-scale 0.75"`。lower 提到 0.9 走路距离 0.57→0.83 m,但整体跟踪
变差(0.114→0.179 rad)且深蹲+旋转段更易瘫,不推荐。

**遗留问题**:

- 走路距离仍只有参考的 ~20-30%(0.57-0.83 m / 2.92 m),方向与转身正确。平移在
  该链路是开环的(deploy 无里程计,`base_trans_measured` 是固定常量),距离误差
  无法闭环修正;进一步提升需要 planner 模式承担移动、或参考步态合成/里程计。
- 深蹲(参考 z 0.51)+ 快速旋转的组合段超出策略能力,会瘫倒(自动复位后恢复);
  单纯浅蹲(z≈0.84)正常。
- deploy 回放稳态滞后流头 ~3.2s(启动瞬态遗留,merger 窗口顶格 160 帧),
  影响实时性但不影响跟踪正确性,待后续优化。

**量化验证工具**:`SONIC_SIM_BASE_POSE_PORT`(默认 5658)输出仿真底座真值
UDP JSON;g1_debug 的 `base_quat_*` 为 wxyz 序;对齐 JSON 循环用参考 z 曲线
互相关。

## planner 跟随模式:走路距离问题的解法(2026-07-03,分支 feature/json-stream-planner-follow)

pose 模式的平移是开环的(deploy 无里程计),走路距离只能到参考的 20-30%。
参照 PICO 的驱动方式(速度指令走 planner 闭环、姿态只管上肢),把 BVH 分支的
`--follow-trajectory` 因果化移植到了 JSON 流:

- `gear_sonic/utils/teleop/root_trajectory_follower.py`:在线把流式 root 轨迹
  差分成 planner 的 `mode/movement/facing/speed`——EMA 平滑速度、首移方向对齐
  +X、限速(1.5 rad/s)稳定朝向、sender 循环回绕防护;
- 朝向源两档:`travel`(跟行进方向,稳,默认)/`root_yaw`(跟参考 root yaw,
  连续解卷绕,能复现原地整圈旋转);
- manager 加 `--planner-follow-stream-root`(planner 模式下替代 stdin move/face),
  一键脚本 `CONTROL_MODE=planner` 直接启用。

启动:

```bash
REPLACE=1 CONTROL_MODE=planner \
MANAGER_EXTRA_ARGS="--planner-follow-facing-source root_yaw" \
scripts/launch_sonic_json_mujoco_closed_loop.sh --backend mujoco /home/nolo/saveBoneData_Yup20260702.json
```

实测(saveBoneData_Yup20260702,底座真值):

| 指标 | pose 模式 | planner 跟随模式 |
|---|---|---|
| 走路段净位移(参考 2.92 m) | 0.57–0.83 m | **2.88 m(99%)** |
| 走路段转身(参考 ~180°) | +178° | +177°(指令 +181°) |
| 原地 365° 旋转 | +365°(但位置漂 3-4 m) | **+347°,原地(漂移 0.26 m)** |
| 摔倒 | 深蹲+旋转段偶发 | 0 |

**两个后续修复(2026-07-03 同日,已并入本分支)**:

1. **横向行走**:mocopi root 四元数 +X 与人的真实朝向恒差 ~90°(骨盆轴约定)。
   facing 用 root yaw、movement 用行进方向 → 两者恒差固定角,机器人横着走
   (实测行进-朝向差中位 +87°)。修复:用**双肩连线法向**推真实朝向(约定无关),
   follower 的 facing 与 movement 同减首帧朝向保持同系。修复后中位 +3°,
   走路段 +11°/+19°。
2. **planner 模式手臂**:bvh_stream 帧现在携带 head/l_hand/r_hand 关节
   (跑步机局部化:减 root 水平位移 + 按肩线朝向反旋),VR3PointRetargeter
   直接工作,vr_3pt 目标进 planner 消息(同 PICO 的 PLANNER_VR_3PT 通道)。
   **注意必须局部化**——VR3pt 标定只减常量偏移,若保留行走平移,手臂目标会
   随人走远漂几米,把机器人拽倒(实测 181 次摔倒的事故就是这个)。

**精度取舍**:planner 模式手臂走 3 点(头+双手)IK,方向正确、形态近似,
不如 pose 模式的全身逐关节跟踪精细;下蹲在 planner 模式仍不生效。
**要手臂/下蹲最准用 pose 模式,要真走路用 planner 模式**。后续方向:
`upper_body_position`(PLANNER_FROZEN_UPPER_BODY 字段)接 pose 级上肢参考、
下蹲试 planner `height` 指令或参考 root z 低时切 `IDLE_SQUAT`。

## 2026-07-02 验证记录

验证文件：

```text
/home/nolo/saveBoneData_Yup20260702.json
```

静态检查：

- `name` 长度 27；
- `position` / `rotation` 长度 197235；
- 共 7305 帧；
- root / head / toes 高度主要落在 `y` 轴，因此接收侧使用 `left_handed_yup`。

隔离端口验证：

```text
bvh_stream UDP: 12362
manager ZMQ:   5656
deploy debug:  5657
```

日志指纹：

- sender 解析并按 50 Hz 发出 7305 帧 raw JSON；
- manager `recv_fps` 接近 50，`pose=sent`，`dropped=0`；
- deploy 看到 `Protocol version: 1`、`Merged streamed data`、`LowState age` 为毫秒级；
- 未见 traceback、safety check failed、lost LowState。

日志目录：

```text
logs/sony_json_yup_20260702_isolated/
```
