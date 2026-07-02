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
  -> ZMQ pose protocol v1/g1 或显式 opt-in 的 v3/smpl
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

发送链路直接读取 `saveBoneData*.json`，不需要先转换为逐帧 JSONL。

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

`frame_index` 必须逐帧递增（外部发送端同样适用）。当前接收器会在字段缺失时用
接收序号兜底，但如果外部发送端显式发送恒定 `frame_index`，POSE 发布器仍会按重复帧
丢弃，导致机器人定格在第一帧，见下文"故障排查"。

## 单独启动 sender

```bash
cd /home/nolo/GR00T-WholeBodyControl-v3-old-effect-20260702
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

默认稳定主线仍可走 BVH-G1 POSE v1。当前 v3 验证脚本会在同一个 receiver 上显式
切到 `--pose-protocol-version 3 --pose-encoder-mode smpl --allow-sony-pose-v3`。
BoneData raw frame 的坐标/四元数转换参数放在 receiver 侧：

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

## v3 MuJoCo / IsaacLab 验证

```bash
cd /home/nolo/GR00T-WholeBodyControl-v3-old-effect-20260702

# MuJoCo + v3 manager + deploy + raw JSON sender + metrics
scripts/launch_sonic_v3_mujoco_closed_loop.py \
  --input-source sony_json \
  --json-file /home/nolo/saveBoneData_Yup20260702.json

# IsaacLab v3 tuning stack + raw JSON sender
scripts/launch_sonic_v3_tuning_closed_loop.py \
  --input-source sony_json \
  --json-file /home/nolo/saveBoneData_Yup20260702.json \
  --replace --no-attach

# 只打印 v3 MuJoCo 命令，不启动 tmux
scripts/launch_sonic_v3_mujoco_closed_loop.py \
  --input-source sony_json \
  --json-file /home/nolo/saveBoneData_Yup20260702.json \
  --dry-run
```

v3 MuJoCo 默认端口：

| 项 | 默认值 |
|---|---:|
| raw JSON UDP / `bvh_stream` | `12413` |
| manager ZMQ | `6156` |
| deploy debug ZMQ | `6157` |
| MuJoCo metrics ZMQ | `6158` |
| receiver coordinate frame | `left_handed_yup` |

v3 IsaacLab tuning 默认端口：

| 项 | 默认值 |
|---|---:|
| raw JSON UDP / `bvh_stream` | `12403` |
| manager ZMQ | `6056` |
| deploy debug ZMQ | `6057` |
| IsaacLab state ZMQ | `6060` |

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

1. 早期 raw payload 缺 `frame_index` 时，接收侧转换函数会把它默认成 **0**。
2. `bvh_stream_source.py` 的 `_payload_to_frame` 原本有兜底
   `payload.get("frame_index", receive_sequence)`，但转换先执行、payload 已被塞进
   `frame_index=0`，兜底不会触发。
3. `PoseStreamPublisher.publish()`（`zmq_pose_sender.py`）按 `frame_index` 去重：
   与上一帧相同直接丢弃。首帧走 bootstrap 通道把机器人摆到正确姿势，之后每帧都被
   判为重复帧丢掉，POSE 缓冲永远 `0/window`，机器人定格。

坐标转换、retarget 链路全部正常——"姿势是对的"正说明只有去重环节卡死。

**处理结果**（2026-07-02）：仓库 sender 每帧携带递增 `frame_index`；接收侧也在
字段缺失时用 `receive_sequence` 兜底。外部 Unity/Sony 发送端仍应按协议每帧发送递增
`frame_index`，不要显式发送常量值。由此固化为**协议要求**：

- 外部 raw `sony_bonedata_json_v1` 发送端必须每帧携带递增 `frame_index`
  （建议同时带 `source_time_ns`，便于丢包/乱序诊断）；
- 接收侧对缺失的 `frame_index` 仍默认 0，不做 `receive_sequence` 回填——发送端
  缺失或只带常量 `frame_index=0` 都会复现本症状，属发送端 bug；
- `joint_vel` 差分依赖 `source_frame_index` 递增，发送端带递增帧号后该差分同时
  恢复正常。

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
