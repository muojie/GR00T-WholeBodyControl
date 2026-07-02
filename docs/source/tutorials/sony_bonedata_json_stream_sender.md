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
