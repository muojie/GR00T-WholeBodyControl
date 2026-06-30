# Sony BoneData JSON stream sender

本文说明当前 Sony mocopi BoneData JSON 是怎么送进 SONIC 的。重点是：JSON
文件不是直接给 deploy 或 MuJoCo/IsaacLab 消费，而是先被转换成现有
`bvh_stream_v1` UDP skeleton frame。

## 数据链路

```text
Sony BoneData JSON
  -> gear_sonic/scripts/sony_bonedata_json_stream_sender.py
  -> UDP bvh_stream_v1, 默认 127.0.0.1:12352
  -> mocap_manager_server.py --source bvh_stream
  -> skeleton retarget / BVH-G1 route
  -> ZMQ pose protocol v1, encoder_mode=g1, 默认 5556
  -> g1_deploy_onnx_ref
  -> MuJoCo / IsaacLab closed loop
```

`mocap_manager_server.py` 里仍然使用 `--source bvh_stream`。这里的
`bvh_stream` 指的是实时 skeleton wire protocol，不表示输入一定是 BVH 文件。
BoneData JSON sender 复用这条协议，因此 manager 和 deploy 下游不需要新增一套
JSON 专用输入类型。

## 输入 JSON 结构

当前 sender 支持的 BoneData JSON 是一个顶层 dict，包含三组等长数组：

| 字段 | 类型 | 说明 |
|---|---|---|
| `name` | string array | 每个 bone / joint 的名字 |
| `position` | object array | 每项为 `{x, y, z}` |
| `rotation` | object array | 每项为 `{x, y, z, w}`，即 `xyzw` 四元数 |

数据按帧展平成一维数组。当前 Sony mocopi 样例每帧 27 个节点，因此：

```text
len(name) == len(position) == len(rotation)
frame_count = len(name) / 27
frame[i] = entries[i * 27 : (i + 1) * 27]
```

同一个 sender 会话中，每帧的 27 个 `name` 顺序必须一致。sender 会把第一帧的
名字作为 `joint_names`，后续帧如果名字集合或顺序不一致，应视为输入数据错误。

当前样例的 27 个节点为：

```text
root, torso_1, torso_2, torso_3, torso_4, torso_5, torso_6, torso_7,
neck_1, neck_2, head,
l_shoulder, l_up_arm, l_low_arm, l_hand,
r_shoulder, r_up_arm, r_low_arm, r_hand,
l_up_leg, l_low_leg, l_foot, l_toes,
r_up_leg, r_low_leg, r_foot, r_toes
```

## sender 做的转换

`sony_bonedata_json_stream_sender.py` 每次取一帧 27 个节点，生成一个
`bvh_stream_v1` payload，并通过 UDP 发送。

转换规则：

| 输入 | 输出 | 规则 |
|---|---|---|
| `name[J]` | `joint_names[J]` | 保持同一帧内顺序 |
| `position[J].x/y/z` | `world_positions[J,3]` | 转成 float，乘 `--position-scale` |
| `rotation[J].x/y/z/w` | `world_quat_wxyz[J,4]` | 从 `xyzw` 改排为 `wxyz` |
| JSON 帧号 | `source_frame_index` | 使用原始 JSON 帧号 |
| sender 输出帧号 | `frame_index` | 从 0 开始单调递增，循环时继续递增 |

默认假设 BoneData JSON 已经是 SONIC/MuJoCo 主线需要的 Z-up、米制世界坐标，
所以 `--position-scale` 默认是 `1.0`。如果上游 app 输出厘米制，再显式使用
`--position-scale 0.01`。

默认保留全局 root translation。只有指定 `--local-root` 时，sender 才会用当前帧
root 位置减掉所有节点的平移，输出 root-local positions。

## UDP payload

sender 输出的 payload 与 `bvh_stream_sender.py` 保持同一协议名：

```json
{
  "format": "bvh_stream_v1",
  "schema_version": 1,
  "path": "/home/nolo/下载/saveBoneData0629.json",
  "motion_name": "saveBoneData0629",
  "joint_names": ["root", "torso_1", "torso_2"],
  "frame_index": 0,
  "source_frame_index": 0,
  "source_fps": 50.0,
  "fps": 50.0,
  "frame_stride": 1,
  "source_time_ns": 1782791000000000000,
  "world_positions": [[0.0, 0.0, 1.86]],
  "world_quat_wxyz": [[1.0, 0.0, 0.0, 0.0]],
  "packet_format": "msgpack"
}
```

真实 payload 的 `joint_names`、`world_positions`、`world_quat_wxyz` 长度应一致。
默认包格式是 msgpack；`--format json` 只用于抓包或人工调试。

## 单独启动 JSON sender

在仓库根目录运行：

```bash
.venv_teleop/bin/python -u gear_sonic/scripts/sony_bonedata_json_stream_sender.py \
  --json-file /home/nolo/下载/saveBoneData0629.json \
  --host 127.0.0.1 \
  --port 12352 \
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
| `--joints-per-frame` | 自动推断 | 当前 Sony 样例是 27 |
| `--position-scale` | `1.0` | position 乘法缩放 |
| `--input-quat-order` | `xyzw` | 当前 JSON rotation 字段顺序 |
| `--local-root` | false | 是否去掉每帧 root translation |
| `--max-frames` | 不限制 | 调试时只发送前 N 帧 |

## manager / deploy 配套命令

JSON sender 只负责发 UDP skeleton frame。manager 仍用 BVH-G1 POSE v1 主线：

```bash
.venv_teleop/bin/python -u gear_sonic/scripts/mocap_manager_server.py \
  --source bvh_stream \
  --bvh-stream-port 12352 \
  --control-mode pose \
  --pose-window-size 80 \
  --pose-encoder-mode g1 \
  --pose-protocol-version 1 \
  --zmq-port 5556 \
  --log-interval-s 1.0
```

MuJoCo deploy 侧仍订阅 manager 的 ZMQ：

```bash
cd gear_sonic_deploy
stdbuf -oL -eL bash deploy.sh \
  --input-type zmq_manager \
  --zmq-host localhost \
  --zmq-port 5556 \
  sim
```

## 日志判断

sender 正常时会打印类似：

```text
[SonyBoneDataJsonStreamSender] frames=624 joints=27 loop=True position_scale=1 local_root=False
[SonyBoneDataJsonStreamSender] sent=250 source_frame=249 bytes=2270
```

manager 正常时应看到：

```text
[MocapManager] recv_fps=50.0 ... pose=sent:N encoder=g1 ... dropped=0 ... source_frame=...
```

deploy 正常时应看到：

```text
[ZMQEndpointInterface] Protocol version: 1
[ZMQEndpointInterface] Requested encoder_mode: 0
[ZMQEndpointInterface] Merged streamed data: ...
```

如果 manager 一直没有 `pose=sent`，优先检查 JSON sender 是否发到同一个端口
`12352`，以及 JSON 中每帧 `name/position/rotation` 数量是否一致。

## 给实时 app 的建议

如果上游是实时 Sony mocopi app，不建议先持续写一个大 JSON 文件再让 Python
读取。更稳的方式是直接在 app 内复刻本页的 sender 逻辑：

```text
每个采样时刻:
  1. 生成稳定顺序的 joint_names
  2. 生成 Z-up 米制 world_positions[J,3]
  3. 生成归一化 world_quat_wxyz[J,4]
  4. 组 bvh_stream_v1 payload
  5. msgpack 后 UDP sendto(manager_host, 12352)
```

这样下游 manager、deploy、MuJoCo 和 IsaacLab 都不需要知道上游到底来自 BVH、
保存的 JSON，还是实时 Sony mocopi app。
