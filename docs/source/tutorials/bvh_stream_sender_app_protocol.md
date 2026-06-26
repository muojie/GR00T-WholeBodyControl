# BVH stream sender app 协议

本文给外部 sender app 开发者使用。目标是复刻当前命令的行为：

```bash
.venv_teleop/bin/python -u gear_sonic/scripts/bvh_stream_sender.py \
  --bvh-file /home/nolo/RAYNOS_Motion1.bvh \
  --host 127.0.0.1 \
  --port 12352 \
  --loop
```

## 边界

sender app 只负责把逐帧 skeleton 数据发到 manager 的 UDP 端口。它不直接给 deploy 发 ZMQ，也不直接实现 G1 retarget。

```text
sender app
  -> UDP bvh_stream_v1, 默认 127.0.0.1:12352
  -> mocap_manager_server.py --source bvh_stream
  -> BvhStreamUdpSource
  -> BVH/canonical skeleton 到 G1 29 维 joint reference
  -> ZMQ pose protocol v1, encoder_mode=g1, 默认 5556
  -> deploy StreamedMotionMerger
```

manager / deploy 侧的稳定主线固定为：

```bash
.venv_teleop/bin/python -u gear_sonic/scripts/mocap_manager_server.py \
  --source bvh_stream \
  --bvh-stream-port 12352 \
  --control-mode pose \
  --pose-window-size 80 \
  --pose-encoder-mode g1 \
  --pose-protocol-version 1 \
  --zmq-port 5556
```

`--source bvh_stream` 会强制要求 `--control-mode pose`、`--pose-protocol-version 1`、`--pose-encoder-mode g1`。这条链路发送的是 G1 joint reference，不是 SMPL pose，也不是 planner VR3PT 三点目标。

## UDP 传输

| 项 | 约定 |
|----|------|
| transport | UDP datagram，一帧一个 datagram |
| 默认目的端口 | `12352` |
| manager 绑定地址 | `--bvh-stream-host`，默认 `0.0.0.0` |
| sender 目的地址 | 本机联调用 `127.0.0.1`，跨机器时填 manager 主机 IP |
| 最大接收包 | `--bvh-stream-recv-size`，默认 `262144` bytes |
| 包格式 | `msgpack` 推荐，`json` 仅用于调试 |
| 自动识别 | manager 的 `auto` 模式看到首个非空白字节是 `{` 就按 JSON，否则按 msgpack |

UDP 不保证可靠到达和顺序。sender 需要按固定 FPS 持续发送最新帧；manager 只消费最新收到的 payload。不要依赖重传，也不要把多帧合并到一个 UDP 包。

## 包字段

payload 是一个 map / dict。当前协议名是 `bvh_stream_v1`。

| 字段 | 类型 / shape | 必需 | 用途 |
|------|--------------|------|------|
| `format` | string | 是 | 固定为 `bvh_stream_v1`；旧值 `bvh_stream` 仍被接收 |
| `schema_version` | int | 建议 | 当前填 `1` |
| `joint_names` | string list, length `J` | 是 | skeleton 关节名，顺序必须和数组第 0 维一致 |
| `world_positions` | float array `[J, 3]` | 是 | 每个关节的世界坐标，单位米 |
| `world_quat_wxyz` | float array `[J, 4]` | 是 | 每个关节的世界姿态四元数，顺序是 `w, x, y, z` |
| `frame_index` | int | 是 | sender 输出帧序号；同一动作内单调递增 |
| `source_frame_index` | int | 建议 | 原始数据帧号；用于估计 `joint_vel` |
| `fps` | float | 建议 | sender 实际发送 FPS / playback FPS |
| `source_fps` | float | 建议 | 原始源 FPS；没有重采样时等于 `fps` |
| `frame_stride` | int | 可选 | 从高 FPS 源跳帧时的 stride |
| `source_time_ns` | int | 建议 | sender 采样或发包时间，纳秒 |
| `path` | string | 可选 | 数据来源路径或 app source id，仅用于诊断 |
| `motion_name` | string | 可选 | 动作名，仅用于诊断 |
| `packet_format` | string | 可选 | `msgpack` 或 `json`，仅用于诊断 |

`world_positions` 和 `world_quat_wxyz` 可以是普通嵌套 list，也可以是 msgpack-numpy 编码的 ndarray。manager 会转换成 `float32` 并校验形状。

## JSON 示例

JSON 适合抓包和手工检查。真实 app 推荐改用 msgpack，避免包过大。

```json
{
  "format": "bvh_stream_v1",
  "schema_version": 1,
  "path": "sender_app_live",
  "motion_name": "live_motion",
  "joint_names": [
    "root",
    "chest",
    "left_upper_leg",
    "left_lower_leg",
    "right_upper_leg",
    "right_lower_leg"
  ],
  "frame_index": 0,
  "source_frame_index": 0,
  "source_fps": 50.0,
  "fps": 50.0,
  "frame_stride": 1,
  "source_time_ns": 1792740000000000000,
  "world_positions": [
    [0.0, 0.0, 0.9],
    [0.0, 0.0, 1.2],
    [0.08, 0.05, 0.8],
    [0.08, 0.05, 0.45],
    [-0.08, 0.05, 0.8],
    [-0.08, 0.05, 0.45]
  ],
  "world_quat_wxyz": [
    [1.0, 0.0, 0.0, 0.0],
    [1.0, 0.0, 0.0, 0.0],
    [1.0, 0.0, 0.0, 0.0],
    [1.0, 0.0, 0.0, 0.0],
    [1.0, 0.0, 0.0, 0.0],
    [1.0, 0.0, 0.0, 0.0]
  ],
  "packet_format": "json"
}
```

上面的例子只展示 schema。实际 G1 retarget 至少需要有效下肢和胸部关节；要得到上肢和腕部动作，还应发送肩、肘、手等关节。

## msgpack Python 参考

Python 版可以直接发送标准 list；不依赖 msgpack-numpy 也能被 manager 解析。

```python
import socket
import time

import msgpack

payload = {
    "format": "bvh_stream_v1",
    "schema_version": 1,
    "joint_names": joint_names,
    "frame_index": frame_index,
    "source_frame_index": source_frame_index,
    "source_fps": 50.0,
    "fps": 50.0,
    "frame_stride": 1,
    "source_time_ns": time.time_ns(),
    "world_positions": world_positions_meters,     # list[J][3]
    "world_quat_wxyz": world_quat_wxyz,            # list[J][4]
    "packet_format": "msgpack",
}

packet = msgpack.packb(payload, use_bin_type=True)
sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
sock.sendto(packet, ("127.0.0.1", 12352))
```

如果 app 使用 C++、C#、Unity 或移动端实现，只要发出的 msgpack 是普通 map + array 结构即可。字段名必须完全匹配。

## 关节命名

manager 通过关节名匹配 skeleton 语义，`joint_names` 的长度和顺序必须在同一个 app 会话中保持稳定。首次收到包或关节名集合变化时，manager 会重建 BVH-to-G1 retarget context。

推荐 sender app 直接使用下列语义名，减少 alias 依赖：

| 语义 | 推荐名 | 常见已支持 alias |
|------|--------|------------------|
| root / pelvis | `root` | `Hips`, `Hip`, `Pelvis`, `Root` |
| spine | `spine` | `torso_2`, `Spine`, `Spine1`, `Spine01` |
| chest | `chest` | `torso_6`, `torso_5`, `UpperChest`, `Chest`, `Spine2`, `Spine3` |
| left upper arm | `left_upper_arm` | `l_up_arm`, `LeftArm`, `LeftUpperArm`, `L_UpperArm` |
| left lower arm | `left_lower_arm` | `l_low_arm`, `LeftForeArm`, `LeftLowerArm`, `L_ForeArm` |
| left hand | `left_hand` | `l_hand`, `LeftHand`, `LeftWrist`, `L_Hand` |
| right upper arm | `right_upper_arm` | `r_up_arm`, `RightArm`, `RightUpperArm`, `R_UpperArm` |
| right lower arm | `right_lower_arm` | `r_low_arm`, `RightForeArm`, `RightLowerArm`, `R_ForeArm` |
| right hand | `right_hand` | `r_hand`, `RightHand`, `RightWrist`, `R_Hand` |
| left upper leg | `left_upper_leg` | `l_up_leg`, `LeftUpLeg`, `LeftUpperLeg`, `LeftThigh` |
| left lower leg | `left_lower_leg` | `l_low_leg`, `LeftLeg`, `LeftLowerLeg`, `LeftShin` |
| left foot | `left_foot` | `l_foot`, `LeftFoot`, `LeftAnkle`, `L_Foot` |
| right upper leg | `right_upper_leg` | `r_up_leg`, `RightUpLeg`, `RightUpperLeg`, `RightThigh` |
| right lower leg | `right_lower_leg` | `r_low_leg`, `RightLeg`, `RightLowerLeg`, `RightShin` |
| right foot | `right_foot` | `r_foot`, `RightFoot`, `RightAnkle`, `R_Foot` |

当前 skeleton retarget 的硬性必需项是：

```text
left_upper_leg
left_lower_leg
right_upper_leg
right_lower_leg
chest
```

实际联调建议同时提供 `root`、左右脚、spine/chest、左右上臂、左右前臂、左右手。缺少上肢或脚部时，manager 仍可能能出包，但动作复原度会明显下降。

## 坐标系和单位

manager 接收的是已经标准化后的世界系数据：

- `world_positions` 单位是米。
- `world_quat_wxyz` 是世界姿态四元数，必须归一化。
- SONIC / MuJoCo 主线使用 Z-up。
- 如果上游是常见 BVH Y-up、厘米制数据，先做厘米到米，再做 Y-up 到 Z-up。

当前 Python sender 默认行为：

```text
position_m = bvh_position * 0.01
Y-up -> Z-up: (x, y, z) -> (x, -z, y)
quaternion: xyzw -> wxyz after the same basis conversion
```

如果 app 本身已经输出 Z-up 米制数据，不要重复做 Y-up 转换。

默认 sender 保留 BVH 的全局 root motion。只有显式加 `--local-root` 时才会把 root translation 从所有关节位置中减掉。manager 的 `bvh_stream` 默认还会做首帧 root 对齐，用来降低初始朝向和位置跳变。

## 帧序和切换动作

`frame_index` 表示 sender 输出帧序号，推荐每个 sender 会话从 `0` 开始并逐帧加 `1`。

同一动作内：

- `frame_index` 单调递增。
- `source_frame_index` 可以等于 `frame_index`。
- 如果从高 FPS 源跳帧，`source_frame_index` 按原始帧号增长，`frame_index` 按发送帧增长。

循环播放同一个动作时：

- `frame_index` 继续递增。
- `source_frame_index` 可以回到动作开头。
- 这样 manager 不会把循环误判成新动作；循环边界的 `joint_vel` 会自动避免使用负 frame delta。

热切换新动作时：

- 只重启 sender app。
- 新 app 从 `frame_index=0` 开始。
- MuJoCo、deploy、manager 可以常驻。
- manager / deploy 会通过 frame reset 和 catch-up 逻辑切到新的 streamed motion window。

## 频率

推荐 `50 Hz`。如果机器负载较高，可以统一降到 `25 Hz`，但 sender 的实际发送 FPS、payload 里的 `fps`、manager 的 `--target-fps` 应保持一致或接近。

当前 Python sender 的 `--fps` 逻辑是：如果目标 FPS 低于 BVH 原始 FPS，就用 `frame_stride=round(source_fps / target_fps)` 跳帧，并把 `fps` 写成实际 playback FPS。

## 下游 POSE v1 字段

sender app 不需要实现这一层，但联调时要知道 manager 最终发给 deploy 的字段：

| 字段 | shape | dtype | 说明 |
|------|-------|-------|------|
| `body_pos` | `[window, 14, 3]` | `float32` | G1 body FK 位置参考 |
| `body_quat_w` | `[window, 4]` | `float32` | root/world body quaternion |
| `joint_pos` | `[window, 29]` | `float32` | G1 29 维 joint reference |
| `joint_vel` | `[window, 29]` | `float32` | G1 29 维 joint velocity reference |
| `frame_index` | `[window]` | `int64` | 滑动窗口帧号 |
| `catch_up` | `[1]` | `bool` | deploy catch-up / reset hint |
| `encoder_mode` | `[1]` | `int32` | `g1` 对应 `0` |

ZMQ `pose` 消息的底层格式是：

```text
topic bytes "pose"
1280-byte JSON header
little-endian binary field payload
```

这个 ZMQ 协议由 `PoseStreamPublisher` 和 `pack_pose_message()` 负责。普通 sender app 不应绕过 manager 直接发 ZMQ，除非明确要替换 manager 侧 retarget。

## 联调检查

manager 启动后应看到：

```text
control_mode=pose pose_stream=1 pose_protocol=v1 pose_window=80 pose_encoder=g1
listening for BVH stream UDP on 0.0.0.0:12352
```

sender 启动后应看到类似：

```text
[BvhStreamSender] streaming /home/nolo/RAYNOS_Motion1.bvh to udp://127.0.0.1:12352 format=msgpack fps=50.0
[BvhStreamSender] sent=51 source_frame=51 bytes=...
```

manager 周期日志里重点看：

| 字段 | 正常判断 |
|------|----------|
| `recv_fps` | 接近 sender FPS，例如 50 |
| `dropped` | 应保持 0 |
| `pose=sent` | 应开始增长 |
| `q=[min,max]` | 29 维 G1 关节目标范围合理 |
| `dq_abs` | 不应有明显尖峰 |
| `lower_dq` | 不应长期接近 0，否则下肢没有进入 G1 retarget 主线 |
| `root_z` | root 高度合理 |
| `root_tilt` | 不应在切入时过大 |

常见问题：

| 现象 | 优先检查 |
|------|----------|
| `recv=0` | sender 目的 IP/端口、manager 是否先启动、是否被防火墙拦截 |
| `dropped` 增长 | payload 不是 dict、`format` 不对、数组 shape 不对、JSON/msgpack 格式选错 |
| 报 missing source joints | `joint_names` 缺少必需语义或 alias 不匹配 |
| 有包但不出 pose | manager 是否使用 `--control-mode pose --pose-protocol-version 1 --pose-encoder-mode g1` |
| 动作抖动 | `fps`、实际发送频率、manager `--target-fps` 是否长期不一致 |
| 换动作后还跟旧动作 | 新 sender 是否从 `frame_index=0` 开始 |

