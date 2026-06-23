# Sony mocopi / `--source bvh_stream` 在线 BVH UDP 流

本文只记录 `mocap_manager_server.py --source bvh_stream`。它和 [`--source bvh`](mocopi_source_bvh_file.md) 是两个不同输入源，不应互相覆盖。

## 定位

`--source bvh_stream` 让 manager 监听 UDP skeleton frame。BVH 文件由另一个 sender 进程读取和发送：

```text
BVH file
  -> bvh_stream_sender.py
  -> UDP bvh_stream_v1
  -> BvhStreamUdpSource
  -> manager 侧 G1 retarget
  -> POSE v1 + encoder_mode=g1
```

特点：

- manager 不绑定具体 BVH 文件。
- MuJoCo、deploy、manager 可以常驻。
- 换 BVH 文件只需要重启 `bvh_stream_sender.py`。
- 当前主要用于 [BVH-G1 POSE v1 路线](mocopi_pose_bvh_g1_v1_route.md)。

## Manager 命令

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

预期日志：

```text
control_mode=pose pose_stream=1 pose_protocol=v1 pose_window=80 pose_encoder=g1
listening for BVH stream UDP on 0.0.0.0:12352
```

## Sender 命令

```bash
.venv_teleop/bin/python -u gear_sonic/scripts/bvh_stream_sender.py \
  --bvh-file /home/nolo/RAYNOS_Motion1.bvh \
  --host 127.0.0.1 \
  --port 12352 \
  --loop
```

预期日志：

```text
[BvhStreamSender] streaming /home/nolo/RAYNOS_Motion1.bvh to udp://127.0.0.1:12352 format=msgpack fps=50.0
```

## 热切换

换 BVH 文件时：

1. 保持 MuJoCo、deploy、manager 不动。
2. 停止当前 `bvh_stream_sender.py`。
3. 用新的 `--bvh-file` 重启 sender。
4. 必要时在 MuJoCo viewer 中按 `Backspace` reset 机器人状态。

新的 sender 从 `frame_index=0` 开始时，deploy 的 streamed-motion merger 会通过 catch-up reset 切到新 motion window。

## 接收指纹

manager 应看到：

```text
recv_fps≈50
dropped=0
pose=sent:N
```

如果 sender 先于 manager 启动，UDP 前几帧可能丢失。实际操作时先等 manager 打印 `listening for BVH stream UDP...`，再启动 sender。

## 和 `bvh` 的区别

| 项目 | `--source bvh_stream` | `--source bvh` |
|------|------------------------|----------------|
| 输入 | UDP skeleton stream | 本地 BVH 文件 |
| 文件读取者 | `bvh_stream_sender.py` | manager 进程 |
| 换文件 | 只重启 sender | 重启 manager |
| 当前主用途 | BVH-G1 POSE v1 在线验证 | 文件回放、VR3PT、SMPL v3 研究 |
| 是否模拟实时动捕 | 是 | 否 |
