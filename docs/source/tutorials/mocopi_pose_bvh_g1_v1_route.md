# Sony mocopi / BVH-G1 POSE v1 路线

本文只记录当前跑通的工程路线：输入源先在 Python manager 侧重定向成 G1 关节参考，再通过 POSE protocol v1 发给 deploy。

## 定位

这条路线使用 SONIC / GR00T 的 deploy、MuJoCo、ZMQ stream、MotionSequence 和 policy runtime，但不走 SONIC 原生 SMPL encoder 语义。

```text
BVH / canonical skeleton frame
  -> manager 侧 BVH/mocopi-to-G1 retarget
  -> G1 29 维 joint_pos / joint_vel
  -> pose topic, protocol v1, encoder_mode=g1
  -> deploy StreamedMotionMerger
  -> policy 跟随 G1 joint reference
```

BVH 或 mocopi 原始数据不会直接交给 deploy；deploy 收到的是已经由 manager 侧算好的 G1 reference 窗口。

## 在线 BVH stream

当前推荐把输入源和 manager 拆成两个进程。这里使用的是 [`--source bvh_stream`](mocopi_bvh_sources.md)，不是同页记录的 `--source bvh`：

```text
BVH file
  -> bvh_stream_sender.py
  -> UDP bvh_stream_v1, 默认 12352
  -> mocap_manager_server.py --source bvh_stream
  -> BvhStreamUdpSource
  -> G1 retarget
  -> ZMQ pose protocol v1, 默认 5556
```

manager 命令：

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

sender 命令：

```bash
.venv_teleop/bin/python -u gear_sonic/scripts/bvh_stream_sender.py \
  --bvh-file /home/nolo/RAYNOS_Motion1.bvh \
  --host 127.0.0.1 \
  --port 12352 \
  --loop
```

换 BVH 时只重启 sender。MuJoCo、deploy、manager 可以常驻；MuJoCo 中需要清空状态时，在 viewer 里按 `Backspace`。

## POSE v1 字段

当前主要发送：

```text
joint_pos
joint_vel
body_pos
body_quat_w
frame_index
catch_up
encoder_mode
```

关键约束：

- `joint_pos` / `joint_vel` 是 G1 29 维关节参考。
- `encoder_mode=g1`。
- `pose_window_size` 建议 80，避免 future observation 不够。
- 新 sender 从 `frame_index=0` 开始时，deploy 侧通过 catch-up reset 切到新 motion window。

## 单进程回归路径

如果不需要热切换，可以用 `bvh_g1`：

```bash
.venv_teleop/bin/python -u gear_sonic/scripts/mocap_manager_server.py \
  --source bvh_g1 \
  --bvh-file /home/nolo/RAYNOS_Motion1.bvh \
  --bvh-loop \
  --control-mode pose \
  --pose-window-size 80 \
  --pose-encoder-mode g1 \
  --pose-protocol-version 1 \
  --zmq-port 5556
```

这个模式仍是在线逐帧 retarget，但 manager 绑定具体 BVH 文件，换文件需要重启 manager。

## 运行指纹

manager 侧应看到：

```text
control_mode=pose pose_stream=1 pose_protocol=v1 pose_window=80 pose_encoder=g1
listening for BVH stream UDP on 0.0.0.0:12352
pose=sent:1
recv_fps≈50 dropped=0
```

诊断字段重点：

| 字段 | 判断 |
|------|------|
| `q=[min,max]` | 29 维 G1 关节目标范围 |
| `dq_abs` | 关节速度是否有尖峰 |
| `lower_dq` | 下肢 G1 关节相对默认站姿的偏差；当前路线不应长期为 0 |
| `root_z` | root 高度是否合理 |
| `root_tilt` | 切入时 root 倾斜是否过大 |

## 后续工作

这条路线下一步应把 `bvh_stream_v1` 的 skeleton packet 抽象成 mocopi 实时输入也能复用的 canonical skeleton frame，并继续优化 wrist、root heading、脚踝/脚尖和速度限幅。
