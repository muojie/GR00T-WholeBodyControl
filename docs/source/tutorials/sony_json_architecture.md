# Sony JSON 一键脚本技术架构

## 系统架构图

```
┌─────────────────────────────────────────────────────────────────────────┐
│                          Sony JSON 一键启动脚本                           │
│                   launch_sonic_json_mujoco_closed_loop.sh                │
│                   launch_sonic_json_isaaclab_closed_loop.sh              │
└─────────────────────────────────────────────────────────────────────────┘
                                    │
                    ┌───────────────┴───────────────┐
                    │  环境变量: POSE_PROTOCOL_VERSION │
                    │  • v1 (默认) → bvh_stream + g1  │
                    │  • v3 → sony_pico + smpl        │
                    └───────────────┬───────────────┘
                                    │
        ┌───────────────────────────┼───────────────────────────┐
        │                           │                           │
        ▼                           ▼                           ▼
┌──────────────┐          ┌──────────────────┐        ┌──────────────┐
│ json_sender  │          │ mocap_manager    │        │   deploy     │
│              │          │                  │        │              │
│ 发送 UDP      │─────────▶│ 接收 + 转换      │───────▶│ ONNX 推理    │
│ 帧数据        │  12362   │ 姿态编码         │  5656  │ + 规划器     │
└──────────────┘          └──────────────────┘        └──────┬───────┘
        │                           │                           │
        │                           │                           │
        ▼                           ▼                           ▼
  JSON 文件                  ZMQ pose topic           ZMQ g1_debug (5657)
  • saveBoneData                                              │
  • saveBoneAllData                                           │
                                                              ▼
                                                    ┌──────────────────┐
                                                    │  MuJoCo/IsaacLab │
                                                    │  仿真环境         │
                                                    └──────────────────┘
```

## 数据流详解

### 1. JSON Sender（发送端）

**输入**：
- `saveBoneData.json` - 扁平数组格式
- `saveBoneAllData.json` - 嵌套 boneInfos 格式（待支持）

**处理**：
```python
# sony_bonedata_json.py::load_sony_bonedata_json_raw()
1. 加载 JSON 文件
2. 验证格式（name/position/rotation 长度一致）
3. 切分成逐帧 payload
4. 通过 UDP 发送到 manager (端口 12362)
```

**输出 UDP Payload**：
```python
{
    "format": "sony_bonedata_json_v1",
    "schema_version": 1,
    "frame_index": 0,
    "joints_per_frame": 27,
    "fps": 50.0,
    "name": ["root", "torso_1", ...],       # 27 个关节名
    "position": [{x,y,z}, ...],             # 27 个位置
    "rotation": [{x,y,z,w}, ...]            # 27 个旋转（Unity 原生四元数）
}
```

### 2. Mocap Manager（中间层）

**接收端口**：12362 (UDP)

**协议分支**：

#### POSE v1 路线（bvh_stream）

```
Sony UDP Payload
    │
    ├─▶ convert_sony_bonedata_payload_to_bvh_stream_payload()
    │   ├─ coordinate_frame: left_handed_yup → sonic_zup
    │   ├─ position_scale: 1.0
    │   ├─ input_quat_order: xyzw → wxyz
    │   └─ rotation_mode: input
    │
    ├─▶ BVH skeleton (27 bone → BVH tree)
    │
    ├─▶ G1 Retarget (BVH → G1 骨架)
    │   └─ 14 个 G1 FK 关键点
    │
    └─▶ SMPL Projection (可选，v3 with bvh_stream)
        ├─ g1_fk_to_smpl_joints (14 → 24 SMPL 槽位)
        └─ body_n=14, smpl_lspan≈0.66m
```

**日志特征**：
```
encoder=g1, body_n=14
```

#### POSE v3 路线（sony_pico）

```
Sony UDP Payload
    │
    ├─▶ zflip 基变换: diag(1,1,-1)
    │   └─ Unity 左手系 → XRT SDK 右手约定
    │
    ├─▶ 27→24 直接搬全局旋转
    │   ├─ mocopi bind frame 世界对齐
    │   ├─ SMPL FK 只消费旋转，用标准骨长重建关节
    │   └─ 骨架比例自动归一
    │
    ├─▶ SMPL 22/23 hand 槽位复用 wrist bone
    │   └─ 局部旋转被 [:63] 截断，无影响
    │
    └─▶ PICO 腕关节投影
        ├─ smpl_pose_to_g1_wrist_joint_pos()
        ├─ 消除 Sony/PICO 腕语义差异（p95 2.25 rad）
        └─ body_n=1, smpl_lspan≈0.92m
```

**日志特征**：
```
encoder=smpl, body_n=1, smpl_lspan=0.92m
```

**输出 ZMQ Message**（端口 5656）：
```python
{
    "topic": "pose",
    "protocol_version": 1 or 3,
    "encode_mode": 0 (g1) or 2 (smpl),
    "joint_pos": [30个关节位置],       # G1 格式
    "smpl_joints": [24个SMPL关节],     # v3 专有
    "smpl_pose": [23×3 局部旋转],      # v3 专有
    "timestamp_ns": ...
}
```

### 3. Deploy（推理端）

**输入端口**：5656 (ZMQ)

**处理流程**：
```
ZMQ pose message
    │
    ├─▶ 协议版本检测
    │   ├─ v1: 使用 joint_pos (30 个关节)
    │   └─ v3: 使用 smpl_joints + smpl_pose
    │
    ├─▶ ONNX Encoder
    │   └─ 历史窗口特征提取
    │
    ├─▶ ONNX Policy
    │   └─ 动作生成
    │
    ├─▶ Planner (planner_sonic.onnx)
    │   └─ 目标速度规划
    │
    └─▶ 输出关节指令
```

**输出**：
- `g1_debug` ZMQ topic (端口 5657)
- 发送给 MuJoCo/IsaacLab 的关节指令

**日志特征**：
```
[ZMQEndpointInterface] active_protocol_version_=3
[ZMQEndpointInterface] result.motion->GetEncodeMode()=2
Frame[17] (idx=1545) joint_pos: [-0.312000, -0.312000], 
smpl_joints: [(-0.009844, -0.351365, 0.009359)], 
smpl_pose: [(-0.040017, -0.063219, 0.073330)]
```

### 4. Simulation Backend

**MuJoCo 模式**：
```
run_sim_loop.py
    ├─ 监听 g1_debug ZMQ topic
    ├─ 接收关节指令
    ├─ MuJoCo 仿真
    └─ 可视化
```

**IsaacLab 模式**：
```
IsaacLab Python 环境
    ├─ 监听 sonic_state ZMQ (端口 5560)
    ├─ 接收状态数据
    ├─ GPU 并行仿真
    └─ 渲染输出
```

## 协议版本对比表

| 维度 | POSE v1 (bvh_stream + g1) | POSE v3 (sony_pico + smpl) |
|------|--------------------------|---------------------------|
| **Source** | `bvh_stream` | `sony_pico` |
| **Encoder Mode** | `g1` | `smpl` |
| **转换路径** | Sony → BVH → G1 Retarget | Sony → PICO 直通 |
| **中间层** | BVH skeleton (27 bone tree) | 无中间层 |
| **关键点来源** | 14 个 G1 FK 关键点 | PICO 转换栈（zflip + 27→24） |
| **腕关节语义** | G1 原生 IK | PICO SMPL 腕投影 |
| **腕语义差异** | 与 PICO 存在差异（p95 2.25 rad） | 与 PICO 头显同构（差异消除） |
| **坐标系转换** | 需要 coordinate-frame 参数 | PICO 栈内部处理 |
| **body_n 指标** | 14 | 1 |
| **smpl_lspan** | ≈0.66m | ≈0.92m |
| **适用场景** | 通用 BVH 动作 | Sony mocopi / PICO VR 专用 |

## 端口分配

| 组件 | 默认端口 (MuJoCo) | 默认端口 (IsaacLab) | 协议 | 用途 |
|------|------------------|-------------------|------|------|
| json_sender → manager | 12362 | 12352 | UDP | Sony BoneData 原始帧 |
| manager → deploy | 5656 | 5556 | ZMQ | 编码后的姿态数据 |
| deploy → debug | 5657 | 5557 | ZMQ | 调试输出 |
| IsaacLab → manager | - | 5560 | ZMQ | 机器人状态反馈 |

## 关键代码位置

```
scripts/
├── launch_sonic_json_mujoco_closed_loop.sh      # MuJoCo 后端启动脚本
├── launch_sonic_json_isaaclab_closed_loop.sh    # IsaacLab 后端启动脚本
└── launch_sonic_local_isaaclab_closed_loop.py   # IsaacLab Python 启动器

gear_sonic/scripts/
├── sony_bonedata_json_stream_sender.py          # JSON 文件发送器
├── mocap_manager_server.py                      # Manager 主程序
└── run_sim_loop.py                              # MuJoCo 仿真循环

gear_sonic/utils/teleop/sources/
├── sony_bonedata_json.py                        # JSON 格式解析 + 转换
├── bvh_stream_source.py                         # BVH-G1 路线 (v1)
└── sony_pico_smpl_source.py                     # Sony-PICO 路线 (v3)

gear_sonic_deploy/
├── src/g1_deploy_onnx_ref.rs                    # Deploy Rust 主程序
└── planner/target_vel/V2/planner_sonic.onnx     # 规划器模型
```

## 启动命令

### MuJoCo 后端

```bash
# v1 协议（默认）
./scripts/launch_sonic_json_mujoco_closed_loop.sh ~/saveBoneData.json

# v3 协议
POSE_PROTOCOL_VERSION=3 ./scripts/launch_sonic_json_mujoco_closed_loop.sh ~/saveBoneData.json
```

### IsaacLab 后端

```bash
# 本地 IsaacLab + v3 协议
POSE_PROTOCOL_VERSION=3 ./scripts/launch_sonic_json_isaaclab_closed_loop.sh ~/saveBoneData.json

# 远程 Windows IsaacLab + v3 协议
POSE_PROTOCOL_VERSION=3 ./scripts/launch_sonic_json_isaaclab_closed_loop.sh \
  --no-isaaclab \
  --windows-ip 192.168.50.100 \
  ~/saveBoneData.json
```

## Tmux 窗口布局

```
sonic_json_yup_mujoco (session)
├── [0] mujoco        # MuJoCo 仿真
├── [1] manager       # Manager (接收 + 编码)
├── [2] deploy        # Deploy (推理 + 规划)
└── [3] json_sender   # JSON 文件发送器
```

**切换窗口**：`Ctrl+B` 然后按 `0/1/2/3`

## 日志位置

```
logs/sony_json_mujoco_sonic_json_yup_mujoco/
├── mujoco.log        # MuJoCo 仿真日志
├── manager.log       # Manager 日志（重点关注）
├── deploy.log        # Deploy 日志（重点关注）
└── json_sender.log   # Sender 日志
```

## 故障排查流程

```
问题：机器人不动/动作不对

1. 检查 json_sender 日志
   ├─ 发送帧率是否正常？(应为 50.0 fps)
   └─ 是否有错误？

2. 检查 manager 日志
   ├─ recv_fps 是否正常？(应为 50.0 fps)
   ├─ encoder 是否正确？(v1=g1, v3=smpl)
   ├─ body_n 是否正确？(v1=14, v3=1)
   ├─ smpl_lspan 是否正确？(v1≈0.66m, v3≈0.92m)
   └─ dropped 是否为 0？

3. 检查 deploy 日志
   ├─ active_protocol_version_ 是否匹配？
   ├─ GetEncodeMode() 是否正确？(v1=0, v3=2)
   ├─ 是否有 smpl_joints/smpl_pose 输出？(v3 专有)
   └─ 是否有 CRC 错误？

4. 检查端口占用
   └─ ss -ltnup | grep -E ":(12362|5656|5657)"
```

## 相关文档

- [POSE v3 协议支持](sony_json_pose_v3_support.md)
- [一键启动脚本使用指南](launch_scripts_usage_guide.md)
- [mocopi POSE SMPL v3 route](mocopi_pose_smpl_v3_route.md)
- [Sony BoneData JSON 格式](sony_bonedata_json_stream_sender.md)
