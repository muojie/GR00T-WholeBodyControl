# xiaoyang-udp 四端遥操栈一键启动

`scripts/start_xiaoyang_udp_stack.sh` 一条命令拉起 xiaoyang-udp 分支的完整
PICO → GR00T → IsaacLab 遥操链路(tmux 四 pane),并自动处理确认提示与环境防护。

## 数据流

```
Pico 4U 头显 ──XRoboToolkit App──> RoboticsService(PC Service)
                                        │  XROBO_TRANSPORT=sdk
                                        v
                              pico_manager (:5556 ZMQ)      [pane 2, .venv_teleop]
                                        │ pose/command
                                        v
                              deploy.sh zmq_manager sim      [pane 1, gear_sonic_deploy]
                                   │            ^
                     UDP :5557 g1_debug      DDS rt/lowcmd·lowstate
                     UDP :5558 g1_root          │
                           │                    v
                           │            run_sim_loop MuJoCo   [pane 0, .venv_sim]
                           v
                  IsaacLab teleop_se3_agent(--xr)            [pane 3, conda env_isaaclab]
```

## 用法

```bash
scripts/start_xiaoyang_udp_stack.sh              # 启动并 attach 进 tmux
scripts/start_xiaoyang_udp_stack.sh --no-attach  # 启动后留在后台
scripts/start_xiaoyang_udp_stack.sh --dry-run    # 只打印各 pane 命令
scripts/start_xiaoyang_udp_stack.sh --kill       # 杀掉整个 session
```

tmux 常用键:`Ctrl+b d` 退出不杀进程;`Ctrl+b 方向键` 切 pane;鼠标点选已开启。
deploy 的 `Proceed with deployment? [Y/n]` 提示由脚本自动应答(最多等 120s)。

## 环境变量(均有默认值)

| 变量 | 默认 | 说明 |
| --- | --- | --- |
| `GROOT_REPO_ROOT` | 脚本所在检出 | GR00T-WholeBodyControl 仓库根 |
| `ISAACLAB_ROOT` | `~/xiaoyang_IssacLab/IsaacLab` | IsaacLab 检出(注意目录名拼写) |
| `ISAAC_TASK` | `Isaac-PickPlace-Locomanipulation-G1-Abs-v0` | IsaacLab 任务名 |
| `CONDA_ENV` / `CONDA_BASE` | `env_isaaclab` / 自动探测 | IsaacLab 的 python 环境 |
| `XROBO_TRANSPORT` | `sdk` | 见下节 |

## XROBO_TRANSPORT:sdk 与 udp 两条头显数据链

- **`sdk`(默认)**:走官方 XRoboToolkit PC Service。头显装官方
  `com.xrobotoolkit.client` App 并与本机 RoboticsService 连通即可,开箱能用。
- **`udp`**:xiaoyang 低延迟直发协议
  (`decoupled_wbc/control/teleop/device/pico/xr_client.py`,监听 63901/udp,
  0x3F 帧头)。**需要头显安装配套的定制 Unity 发送端 App** 并把目标 IP 配成
  本机;没有它 manager 会一直 `waiting for body data...`,ZMQ 5556 不 bind,
  按键也全部无响应(按键处理在 body data 等待之后)。

```bash
XROBO_TRANSPORT=udp scripts/start_xiaoyang_udp_stack.sh   # 测 UDP 协议时
```

## 前置条件

1. `.venv_sim` —— `bash install_scripts/install_mujoco_sim.sh`(run_sim_loop 用;
   `.venv_teleop` 未装 sim 依赖,跑不了 sim)
2. `.venv_teleop` —— `bash install_scripts/install_pico.sh`(pico_manager 用)
3. IsaacLab 检出 + conda `env_isaaclab`
4. 头显与本机同网段,XRoboToolkit App 已连上本机 PC Service
   (可用 `adb devices` / `ss -tnp | grep -i robotics` 核验)

脚本启动前会自检以上路径,缺什么直接报错并给出安装/恢复命令。

## 网络配置(单机闭环已本地化)

两侧 env 的 robot-1 IP 已改为 127.0.0.1(原双机拓扑值留在各文件注释里):

- GR00T 侧:`config/g1_udp_network.env`(deploy.sh 与 MuJoCo bridge 加载)
- IsaacLab 侧:`<ISAACLAB_ROOT>/scripts/gr00t_wbc/g1_udp_network.env`
  (`GR00T_WBC_ROOT` 同时已从 Windows 的 `F:/...` 改为本机检出路径)

回双机部署时把这两个文件里的 IP 改回注释中的原值即可。

## 已知坑(xiaoyang-udp 分支特有)

1. **`.gitignore` 的 `data/` 规则误伤 `gear_sonic/data/`**:基线提交 14f8bf1
   重建时整个 `gear_sonic/data/`(robot_model、assets、robots、human 等 700+
   文件,含 `g1_43dof.usd`)因该规则被排除,分支上没有这些文件,但代码 import
   和 IsaacLab USD finder 都需要。已在工作树按"只补缺失不覆盖已有"从 main 恢复;
   若再丢(切分支/`git clean`),恢复命令:

   ```bash
   git ls-tree -r main --name-only gear_sonic/data/ | while IFS= read -r f; do
     [ -e "$f" ] || git restore --source=main -- "$f"
   done
   ```

   `g1_43dof.usd` 的材质补丁版(本地 mdl)在 `stash@{0}`(2026-07-16),
   如需单独恢复:`git show 'stash@{0}:gear_sonic/data/robots/g1/g1_43dof.usd' | git lfs smudge > <目标>`。
2. **isaaclab.sh 不能裸跑**:需 `conda activate env_isaaclab` + CUDA 12.5/nvJitLink
   冲突防护(bashrc 的 cuda-12.5 缺 `__nvJitLinkCreate_12_8` 符号会崩 Kit)。
   脚本的 `--isaaclab-pane` 内部入口已包好,勿绕过脚本手动裸跑。
3. **IsaacLab 报"找不到 43dof"**:USD finder 在
   `locomanipulation_g1_env_cfg.py:_find_gr00t_g1_43dof_usd`,依赖
   `GR00T_WBC_ROOT` 指向本机检出且 `g1_43dof.usd` 存在(见坑 1 与网络配置节)。

## 故障排查速查

| 症状 | 原因 → 处置 |
| --- | --- |
| manager 一直 `waiting for body data...` | UDP 模式无源 → 用默认 sdk;或头显 App 未连 PC Service |
| 按键 A/B/X/Y 无响应 | 同上(按键处理在 body data 之后);日志找 `Poll alive: mode=... A=..` 确认轮询已活 |
| sim pane `ModuleNotFoundError: tyro` | 用了 .venv_teleop → 脚本已固定 .venv_sim |
| `No module named gear_sonic.data...` / 找不到 `human_joints_info.pkl` | 坑 1,按恢复命令补 |
| IsaacLab `undefined symbol __nvJitLinkCreate_12_8` | 坑 2,走脚本启动 |
| IsaacLab `Could not locate GR00T G1 43-DoF USD` | 坑 3 |
| deploy 卡确认提示 | 脚本 120s 内自动喂 y;超时后 attach 进 pane 1 手动回车 |
