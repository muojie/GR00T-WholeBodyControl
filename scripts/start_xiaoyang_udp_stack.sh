#!/usr/bin/env bash
# scripts/start_xiaoyang_udp_stack.sh
#
# xiaoyang-udp 分支四端遥操栈一键启动（tmux 四 pane）：
#   pane 0  sim      : MuJoCo run_sim_loop.py                （.venv_sim）
#   pane 1  deploy   : deploy.sh --input-type zmq_manager --zmq-host localhost sim
#                      （自动应答 "Proceed with deployment? [Y/n]"）
#   pane 2  manager  : pico_manager_thread_server.py --manager --port 5556（.venv_teleop）
#   pane 3  isaaclab : IsaacLab teleop_se3_agent.py --xr --teleop_device motion_controllers
#                      （XR runtime 默认 SteamVR，可切 CloudXR，见 ISAACLAB_XR_RUNTIME）
#
# 注意: run_sim_loop 用 .venv_sim（install_scripts/install_mujoco_sim.sh 创建），
#       .venv_teleop 没装 sim 依赖（tyro/mujoco/onnxruntime 等），跑不了 sim。
#
# 用法:
#   scripts/start_xiaoyang_udp_stack.sh              # 启动并 attach 进 tmux
#   scripts/start_xiaoyang_udp_stack.sh --no-attach  # 启动后留在后台
#   scripts/start_xiaoyang_udp_stack.sh --dry-run    # 只打印各 pane 命令
#   scripts/start_xiaoyang_udp_stack.sh --kill       # 杀掉整个 session
#
# 环境变量（有默认值，可覆盖）:
#   GROOT_REPO_ROOT  默认取本脚本所在检出
#   ISAACLAB_ROOT    默认 ~/xiaoyang_IssacLab/IsaacLab
#   ISAAC_TASK       默认 Isaac-PickPlace-Locomanipulation-G1-Abs-v0
#   CONDA_ENV        默认 env_isaaclab（IsaacLab 的 python 环境）
#   ISAACLAB_XR_RUNTIME  默认 steamvr（Isaac Sim→SteamVR→NOLO driver→PICO 无线串流）;
#                    设 cloudxr 切回 CloudXR runtime（WebXR/WebRTC 通路）
#   STEAMVR_XR_JSON  SteamVR 的 OpenXR manifest，默认
#                    ~/.steam/steam/steamapps/common/SteamVR/steamxr_linux64.json
#
# tmux 常用键: Ctrl+b d 退出不杀进程 / Ctrl+b 方向键 切 pane / 鼠标点选已开启

set -uo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO_ROOT="${GROOT_REPO_ROOT:-$(cd "$SCRIPT_DIR/.." && pwd)}"
ISAACLAB_ROOT="${ISAACLAB_ROOT:-$HOME/xiaoyang_IssacLab/IsaacLab}"
ISAAC_TASK="${ISAAC_TASK:-Isaac-PickPlace-Locomanipulation-G1-Abs-v0}"
CONDA_ENV="${CONDA_ENV:-env_isaaclab}"
CONDA_BASE="${CONDA_BASE:-$(conda info --base 2>/dev/null || echo "$HOME/miniconda3")}"
CONDA_SH="$CONDA_BASE/etc/profile.d/conda.sh"
# XR runtime 二选一（移植自 IsaacLab 提交 1bef53791/c8b6a295f，默认 SteamVR）
ISAACLAB_XR_RUNTIME="${ISAACLAB_XR_RUNTIME:-steamvr}"
STEAMVR_XR_JSON="${STEAMVR_XR_JSON:-$HOME/.steam/steam/steamapps/common/SteamVR/steamxr_linux64.json}"
SESSION=xiaoyang_udp

# ---- IsaacLab pane 内部入口（由 tmux pane 调用，勿手动使用）----
# isaaclab.sh 不能裸跑：需要先 conda activate + CUDA 库冲突防护，
# 与 sony-pico-smpl-route:scripts/launch_sony_isaaclab_closed_loop.py 保持一致。
if [[ "${1:-}" == "--isaaclab-pane" ]]; then
  cd "$ISAACLAB_ROOT" || exit 1
  source "$CONDA_SH" || exit 1
  conda activate "$CONDA_ENV" || exit 1
  # CUDA 库冲突防护（勿删）：~/.bashrc 把系统 CUDA 12.5 塞进 LD_LIBRARY_PATH，
  # 其 libnvJitLink.so.12 缺 __nvJitLinkCreate_12_8，torch(cu128) 的 libcusparse
  # 一加载就崩 Kit（undefined symbol）。仅本次启动生效、不动 bashrc：
  # ① 剔除 cuda-12.5 ② 预加载 conda env 自带的 12.8。
  export LD_LIBRARY_PATH="$(printf '%s' "${LD_LIBRARY_PATH:-}" | tr ':' '\n' | grep -v 'cuda-12\.5' | paste -sd: || true)"
  NVJITLINK="$(ls "${CONDA_PREFIX:-$CONDA_BASE/envs/$CONDA_ENV}"/lib/python*/site-packages/nvidia/nvjitlink/lib/libnvJitLink.so.12 2>/dev/null | head -1)"
  [[ -n "$NVJITLINK" ]] && export LD_PRELOAD="$NVJITLINK${LD_PRELOAD:+:$LD_PRELOAD}"
  export PYTHONUNBUFFERED=1
  # ---- OpenXR runtime（--xr 必需，勿删；移植自 IsaacLab 提交 1bef53791）----
  # Kit 只在进程启动时读一次 XR_RUNTIME_JSON，且优先级高于
  # ~/.config/openxr/1/active_runtime.json；不设或设错会报
  # "Cannot start OpenXR! No valid active runtime is set"，XR 视口黑屏无响应。
  case "$ISAACLAB_XR_RUNTIME" in
    steamvr)
      # Isaac Sim → SteamVR(OpenXR runtime) → NOLO driver → PICO 无线串流
      # KB: NVIDIA/CloudXR-OpenXR/NOLO-XRLink-SteamVR-driver部署实战.md
      [[ -f "$STEAMVR_XR_JSON" ]] \
        || { echo "✗ 缺 SteamVR OpenXR manifest: $STEAMVR_XR_JSON（SteamVR 装了吗？）" >&2; exit 1; }
      export XR_RUNTIME_JSON="$STEAMVR_XR_JSON"
      # SteamVR 必须已在运行：vrclient.so 要连 vrserver，没起来 OpenXR 会话建不起来
      if ! pgrep -x vrserver >/dev/null 2>&1; then
        echo "⚠ SteamVR(vrserver) 未运行，--xr 会黑屏。请在另一终端启动：" >&2
        echo "    env -u http_proxy -u https_proxy -u all_proxy DISPLAY=:0 setsid /usr/games/steam steam://run/250820" >&2
        echo "    （必须走 steam:// 让它跑在 sniper 容器里，裸跑 vrstartup 会崩）" >&2
        echo "  等待 vrserver 出现（最多 180s，Ctrl+C 放弃）..." >&2
        for _ in $(seq 1 90); do
          pgrep -x vrserver >/dev/null 2>&1 && break
          sleep 2
        done
        pgrep -x vrserver >/dev/null 2>&1 \
          || { echo "✗ 等待超时；启动 SteamVR 后在本 pane 按 ↑ + Enter 重跑" >&2; exit 1; }
      fi
      echo "XR 通路: steamvr（vrserver pid $(pgrep -x vrserver | head -1)）"
      echo "XR_RUNTIME_JSON=$XR_RUNTIME_JSON"
      ;;
    cloudxr)
      # 原 CloudXR 通路：Isaac Sim → CloudXR runtime → WebXR/WebRTC → 头显
      CLOUDXR_ENV="$HOME/.cloudxr/run/cloudxr.env"
      if [[ -f "$CLOUDXR_ENV" ]]; then
        # shellcheck disable=SC1090
        source "$CLOUDXR_ENV"
      else
        echo "⚠ 未找到 $CLOUDXR_ENV，CloudXR runtime 可能没启动，--xr 会黑屏" >&2
      fi
      [[ -S "$HOME/.cloudxr/run/ipc_cloudxr" ]] \
        || echo "⚠ CloudXR 的 ipc socket 不存在，请先启动 runtime（isaacteleop.cloudxr.runtime）" >&2
      echo "XR 通路: cloudxr; XR_RUNTIME_JSON=${XR_RUNTIME_JSON:-<未设置>}"
      ;;
    *)
      echo "✗ ISAACLAB_XR_RUNTIME 只能是 steamvr 或 cloudxr，当前: $ISAACLAB_XR_RUNTIME" >&2
      exit 1 ;;
  esac
  exec ./isaaclab.sh -p scripts/environments/teleoperation/teleop_se3_agent.py \
    --xr --device cuda:0 --task "$ISAAC_TASK" \
    --teleop_device motion_controllers --enable_pinocchio
fi

DRY_RUN=0
ATTACH=1
while [[ $# -gt 0 ]]; do
  case "$1" in
    --dry-run)   DRY_RUN=1 ;;
    --no-attach) ATTACH=0 ;;
    --kill)      tmux kill-session -t "$SESSION" 2>/dev/null \
                   && echo "已杀掉 session $SESSION" || echo "session $SESSION 不存在"
                 exit 0 ;;
    -h|--help)   awk 'NR>1 && !/^#/{exit} NR>1{sub(/^# ?/,""); print}' "$0"; exit 0 ;;
    *)           echo "未知参数: $1（-h 看用法）" >&2; exit 2 ;;
  esac
  shift
done

VENV_ACTIVATE="$REPO_ROOT/.venv_teleop/bin/activate"
VENV_SIM_ACTIVATE="$REPO_ROOT/.venv_sim/bin/activate"

SIM_CMD="cd $REPO_ROOT && source $VENV_SIM_ACTIVATE && python gear_sonic/scripts/run_sim_loop.py"
DEPLOY_CMD="cd $REPO_ROOT/gear_sonic_deploy && bash deploy.sh --input-type zmq_manager --zmq-host localhost sim"
# XROBO_TRANSPORT: sdk=官方 XRoboToolkit PC Service(头显 App 即可用);
#                   udp=xiaoyang 低延迟 UDP 直发协议(需头显装定制 Unity 发送端,发往本机 63901/udp)
XROBO_TRANSPORT="${XROBO_TRANSPORT:-sdk}"
MANAGER_CMD="cd $REPO_ROOT && source $VENV_ACTIVATE && XROBO_TRANSPORT=$XROBO_TRANSPORT python gear_sonic/scripts/pico_manager_thread_server.py --manager --port 5556"
# tmux pane 的 shell 不继承本脚本环境，覆盖值随命令显式透传给内部入口
ISAAC_CMD="ISAACLAB_ROOT=$(printf %q "$ISAACLAB_ROOT") ISAAC_TASK=$(printf %q "$ISAAC_TASK")"
ISAAC_CMD+=" CONDA_ENV=$(printf %q "$CONDA_ENV") CONDA_BASE=$(printf %q "$CONDA_BASE")"
ISAAC_CMD+=" ISAACLAB_XR_RUNTIME=$(printf %q "$ISAACLAB_XR_RUNTIME")"
ISAAC_CMD+=" STEAMVR_XR_JSON=$(printf %q "$STEAMVR_XR_JSON")"
ISAAC_CMD+=" bash $(printf %q "$SCRIPT_DIR/$(basename "${BASH_SOURCE[0]}")") --isaaclab-pane"

if [[ "$DRY_RUN" == 1 ]]; then
  echo "pane 0 sim      : $SIM_CMD"
  echo "pane 1 deploy   : $DEPLOY_CMD"
  echo "pane 2 manager  : $MANAGER_CMD"
  echo "pane 3 isaaclab : $ISAAC_CMD"
  echo "  （--isaaclab-pane 内部做: cd $ISAACLAB_ROOT && conda activate $CONDA_ENV"
  echo "    && CUDA 12.5/nvJitLink 冲突防护 && XR runtime 准备 && ./isaaclab.sh -p .../teleop_se3_agent.py"
  echo "    --xr --device cuda:0 --task $ISAAC_TASK --teleop_device motion_controllers --enable_pinocchio）"
  echo "  XR 通路: $ISAACLAB_XR_RUNTIME（切换: ISAACLAB_XR_RUNTIME=steamvr|cloudxr）"
  exit 0
fi

# ---- 前置检查 ----
errs=0
command -v tmux >/dev/null || { echo "❌ 缺 tmux（sudo apt install tmux）" >&2; errs=$((errs+1)); }
[[ -f "$VENV_ACTIVATE" ]] || { echo "❌ 缺 venv: $VENV_ACTIVATE（bash install_scripts/install_pico.sh）" >&2; errs=$((errs+1)); }
[[ -f "$VENV_SIM_ACTIVATE" ]] || { echo "❌ 缺 venv: $VENV_SIM_ACTIVATE（bash install_scripts/install_mujoco_sim.sh）" >&2; errs=$((errs+1)); }
[[ -d "$REPO_ROOT/gear_sonic/data/robot_model/instantiation" ]] \
  && find "$REPO_ROOT/gear_sonic/data/robot_model" -name "*.py" | grep -q . \
  || { echo "❌ 缺 gear_sonic/data/robot_model/ 源码（xiaoyang 基线漏同步，可从 main 恢复:" >&2
       echo "     git restore --source=main -- gear_sonic/data/robot_model/）" >&2; errs=$((errs+1)); }
[[ -f "$REPO_ROOT/gear_sonic/scripts/run_sim_loop.py" ]] \
  || { echo "❌ 缺 $REPO_ROOT/gear_sonic/scripts/run_sim_loop.py" >&2; errs=$((errs+1)); }
[[ -f "$REPO_ROOT/gear_sonic_deploy/deploy.sh" ]] \
  || { echo "❌ 缺 $REPO_ROOT/gear_sonic_deploy/deploy.sh" >&2; errs=$((errs+1)); }
[[ -f "$REPO_ROOT/gear_sonic/scripts/pico_manager_thread_server.py" ]] \
  || { echo "❌ 缺 $REPO_ROOT/gear_sonic/scripts/pico_manager_thread_server.py" >&2; errs=$((errs+1)); }
[[ -x "$ISAACLAB_ROOT/isaaclab.sh" ]] \
  || { echo "❌ 缺 $ISAACLAB_ROOT/isaaclab.sh（用 ISAACLAB_ROOT 覆盖路径）" >&2; errs=$((errs+1)); }
[[ -f "$CONDA_SH" ]] \
  || { echo "❌ 缺 conda.sh: $CONDA_SH（用 CONDA_BASE 覆盖）" >&2; errs=$((errs+1)); }
[[ -d "$CONDA_BASE/envs/$CONDA_ENV" ]] \
  || { echo "❌ 缺 conda 环境 $CONDA_ENV（$CONDA_BASE/envs/ 下未找到，用 CONDA_ENV 覆盖）" >&2; errs=$((errs+1)); }
# XR runtime 预检（详细校验在 --isaaclab-pane 内部再做一次）
case "$ISAACLAB_XR_RUNTIME" in
  steamvr)
    [[ -f "$STEAMVR_XR_JSON" ]] \
      || { echo "❌ 缺 SteamVR OpenXR manifest: $STEAMVR_XR_JSON" >&2
           echo "     （SteamVR 没装？或 ISAACLAB_XR_RUNTIME=cloudxr 切 CloudXR 通路）" >&2; errs=$((errs+1)); }
    pgrep -x vrserver >/dev/null 2>&1 \
      || echo "⚠ SteamVR(vrserver) 未运行——isaaclab pane 会等它最多 180s，启动命令见该 pane 提示" >&2
    ;;
  cloudxr)
    [[ -S "$HOME/.cloudxr/run/ipc_cloudxr" ]] \
      || echo "⚠ CloudXR runtime 的 ipc socket 不存在，isaaclab pane 的 --xr 可能黑屏" >&2
    ;;
  *)
    echo "❌ ISAACLAB_XR_RUNTIME 只能是 steamvr 或 cloudxr，当前: $ISAACLAB_XR_RUNTIME" >&2
    errs=$((errs+1)) ;;
esac
[[ "$errs" == 0 ]] || exit 1

# ---- 启动 tmux 四 pane（2x2）----
tmux kill-session -t "$SESSION" 2>/dev/null
tmux new-session -d -s "$SESSION" -x 220 -y 50
tmux set-option -t "$SESSION" -g mouse on
tmux rename-window -t "$SESSION:0" stack
tmux split-window -t "$SESSION:0" -h          # 0|1
tmux split-window -t "$SESSION:0.0" -v        # 0 上 / 2 下
tmux split-window -t "$SESSION:0.2" -v        # 1 上 / 3 下
# 布局: 0=sim(左上) 2=manager(左下) 1=deploy(右上) 3=isaaclab(右下)

tmux send-keys -t "$SESSION:0.0" "$SIM_CMD" C-m
tmux send-keys -t "$SESSION:0.2" "$MANAGER_CMD" C-m
tmux send-keys -t "$SESSION:0.1" "$DEPLOY_CMD" C-m
tmux send-keys -t "$SESSION:0.3" "$ISAAC_CMD" C-m

echo "== 已启动 tmux session: $SESSION =="
echo "   pane 0 sim / pane 2 manager / pane 1 deploy / pane 3 isaaclab"

# ---- 自动应答 deploy 的 Proceed 确认（最多等 120s）----
confirmed=0
for _ in $(seq 1 60); do
  if tmux capture-pane -t "$SESSION:0.1" -p 2>/dev/null \
       | grep -q 'Proceed with deployment'; then
    tmux send-keys -t "$SESSION:0.1" 'y' C-m
    confirmed=1
    echo "   （已自动确认 deploy 的 Proceed 提示）"
    break
  fi
  # session 没了（比如手动杀掉）就别干等
  tmux has-session -t "$SESSION" 2>/dev/null || break
  sleep 2
done
[[ "$confirmed" == 1 ]] \
  || echo "   ⚠ 120s 内没等到 deploy 的 Proceed 提示，请 attach 后自查 pane 1"

echo "   attach: tmux attach -t $SESSION（Ctrl+b d 退出不杀进程）"
echo "   停止:   $0 --kill"

[[ "$ATTACH" == 1 ]] && exec tmux attach -t "$SESSION"
exit 0
