#!/usr/bin/env bash
# scripts/verify_pico_isaaclab_closed_loop.sh
#
# PICO 闭环一键启动 + 自动核验（对比验证基线脚本），支持 IsaacLab / MuJoCo / none 三后端
#
# 固化 sonic-physics-review-fixes 谱系已验证的 PICO 通路，启动后自动轮询日志
# 核验健康判据，并把基线快照+结果存档，以后改代码/换分支后重跑本脚本即可对比。
#   isaaclab 后端（默认）: 四组件 IsaacLab / C++ proxy / deploy(zmq_manager) / PICO streamer
#   mujoco   后端       : 三组件 MuJoCo run_sim_loop / deploy(zmq_manager) / PICO streamer
#   none     后端       : 三组件 C++ proxy / deploy(zmq_manager) / PICO streamer，
#                         IsaacLab 由你手动启动（须 export SONIC_PUBLISH_STATE_ZMQ=1）
# 头显手动连 XRoboToolkit。
#
# 用法:
#   scripts/verify_pico_isaaclab_closed_loop.sh                    # isaaclab 后端一键启动+核验
#   scripts/verify_pico_isaaclab_closed_loop.sh --backend mujoco   # mujoco sim2sim 后端
#   scripts/verify_pico_isaaclab_closed_loop.sh --backend none     # proxy+deploy+PICO，Isaac 手动起
#   scripts/verify_pico_isaaclab_closed_loop.sh --check-only       # 不启动，只核验当前最新日志
#   scripts/verify_pico_isaaclab_closed_loop.sh --dry-run          # 只打印各 pane 命令
#   scripts/verify_pico_isaaclab_closed_loop.sh --timeout 600      # 核验轮询超时秒数（默认 360）
#   其余参数原样透传给外置 launcher
#     （如 --task Isaac-SonicFullscene-Locomanipulation-G1-v0）
#
# 依赖: 仓内 tmux launcher scripts/launch_sony_isaaclab_closed_loop.py
#   （与本脚本同目录；可用 SONY_SONIC_LAUNCHER 环境变量覆盖位置）。
#   GR00T 仓库根默认取本脚本所在检出（worktree 里跑即验证该 worktree），
#   可用 GROOT_REPO_ROOT 覆盖；IsaacLab 用 ISAACLAB_ROOT 覆盖。
#
# isaaclab 判据（知识库 SONIC闭环三端启动手册与PICO接入阶段 §2.4）:
#   1. proxy 日志 src=isaac（不是 synthetic）
#   2. IsaacLab 日志出现 root pose stabilized（STABILIZE_ROOT 生效）
#   3. IsaacLab env_hz >= 45（solo/fullscene 实时目标 50）
#   4. IsaacLab real_root tilt < 10°（解锁前锁根站立姿态健康）
#   5. 无 NaN 毒目标（first target parsed ... nan = 残留旧 deploy）
#   6. deploy 日志已产生输出
# mujoco 判据:
#   1. run_sim_loop 日志已产生输出
#   2. mujoco 日志无 Traceback
#   3. deploy 日志已产生输出
# none 判据（IsaacLab 手动启动，其日志位置未知，只核验脚本管的两端）:
#   1. proxy 日志 src=isaac（= 你手动起的 IsaacLab 已通过 5560 接上）
#   2. deploy 日志已产生输出
#
# 自动核验通过后，剩余步骤是人工的:
#   PICO 头显 A+B+X+Y（校准+CONTROL）→ deploy packets 增长
#   （isaaclab/none 后端还要在 IsaacLab 窗口按 U 解锁）

set -uo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
GROOT_ROOT="${GROOT_REPO_ROOT:-$(cd "$SCRIPT_DIR/.." && pwd)}"
LAUNCHER="${SONY_SONIC_LAUNCHER:-$SCRIPT_DIR/launch_sony_isaaclab_closed_loop.py}"
HISTORY_DIR="${SONIC_VERIFY_LOG_DIR:-$HOME/.sonic_verify_logs}"

ISAACLAB_ROOT_DIR="${ISAACLAB_ROOT:-$HOME/xiaoyang_IssacLab/IsaacLab}"
PROXY_BIN="$HOME/bin/sonic_unitree_lowstate_cpp_proxy"
DEPLOY_BIN="$GROOT_ROOT/gear_sonic_deploy/target/release/g1_deploy_onnx_ref"

BACKEND=isaaclab
TIMEOUT=360
CHECK_ONLY=0
DRY_RUN=0
EXTRA_ARGS=()

while [[ $# -gt 0 ]]; do
  case "$1" in
    --backend)    BACKEND="${2:?--backend 需要 isaaclab|mujoco}"; shift ;;
    --check-only) CHECK_ONLY=1 ;;
    --dry-run)    DRY_RUN=1 ;;
    --timeout)    TIMEOUT="${2:?--timeout 需要秒数}"; shift ;;
    -h|--help)    awk 'NR>1 && !/^#/{exit} NR>1{sub(/^# ?/,""); print}' "$0"; exit 0 ;;
    *)            EXTRA_ARGS+=("$1") ;;
  esac
  shift
done

case "$BACKEND" in
  isaaclab) CHECK_KEYS=(proxy_src stabilized env_hz tilt nan deploy) ;;
  mujoco)   CHECK_KEYS=(mujoco mujoco_tb deploy) ;;
  none)     CHECK_KEYS=(proxy_src deploy) ;;
  *) echo "❌ 未知 backend: $BACKEND（支持 isaaclab|mujoco|none）" >&2; exit 2 ;;
esac

if [[ ! -x "$LAUNCHER" ]]; then
  echo "❌ 找不到外置 launcher: $LAUNCHER" >&2
  echo "   （设 SONY_SONIC_LAUNCHER 指向 launch_sony_isaaclab_closed_loop.py）" >&2
  exit 2
fi

# launcher 与本脚本对同一检出取证
LAUNCHER_ARGS=(--backend "$BACKEND" --input-source pico --repo-root "$GROOT_ROOT")

# ---------- 基线快照（对比验证的记录锚点） ----------

snapshot() {
  echo "== 基线快照 $(date '+%Y-%m-%d %H:%M:%S')  backend=$BACKEND =="
  local repos=("$GROOT_ROOT")
  # none 后端 Isaac 虽是手动起，但一般也是这份检出，快照仍有对比价值
  [[ "$BACKEND" != mujoco ]] && repos+=("$ISAACLAB_ROOT_DIR")
  local d
  for d in "${repos[@]}"; do
    if git -C "$d" rev-parse --git-dir >/dev/null 2>&1; then
      echo "  $d"
      echo "    branch: $(git -C "$d" branch --show-current 2>/dev/null || echo '?')"
      echo "    commit: $(git -C "$d" log --oneline -1 2>/dev/null || echo '?')"
      local dirty
      dirty=$(git -C "$d" status --porcelain 2>/dev/null | wc -l)
      echo "    dirty:  ${dirty} 个未提交变更"
    else
      echo "  $d (非 git 仓库或不存在)"
    fi
  done
  local bins=("$DEPLOY_BIN")
  [[ "$BACKEND" != mujoco ]] && bins+=("$PROXY_BIN")
  local b
  for b in "${bins[@]}"; do
    if [[ -x "$b" ]]; then
      echo "  $b  ($(stat -c '%y' "$b" | cut -d. -f1))"
    else
      echo "  ⚠️ 缺二进制: $b"
    fi
  done
}

# ---------- 日志定位 ----------

MARKER="$(mktemp /tmp/sonic_verify_marker.XXXXXX)"

# 找一份日志: $1=前缀(isaac_sonic|sonic_proxy|sonic_deploy|mujoco_sonic)
# 启动模式下只认 marker 之后新建的文件（防接到旧场日志——手册明令每场全新起）；
# --check-only 模式取 mtime 最新的一份。
find_log() {
  local prefix="$1"
  if [[ "$CHECK_ONLY" == 1 ]]; then
    ls -t /tmp/${prefix}_*.log 2>/dev/null | head -1
  else
    find /tmp -maxdepth 1 -name "${prefix}_*.log" -newer "$MARKER" 2>/dev/null \
      | xargs -r ls -t 2>/dev/null | head -1
  fi
}

# ---------- 核验 ----------

declare -A RESULT DETAIL

check_deploy_log() {
  local deploy_log="$1"
  if [[ -n "$deploy_log" && -s "$deploy_log" ]]; then
    RESULT[deploy]=PASS; DETAIL[deploy]="$deploy_log"
  else
    RESULT[deploy]=WAIT; DETAIL[deploy]="${deploy_log:-日志未出现}"
  fi
}

check_proxy_src() {
  local proxy_log="$1"
  if [[ -n "$proxy_log" ]] && grep -q 'src=isaac' "$proxy_log"; then
    RESULT[proxy_src]=PASS; DETAIL[proxy_src]="$proxy_log"
  else
    RESULT[proxy_src]=WAIT; DETAIL[proxy_src]="${proxy_log:-日志未出现}"
    [[ -n "$proxy_log" ]] && grep -q 'src=synthetic' "$proxy_log" \
      && DETAIL[proxy_src]="$proxy_log 仍是 synthetic（5560 没通，查 SONIC_PUBLISH_STATE_ZMQ）"
  fi
}

run_checks_isaaclab() {
  local isaac_log proxy_log deploy_log
  isaac_log="$(find_log isaac_sonic)"
  proxy_log="$(find_log sonic_proxy)"
  deploy_log="$(find_log sonic_deploy)"

  # 1. proxy src=isaac
  check_proxy_src "$proxy_log"

  # 2. root pose stabilized
  if [[ -n "$isaac_log" ]] && grep -q 'root pose stabilized' "$isaac_log"; then
    RESULT[stabilized]=PASS
    DETAIL[stabilized]="$(grep -m1 'root pose stabilized' "$isaac_log" | tail -c 80)"
  else
    RESULT[stabilized]=WAIT
    DETAIL[stabilized]="缺该行=SONIC_DEPLOY_STABILIZE_ROOT 没带上（启动即倒+U 失效）"
  fi

  # 3. env_hz >= 45
  local hz
  hz=$([[ -n "$isaac_log" ]] && grep -oE 'env_hz[=: ]+[0-9]+\.?[0-9]*' "$isaac_log" \
        | tail -1 | grep -oE '[0-9]+\.?[0-9]*$' || true)
  if [[ -n "${hz:-}" ]]; then
    if awk -v h="$hz" 'BEGIN{exit !(h>=45)}'; then
      RESULT[env_hz]=PASS; DETAIL[env_hz]="env_hz=$hz"
    else
      RESULT[env_hz]=FAIL
      DETAIL[env_hz]="env_hz=$hz（30=vsync 没关; ~18=场景超载; 目标 50）"
    fi
  else
    RESULT[env_hz]=WAIT; DETAIL[env_hz]="尚无 env_hz 调试行"
  fi

  # 4. real_root tilt < 10°
  local tilt
  tilt=$([[ -n "$isaac_log" ]] && grep -oE 'tilt=[0-9]+\.?[0-9]*' "$isaac_log" \
          | tail -1 | grep -oE '[0-9]+\.?[0-9]*' || true)
  if [[ -n "${tilt:-}" ]]; then
    if awk -v t="$tilt" 'BEGIN{exit !(t<10)}'; then
      RESULT[tilt]=PASS; DETAIL[tilt]="tilt=${tilt}°"
    else
      RESULT[tilt]=FAIL; DETAIL[tilt]="tilt=${tilt}°（解锁前应为个位数，机器人可能已倒）"
    fi
  else
    RESULT[tilt]=WAIT; DETAIL[tilt]="尚无 real_root 行（9a3b503e8 仪表应在岗）"
  fi

  # 5. NaN 毒目标（阴性判据: 出现即 FAIL）
  if [[ -n "$isaac_log" ]]; then
    if grep -iq 'first target parsed.*nan' "$isaac_log"; then
      RESULT[nan]=FAIL
      DETAIL[nan]="检测到 NaN 毒目标=残留旧 deploy 在发包，三端全杀后重启"
    else
      RESULT[nan]=PASS; DETAIL[nan]="无毒目标"
    fi
  else
    RESULT[nan]=WAIT; DETAIL[nan]="IsaacLab 日志未出现"
  fi

  # 6. deploy 已产出日志
  check_deploy_log "$deploy_log"
}

run_checks_mujoco() {
  local mujoco_log deploy_log
  mujoco_log="$(find_log mujoco_sonic)"
  deploy_log="$(find_log sonic_deploy)"

  # 1. run_sim_loop 已产出日志
  if [[ -n "$mujoco_log" && -s "$mujoco_log" ]]; then
    RESULT[mujoco]=PASS; DETAIL[mujoco]="$mujoco_log"
  else
    RESULT[mujoco]=WAIT; DETAIL[mujoco]="${mujoco_log:-日志未出现}"
  fi

  # 2. mujoco 无 Traceback（阴性判据: 出现即 FAIL）
  if [[ -n "$mujoco_log" ]]; then
    if grep -q 'Traceback' "$mujoco_log"; then
      RESULT[mujoco_tb]=FAIL
      DETAIL[mujoco_tb]="$(grep -A1 -m1 'Traceback' "$mujoco_log" | tail -1 | tail -c 80)"
    else
      RESULT[mujoco_tb]=PASS; DETAIL[mujoco_tb]="无 Traceback"
    fi
  else
    RESULT[mujoco_tb]=WAIT; DETAIL[mujoco_tb]="mujoco 日志未出现"
  fi

  # 3. deploy 已产出日志
  check_deploy_log "$deploy_log"
}

run_checks_none() {
  local proxy_log deploy_log
  proxy_log="$(find_log sonic_proxy)"
  deploy_log="$(find_log sonic_deploy)"

  # 1. proxy src=isaac（= 手动起的 IsaacLab 已接上 5560）
  check_proxy_src "$proxy_log"
  [[ "${RESULT[proxy_src]}" == WAIT ]] \
    && DETAIL[proxy_src]="${DETAIL[proxy_src]}（none 后端: 等你手动启动 IsaacLab，须 SONIC_PUBLISH_STATE_ZMQ=1）"

  # 2. deploy 已产出日志
  check_deploy_log "$deploy_log"
}

run_checks() {
  case "$BACKEND" in
    isaaclab) run_checks_isaaclab ;;
    mujoco)   run_checks_mujoco ;;
    none)     run_checks_none ;;
  esac
}

all_settled() {
  local k
  for k in "${CHECK_KEYS[@]}"; do
    [[ "${RESULT[$k]}" == WAIT ]] && return 1
  done
  return 0
}

print_report() {
  local label_proxy_src="proxy src=isaac"
  local label_stabilized="root pose stabilized"
  local label_env_hz="env_hz>=45"
  local label_tilt="real_root tilt<10°"
  local label_nan="无 NaN 毒目标"
  local label_deploy="deploy 已启动"
  local label_mujoco="run_sim_loop 已启动"
  local label_mujoco_tb="mujoco 无 Traceback"
  local fails=0 k mark var
  echo
  echo "== 核验结果 (backend=$BACKEND) =="
  for k in "${CHECK_KEYS[@]}"; do
    case "${RESULT[$k]}" in
      PASS) mark="✅ PASS" ;;
      FAIL) mark="❌ FAIL"; fails=$((fails+1)) ;;
      *)    mark="⏳ 超时未出现"; fails=$((fails+1)) ;;
    esac
    var="label_$k"
    printf '  %-10s %-22s %s\n' "$mark" "${!var}" "${DETAIL[$k]}"
  done
  return "$fails"
}

print_manual_steps() {
  echo
  echo "== 后续人工步骤（自动核验只覆盖启动段） =="
  if [[ "$BACKEND" == none ]]; then
    echo "  0. 手动启动 IsaacLab（若核验时还没起）: export SONIC_PUBLISH_STATE_ZMQ=1 及"
    echo "     SONIC_DEPLOY_STABILIZE_ROOT=1 等环境变量后跑 teleop_se3_agent.py"
  fi
  echo "  1. PICO 头显连 XRoboToolkit（同网段）"
  echo "  2. 头显按 A+B+X+Y = 三点校准 + CONTROL（⚠️ 人站在追踪原点正中，防前冲）"
  echo "  3. deploy 窗确认 packets 开始增长"
  if [[ "$BACKEND" == mujoco ]]; then
    echo "  4. MuJoCo viewer 观察跟随（无 U 解锁步骤，plant 从启动即自由物理）"
  else
    echo "  4. IsaacLab 窗口按 U 解锁 → 闭环自由站立，摇杆/姿态驱动"
    echo "  摔倒恢复: 三端全新起（残留 deploy 会发 NaN 毒目标）"
  fi
}

# ---------- 主流程 ----------

if [[ "$DRY_RUN" == 1 ]]; then
  exec "$LAUNCHER" "${LAUNCHER_ARGS[@]}" --dry-run "${EXTRA_ARGS[@]}"
fi

SNAPSHOT_TXT="$(snapshot)"
echo "$SNAPSHOT_TXT"
echo

if [[ "$CHECK_ONLY" == 0 ]]; then
  echo "== 一键启动（tmux session: sony_sonic; backend=$BACKEND） =="
  "$LAUNCHER" "${LAUNCHER_ARGS[@]}" --replace --no-attach "${EXTRA_ARGS[@]}" \
    || { echo "❌ launcher 启动失败（preflight 未过?）"; rm -f "$MARKER"; exit 2; }
  echo "  已启动。attach: tmux attach -t sony_sonic"
  echo
fi

if [[ "$BACKEND" == none && "$CHECK_ONLY" == 0 ]]; then
  echo "⚠️  none 后端: 请现在手动启动 IsaacLab（export SONIC_PUBLISH_STATE_ZMQ=1），"
  echo "    核验将在 ${TIMEOUT}s 内等待 proxy 出现 src=isaac。"
  echo
fi

echo "== 自动核验（轮询 ${TIMEOUT}s） =="
deadline=$(( $(date +%s) + TIMEOUT ))
while :; do
  run_checks
  all_settled && break
  [[ $(date +%s) -ge $deadline ]] && break
  sleep 5
done
rm -f "$MARKER"

print_report
FAILS=$?

print_manual_steps

# 存档本次验证结果，供以后 diff 对比
mkdir -p "$HISTORY_DIR"
REPORT_FILE="$HISTORY_DIR/verify_$(date +%Y%m%d_%H%M%S).txt"
{
  echo "$SNAPSHOT_TXT"
  print_report 2>/dev/null || true
} > "$REPORT_FILE"
echo
echo "结果已存档: $REPORT_FILE（历次结果 diff 即可对比）"

exit "$FAILS"
