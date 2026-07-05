#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR=$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)
REPO_ROOT=$(cd "${SCRIPT_DIR}/.." && pwd)

SESSION=${SESSION:-sonic_json_isaaclab_v3}
JSON_FILE=${JSON_FILE:-/home/nolo/saveBoneData_Yup20260702.json}
INPUT_SOURCE=${INPUT_SOURCE:-sony_json}
NO_ISAAC=${NO_ISAAC:-0}
NO_JSON_SENDER=${NO_JSON_SENDER:-0}
WINDOWS_BRIDGE_IP=${WINDOWS_BRIDGE_IP:-}
REPLACE=${REPLACE:-1}
NO_ATTACH=${NO_ATTACH:-0}
DRY_RUN=${DRY_RUN:-0}

BASE_LAUNCHER=${BASE_LAUNCHER:-${REPO_ROOT}/scripts/launch_sonic_local_isaaclab_closed_loop.py}
V3_LAUNCHER=${V3_LAUNCHER:-${REPO_ROOT}/scripts/launch_sonic_v3_tuning_closed_loop.py}

usage() {
  cat <<EOF
Usage:
  $0 [OPTIONS] [JSON_FILE]

启动 v3 SMPL POSE 版本的 Sony JSON IsaacLab 闭环栈。

Options:
  --no-isaac               不启动本地 IsaacLab，只启动 manager + deploy。
  --no-isaaclab            同 --no-isaac（兼容旧参数）。
  --no-json-sender         不启动 JSON sender（假设外部已在发送）。
  --windows-ip HOST        Windows 桥接机器的 IP（用于跨机 IsaacLab）。
  --windows-bridge-ip HOST 同 --windows-ip。
  --json-file PATH         Sony BoneData JSON 文件路径。
  --input-source SOURCE    输入源: sony_json (默认) 或 bvh。
  --session NAME           tmux 会话名。默认: ${SESSION}
  --replace                如果会话已存在则先杀掉。默认启用。
  --no-replace             如果会话已存在则报错退出。
  --no-attach              启动后不 attach 到 tmux 会话。
  --dry-run                只打印命令不实际启动。
  -h, --help               显示此帮助。

Environment overrides:
  SESSION=${SESSION}
  JSON_FILE=${JSON_FILE}
  INPUT_SOURCE=${INPUT_SOURCE}
  NO_ISAAC=${NO_ISAAC}
  NO_JSON_SENDER=${NO_JSON_SENDER}
  WINDOWS_BRIDGE_IP=${WINDOWS_BRIDGE_IP}
  REPLACE=${REPLACE}
  NO_ATTACH=${NO_ATTACH}
  DRY_RUN=${DRY_RUN}

Examples:
  # 完整本地栈（推荐，使用 v3 tuning launcher）
  $0 /home/nolo/saveBoneData_Yup20260702.json

  # 只启动接收端，对接 Windows 上的 IsaacLab（使用底层 launcher）
  $0 --no-isaac --no-json-sender --windows-ip 192.168.1.137

  # 只启动接收端，JSON sender 在别处跑（使用底层 launcher）
  $0 --no-json-sender

  # 打印命令不执行
  $0 --dry-run
EOF
}

json_arg_seen=0
while [[ $# -gt 0 ]]; do
  case "$1" in
    -h|--help)
      usage
      exit 0
      ;;
    --no-isaac|--no-isaaclab)
      NO_ISAAC=1
      shift
      ;;
    --no-json-sender)
      NO_JSON_SENDER=1
      shift
      ;;
    --windows-ip|--windows-bridge-ip)
      WINDOWS_BRIDGE_IP="$2"
      shift 2
      ;;
    --json-file)
      JSON_FILE="$2"
      json_arg_seen=1
      shift 2
      ;;
    --input-source)
      INPUT_SOURCE="$2"
      shift 2
      ;;
    --session)
      SESSION="$2"
      shift 2
      ;;
    --replace)
      REPLACE=1
      shift
      ;;
    --no-replace)
      REPLACE=0
      shift
      ;;
    --no-attach)
      NO_ATTACH=1
      shift
      ;;
    --dry-run)
      DRY_RUN=1
      shift
      ;;
    -*)
      echo "Unknown option: $1" >&2
      usage
      exit 1
      ;;
    *)
      if [[ $json_arg_seen -eq 0 ]]; then
        JSON_FILE="$1"
        json_arg_seen=1
      else
        echo "Unexpected positional argument: $1" >&2
        usage
        exit 1
      fi
      shift
      ;;
  esac
done

# 根据参数选择使用哪个 launcher
# 如果需要禁用 Isaac 或 sender，使用底层 launcher
# 否则使用 v3 tuning launcher
if [[ $NO_ISAAC -eq 1 ]] || [[ $NO_JSON_SENDER -eq 1 ]] || [[ -n "${WINDOWS_BRIDGE_IP}" ]]; then
  USE_BASE_LAUNCHER=1
else
  USE_BASE_LAUNCHER=0
fi

if [[ $USE_BASE_LAUNCHER -eq 1 ]]; then
  if [[ ! -f "${BASE_LAUNCHER}" ]]; then
    echo "ERROR: base launcher not found: ${BASE_LAUNCHER}" >&2
    exit 1
  fi

  # 使用底层 launcher，支持 --no-isaaclab 和跨机器部署
  cmd=(
    "${BASE_LAUNCHER}"
    --session "${SESSION}"
    --bvh-file "${JSON_FILE}"
    --zmq-port 6056
    --debug-port 6057
    --state-port 6060
    --bvh-stream-port 12403
    --manager-extra-args "--pose-protocol-version 3 --pose-encoder-mode smpl --allow-sony-pose-v3 --control-mode pose --source bvh_stream --target-fps 50 --bvh-stream-bonedata-coordinate-frame left_handed_yup --bvh-stream-bonedata-input-quat-order xyzw"
  )

  if [[ $NO_ISAAC -eq 1 ]]; then
    cmd+=(--no-isaaclab)
  fi

  if [[ $NO_JSON_SENDER -eq 1 ]]; then
    cmd+=(--no-bvh-stream-sender)
  fi

  if [[ -n "${WINDOWS_BRIDGE_IP}" ]]; then
    cmd+=(--isaac-state-host "${WINDOWS_BRIDGE_IP}")
  fi

  if [[ $REPLACE -eq 1 ]]; then
    cmd+=(--replace)
  fi

  if [[ $NO_ATTACH -eq 1 ]]; then
    cmd+=(--no-attach)
  fi

  if [[ $DRY_RUN -eq 1 ]]; then
    cmd+=(--dry-run)
  fi

  echo "========================================" >&2
  echo "Launching v3 SMPL POSE IsaacLab stack (base launcher)" >&2
else
  if [[ ! -f "${V3_LAUNCHER}" ]]; then
    echo "ERROR: v3 launcher not found: ${V3_LAUNCHER}" >&2
    exit 1
  fi

  # 使用 v3 tuning launcher，完整本地栈
  cmd=(
    "${V3_LAUNCHER}"
    --session "${SESSION}"
    --input-source "${INPUT_SOURCE}"
    --json-file "${JSON_FILE}"
  )

  if [[ $REPLACE -eq 1 ]]; then
    cmd+=(--replace)
  fi

  if [[ $NO_ATTACH -eq 1 ]]; then
    cmd+=(--no-attach)
  fi

  if [[ $DRY_RUN -eq 1 ]]; then
    cmd+=(--dry-run)
  fi

  echo "========================================" >&2
  echo "Launching v3 SMPL POSE IsaacLab stack (v3 tuning)" >&2
fi

echo "========================================" >&2
echo "Session:      ${SESSION}" >&2
echo "JSON file:    ${JSON_FILE}" >&2
echo "Input source: ${INPUT_SOURCE}" >&2
echo "No Isaac:     ${NO_ISAAC}" >&2
echo "No sender:    ${NO_JSON_SENDER}" >&2
if [[ -n "${WINDOWS_BRIDGE_IP}" ]]; then
  echo "Windows IP:   ${WINDOWS_BRIDGE_IP}" >&2
fi
echo "Launcher:     $(basename "${cmd[0]}")" >&2
echo "Command:      ${cmd[*]}" >&2
echo "========================================" >&2
echo >&2

exec "${cmd[@]}"
