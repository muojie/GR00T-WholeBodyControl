#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR=$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)
REPO_ROOT=$(cd "${SCRIPT_DIR}/.." && pwd)

SESSION=${SESSION:-sonic_json_mujoco_v3}
JSON_FILE=${JSON_FILE:-/home/nolo/saveBoneData_Yup20260702.json}
INPUT_SOURCE=${INPUT_SOURCE:-sony_json}
NO_JSON_SENDER=${NO_JSON_SENDER:-0}
REPLACE=${REPLACE:-1}
NO_ATTACH=${NO_ATTACH:-0}
DRY_RUN=${DRY_RUN:-0}

V3_LAUNCHER=${V3_LAUNCHER:-${REPO_ROOT}/scripts/launch_sonic_v3_mujoco_closed_loop.py}

usage() {
  cat <<EOF
Usage:
  $0 [OPTIONS] [JSON_FILE]

启动 v3 SMPL POSE 版本的 Sony JSON MuJoCo 闭环栈。

Options:
  --no-json-sender         不启动 JSON sender（假设外部已在发送）。
  --receiver-only          同 --no-json-sender（兼容旧参数）。
  --sender-only            只启动 JSON sender（不启动 MuJoCo/manager/deploy）。
  --print-sender-command   只打印 sender 命令并退出。
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
  NO_JSON_SENDER=${NO_JSON_SENDER}
  REPLACE=${REPLACE}
  NO_ATTACH=${NO_ATTACH}
  DRY_RUN=${DRY_RUN}

Examples:
  # 完整本地栈（MuJoCo + manager + deploy + JSON sender + metrics）
  $0 /home/nolo/saveBoneData_Yup20260702.json

  # 只启动接收端，JSON sender 在别处跑
  $0 --no-json-sender
  $0 --receiver-only

  # 只启动 JSON sender
  $0 --sender-only /home/nolo/saveBoneData_Yup20260702.json

  # 打印 sender 命令
  $0 --print-sender-command /home/nolo/saveBoneData_Yup20260702.json

  # 打印完整命令不执行
  $0 --dry-run
EOF
}

json_arg_seen=0
sender_only=0
print_sender_command=0

while [[ $# -gt 0 ]]; do
  case "$1" in
    -h|--help)
      usage
      exit 0
      ;;
    --no-json-sender|--receiver-only)
      NO_JSON_SENDER=1
      shift
      ;;
    --sender-only)
      sender_only=1
      shift
      ;;
    --print-sender-command)
      print_sender_command=1
      shift
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

if [[ ! -f "${V3_LAUNCHER}" ]]; then
  echo "ERROR: v3 launcher not found: ${V3_LAUNCHER}" >&2
  exit 1
fi

# Handle --sender-only mode
if [[ $sender_only -eq 1 ]]; then
  SENDER_PY=${SENDER_PY:-${REPO_ROOT}/.venv_teleop/bin/python}
  SENDER_SCRIPT=${SENDER_SCRIPT:-${REPO_ROOT}/gear_sonic/scripts/sony_bonedata_json_stream_sender.py}

  sender_cmd=(
    "${SENDER_PY}" -u "${SENDER_SCRIPT}"
    --json-file "${JSON_FILE}"
    --loop
    --fps 30
    --udp-host localhost
    --udp-port 12403
    --verbose
  )

  if [[ $print_sender_command -eq 1 ]]; then
    echo "${sender_cmd[*]}"
    exit 0
  fi

  echo "========================================" >&2
  echo "Starting JSON sender only" >&2
  echo "========================================" >&2
  echo "JSON file: ${JSON_FILE}" >&2
  echo "Command:   ${sender_cmd[*]}" >&2
  echo "========================================" >&2
  echo >&2

  exec "${sender_cmd[@]}"
fi

# Handle --print-sender-command without --sender-only
if [[ $print_sender_command -eq 1 ]]; then
  SENDER_PY=${SENDER_PY:-${REPO_ROOT}/.venv_teleop/bin/python}
  SENDER_SCRIPT=${SENDER_SCRIPT:-${REPO_ROOT}/gear_sonic/scripts/sony_bonedata_json_stream_sender.py}

  echo "${SENDER_PY} -u ${SENDER_SCRIPT} --json-file ${JSON_FILE} --loop --fps 30 --udp-host localhost --udp-port 12403 --verbose"
  exit 0
fi

# Build main launcher command
cmd=(
  "${V3_LAUNCHER}"
  --session "${SESSION}"
  --input-source "${INPUT_SOURCE}"
  --json-file "${JSON_FILE}"
)

if [[ $NO_JSON_SENDER -eq 1 ]]; then
  cmd+=(--no-json-sender)
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
echo "Launching v3 SMPL POSE MuJoCo stack" >&2
echo "========================================" >&2
echo "Session:      ${SESSION}" >&2
echo "JSON file:    ${JSON_FILE}" >&2
echo "Input source: ${INPUT_SOURCE}" >&2
echo "No sender:    ${NO_JSON_SENDER}" >&2
echo "Command:      ${cmd[*]}" >&2
echo "========================================" >&2
echo >&2

exec "${cmd[@]}"
