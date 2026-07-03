#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR=$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)
SENDER_REPO=$(cd "${SCRIPT_DIR}/.." && pwd)

SESSION=${SESSION:-sonic_json_isaaclab}
JSON_FILE=${JSON_FILE:-/home/nolo/下载/saveBoneData0629.json}
BVH_STREAM_PORT=${BVH_STREAM_PORT:-12352}
MOCAP_ZMQ_PORT=${MOCAP_ZMQ_PORT:-5556}
DEBUG_PORT=${DEBUG_PORT:-5557}
STATE_PORT=${STATE_PORT:-5560}
FPS=${FPS:-50}
COORDINATE_FRAME=${COORDINATE_FRAME:-left_handed_yup}
BONEDATA_POSITION_SCALE=${BONEDATA_POSITION_SCALE:-1.0}
BONEDATA_INPUT_QUAT_ORDER=${BONEDATA_INPUT_QUAT_ORDER:-xyzw}
BONEDATA_ROTATION_MODE=${BONEDATA_ROTATION_MODE:-input}

ISAACLAB_ROOT=${ISAACLAB_ROOT:-/home/nolo/xiaoyang_IssacLab/IsaacLab}
CONDA_SH=${CONDA_SH:-/home/nolo/miniconda3/etc/profile.d/conda.sh}
CONDA_ENV=${CONDA_ENV:-env_isaaclab}
DEVICE=${DEVICE:-cpu}
DDS_INTERFACE=${DDS_INTERFACE:-lo}
DDS_DOMAIN_ID=${DDS_DOMAIN_ID:-0}

LAUNCHER_PY=${LAUNCHER_PY:-${SENDER_REPO}/.venv_teleop/bin/python}
LAUNCHER=${LAUNCHER:-${SENDER_REPO}/scripts/launch_sonic_local_isaaclab_closed_loop.py}
SENDER_PY=${SENDER_PY:-${SENDER_REPO}/.venv_teleop/bin/python}
SENDER_SCRIPT=${SENDER_SCRIPT:-${SENDER_REPO}/gear_sonic/scripts/sony_bonedata_json_stream_sender.py}

MODE=${MODE:-all}
REPLACE=${REPLACE:-1}
NO_ISAACLAB=${NO_ISAACLAB:-0}
ISAAC_STATE_HOST=${ISAAC_STATE_HOST:-127.0.0.1}
ISAAC_STATE_ENDPOINT=${ISAAC_STATE_ENDPOINT:-}

usage() {
  cat <<EOF
Usage:
  $0 [OPTIONS] [JSON_FILE]

Options:
  --no-isaaclab            Do not add the local IsaacLab tmux window.
  --with-isaaclab          Add the local IsaacLab tmux window. Default.
  --receiver-only          Start input/proxy/deploy, but do not start JSON sender.
  --no-json-sender         Alias for --receiver-only.
  --sender-only            Start only the Sony BoneData JSON sender in tmux.
  --print-sender-command   Print the standalone JSON sender command and exit.
  --isaac-state-host HOST  Host/IP where external IsaacLab publishes sonic_state.
  --windows-ip HOST        Alias for --isaac-state-host.
  --isaac-state-endpoint ENDPOINT
                            Full sonic_state endpoint, e.g. tcp://192.168.1.20:5560.
  --json-file PATH         JSON file path. A positional JSON_FILE is also accepted.
  --session NAME           tmux session name. Default: ${SESSION}
  --replace                Kill an existing tmux session first. Default when REPLACE=1.
  --no-replace             Fail if the tmux session already exists.
  -h, --help               Show this help text.

Environment overrides:
  SESSION=${SESSION}
  MODE=${MODE}                         # all, receiver, sender, print_sender
  NO_ISAACLAB=${NO_ISAACLAB}
  ISAAC_STATE_HOST=${ISAAC_STATE_HOST}
  ISAAC_STATE_ENDPOINT=${ISAAC_STATE_ENDPOINT}
  BVH_STREAM_PORT=${BVH_STREAM_PORT}
  MOCAP_ZMQ_PORT=${MOCAP_ZMQ_PORT}
  DEBUG_PORT=${DEBUG_PORT}
  STATE_PORT=${STATE_PORT}
  FPS=${FPS}

Examples:
  $0 /home/nolo/saveBoneData_Yup20260702.json
  $0 --no-isaaclab --windows-ip 192.168.1.20 /home/nolo/saveBoneData_Yup20260702.json
  $0 --no-json-sender
  $0 --no-isaaclab --no-json-sender --isaac-state-host 192.168.1.20
  $0 --sender-only /home/nolo/saveBoneData_Yup20260702.json
EOF
}

json_arg_seen=0
while [[ $# -gt 0 ]]; do
  case "$1" in
    -h|--help)
      usage
      exit 0
      ;;
    --no-isaaclab)
      NO_ISAACLAB=1
      shift
      ;;
    --with-isaaclab)
      NO_ISAACLAB=0
      shift
      ;;
    --receiver-only|--no-json-sender)
      MODE=receiver
      shift
      ;;
    --sender-only)
      MODE=sender
      shift
      ;;
    --print-sender-command)
      MODE=print_sender
      shift
      ;;
    --isaac-state-host|--windows-ip)
      if [[ $# -lt 2 ]]; then
        echo "ERROR: $1 requires a host/IP" >&2
        exit 2
      fi
      ISAAC_STATE_HOST=$2
      shift 2
      ;;
    --isaac-state-endpoint)
      if [[ $# -lt 2 ]]; then
        echo "ERROR: --isaac-state-endpoint requires an endpoint" >&2
        exit 2
      fi
      ISAAC_STATE_ENDPOINT=$2
      shift 2
      ;;
    --json-file)
      if [[ $# -lt 2 ]]; then
        echo "ERROR: --json-file requires a path" >&2
        exit 2
      fi
      JSON_FILE=$2
      json_arg_seen=1
      shift 2
      ;;
    --session)
      if [[ $# -lt 2 ]]; then
        echo "ERROR: --session requires a name" >&2
        exit 2
      fi
      SESSION=$2
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
    --)
      shift
      while [[ $# -gt 0 ]]; do
        if [[ "${json_arg_seen}" == "1" ]]; then
          echo "ERROR: multiple JSON files specified: ${JSON_FILE} and $1" >&2
          exit 2
        fi
        JSON_FILE=$1
        json_arg_seen=1
        shift
      done
      ;;
    -*)
      echo "ERROR: unknown option: $1" >&2
      usage >&2
      exit 2
      ;;
    *)
      if [[ "${json_arg_seen}" == "1" ]]; then
        echo "ERROR: multiple JSON files specified: ${JSON_FILE} and $1" >&2
        exit 2
      fi
      JSON_FILE=$1
      json_arg_seen=1
      shift
      ;;
  esac
done

case "${MODE}" in
  all|receiver|sender|print_sender)
    ;;
  receiver-only|no-json-sender)
    MODE=receiver
    ;;
  sender-only)
    MODE=sender
    ;;
  print-sender-command)
    MODE=print_sender
    ;;
  *)
    echo "ERROR: unsupported MODE=${MODE}; expected all, receiver, sender, or print_sender" >&2
    exit 2
    ;;
esac

case "${NO_ISAACLAB}" in
  1|true|TRUE|yes|YES|on|ON)
    NO_ISAACLAB=1
    ;;
  0|false|FALSE|no|NO|off|OFF|"")
    NO_ISAACLAB=0
    ;;
  *)
    echo "ERROR: unsupported NO_ISAACLAB=${NO_ISAACLAB}; expected 0 or 1" >&2
    exit 2
    ;;
esac

RUNTIME_DIR=${RUNTIME_DIR:-/tmp/sonic_json_isaaclab_${SESSION}}
mkdir -p "${RUNTIME_DIR}"

quote_arg() {
  printf "%q" "$1"
}

print_sender_command() {
  cat <<EOF
cd $(quote_arg "${SENDER_REPO}")
export PYTHONUNBUFFERED=1
export PYTHONPATH=$(quote_arg "${SENDER_REPO}"):\${PYTHONPATH:-}
$(quote_arg "${SENDER_PY}") -u $(quote_arg "${SENDER_SCRIPT}") \\
  --json-file $(quote_arg "${JSON_FILE}") \\
  --host 127.0.0.1 \\
  --port $(quote_arg "${BVH_STREAM_PORT}") \\
  --fps $(quote_arg "${FPS}") \\
  --loop \\
  |& tee "/tmp/sonic_local_sony_json_sender_\$(date +%Y%m%d_%H%M%S).log"
EOF
}

if [[ "${MODE}" != "receiver" && ! -f "${JSON_FILE}" ]]; then
  echo "ERROR: JSON file not found: ${JSON_FILE}" >&2
  exit 2
fi
if [[ "${MODE}" != "sender" && "${MODE}" != "print_sender" && ! -x "${LAUNCHER_PY}" ]]; then
  echo "ERROR: launcher python not executable: ${LAUNCHER_PY}" >&2
  exit 2
fi
if [[ "${MODE}" != "sender" && "${MODE}" != "print_sender" && ! -f "${LAUNCHER}" ]]; then
  echo "ERROR: launcher not found: ${LAUNCHER}" >&2
  exit 2
fi
if [[ "${MODE}" != "receiver" && ! -x "${SENDER_PY}" ]]; then
  echo "ERROR: sender python not executable: ${SENDER_PY}" >&2
  exit 2
fi
if [[ "${MODE}" != "receiver" && ! -f "${SENDER_SCRIPT}" ]]; then
  echo "ERROR: sender script not found: ${SENDER_SCRIPT}" >&2
  exit 2
fi
if [[ "${MODE}" == "print_sender" ]]; then
  print_sender_command
  exit 0
fi

if [[ "${MODE}" != "receiver" ]]; then
  cat > "${RUNTIME_DIR}/sony_json_sender.sh" <<EOF
#!/usr/bin/env bash
set -o pipefail
cd "${SENDER_REPO}"
export PYTHONUNBUFFERED=1
export PYTHONPATH="${SENDER_REPO}:\${PYTHONPATH:-}"
"${SENDER_PY}" -u "${SENDER_SCRIPT}" \
  --json-file "${JSON_FILE}" \
  --host 127.0.0.1 \
  --port "${BVH_STREAM_PORT}" \
  --fps "${FPS}" \
  --loop \
  |& tee "/tmp/sonic_local_sony_json_sender_\$(date +%Y%m%d_%H%M%S).log"
status=\$?
echo
echo "[sonic-json] sony_json_sender exited with status \${status}"
exec bash
EOF
  chmod +x "${RUNTIME_DIR}/sony_json_sender.sh"
fi

if [[ "${NO_ISAACLAB}" == "0" && "${MODE}" != "sender" ]]; then
  cat > "${RUNTIME_DIR}/isaaclab.sh" <<EOF
#!/usr/bin/env bash
set -o pipefail

deadline=\$((SECONDS + 90))
echo "[sonic-json] waiting for deploy debug port 127.0.0.1:${DEBUG_PORT}"
until (echo >/dev/tcp/127.0.0.1/${DEBUG_PORT}) >/dev/null 2>&1; do
  if [[ "\${SECONDS}" -ge "\${deadline}" ]]; then
    echo "[sonic-json] warning: timeout waiting for deploy debug port; starting IsaacLab anyway" >&2
    break
  fi
  sleep 0.5
done

cd "${ISAACLAB_ROOT}"
source "${CONDA_SH}"
conda activate "${CONDA_ENV}"

export PYTHONUNBUFFERED=1
export UNITREE_DDS_INTERFACE="${DDS_INTERFACE}"
export UNITREE_DDS_DOMAIN_ID="${DDS_DOMAIN_ID}"
export SONIC_DEPLOY_TRANSPORT=zmq
export SONIC_DEPLOY_ENDPOINT="tcp://127.0.0.1:${DEBUG_PORT}"
export SONIC_DEPLOY_TOPIC=g1_debug
export SONIC_DEPLOY_TARGET_FIELD=last_action
export SONIC_DEPLOY_REFERENCE_TARGET_FIELD=body_q_target
export SONIC_PUBLISH_STATE_ZMQ=1
export SONIC_STATE_ZMQ_BIND="tcp://*:${STATE_PORT}"
export SONIC_STATE_ZMQ_TOPIC=sonic_state
export SONIC_G1_PHYSICS_MODE=1
export SONIC_G1_VISUAL_SERVO_MODE=0
export SONIC_G1_SELF_COLLISIONS=0
export SONIC_DEPLOY_STABILIZE_ROOT=1
export SONIC_DEPLOY_TARGET_RATE_LIMIT=0.003

./isaaclab.sh -p scripts/environments/teleoperation/teleop_se3_agent.py \
  --task Isaac-SonicSolo-Locomanipulation-G1-v0 \
  --device "${DEVICE}" \
  --kit_args "--/app/vsync=false --/app/runLoops/main/rateLimitEnabled=false" \
  |& tee "/tmp/sonic_local_isaaclab_json_\$(date +%Y%m%d_%H%M%S).log"
status=\$?
echo
echo "[sonic-json] isaaclab exited with status \${status}"
exec bash
EOF
  chmod +x "${RUNTIME_DIR}/isaaclab.sh"
fi

if [[ -n "${ISAAC_STATE_ENDPOINT}" ]]; then
  STATE_ENDPOINT_DISPLAY="${ISAAC_STATE_ENDPOINT}"
else
  STATE_ENDPOINT_DISPLAY="tcp://${ISAAC_STATE_HOST}:${STATE_PORT}"
fi

echo "[sonic-json] session=${SESSION}"
echo "[sonic-json] mode=${MODE}"
if [[ "${MODE}" != "receiver" ]]; then
  echo "[sonic-json] json=${JSON_FILE}"
else
  echo "[sonic-json] json_sender=disabled"
fi
if [[ "${NO_ISAACLAB}" == "1" ]]; then
  echo "[sonic-json] local_isaaclab=disabled"
  echo "[sonic-json] external_isaac_state_endpoint=${STATE_ENDPOINT_DISPLAY}"
else
  echo "[sonic-json] local_isaaclab=enabled"
fi
echo "[sonic-json] receiver_coordinate_frame=${COORDINATE_FRAME}"
echo "[sonic-json] bvh_stream_port=${BVH_STREAM_PORT} mocap_zmq_port=${MOCAP_ZMQ_PORT}"

launcher_args=(
  "${LAUNCHER_PY}" -u "${LAUNCHER}"
  --session "${SESSION}" \
  --no-attach \
  --repo-root "${SENDER_REPO}" \
  --no-isaaclab \
  --no-bvh-stream-sender \
  --bvh-stream-port "${BVH_STREAM_PORT}" \
  --bvh-stream-bonedata-coordinate-frame "${COORDINATE_FRAME}" \
  --bvh-stream-bonedata-position-scale "${BONEDATA_POSITION_SCALE}" \
  --bvh-stream-bonedata-input-quat-order "${BONEDATA_INPUT_QUAT_ORDER}" \
  --bvh-stream-bonedata-rotation-mode "${BONEDATA_ROTATION_MODE}" \
  --zmq-port "${MOCAP_ZMQ_PORT}" \
  --debug-port "${DEBUG_PORT}" \
  --state-port "${STATE_PORT}"
)
if [[ "${REPLACE}" == "1" ]]; then
  launcher_args+=(--replace)
fi
if [[ -n "${ISAAC_STATE_ENDPOINT}" ]]; then
  launcher_args+=(--isaac-state-endpoint "${ISAAC_STATE_ENDPOINT}")
else
  launcher_args+=(--isaac-state-host "${ISAAC_STATE_HOST}")
fi

if [[ "${MODE}" == "sender" ]]; then
  if tmux has-session -t "${SESSION}" 2>/dev/null; then
    if [[ "${REPLACE}" == "1" ]]; then
      tmux kill-session -t "${SESSION}"
    else
      echo "ERROR: tmux session already exists: ${SESSION}" >&2
      exit 2
    fi
  fi
  tmux new-session -d -s "${SESSION}" -n sony_json_sender "bash '${RUNTIME_DIR}/sony_json_sender.sh'"
  echo "[sonic-json] started sender-only tmux session: ${SESSION}"
  echo "[sonic-json] attach with: tmux attach-session -t ${SESSION}"
  echo "[sonic-json] stop with: tmux kill-session -t ${SESSION}"
  exit 0
fi

"${launcher_args[@]}"

if [[ "${MODE}" != "receiver" ]]; then
  tmux new-window -t "${SESSION}" -n sony_json_sender "bash '${RUNTIME_DIR}/sony_json_sender.sh'"
fi
if [[ "${NO_ISAACLAB}" == "0" ]]; then
  tmux new-window -t "${SESSION}" -n isaaclab "bash '${RUNTIME_DIR}/isaaclab.sh'"
fi

if [[ "${NO_ISAACLAB}" == "0" ]]; then
  tmux select-window -t "${SESSION}:isaaclab"
elif [[ "${MODE}" != "receiver" ]]; then
  tmux select-window -t "${SESSION}:sony_json_sender"
else
  tmux select-window -t "${SESSION}:proxy"
fi

echo "[sonic-json] started tmux session: ${SESSION}"
echo "[sonic-json] attach with: tmux attach-session -t ${SESSION}"
echo "[sonic-json] stop with: tmux kill-session -t ${SESSION}"
if [[ "${MODE}" == "receiver" ]]; then
  echo "[sonic-json] JSON sender was not started. Start it later with:"
  echo "  ${SCRIPT_DIR}/$(basename "$0") --sender-only ${JSON_FILE}"
fi
