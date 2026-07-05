#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR=$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)
REPO_ROOT=$(cd "${SCRIPT_DIR}/.." && pwd)

SESSION_WAS_SET=0
if [[ -n "${SESSION+x}" ]]; then
  SESSION_WAS_SET=1
fi
BVH_STREAM_PORT_WAS_SET=0
if [[ -n "${BVH_STREAM_PORT+x}" ]]; then
  BVH_STREAM_PORT_WAS_SET=1
fi
MOCAP_ZMQ_PORT_WAS_SET=0
if [[ -n "${MOCAP_ZMQ_PORT+x}" ]]; then
  MOCAP_ZMQ_PORT_WAS_SET=1
fi
DEBUG_PORT_WAS_SET=0
if [[ -n "${DEBUG_PORT+x}" ]]; then
  DEBUG_PORT_WAS_SET=1
fi
SESSION=${SESSION:-sonic_json_yup_mujoco}
BACKEND=${BACKEND:-mujoco}
JSON_FILE=${JSON_FILE:-/home/nolo/saveBoneData_Yup20260702.json}
BVH_STREAM_PORT=${BVH_STREAM_PORT:-12362}
MOCAP_ZMQ_PORT=${MOCAP_ZMQ_PORT:-5656}
DEBUG_PORT=${DEBUG_PORT:-5657}
BASE_POSE_PORT=${BASE_POSE_PORT:-5658}
BAND_RELEASE_S=${BAND_RELEASE_S:-90}
STATE_PORT=${STATE_PORT:-5560}
FPS=${FPS:-50}
COORDINATE_FRAME=${COORDINATE_FRAME:-left_handed_yup}
BONEDATA_POSITION_SCALE=${BONEDATA_POSITION_SCALE:-1.0}
BONEDATA_INPUT_QUAT_ORDER=${BONEDATA_INPUT_QUAT_ORDER:-xyzw}
BONEDATA_ROTATION_MODE=${BONEDATA_ROTATION_MODE:-input}
POSE_PROTOCOL_VERSION=${POSE_PROTOCOL_VERSION:-1}
REPLACE=${REPLACE:-1}
MODE=${MODE:-all}
NO_ISAACLAB=${NO_ISAACLAB:-0}
ISAAC_STATE_HOST=${ISAAC_STATE_HOST:-127.0.0.1}
ISAAC_STATE_ENDPOINT=${ISAAC_STATE_ENDPOINT:-}

SIM_PY=${SIM_PY:-${REPO_ROOT}/.venv_sim/bin/python}
TELEOP_PY=${TELEOP_PY:-${REPO_ROOT}/.venv_teleop/bin/python}
SENDER_SCRIPT=${SENDER_SCRIPT:-${REPO_ROOT}/gear_sonic/scripts/sony_bonedata_json_stream_sender.py}
MANAGER_SCRIPT=${MANAGER_SCRIPT:-${REPO_ROOT}/gear_sonic/scripts/mocap_manager_server.py}
MANAGER_EXTRA_ARGS=${MANAGER_EXTRA_ARGS:-}
DEPLOY_ROOT=${DEPLOY_ROOT:-${REPO_ROOT}/gear_sonic_deploy}

usage() {
  cat <<EOF
Usage:
  $0 [OPTIONS] [JSON_FILE]

Options:
  --backend mujoco|isaaclab  Select simulation backend. Default: mujoco.
  --mujoco                   Alias for --backend mujoco.
  --isaaclab                 Alias for --backend isaaclab.
  --receiver-only          Start MuJoCo, manager, and deploy, but do not start JSON sender.
  --no-json-sender         Alias for --receiver-only.
  --no-isaaclab            IsaacLab backend only: do not add local IsaacLab window.
  --isaac-state-host HOST  IsaacLab backend only: external sonic_state host/IP.
  --windows-ip HOST        Alias for --isaac-state-host.
  --isaac-state-endpoint ENDPOINT
                            IsaacLab backend only: full sonic_state endpoint.
  --sender-only            Start only the Sony BoneData JSON sender in tmux.
  --print-sender-command   Print the standalone sender command and exit.
  --json-file PATH         JSON file path. A positional JSON_FILE is also accepted.
  -h, --help               Show this help text.

Environment overrides:
  SESSION=${SESSION}
  BACKEND=${BACKEND}
  MODE=${MODE}
  NO_ISAACLAB=${NO_ISAACLAB}              # IsaacLab backend only
  ISAAC_STATE_HOST=${ISAAC_STATE_HOST}    # IsaacLab backend only
  ISAAC_STATE_ENDPOINT=${ISAAC_STATE_ENDPOINT}
  BVH_STREAM_PORT=${BVH_STREAM_PORT}
  MOCAP_ZMQ_PORT=${MOCAP_ZMQ_PORT}
  DEBUG_PORT=${DEBUG_PORT}
  STATE_PORT=${STATE_PORT}                  # IsaacLab backend only
  FPS=${FPS}
  COORDINATE_FRAME=${COORDINATE_FRAME}       # receiver-side raw BoneData conversion
  BONEDATA_POSITION_SCALE=${BONEDATA_POSITION_SCALE}
  BONEDATA_INPUT_QUAT_ORDER=${BONEDATA_INPUT_QUAT_ORDER}
  BONEDATA_ROTATION_MODE=${BONEDATA_ROTATION_MODE}
  POSE_PROTOCOL_VERSION=${POSE_PROTOCOL_VERSION}
  REPLACE=${REPLACE}

Examples:
  $0 --backend mujoco /home/nolo/saveBoneData_Yup20260702.json
  $0 --backend isaaclab /home/nolo/saveBoneData_Yup20260702.json
  $0 --backend isaaclab --no-isaaclab --windows-ip 192.168.1.20
  $0 --backend isaaclab --no-json-sender
  $0 --receiver-only
  $0 --sender-only /home/nolo/saveBoneData_Yup20260702.json
  $0 --print-sender-command /home/nolo/saveBoneData_Yup20260702.json
  COORDINATE_FRAME=sonic_zup $0 /path/to/saveBoneData.json
  POSE_PROTOCOL_VERSION=3 $0 /home/nolo/saveBoneData_Yup20260702.json
  tmux kill-session -t ${SESSION}
EOF
}

json_arg_seen=0
while [[ $# -gt 0 ]]; do
  case "$1" in
    -h|--help)
      usage
      exit 0
      ;;
    --sender-only)
      MODE=sender
      shift
      ;;
    --receiver-only|--no-json-sender)
      MODE=receiver
      shift
      ;;
    --no-isaaclab)
      NO_ISAACLAB=1
      shift
      ;;
    --with-isaaclab)
      NO_ISAACLAB=0
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
    --print-sender-command)
      MODE=print_sender
      shift
      ;;
    --backend)
      if [[ $# -lt 2 ]]; then
        echo "ERROR: --backend requires mujoco or isaaclab" >&2
        exit 2
      fi
      BACKEND=$2
      shift 2
      ;;
    --mujoco)
      BACKEND=mujoco
      shift
      ;;
    --isaaclab|--isaac)
      BACKEND=isaaclab
      shift
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
    --)
      shift
      break
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

if [[ $# -gt 0 ]]; then
  echo "ERROR: unexpected argument: $1" >&2
  exit 2
fi

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

case "${BACKEND}" in
  mujoco|isaaclab)
    ;;
  isaac)
    BACKEND=isaaclab
    ;;
  *)
    echo "ERROR: unsupported BACKEND=${BACKEND}; expected mujoco or isaaclab" >&2
    exit 2
    ;;
esac

if [[ "${BACKEND}" == "isaaclab" ]]; then
  if [[ "${SESSION_WAS_SET}" == "0" ]]; then
    SESSION=sonic_json_isaaclab
  fi
  if [[ "${BVH_STREAM_PORT_WAS_SET}" == "0" ]]; then
    BVH_STREAM_PORT=12352
  fi
  if [[ "${MOCAP_ZMQ_PORT_WAS_SET}" == "0" ]]; then
    MOCAP_ZMQ_PORT=5556
  fi
  if [[ "${DEBUG_PORT_WAS_SET}" == "0" ]]; then
    DEBUG_PORT=5557
  fi
else
  if [[ "${SESSION_WAS_SET}" == "0" ]]; then
    SESSION=sonic_json_yup_mujoco
  fi
fi

if [[ "${MODE}" == "sender" && "${SESSION_WAS_SET}" == "0" ]]; then
  SESSION=sonic_json_${BACKEND}_sender
fi

LOG_DIR=${LOG_DIR:-${REPO_ROOT}/logs/sony_json_${BACKEND}_${SESSION}}
RUNTIME_DIR=${RUNTIME_DIR:-/tmp/sony_json_${BACKEND}_${SESSION}}

quote_arg() {
  printf "%q" "$1"
}

print_sender_command() {
  cat <<EOF
mkdir -p $(quote_arg "${LOG_DIR}")
cd $(quote_arg "${REPO_ROOT}")
export PYTHONUNBUFFERED=1
export PYTHONPATH=$(quote_arg "${REPO_ROOT}"):\${PYTHONPATH:-}
$(quote_arg "${TELEOP_PY}") -u $(quote_arg "${SENDER_SCRIPT}") \\
  --json-file $(quote_arg "${JSON_FILE}") \\
  --host 127.0.0.1 \\
  --port $(quote_arg "${BVH_STREAM_PORT}") \\
  --fps $(quote_arg "${FPS}") \\
  --loop \\
  --log-interval-s 1.0 \\
  2>&1 | tee $(quote_arg "${LOG_DIR}/json_sender.log")
EOF
}

if [[ "${MODE}" != "receiver" && ! -f "${JSON_FILE}" ]]; then
  echo "ERROR: JSON file not found: ${JSON_FILE}" >&2
  exit 2
fi
if [[ ! -x "${TELEOP_PY}" ]]; then
  echo "ERROR: teleop python not executable: ${TELEOP_PY}" >&2
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
if [[ "${BACKEND}" == "isaaclab" && "${MODE}" != "sender" && "${MODE}" != "print_sender" ]]; then
  isaaclab_args=()
  if [[ "${MODE}" == "receiver" ]]; then
    isaaclab_args+=(--no-json-sender)
  fi
  if [[ "${NO_ISAACLAB}" == "1" ]]; then
    isaaclab_args+=(--no-isaaclab)
  fi
  if [[ -n "${ISAAC_STATE_ENDPOINT}" ]]; then
    isaaclab_args+=(--isaac-state-endpoint "${ISAAC_STATE_ENDPOINT}")
  else
    isaaclab_args+=(--isaac-state-host "${ISAAC_STATE_HOST}")
  fi
  SESSION="${SESSION}" \
    NO_ISAACLAB="${NO_ISAACLAB}" \
    ISAAC_STATE_HOST="${ISAAC_STATE_HOST}" \
    ISAAC_STATE_ENDPOINT="${ISAAC_STATE_ENDPOINT}" \
    JSON_FILE="${JSON_FILE}" \
    BVH_STREAM_PORT="${BVH_STREAM_PORT}" \
    MOCAP_ZMQ_PORT="${MOCAP_ZMQ_PORT}" \
    DEBUG_PORT="${DEBUG_PORT}" \
    STATE_PORT="${STATE_PORT}" \
    FPS="${FPS}" \
    COORDINATE_FRAME="${COORDINATE_FRAME}" \
    BONEDATA_POSITION_SCALE="${BONEDATA_POSITION_SCALE}" \
    BONEDATA_INPUT_QUAT_ORDER="${BONEDATA_INPUT_QUAT_ORDER}" \
    BONEDATA_ROTATION_MODE="${BONEDATA_ROTATION_MODE}" \
    POSE_PROTOCOL_VERSION="${POSE_PROTOCOL_VERSION}" \
    REPLACE="${REPLACE}" \
    "${SCRIPT_DIR}/launch_sonic_json_isaaclab_closed_loop.sh" "${isaaclab_args[@]}" --json-file "${JSON_FILE}"
  exit 0
fi
if [[ "${MODE}" != "sender" && ! -x "${SIM_PY}" ]]; then
  echo "ERROR: sim python not executable: ${SIM_PY}" >&2
  exit 2
fi
if [[ "${MODE}" != "sender" && ! -f "${MANAGER_SCRIPT}" ]]; then
  echo "ERROR: manager script not found: ${MANAGER_SCRIPT}" >&2
  exit 2
fi
if [[ "${MODE}" != "sender" && ! -x "${DEPLOY_ROOT}/target/release/g1_deploy_onnx_ref" ]]; then
  echo "ERROR: deploy binary not executable: ${DEPLOY_ROOT}/target/release/g1_deploy_onnx_ref" >&2
  exit 2
fi
if [[ "${MODE}" != "sender" && ! -f "${DEPLOY_ROOT}/planner/target_vel/V2/planner_sonic.onnx" ]]; then
  echo "ERROR: planner model not found: ${DEPLOY_ROOT}/planner/target_vel/V2/planner_sonic.onnx" >&2
  exit 2
fi

mkdir -p "${LOG_DIR}" "${RUNTIME_DIR}"

if tmux has-session -t "${SESSION}" 2>/dev/null; then
  if [[ "${REPLACE}" == "1" ]]; then
    tmux kill-session -t "${SESSION}"
  else
    echo "ERROR: tmux session already exists: ${SESSION}" >&2
    echo "Use REPLACE=1 or kill it with: tmux kill-session -t ${SESSION}" >&2
    exit 2
  fi
fi

if [[ "${MODE}" != "sender" ]] && command -v ss >/dev/null; then
  if ss -ltnup | grep -E ":((${BVH_STREAM_PORT})|(${MOCAP_ZMQ_PORT})|(${DEBUG_PORT}))\\b" >/tmp/sony_json_mujoco_ports.$$; then
    echo "ERROR: one or more requested ports are already in use:" >&2
    cat /tmp/sony_json_mujoco_ports.$$ >&2
    rm -f /tmp/sony_json_mujoco_ports.$$
    exit 2
  fi
  rm -f /tmp/sony_json_mujoco_ports.$$
fi

if [[ "${MODE}" != "sender" ]]; then
  cat > "${RUNTIME_DIR}/mujoco.sh" <<EOF
#!/usr/bin/env bash
set -o pipefail
cd "${REPO_ROOT}"
source "${REPO_ROOT}/.venv_sim/bin/activate"
export PYTHONUNBUFFERED=1
export PYTHONPATH="${REPO_ROOT}:\${PYTHONPATH:-}"
export SONIC_SIM_BASE_POSE_PORT="${BASE_POSE_PORT}"
export SONIC_SIM_BAND_RELEASE_S="${BAND_RELEASE_S}"
"${SIM_PY}" -u gear_sonic/scripts/run_sim_loop.py \
  2>&1 | tee "${LOG_DIR}/mujoco.log"
status=\$?
echo
echo "[sonic-json-mujoco] mujoco exited with status \${status}"
exec bash
EOF

  # v3 requires smpl encoder mode and --allow-sony-pose-v3
  if [[ "${POSE_PROTOCOL_VERSION}" == "3" ]]; then
    POSE_ENCODER_MODE=smpl
    POSE_V3_FLAG="--allow-sony-pose-v3"
  else
    POSE_ENCODER_MODE=g1
    POSE_V3_FLAG=""
  fi

  cat > "${RUNTIME_DIR}/manager.sh" <<EOF
#!/usr/bin/env bash
set -o pipefail
cd "${REPO_ROOT}"
export PYTHONUNBUFFERED=1
export PYTHONPATH="${REPO_ROOT}:\${PYTHONPATH:-}"
"${TELEOP_PY}" -u "${MANAGER_SCRIPT}" \
  --source bvh_stream \
  --bvh-stream-host 0.0.0.0 \
  --bvh-stream-port "${BVH_STREAM_PORT}" \
  --bvh-stream-bonedata-coordinate-frame "${COORDINATE_FRAME}" \
  --bvh-stream-bonedata-position-scale "${BONEDATA_POSITION_SCALE}" \
  --bvh-stream-bonedata-input-quat-order "${BONEDATA_INPUT_QUAT_ORDER}" \
  --bvh-stream-bonedata-rotation-mode "${BONEDATA_ROTATION_MODE}" \
  --control-mode pose \
  --pose-window-size 80 \
  --pose-encoder-mode ${POSE_ENCODER_MODE} \
  --pose-protocol-version "${POSE_PROTOCOL_VERSION}" \
  ${POSE_V3_FLAG} \
  --zmq-port "${MOCAP_ZMQ_PORT}" \
  --log-interval-s 1.0 ${MANAGER_EXTRA_ARGS} \
  2>&1 | tee "${LOG_DIR}/manager.log"
status=\$?
echo
echo "[sonic-json-mujoco] manager exited with status \${status}"
exec bash
EOF

  cat > "${RUNTIME_DIR}/deploy.sh" <<EOF
#!/usr/bin/env bash
set -o pipefail
cd "${DEPLOY_ROOT}"
source scripts/setup_env.sh >"${LOG_DIR}/setup_env.log" 2>&1 || true
stdbuf -oL -eL just run g1_deploy_onnx_ref \
  lo \
  policy/release/model_decoder.onnx \
  reference/example \
  --obs-config policy/release/observation_config.yaml \
  --encoder-file policy/release/model_encoder.onnx \
  --planner-file planner/target_vel/V2/planner_sonic.onnx \
  --input-type zmq_manager \
  --output-type all \
  --zmq-host localhost \
  --zmq-port "${MOCAP_ZMQ_PORT}" \
  --zmq-topic pose \
  --zmq-out-port "${DEBUG_PORT}" \
  --zmq-out-topic g1_debug \
  --disable-crc-check \
  2>&1 | tee "${LOG_DIR}/deploy.log"
status=\$?
echo
echo "[sonic-json-mujoco] deploy exited with status \${status}"
exec bash
EOF
fi

if [[ "${MODE}" != "receiver" ]]; then
  cat > "${RUNTIME_DIR}/json_sender.sh" <<EOF
#!/usr/bin/env bash
set -o pipefail
cd "${REPO_ROOT}"
export PYTHONUNBUFFERED=1
export PYTHONPATH="${REPO_ROOT}:\${PYTHONPATH:-}"
"${TELEOP_PY}" -u "${SENDER_SCRIPT}" \
  --json-file "${JSON_FILE}" \
  --host 127.0.0.1 \
  --port "${BVH_STREAM_PORT}" \
  --fps "${FPS}" \
  --loop \
  --log-interval-s 1.0 \
  2>&1 | tee "${LOG_DIR}/json_sender.log"
status=\$?
echo
echo "[sonic-json-mujoco] json_sender exited with status \${status}"
exec bash
EOF
fi

if [[ "${MODE}" != "receiver" ]]; then
  chmod +x "${RUNTIME_DIR}/json_sender.sh"
fi
if [[ "${MODE}" != "sender" ]]; then
  chmod +x "${RUNTIME_DIR}/mujoco.sh" \
    "${RUNTIME_DIR}/manager.sh" \
    "${RUNTIME_DIR}/deploy.sh"
fi

echo "[sonic-json-mujoco] session=${SESSION}"
echo "[sonic-json-mujoco] backend=${BACKEND}"
echo "[sonic-json-mujoco] mode=${MODE}"
echo "[sonic-json-mujoco] json=${JSON_FILE}"
echo "[sonic-json-mujoco] receiver_coordinate_frame=${COORDINATE_FRAME}"
echo "[sonic-json-mujoco] receiver_position_scale=${BONEDATA_POSITION_SCALE}"
echo "[sonic-json-mujoco] ports: bvh_stream=${BVH_STREAM_PORT} zmq=${MOCAP_ZMQ_PORT} debug=${DEBUG_PORT}"
echo "[sonic-json-mujoco] logs=${LOG_DIR}"

if [[ "${MODE}" == "sender" ]]; then
  tmux new-session -d -s "${SESSION}" -n json_sender "bash '${RUNTIME_DIR}/json_sender.sh'"
  echo "[sonic-json-mujoco] started sender-only tmux session: ${SESSION}"
  echo "[sonic-json-mujoco] attach with: tmux attach-session -t ${SESSION}"
  echo "[sonic-json-mujoco] stop with: tmux kill-session -t ${SESSION}"
  exit 0
fi

tmux new-session -d -s "${SESSION}" -n mujoco "bash '${RUNTIME_DIR}/mujoco.sh'"
sleep 5
tmux new-window -t "${SESSION}" -n manager "bash '${RUNTIME_DIR}/manager.sh'"
sleep 2
tmux new-window -t "${SESSION}" -n deploy "bash '${RUNTIME_DIR}/deploy.sh'"
if [[ "${MODE}" == "all" ]]; then
  sleep 8
  tmux new-window -t "${SESSION}" -n json_sender "bash '${RUNTIME_DIR}/json_sender.sh'"
  tmux select-window -t "${SESSION}:json_sender"
else
  tmux select-window -t "${SESSION}:manager"
fi

echo "[sonic-json-mujoco] started tmux session: ${SESSION}"
echo "[sonic-json-mujoco] attach with: tmux attach-session -t ${SESSION}"
echo "[sonic-json-mujoco] stop with: tmux kill-session -t ${SESSION}"
if [[ "${MODE}" == "receiver" ]]; then
  echo "[sonic-json-mujoco] JSON sender was not started. Start it later with:"
  echo "  ${SCRIPT_DIR}/$(basename "$0") --sender-only ${JSON_FILE}"
fi
