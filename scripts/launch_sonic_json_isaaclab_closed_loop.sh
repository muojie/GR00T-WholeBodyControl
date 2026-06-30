#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR=$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)
SENDER_REPO=$(cd "${SCRIPT_DIR}/.." && pwd)

SESSION=${SESSION:-sonic_json_isaaclab}
JSON_FILE=${1:-${JSON_FILE:-/home/nolo/下载/saveBoneData0629.json}}
BVH_STREAM_PORT=${BVH_STREAM_PORT:-12352}
MOCAP_ZMQ_PORT=${MOCAP_ZMQ_PORT:-5556}
DEBUG_PORT=${DEBUG_PORT:-5557}
STATE_PORT=${STATE_PORT:-5560}
FPS=${FPS:-50}

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

RUNTIME_DIR=${RUNTIME_DIR:-/tmp/sonic_json_isaaclab_${SESSION}}
mkdir -p "${RUNTIME_DIR}"

if [[ ! -f "${JSON_FILE}" ]]; then
  echo "ERROR: JSON file not found: ${JSON_FILE}" >&2
  exit 2
fi
if [[ ! -x "${LAUNCHER_PY}" ]]; then
  echo "ERROR: launcher python not executable: ${LAUNCHER_PY}" >&2
  exit 2
fi
if [[ ! -f "${LAUNCHER}" ]]; then
  echo "ERROR: launcher not found: ${LAUNCHER}" >&2
  exit 2
fi
if [[ ! -x "${SENDER_PY}" ]]; then
  echo "ERROR: sender python not executable: ${SENDER_PY}" >&2
  exit 2
fi
if [[ ! -f "${SENDER_SCRIPT}" ]]; then
  echo "ERROR: sender script not found: ${SENDER_SCRIPT}" >&2
  exit 2
fi

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

echo "[sonic-json] session=${SESSION}"
echo "[sonic-json] json=${JSON_FILE}"
echo "[sonic-json] bvh_stream_port=${BVH_STREAM_PORT} mocap_zmq_port=${MOCAP_ZMQ_PORT}"

"${LAUNCHER_PY}" -u "${LAUNCHER}" \
  --session "${SESSION}" \
  --replace \
  --no-attach \
  --no-isaaclab \
  --no-bvh-stream-sender \
  --bvh-stream-port "${BVH_STREAM_PORT}" \
  --zmq-port "${MOCAP_ZMQ_PORT}" \
  --debug-port "${DEBUG_PORT}" \
  --state-port "${STATE_PORT}"

tmux new-window -t "${SESSION}" -n sony_json_sender "bash '${RUNTIME_DIR}/sony_json_sender.sh'"
tmux new-window -t "${SESSION}" -n isaaclab "bash '${RUNTIME_DIR}/isaaclab.sh'"
tmux select-window -t "${SESSION}:isaaclab"

echo "[sonic-json] started tmux session: ${SESSION}"
echo "[sonic-json] attach with: tmux attach-session -t ${SESSION}"
