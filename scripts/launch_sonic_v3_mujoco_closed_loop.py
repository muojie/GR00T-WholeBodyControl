#!/usr/bin/env python3
"""Launch an isolated Sony/BVH SMPL POSE v3 stack against MuJoCo."""

from __future__ import annotations

import argparse
import shlex
import shutil
import socket
import subprocess
import sys
import time
from dataclasses import dataclass
from pathlib import Path


DEFAULT_SESSION = "sonic_v3_mujoco_tuning"
DEFAULT_REPO_ROOT = Path(__file__).resolve().parents[1]
DEFAULT_BVH_FILE = Path.home() / "RAYNOS_Motion1.bvh"
DEFAULT_JSON_FILE = Path.home() / "saveBoneData_Yup20260702.json"
ORIGINAL_REPO_ROOT = Path.home() / "GR00T-WholeBodyControl"

DEFAULT_ZMQ_PORT = 6156
DEFAULT_DEBUG_PORT = 6157
DEFAULT_SIM_METRICS_PORT = 6158
DEFAULT_BVH_STREAM_PORT = 12413
DEFAULT_DOMAIN_ID = 4


@dataclass(frozen=True)
class WindowCommand:
    name: str
    command: str


def _quote(value: str | Path | int | float) -> str:
    return shlex.quote(str(value))


def _run(cmd: list[str]) -> None:
    subprocess.run(cmd, check=True)


def _tmux_session_exists(session: str) -> bool:
    return subprocess.run(
        ["tmux", "has-session", "-t", session],
        stdout=subprocess.DEVNULL,
        stderr=subprocess.DEVNULL,
    ).returncode == 0


def _port_is_available(port: int) -> bool:
    sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    try:
        sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        sock.bind(("0.0.0.0", port))
        return True
    except OSError:
        return False
    finally:
        sock.close()


def _shell_window(command: str) -> str:
    wrapped = (
        "set -o pipefail; "
        f"{command}; "
        "status=$?; "
        "echo; "
        'echo "[sonic-v3-mujoco] process exited with status ${status}"; '
        "exec bash"
    )
    return f"bash -lc {_quote(wrapped)}"


def _log_path(name: str) -> str:
    stamp = time.strftime("%Y%m%d_%H%M%S")
    safe_name = "".join(ch if ch.isalnum() or ch in "-_" else "_" for ch in name)
    return f"/tmp/sonic_v3_mujoco_{safe_name}_{stamp}.log"


def _with_log(command: str, name: str) -> str:
    return f"{command} |& tee {_quote(_log_path(name))}"


def _default_metric_path(kind: str) -> Path:
    stamp = time.strftime("%Y%m%d_%H%M%S")
    suffix = "summary.json" if kind == "summary" else "samples.jsonl"
    return Path(f"/tmp/sony_pose_v3_mujoco_{stamp}_{suffix}")


def _resolve_release_file(repo_root: Path, relative_path: str) -> Path:
    local = repo_root / "gear_sonic_deploy" / relative_path
    if local.exists():
        return local
    original = ORIGINAL_REPO_ROOT / "gear_sonic_deploy" / relative_path
    if original.exists():
        return original
    return local


def _path_arg(path: Path, base: Path) -> str:
    try:
        return str(path.relative_to(base))
    except ValueError:
        return str(path)


def _wait_for_tcp_port_command(host: str, port: int, timeout_s: float, label: str) -> str:
    timeout_i = max(1, int(timeout_s + 0.999))
    return (
        f"echo {_quote(f'[sonic-v3-mujoco] waiting for {label} {host}:{port}')}"
        f" && deadline=$((SECONDS+{timeout_i}))"
        f" && until (echo >/dev/tcp/{host}/{int(port)}) >/dev/null 2>&1; do"
        f" if [ \"$SECONDS\" -ge \"$deadline\" ]; then"
        f" echo {_quote(f'[sonic-v3-mujoco] timeout waiting for {label} {host}:{port}')} >&2; exit 2;"
        f" fi;"
        f" sleep 0.5;"
        f" done"
        f" && echo {_quote(f'[sonic-v3-mujoco] {label} ready')}"
    )


def _build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        description="Launch isolated Sony/BVH SMPL POSE v3 closed-loop tuning against MuJoCo.",
        formatter_class=argparse.ArgumentDefaultsHelpFormatter,
    )
    parser.add_argument("--session", default=DEFAULT_SESSION)
    parser.add_argument("--replace", action="store_true")
    parser.add_argument("--no-attach", action="store_true")
    parser.add_argument("--dry-run", action="store_true")
    parser.add_argument("--ignore-port-check", action="store_true")

    parser.add_argument("--repo-root", type=Path, default=DEFAULT_REPO_ROOT)
    parser.add_argument("--interface", default="lo")
    parser.add_argument("--domain-id", type=int, default=DEFAULT_DOMAIN_ID)
    parser.add_argument("--zmq-port", type=int, default=DEFAULT_ZMQ_PORT)
    parser.add_argument("--debug-port", type=int, default=DEFAULT_DEBUG_PORT)
    parser.add_argument("--debug-topic", default="g1_debug")
    parser.add_argument("--sim-metrics-port", type=int, default=DEFAULT_SIM_METRICS_PORT)
    parser.add_argument("--sim-metrics-topic", default="mujoco_metrics")
    parser.add_argument("--sim-metrics-hz", type=float, default=50.0)
    parser.add_argument("--no-sim-metrics", action="store_true")
    parser.add_argument("--zmq-topic", default="pose")
    parser.add_argument("--bvh-stream-host", default="0.0.0.0")
    parser.add_argument("--bvh-stream-sender-host", default="127.0.0.1")
    parser.add_argument("--bvh-stream-port", type=int, default=DEFAULT_BVH_STREAM_PORT)

    parser.add_argument("--input-source", choices=["bvh", "sony_json"], default="bvh")
    parser.add_argument("--bvh-file", type=Path, default=DEFAULT_BVH_FILE)
    parser.add_argument("--bvh-fps", type=float)
    parser.add_argument("--no-bvh-loop", action="store_true")
    parser.add_argument("--bvh-wait-timeout-s", type=float, default=180.0)
    parser.add_argument("--bvh-start-delay-s", type=float, default=1.0)
    parser.add_argument(
        "--json-file",
        type=Path,
        default=DEFAULT_JSON_FILE,
        help="Sony mocopi saveBoneData JSON file when --input-source sony_json.",
    )
    parser.add_argument("--sony-bonedata-packet-format", choices=["msgpack", "json"], default="msgpack")
    parser.add_argument("--sony-bonedata-joints-per-frame", type=int, default=27)
    parser.add_argument("--sony-bonedata-fps", type=float, help="Sony BoneData sender FPS; defaults to --bvh-fps or 50.")
    parser.add_argument("--sony-bonedata-source-fps", type=float, help="Original Sony BoneData capture FPS.")
    parser.add_argument("--sony-bonedata-start-frame", type=int, default=0)
    parser.add_argument(
        "--bvh-stream-bonedata-coordinate-frame",
        choices=["sonic_zup", "left_handed_zup", "left_handed_yup", "zup_flip_xy"],
        default="left_handed_yup",
        help="receiver-side coordinate conversion for raw sony_bonedata_json_v1 packets.",
    )
    parser.add_argument("--bvh-stream-bonedata-position-scale", type=float, default=1.0)
    parser.add_argument("--bvh-stream-bonedata-input-quat-order", choices=["xyzw", "wxyz"], default="xyzw")
    parser.add_argument("--bvh-stream-bonedata-rotation-mode", choices=["input", "identity"], default="input")
    parser.add_argument("--bvh-stream-bonedata-local-root", action="store_true")

    parser.add_argument("--onscreen", action="store_true", help="show the MuJoCo viewer")
    parser.add_argument("--offscreen", action="store_true", help="enable MuJoCo offscreen rendering")
    parser.add_argument(
        "--no-mujoco-elastic-band",
        action="store_true",
        help="disable the MuJoCo root elastic band for free-root walking diagnostics",
    )
    parser.add_argument("--sim-frequency", type=int, default=200)
    parser.add_argument("--control-frequency", type=int, default=50)

    parser.add_argument(
        "--pose-filter-profile",
        choices=["stable", "balanced", "responsive", "off"],
        default="stable",
    )
    parser.add_argument("--control-mode", choices=["pose", "planner"], default="pose")
    parser.add_argument("--planner-from-root", action="store_true")
    parser.add_argument("--planner-root-speed-scale", type=float, default=1.0)
    parser.add_argument("--planner-root-speed-alpha", type=float, default=0.25)
    parser.add_argument("--planner-root-speed-deadband", type=float, default=0.04)
    parser.add_argument("--planner-root-speed-min", type=float, default=0.12)
    parser.add_argument("--planner-root-speed-max", type=float, default=0.8)
    parser.add_argument("--planner-root-locomotion-mode", type=int, default=2)
    parser.add_argument(
        "--pose-encoder-mode",
        choices=["g1", "teleop", "smpl"],
        default="smpl",
        help="encoder branch requested in the streamed POSE v3 messages",
    )
    parser.add_argument(
        "--pose-protocol-version",
        type=int,
        choices=[1, 3],
        default=3,
        help="POSE stream protocol version sent to deploy",
    )
    parser.add_argument(
        "--no-root-yaw-only",
        action="store_true",
        help="do not pass --pose-root-yaw-only to mocap_manager_server.py",
    )
    parser.add_argument("--pose-window-size", type=int, default=80)
    parser.add_argument("--mocap-log-interval-s", type=float, default=1.0)
    parser.add_argument("--bvh-g1-retarget-scale", type=float, default=1.0)
    parser.add_argument("--bvh-g1-lower-scale", type=float, default=0.60)
    parser.add_argument("--bvh-g1-upper-scale", type=float, default=0.85)
    parser.add_argument("--bvh-g1-wrist-scale", type=float, default=0.55)
    parser.add_argument("--bvh-g1-waist-scale", type=float, default=0.25)
    parser.add_argument("--bvh-g1-max-joint-velocity", type=float, default=5.5)
    parser.add_argument("--bvh-g1-max-joint-step", type=float, default=0.0)
    parser.add_argument("--bvh-g1-joint-filter-alpha", type=float, default=0.35)
    parser.add_argument("--bvh-g1-joint-delta-limit-scale", type=float, default=0.8)
    parser.add_argument("--bvh-g1-min-root-height", type=float, default=0.74)
    parser.add_argument(
        "--bvh-g1-no-align-root",
        action="store_true",
        help="Do not align BVH root to first frame; preserve global root translation for JSON playback"
    )
    parser.add_argument(
        "--bvh-g1-smpl-joints-source",
        choices=("g1_fk", "skeleton", "smpl_model", "canonical"),
        default="g1_fk",
        help="smpl_joints source for Sony/BVH POSE v3 (forwarded to mocap_manager_server.py)",
    )

    parser.add_argument("--decoder", type=Path)
    parser.add_argument("--encoder", type=Path)
    parser.add_argument("--planner-file", type=Path)
    parser.add_argument("--obs-config", type=Path)
    parser.add_argument("--motion-data", type=Path)

    parser.add_argument("--no-metrics", action="store_true")
    parser.add_argument("--metrics-duration-s", type=float, default=45.0)
    parser.add_argument("--metrics-startup-timeout-s", type=float, default=180.0)
    parser.add_argument("--metrics-warmup-s", type=float, default=2.0)
    parser.add_argument("--metrics-sample-hz", type=float, default=60.0)
    parser.add_argument("--metrics-report-interval-s", type=float, default=2.0)
    parser.add_argument("--metrics-min-samples", type=int, default=30)
    parser.add_argument("--metrics-min-deploy-fps", type=float, default=15.0)
    parser.add_argument("--metrics-max-joint-velocity-radps", type=float, default=35.0)
    parser.add_argument("--metrics-max-joint-velocity-p95-radps", type=float, default=25.0)
    parser.add_argument("--metrics-max-target-step-rad", type=float, default=1.25)
    parser.add_argument("--metrics-max-target-velocity-radps", type=float, default=45.0)
    parser.add_argument("--metrics-max-sim-sample-age-s", type=float, default=0.5)
    parser.add_argument("--metrics-max-foot-slip-speed-mps", type=float, default=8.0)
    parser.add_argument("--metrics-max-support-foot-drift-m", type=float, default=1.0)
    parser.add_argument("--metrics-foot-support-height-margin-m", type=float, default=0.04)
    parser.add_argument("--metrics-min-any-foot-contact-ratio", type=float, default=0.05)
    parser.add_argument("--metrics-max-no-foot-contact-ratio", type=float, default=0.95)
    parser.add_argument("--metrics-top-k-joints", type=int, default=5)
    parser.add_argument("--metrics-no-per-joint-jsonl", action="store_true")
    parser.add_argument("--metrics-summary-json", type=Path)
    parser.add_argument("--metrics-samples-jsonl", type=Path)

    parser.add_argument("--wait-after-mujoco", type=float, default=2.0)
    parser.add_argument("--wait-after-input", type=float, default=1.0)
    parser.add_argument("--wait-after-deploy", type=float, default=2.0)
    return parser


def _normalize_args(args: argparse.Namespace) -> None:
    args.repo_root = args.repo_root.expanduser().resolve()
    args.bvh_file = args.bvh_file.expanduser().resolve()
    args.json_file = args.json_file.expanduser().resolve()
    deploy_root = args.repo_root / "gear_sonic_deploy"
    if args.decoder is None:
        args.decoder = _resolve_release_file(args.repo_root, "policy/release/model_decoder.onnx")
    if args.encoder is None:
        args.encoder = _resolve_release_file(args.repo_root, "policy/release/model_encoder.onnx")
    if args.planner_file is None:
        args.planner_file = _resolve_release_file(args.repo_root, "planner/target_vel/V2/planner_sonic.onnx")
    if args.obs_config is None:
        args.obs_config = _resolve_release_file(args.repo_root, "policy/release/observation_config.yaml")
    if args.motion_data is None:
        args.motion_data = _resolve_release_file(args.repo_root, "reference/example")
    for name in ["decoder", "encoder", "planner_file", "obs_config", "motion_data"]:
        setattr(args, name, getattr(args, name).expanduser().resolve())
    if args.metrics_summary_json is None:
        args.metrics_summary_json = _default_metric_path("summary")
    else:
        args.metrics_summary_json = args.metrics_summary_json.expanduser().resolve()
    if args.metrics_samples_jsonl is None:
        args.metrics_samples_jsonl = _default_metric_path("samples")
    else:
        args.metrics_samples_jsonl = args.metrics_samples_jsonl.expanduser().resolve()
    args.deploy_bin = deploy_root / "target" / "release" / "g1_deploy_onnx_ref"


def _preflight(args: argparse.Namespace) -> None:
    errors: list[str] = []
    if shutil.which("tmux") is None:
        errors.append("tmux is not installed or not in PATH")
    if not (args.repo_root / "gear_sonic_deploy" / ".justfile").exists():
        errors.append(f"repo root does not look valid: {args.repo_root}")
    if not (args.repo_root / ".venv_teleop" / "bin" / "python").exists():
        errors.append(f"teleop venv python not found: {args.repo_root / '.venv_teleop/bin/python'}")
    if not (args.repo_root / ".venv_sim" / "bin" / "python").exists():
        errors.append(f"sim venv python not found: {args.repo_root / '.venv_sim/bin/python'}")
    if not args.deploy_bin.exists():
        errors.append(f"deploy binary not found: {args.deploy_bin}; run `cd gear_sonic_deploy && just build`")
    required_paths = ["decoder", "encoder", "planner_file", "obs_config", "motion_data"]
    if args.input_source == "bvh":
        required_paths.append("bvh_file")
    else:
        required_paths.append("json_file")
        if not (args.repo_root / "gear_sonic" / "scripts" / "sony_bonedata_json_stream_sender.py").exists():
            errors.append("Sony BoneData JSON sender script not found in repo")
    for path_name in required_paths:
        path = getattr(args, path_name)
        if not path.exists():
            errors.append(f"{path_name.replace('_', '-')} not found: {path}")
    if not args.ignore_port_check:
        ports_to_check = _ports_to_check(args)
        busy_ports = [port for port in ports_to_check if not _port_is_available(port)]
        if busy_ports:
            errors.append("required TCP port(s) already in use: " + ", ".join(str(port) for port in busy_ports))
    if errors:
        print("Preflight failed:", file=sys.stderr)
        for error in errors:
            print(f"  - {error}", file=sys.stderr)
        print("", file=sys.stderr)
        print("Use --dry-run to inspect commands, or fix the missing prerequisite before launching.", file=sys.stderr)
        sys.exit(2)


def _ports_to_check(args: argparse.Namespace) -> list[int]:
    ports_to_check = [args.zmq_port, args.debug_port]
    if not args.no_metrics and not args.no_sim_metrics:
        ports_to_check.append(args.sim_metrics_port)
    return sorted(set(int(port) for port in ports_to_check))


def _wait_for_cleanup(args: argparse.Namespace, timeout_s: float = 10.0) -> None:
    if args.ignore_port_check:
        return
    deadline = time.monotonic() + max(0.0, timeout_s)
    ports_to_check = _ports_to_check(args)
    while time.monotonic() < deadline:
        if all(_port_is_available(port) for port in ports_to_check):
            return
        time.sleep(0.25)


def _mujoco_command(args: argparse.Namespace) -> str:
    python = args.repo_root / ".venv_sim" / "bin" / "python"
    sim_args: list[str | Path | int] = [
        python,
        "-u",
        "gear_sonic/scripts/run_sim_loop.py",
        "--interface",
        args.interface,
        "--domain-id",
        args.domain_id,
        "--sim-frequency",
        args.sim_frequency,
        "--control-frequency",
        args.control_frequency,
    ]
    sim_args.append("--enable-onscreen" if args.onscreen else "--no-enable-onscreen")
    if args.offscreen:
        sim_args.append("--enable-offscreen")
    if args.no_mujoco_elastic_band:
        sim_args.append("--no-enable-elastic-band")
    if not args.no_metrics and not args.no_sim_metrics:
        sim_args.extend(
            [
                "--mujoco-metrics-zmq-bind",
                f"tcp://*:{args.sim_metrics_port}",
                "--mujoco-metrics-zmq-topic",
                args.sim_metrics_topic,
                "--mujoco-metrics-hz",
                args.sim_metrics_hz,
            ]
        )
    command = " && ".join(
        [
            f"cd {_quote(args.repo_root)}",
            "export PYTHONUNBUFFERED=1",
            f"export PYTHONPATH={_quote(args.repo_root)}:${{PYTHONPATH:-}}",
            " ".join(_quote(part) for part in sim_args),
        ]
    )
    return _with_log(command, "mujoco")


def _mocap_manager_command(args: argparse.Namespace) -> str:
    python = args.repo_root / ".venv_teleop" / "bin" / "python"
    mocap_args: list[str | Path | int | float] = [
        python,
        "-u",
        "gear_sonic/scripts/mocap_manager_server.py",
        "--source",
        "bvh_stream",
        "--bvh-stream-host",
        args.bvh_stream_host,
        "--bvh-stream-port",
        args.bvh_stream_port,
        "--bvh-stream-bonedata-coordinate-frame",
        args.bvh_stream_bonedata_coordinate_frame,
        "--bvh-stream-bonedata-position-scale",
        args.bvh_stream_bonedata_position_scale,
        "--bvh-stream-bonedata-input-quat-order",
        args.bvh_stream_bonedata_input_quat_order,
        "--bvh-stream-bonedata-rotation-mode",
        args.bvh_stream_bonedata_rotation_mode,
        "--control-mode",
        args.control_mode,
        "--pose-window-size",
        args.pose_window_size,
        "--pose-encoder-mode",
        args.pose_encoder_mode,
        "--pose-protocol-version",
        args.pose_protocol_version,
        "--bvh-g1-smpl-joints-source",
        args.bvh_g1_smpl_joints_source,
        "--pose-filter-profile",
        args.pose_filter_profile,
        "--bvh-g1-retarget-scale",
        args.bvh_g1_retarget_scale,
        "--bvh-g1-lower-scale",
        args.bvh_g1_lower_scale,
        "--bvh-g1-upper-scale",
        args.bvh_g1_upper_scale,
        "--bvh-g1-wrist-scale",
        args.bvh_g1_wrist_scale,
        "--bvh-g1-waist-scale",
        args.bvh_g1_waist_scale,
        "--bvh-g1-max-joint-velocity",
        args.bvh_g1_max_joint_velocity,
        "--bvh-g1-max-joint-step",
        args.bvh_g1_max_joint_step,
        "--bvh-g1-joint-filter-alpha",
        args.bvh_g1_joint_filter_alpha,
        "--bvh-g1-joint-delta-limit-scale",
        args.bvh_g1_joint_delta_limit_scale,
        "--bvh-g1-min-root-height",
        args.bvh_g1_min_root_height,
        "--zmq-port",
        args.zmq_port,
        "--log-interval-s",
        args.mocap_log_interval_s,
    ]
    if args.planner_from_root:
        mocap_args.extend(
            [
                "--planner-from-root",
                "--planner-root-speed-scale",
                args.planner_root_speed_scale,
                "--planner-root-speed-alpha",
                args.planner_root_speed_alpha,
                "--planner-root-speed-deadband",
                args.planner_root_speed_deadband,
                "--planner-root-speed-min",
                args.planner_root_speed_min,
                "--planner-root-speed-max",
                args.planner_root_speed_max,
                "--planner-root-locomotion-mode",
                args.planner_root_locomotion_mode,
            ]
        )
    if args.pose_protocol_version == 3 and args.pose_encoder_mode == "smpl":
        mocap_args.append("--allow-sony-pose-v3")
    if args.bvh_stream_bonedata_local_root:
        mocap_args.append("--bvh-stream-bonedata-local-root")
    if args.bvh_g1_no_align_root:
        mocap_args.append("--bvh-g1-no-align-root")
    if not args.no_root_yaw_only:
        mocap_args.append("--pose-root-yaw-only")
    if args.pose_encoder_mode == "teleop":
        mocap_args.append("--allow-teleop-pose-experiment")
    command = " && ".join(
        [
            f"cd {_quote(args.repo_root)}",
            "export PYTHONUNBUFFERED=1",
            f"export PYTHONPATH={_quote(args.repo_root)}:${{PYTHONPATH:-}}",
            " ".join(_quote(part) for part in mocap_args),
        ]
    )
    return _with_log(command, "input")


def _deploy_command(args: argparse.Namespace) -> str:
    deploy_root = args.repo_root / "gear_sonic_deploy"
    deploy_args: list[str | Path | int] = [
        "just",
        "run",
        "g1_deploy_onnx_ref",
        args.interface,
        _path_arg(args.decoder, deploy_root),
        _path_arg(args.motion_data, deploy_root),
        "--domain-id",
        args.domain_id,
        "--obs-config",
        _path_arg(args.obs_config, deploy_root),
        "--encoder-file",
        _path_arg(args.encoder, deploy_root),
        "--planner-file",
        _path_arg(args.planner_file, deploy_root),
        "--input-type",
        "zmq_manager",
        "--zmq-host",
        "localhost",
        "--zmq-port",
        args.zmq_port,
        "--zmq-topic",
        args.zmq_topic,
        "--output-type",
        "all",
        "--zmq-out-port",
        args.debug_port,
        "--zmq-out-topic",
        args.debug_topic,
        "--disable-crc-check",
    ]
    command = " && ".join(
        [
            f"cd {_quote(deploy_root)}",
            f"export DDS_INTERFACE={_quote(args.interface)}",
            "source scripts/setup_env.sh",
            " ".join(_quote(part) for part in deploy_args),
        ]
    )
    return _with_log(command, "deploy")


def _sender_window_name(args: argparse.Namespace) -> str:
    return "json_sender" if args.input_source == "sony_json" else "bvh_sender"


def _bvh_sender_command(args: argparse.Namespace) -> str:
    python = args.repo_root / ".venv_teleop" / "bin" / "python"
    if args.input_source == "sony_json":
        fps = args.sony_bonedata_fps if args.sony_bonedata_fps is not None else (args.bvh_fps or 50.0)
        sender_args: list[str | Path | int | float] = [
            python,
            "-u",
            "gear_sonic/scripts/sony_bonedata_json_stream_sender.py",
            "--json-file",
            args.json_file,
            "--host",
            args.bvh_stream_sender_host,
            "--port",
            args.bvh_stream_port,
            "--format",
            args.sony_bonedata_packet_format,
            "--fps",
            fps,
            "--joints-per-frame",
            args.sony_bonedata_joints_per_frame,
            "--start-frame",
            args.sony_bonedata_start_frame,
            "--log-interval-s",
            1,
        ]
        if args.sony_bonedata_source_fps is not None:
            sender_args.extend(["--source-fps", args.sony_bonedata_source_fps])
    else:
        sender_args = [
            python,
            "-u",
            "gear_sonic/scripts/bvh_stream_sender.py",
            "--bvh-file",
            args.bvh_file,
            "--host",
            args.bvh_stream_sender_host,
            "--port",
            args.bvh_stream_port,
            "--log-interval-s",
            1,
        ]
    if not args.no_bvh_loop:
        sender_args.append("--loop")
    if args.input_source == "bvh" and args.bvh_fps is not None:
        sender_args.extend(["--fps", args.bvh_fps])
    command_parts = [
        f"cd {_quote(args.repo_root)}",
        "export PYTHONUNBUFFERED=1",
        f"export PYTHONPATH={_quote(args.repo_root)}:${{PYTHONPATH:-}}",
        _wait_for_tcp_port_command("127.0.0.1", args.debug_port, args.bvh_wait_timeout_s, "deploy debug port"),
    ]
    if args.bvh_start_delay_s > 0.0:
        command_parts.append(f"sleep {float(args.bvh_start_delay_s):.3f}")
    command_parts.append(" ".join(_quote(part) for part in sender_args))
    return _with_log(" && ".join(command_parts), _sender_window_name(args))


def _metrics_command(args: argparse.Namespace) -> str:
    python = args.repo_root / ".venv_teleop" / "bin" / "python"
    metrics_args: list[str | Path | int | float] = [
        python,
        "-u",
        "gear_sonic/scripts/collect_sonic_mujoco_metrics.py",
        "--deploy-endpoint",
        f"tcp://127.0.0.1:{args.debug_port}",
        "--deploy-topic",
        args.debug_topic,
        "--duration-s",
        args.metrics_duration_s,
        "--startup-timeout-s",
        args.metrics_startup_timeout_s,
        "--warmup-s",
        args.metrics_warmup_s,
        "--sample-hz",
        args.metrics_sample_hz,
        "--report-interval-s",
        args.metrics_report_interval_s,
        "--min-samples",
        args.metrics_min_samples,
        "--min-deploy-fps",
        args.metrics_min_deploy_fps,
        "--max-joint-velocity-radps",
        args.metrics_max_joint_velocity_radps,
        "--max-joint-velocity-p95-radps",
        args.metrics_max_joint_velocity_p95_radps,
        "--max-target-step-rad",
        args.metrics_max_target_step_rad,
        "--max-target-velocity-radps",
        args.metrics_max_target_velocity_radps,
        "--max-sim-sample-age-s",
        args.metrics_max_sim_sample_age_s,
        "--max-foot-slip-speed-mps",
        args.metrics_max_foot_slip_speed_mps,
        "--max-support-foot-drift-m",
        args.metrics_max_support_foot_drift_m,
        "--foot-support-height-margin-m",
        args.metrics_foot_support_height_margin_m,
        "--min-any-foot-contact-ratio",
        args.metrics_min_any_foot_contact_ratio,
        "--max-no-foot-contact-ratio",
        args.metrics_max_no_foot_contact_ratio,
        "--top-k-joints",
        args.metrics_top_k_joints,
        "--summary-json",
        args.metrics_summary_json,
        "--samples-jsonl",
        args.metrics_samples_jsonl,
    ]
    if args.metrics_no_per_joint_jsonl:
        metrics_args.append("--no-per-joint-jsonl")
    if not args.no_sim_metrics:
        metrics_args.extend(
            [
                "--sim-endpoint",
                f"tcp://127.0.0.1:{args.sim_metrics_port}",
                "--sim-topic",
                args.sim_metrics_topic,
            ]
        )
    command = " && ".join(
        [
            f"cd {_quote(args.repo_root)}",
            "export PYTHONUNBUFFERED=1",
            f"export PYTHONPATH={_quote(args.repo_root)}:${{PYTHONPATH:-}}",
            " ".join(_quote(part) for part in metrics_args),
        ]
    )
    return _with_log(command, "metrics")


def _build_window_commands(args: argparse.Namespace) -> list[WindowCommand]:
    commands = [
        WindowCommand("mujoco", _mujoco_command(args)),
        WindowCommand("input", _mocap_manager_command(args)),
        WindowCommand("deploy", _deploy_command(args)),
        WindowCommand(_sender_window_name(args), _bvh_sender_command(args)),
    ]
    if not args.no_metrics:
        commands.append(WindowCommand("metrics", _metrics_command(args)))
    return commands


def _print_dry_run(commands: list[WindowCommand]) -> None:
    for item in commands:
        print(f"\n[{item.name}]")
        print(item.command)


def _launch_tmux(args: argparse.Namespace, commands: list[WindowCommand]) -> None:
    if _tmux_session_exists(args.session):
        if not args.replace:
            print(
                f"tmux session already exists: {args.session}\n"
                f"Use --replace to kill and recreate it.",
                file=sys.stderr,
            )
            sys.exit(2)
        _run(["tmux", "kill-session", "-t", args.session])
        _wait_for_cleanup(args)

    first, *rest = commands
    _run(["tmux", "new-session", "-d", "-s", args.session, "-n", first.name, _shell_window(first.command)])
    waits = {
        "mujoco": args.wait_after_mujoco,
        "input": args.wait_after_input,
        "deploy": args.wait_after_deploy,
    }
    previous = first
    for item in rest:
        wait_s = waits.get(previous.name, 0.0)
        if wait_s > 0:
            time.sleep(wait_s)
        _run(["tmux", "new-window", "-t", args.session, "-n", item.name, _shell_window(item.command)])
        previous = item

    _run(["tmux", "select-window", "-t", f"{args.session}:mujoco"])
    if args.no_attach:
        print(f"Started tmux session {args.session}.")
        print(f"Attach with: tmux attach -t {args.session}")
    else:
        _run(["tmux", "attach", "-t", args.session])


def main() -> int:
    parser = _build_parser()
    args = parser.parse_args()
    _normalize_args(args)
    commands = _build_window_commands(args)
    if args.dry_run:
        _print_dry_run(commands)
        return 0
    if args.replace and _tmux_session_exists(args.session):
        _run(["tmux", "kill-session", "-t", args.session])
        _wait_for_cleanup(args)
    _preflight(args)
    _launch_tmux(args, commands)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
