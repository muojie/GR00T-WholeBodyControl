#!/usr/bin/env python3
"""Launch the local SONIC + IsaacLab closed-loop stack in tmux.

This is the single-machine version of the Sony/IsaacLab/deploy bridge:

    BVH sender -> mocap manager :5556 -> deploy :5557 -> IsaacLab
    IsaacLab :5560 -> C++ LowState proxy -> DDS lowstate -> deploy

Defaults intentionally match the current split-machine validation commands, but
use loopback only:

    DDS interface: lo
    IsaacLab task: Isaac-SonicSolo-Locomanipulation-G1-v0
    deploy input: zmq_manager, topic pose, port 5556
    deploy debug: tcp://127.0.0.1:5557, topic g1_debug
    Isaac state: tcp://127.0.0.1:5560, topic sonic_state
"""

from __future__ import annotations

import argparse
import os
import shlex
import shutil
import socket
import subprocess
import sys
import time
from dataclasses import dataclass
from pathlib import Path


DEFAULT_SESSION = "sonic_local_isaaclab"
DEFAULT_REPO_ROOT = Path(__file__).resolve().parents[1]
DEFAULT_TASK = "Isaac-SonicSolo-Locomanipulation-G1-v0"


def _default_host_path(relative_path: Path) -> Path:
    home_path = Path.home() / relative_path
    if home_path.exists():
        return home_path
    checkout_sibling = DEFAULT_REPO_ROOT.parent / relative_path
    if checkout_sibling.exists():
        return checkout_sibling
    return home_path


DEFAULT_ISAACLAB_ROOT = _default_host_path(Path("xiaoyang_IssacLab") / "IsaacLab")
DEFAULT_BVH_FILE = _default_host_path(Path("RAYNOS_Motion1.bvh"))


@dataclass(frozen=True)
class WindowCommand:
    name: str
    command: str


def _quote(value: str | Path) -> str:
    return shlex.quote(str(value))


def _run(cmd: list[str]) -> None:
    subprocess.run(cmd, check=True)


def _capture(cmd: list[str]) -> str:
    return subprocess.run(cmd, check=True, capture_output=True, text=True).stdout.strip()


def _detect_conda_sh() -> Path:
    conda = shutil.which("conda")
    if conda:
        try:
            base = _capture([conda, "info", "--base"])
            candidate = Path(base) / "etc" / "profile.d" / "conda.sh"
            if candidate.exists():
                return candidate
        except subprocess.SubprocessError:
            pass
    return Path.home() / "miniconda3" / "etc" / "profile.d" / "conda.sh"


def _tmux_session_exists(session: str) -> bool:
    return subprocess.run(
        ["tmux", "has-session", "-t", session],
        stdout=subprocess.DEVNULL,
        stderr=subprocess.DEVNULL,
    ).returncode == 0


def _prepare_tmux_session(args: argparse.Namespace) -> None:
    if not _tmux_session_exists(args.session):
        return
    if not args.replace:
        print(
            f"tmux session already exists: {args.session}\n"
            f"Use --replace to kill and recreate it.",
            file=sys.stderr,
        )
        sys.exit(2)
    _run(["tmux", "kill-session", "-t", args.session])


def _port_is_available(port: int) -> bool:
    sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    try:
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
        'echo "[sonic-local] process exited with status ${status}"; '
        "exec bash"
    )
    return f"bash -lc {_quote(wrapped)}"


def _log_path(name: str) -> str:
    stamp = time.strftime("%Y%m%d_%H%M%S")
    safe_name = "".join(ch if ch.isalnum() or ch in "-_" else "_" for ch in name)
    return f"/tmp/sonic_local_{safe_name}_{stamp}.log"


def _with_log(command: str, name: str) -> str:
    return f"{command} |& tee {_quote(_log_path(name))}"


def _build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        description="Launch local IsaacLab + SONIC deploy closed-loop validation in tmux.",
        formatter_class=argparse.ArgumentDefaultsHelpFormatter,
    )
    parser.add_argument("--session", default=DEFAULT_SESSION, help="tmux session name")
    parser.add_argument("--replace", action="store_true", help="kill an existing tmux session first")
    parser.add_argument("--no-attach", action="store_true", help="create tmux session without attaching")
    parser.add_argument("--dry-run", action="store_true", help="print commands without launching tmux")
    parser.add_argument(
        "--ignore-port-check",
        action="store_true",
        help="do not fail if 5556/5557/5560 are already bound",
    )

    parser.add_argument("--repo-root", type=Path, default=DEFAULT_REPO_ROOT)
    parser.add_argument("--isaaclab-root", type=Path, default=DEFAULT_ISAACLAB_ROOT)
    parser.add_argument("--conda-sh", type=Path, default=_detect_conda_sh())
    parser.add_argument("--conda-env", default="env_isaaclab")

    parser.add_argument("--interface", default="lo", help="Unitree DDS interface used by proxy and deploy")
    parser.add_argument("--domain-id", type=int, default=0)

    parser.add_argument("--task", default=DEFAULT_TASK)
    parser.add_argument("--device", default="cpu")
    parser.add_argument(
        "--ask-isaaclab",
        action="store_true",
        help="ask whether to launch IsaacLab in this tmux session",
    )
    parser.add_argument(
        "--no-isaaclab",
        action="store_true",
        help="do not launch local IsaacLab; use an external IsaacLab sonic_state endpoint instead",
    )
    parser.add_argument(
        "--isaac-state-host",
        "--windows-ip",
        dest="isaac_state_host",
        default="127.0.0.1",
        help="host/IP where IsaacLab publishes sonic_state; use Windows IP with --no-isaaclab",
    )
    parser.add_argument(
        "--isaac-state-endpoint",
        help="full IsaacLab sonic_state endpoint override, for example tcp://192.168.1.20:5560",
    )
    parser.add_argument("--headless", action="store_true")
    parser.add_argument("--enable-pinocchio", action="store_true")
    parser.add_argument("--kit-arg", dest="extra_kit_args", action="append", default=[])

    parser.add_argument("--physics-mode", type=int, choices=[0, 1], default=1)
    parser.add_argument("--visual-servo-mode", type=int, choices=[0, 1], default=0)
    parser.add_argument("--self-collisions", type=int, choices=[0, 1], default=0)
    parser.add_argument("--stabilize-root", type=int, choices=[0, 1], default=1)
    parser.add_argument("--target-rate-limit", type=float, default=0.04)

    parser.add_argument("--zmq-port", type=int, default=5556, help="mocap manager -> deploy port")
    parser.add_argument("--debug-port", type=int, default=5557, help="deploy g1_debug ZMQ port")
    parser.add_argument("--state-port", type=int, default=5560, help="IsaacLab sonic_state ZMQ port")
    parser.add_argument("--zmq-topic", default="pose")
    parser.add_argument("--debug-topic", default="g1_debug")
    parser.add_argument("--state-topic", default="sonic_state")

    parser.add_argument("--bvh-file", type=Path, default=DEFAULT_BVH_FILE)
    parser.add_argument("--bvh-stream-host", default="0.0.0.0")
    parser.add_argument("--bvh-stream-port", type=int, default=12352)
    parser.add_argument("--bvh-stream-sender-host", default="127.0.0.1")
    parser.add_argument(
        "--ask-bvh-stream-sender",
        action="store_true",
        help="ask whether to launch bvh_stream_sender in tmux",
    )
    parser.add_argument(
        "--manual-bvh-stream-sender",
        action="store_true",
        help="do not launch bvh_stream_sender; print its command for a separate terminal",
    )
    parser.add_argument(
        "--no-bvh-stream-sender",
        action="store_true",
        help="do not launch bvh_stream_sender and do not print a manual command",
    )
    parser.add_argument("--no-bvh-loop", action="store_true")
    parser.add_argument("--bvh-fps", type=float)
    parser.add_argument("--pose-window-size", type=int, default=80)
    parser.add_argument("--mocap-log-interval-s", type=float, default=1.0)

    parser.add_argument("--lowstate-hz", type=float, default=500.0)
    parser.add_argument("--follow-alpha", type=float, default=0.35)
    parser.add_argument("--proxy-bin", type=Path)
    parser.add_argument("--sdk-root", type=Path)

    parser.add_argument("--decoder", type=Path, default=Path("policy/release/model_decoder.onnx"))
    parser.add_argument("--encoder", type=Path, default=Path("policy/release/model_encoder.onnx"))
    parser.add_argument("--planner-file", type=Path, default=Path("planner/target_vel/V2/planner_sonic.onnx"))
    parser.add_argument("--obs-config", type=Path, default=Path("policy/release/observation_config.yaml"))
    parser.add_argument("--motion-data", type=Path, default=Path("reference/example"))

    parser.add_argument("--wait-after-input", type=float, default=1.0)
    parser.add_argument("--wait-after-isaaclab", type=float, default=2.0)
    parser.add_argument("--wait-after-proxy", type=float, default=1.0)
    parser.add_argument("--wait-after-deploy", type=float, default=1.0)
    return parser


def _resolve_defaults(args: argparse.Namespace) -> None:
    args.repo_root = args.repo_root.expanduser().resolve()
    args.isaaclab_root = args.isaaclab_root.expanduser().resolve()
    args.conda_sh = args.conda_sh.expanduser().resolve()
    args.bvh_file = args.bvh_file.expanduser().resolve()

    if args.proxy_bin is None:
        args.proxy_bin = (
            args.repo_root
            / "gear_sonic_deploy"
            / "build"
            / "tools"
            / "sonic_unitree_lowstate_cpp_proxy"
        )
    args.proxy_bin = args.proxy_bin.expanduser().resolve()

    if args.sdk_root is None:
        args.sdk_root = args.repo_root / "gear_sonic_deploy" / "thirdparty" / "unitree_sdk2"
    args.sdk_root = args.sdk_root.expanduser().resolve()


def _preflight(args: argparse.Namespace) -> None:
    errors: list[str] = []

    if shutil.which("tmux") is None:
        errors.append("tmux is not installed or not in PATH")
    if not (args.repo_root / "gear_sonic_deploy" / ".justfile").exists():
        errors.append(f"repo root does not look valid: {args.repo_root}")
    if not args.no_isaaclab:
        if not (args.isaaclab_root / "isaaclab.sh").exists():
            errors.append(f"IsaacLab root does not contain isaaclab.sh: {args.isaaclab_root}")
        if not args.conda_sh.exists():
            errors.append(f"conda.sh not found: {args.conda_sh}")
    if not args.proxy_bin.exists():
        errors.append(f"C++ proxy binary not found: {args.proxy_bin}")
    if not (args.repo_root / ".venv_teleop" / "bin" / "python").exists():
        errors.append(f"teleop venv python not found: {args.repo_root / '.venv_teleop/bin/python'}")
    if _launches_bvh_stream_sender(args) and not args.bvh_file.exists():
        errors.append(f"BVH file not found: {args.bvh_file}")

    if not args.ignore_port_check:
        local_ports = [args.zmq_port, args.debug_port]
        if not args.no_isaaclab:
            local_ports.append(args.state_port)
        busy_ports = [port for port in local_ports if not _port_is_available(port)]
        if busy_ports:
            ports = ", ".join(str(port) for port in busy_ports)
            errors.append(f"required TCP port(s) already in use: {ports}")

    if errors:
        print("Preflight failed:", file=sys.stderr)
        for error in errors:
            print(f"  - {error}", file=sys.stderr)
        print("", file=sys.stderr)
        print("Use --dry-run to inspect commands, or stop old processes before relaunching.", file=sys.stderr)
        sys.exit(2)


def _prompt_yes_no(label: str, default: bool) -> bool:
    suffix = "Y/n" if default else "y/N"
    while True:
        raw = input(f"{label} [{suffix}]: ").strip().lower()
        if not raw:
            return default
        if raw in {"y", "yes", "1", "true", "on"}:
            return True
        if raw in {"n", "no", "0", "false", "off"}:
            return False
        print("Please enter y or n.")


def _configure_sender_mode(args: argparse.Namespace) -> None:
    explicit_modes = [
        args.manual_bvh_stream_sender,
        args.no_bvh_stream_sender,
    ]
    if sum(1 for enabled in explicit_modes if enabled) > 1:
        print(
            "ERROR: use only one of --manual-bvh-stream-sender or --no-bvh-stream-sender.",
            file=sys.stderr,
        )
        sys.exit(2)

    if not args.ask_bvh_stream_sender or args.manual_bvh_stream_sender or args.no_bvh_stream_sender:
        return

    if not sys.stdin.isatty():
        print(
            "ERROR: --ask-bvh-stream-sender requires an interactive terminal. "
            "Use --manual-bvh-stream-sender for non-interactive manual sender mode.",
            file=sys.stderr,
        )
        sys.exit(2)

    if not _prompt_yes_no("Launch bvh_stream_sender in this tmux session?", True):
        args.manual_bvh_stream_sender = True


def _configure_isaaclab_mode(args: argparse.Namespace) -> None:
    if not args.ask_isaaclab:
        return

    if not sys.stdin.isatty():
        print(
            "ERROR: --ask-isaaclab requires an interactive terminal. "
            "Use --no-isaaclab for non-interactive external IsaacLab mode.",
            file=sys.stderr,
        )
        sys.exit(2)

    if not _prompt_yes_no("Launch IsaacLab in this tmux session?", not args.no_isaaclab):
        args.no_isaaclab = True
        if not args.isaac_state_endpoint:
            raw = input(f"IsaacLab sonic_state host/IP [{args.isaac_state_host}]: ").strip()
            if raw:
                args.isaac_state_host = raw


def _launches_bvh_stream_sender(args: argparse.Namespace) -> bool:
    return not args.no_bvh_stream_sender and not args.manual_bvh_stream_sender


def _isaac_state_endpoint(args: argparse.Namespace) -> str:
    if args.isaac_state_endpoint:
        return args.isaac_state_endpoint
    return f"tcp://{args.isaac_state_host}:{args.state_port}"


def _path_arg(path: Path, base: Path) -> str:
    try:
        return str(path.relative_to(base))
    except ValueError:
        return str(path)


def _isaaclab_command(args: argparse.Namespace) -> str:
    kit_args = [
        "--/app/vsync=false",
        "--/app/runLoops/main/rateLimitEnabled=false",
        *args.extra_kit_args,
    ]
    isaac_args = [
        "./isaaclab.sh",
        "-p",
        "scripts/environments/teleoperation/teleop_se3_agent.py",
        "--task",
        args.task,
        "--device",
        args.device,
        "--kit_args",
        " ".join(kit_args),
    ]
    if args.headless:
        isaac_args.append("--headless")
    if args.enable_pinocchio:
        isaac_args.append("--enable_pinocchio")

    command = " && ".join(
        [
            f"cd {_quote(args.isaaclab_root)}",
            f"source {_quote(args.conda_sh)}",
            f"conda activate {_quote(args.conda_env)}",
            "export PYTHONUNBUFFERED=1",
            f"export UNITREE_DDS_INTERFACE={_quote(args.interface)}",
            f"export UNITREE_DDS_DOMAIN_ID={_quote(args.domain_id)}",
            "export SONIC_DEPLOY_TRANSPORT=zmq",
            f"export SONIC_DEPLOY_ENDPOINT={_quote(f'tcp://127.0.0.1:{args.debug_port}')}",
            f"export SONIC_DEPLOY_TOPIC={_quote(args.debug_topic)}",
            "export SONIC_DEPLOY_TARGET_FIELD=last_action",
            "export SONIC_DEPLOY_REFERENCE_TARGET_FIELD=body_q_target",
            "export SONIC_PUBLISH_STATE_ZMQ=1",
            f"export SONIC_STATE_ZMQ_BIND={_quote(f'tcp://*:{args.state_port}')}",
            f"export SONIC_STATE_ZMQ_TOPIC={_quote(args.state_topic)}",
            f"export SONIC_G1_PHYSICS_MODE={_quote(args.physics_mode)}",
            f"export SONIC_G1_VISUAL_SERVO_MODE={_quote(args.visual_servo_mode)}",
            f"export SONIC_G1_SELF_COLLISIONS={_quote(args.self_collisions)}",
            f"export SONIC_DEPLOY_STABILIZE_ROOT={_quote(args.stabilize_root)}",
            f"export SONIC_DEPLOY_TARGET_RATE_LIMIT={_quote(args.target_rate_limit)}",
            " ".join(_quote(part) for part in isaac_args),
        ]
    )
    return _with_log(command, "isaaclab")


def _proxy_command(args: argparse.Namespace) -> str:
    proxy_args = [
        _quote(args.proxy_bin),
        "--interface",
        '"$DDS_INTERFACE"',
        "--domain-id",
        _quote(args.domain_id),
        "--lowstate-hz",
        _quote(args.lowstate_hz),
        "--follow-alpha",
        _quote(args.follow_alpha),
        "--isaac-state-endpoint",
        _quote(_isaac_state_endpoint(args)),
        "--isaac-state-topic",
        _quote(args.state_topic),
    ]
    command = " && ".join(
        [
            f"cd {_quote(args.repo_root)}",
            f"export DDS_INTERFACE={_quote(args.interface)}",
            f"export SDK={_quote(args.sdk_root)}",
            'export LD_LIBRARY_PATH="$SDK/thirdparty/lib/$(uname -m):$SDK/lib/$(uname -m):${LD_LIBRARY_PATH:-}"',
            " ".join(proxy_args),
        ]
    )
    return _with_log(command, "proxy")


def _deploy_command(args: argparse.Namespace) -> str:
    deploy_root = args.repo_root / "gear_sonic_deploy"
    deploy_args = [
        "just",
        "run",
        "g1_deploy_onnx_ref",
        '"$DDS_INTERFACE"',
        _quote(_path_arg(args.decoder, deploy_root)),
        _quote(_path_arg(args.motion_data, deploy_root)),
        "--obs-config",
        _quote(_path_arg(args.obs_config, deploy_root)),
        "--encoder-file",
        _quote(_path_arg(args.encoder, deploy_root)),
        "--planner-file",
        _quote(_path_arg(args.planner_file, deploy_root)),
        "--input-type",
        "zmq_manager",
        "--zmq-host",
        "localhost",
        "--zmq-port",
        _quote(args.zmq_port),
        "--zmq-topic",
        _quote(args.zmq_topic),
        "--output-type",
        "all",
        "--disable-crc-check",
    ]
    command = " && ".join(
        [
            f"cd {_quote(deploy_root)}",
            f"export DDS_INTERFACE={_quote(args.interface)}",
            "source scripts/setup_env.sh",
            " ".join(deploy_args),
        ]
    )
    return _with_log(command, "deploy")


def _mocap_manager_command(args: argparse.Namespace) -> str:
    python = args.repo_root / ".venv_teleop" / "bin" / "python"
    mocap_args = [
        python,
        "-u",
        "gear_sonic/scripts/mocap_manager_server.py",
        "--source",
        "bvh_stream",
        "--bvh-stream-host",
        args.bvh_stream_host,
        "--bvh-stream-port",
        str(args.bvh_stream_port),
        "--control-mode",
        "pose",
        "--pose-window-size",
        str(args.pose_window_size),
        "--pose-encoder-mode",
        "g1",
        "--pose-protocol-version",
        "1",
        "--zmq-port",
        str(args.zmq_port),
        "--log-interval-s",
        str(args.mocap_log_interval_s),
    ]
    command = " && ".join(
        [
            f"cd {_quote(args.repo_root)}",
            "export PYTHONUNBUFFERED=1",
            " ".join(_quote(part) for part in mocap_args),
        ]
    )
    return _with_log(command, "input")


def _bvh_sender_command(args: argparse.Namespace) -> str:
    python = args.repo_root / ".venv_teleop" / "bin" / "python"
    sender_args: list[str | Path] = [
        python,
        "-u",
        "gear_sonic/scripts/bvh_stream_sender.py",
        "--bvh-file",
        args.bvh_file,
        "--host",
        args.bvh_stream_sender_host,
        "--port",
        str(args.bvh_stream_port),
        "--log-interval-s",
        "1",
    ]
    if not args.no_bvh_loop:
        sender_args.append("--loop")
    if args.bvh_fps is not None:
        sender_args.extend(["--fps", str(args.bvh_fps)])

    command = " && ".join(
        [
            f"cd {_quote(args.repo_root)}",
            "export PYTHONUNBUFFERED=1",
            " ".join(_quote(part) for part in sender_args),
        ]
    )
    return _with_log(command, "bvh_sender")


def _build_window_commands(args: argparse.Namespace) -> list[WindowCommand]:
    commands = [WindowCommand("input", _mocap_manager_command(args))]
    if not args.no_isaaclab:
        commands.append(WindowCommand("isaaclab", _isaaclab_command(args)))
    commands.extend(
        [
            WindowCommand("proxy", _proxy_command(args)),
            WindowCommand("deploy", _deploy_command(args)),
        ]
    )
    if _launches_bvh_stream_sender(args):
        commands.append(WindowCommand("bvh_sender", _bvh_sender_command(args)))
    return commands


def _print_manual_sender_command(args: argparse.Namespace) -> None:
    if not args.manual_bvh_stream_sender:
        return
    print("\nManual bvh_stream_sender command:")
    print(_bvh_sender_command(args))


def _print_dry_run(args: argparse.Namespace, commands: list[WindowCommand]) -> None:
    for item in commands:
        print(f"\n[{item.name}]")
        print(item.command)
    _print_manual_sender_command(args)


def _launch_tmux(args: argparse.Namespace, commands: list[WindowCommand]) -> None:
    first, *rest = commands
    _run(["tmux", "new-session", "-d", "-s", args.session, "-n", first.name, _shell_window(first.command)])

    waits = {
        "input": args.wait_after_input,
        "isaaclab": args.wait_after_isaaclab,
        "proxy": args.wait_after_proxy,
        "deploy": args.wait_after_deploy,
    }
    for item in rest:
        wait_s = waits.get(commands[commands.index(item) - 1].name, 0.0)
        if wait_s > 0:
            time.sleep(wait_s)
        _run(["tmux", "new-window", "-t", args.session, "-n", item.name, _shell_window(item.command)])

    focus_window = "isaaclab" if not args.no_isaaclab else "proxy"
    _run(["tmux", "select-window", "-t", f"{args.session}:{focus_window}"])
    print(f"Started tmux session: {args.session}")
    print("Windows: " + ", ".join(item.name for item in commands))
    if args.no_isaaclab:
        print(f"External IsaacLab sonic_state endpoint: {_isaac_state_endpoint(args)}")
    _print_manual_sender_command(args)
    if not args.no_attach:
        if args.manual_bvh_stream_sender and sys.stdin.isatty():
            input("\nPress Enter to attach tmux after copying the sender command...")
        os.execvp("tmux", ["tmux", "attach-session", "-t", args.session])


def main() -> None:
    parser = _build_parser()
    args = parser.parse_args()
    _resolve_defaults(args)
    _configure_isaaclab_mode(args)
    _configure_sender_mode(args)
    commands = _build_window_commands(args)

    if args.dry_run:
        _print_dry_run(args, commands)
        return

    _prepare_tmux_session(args)
    _preflight(args)
    _launch_tmux(args, commands)


if __name__ == "__main__":
    main()
