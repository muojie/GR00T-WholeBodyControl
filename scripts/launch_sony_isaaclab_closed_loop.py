#!/usr/bin/env python3
"""Launch a teleop input -> SONIC deploy closed-loop stack in tmux.

Run from anywhere:

    ~/tools/sony-isaaclab-sonic-launcher/launch_sony_isaaclab_closed_loop.py

The launcher asks which simulator backend to use:

    IsaacLab: IsaacLab SonicSolo + C++ LowState proxy + deploy + mocap
    MuJoCo:   MuJoCo sim loop + deploy + mocap
    None:     C++ LowState proxy + deploy + mocap; IsaacLab is started manually
              (remember SONIC_PUBLISH_STATE_ZMQ=1 in the manual IsaacLab shell)

It also asks which upstream input source / deploy input type to use:

    sony: BVH input via mocap_manager_server.py; choose bvh_g1, bvh_stream, or bvh
    pico: PICO input via pico_manager_thread_server.py
    keyboard/gamepad/manager/etc: deploy-side input interface; no extra input pane
"""

from __future__ import annotations

import argparse
import os
import platform
import signal
import shlex
import shutil
import subprocess
import sys
import time
from collections.abc import Callable
from dataclasses import dataclass
from pathlib import Path


BACKENDS = ("isaaclab", "mujoco", "none")
EXTERNAL_INPUT_SOURCES = ("sony", "pico")
DEPLOY_ONLY_INPUT_SOURCES = ("keyboard", "gamepad", "gamepad_manager", "manager", "zmq", "zmq_manager")
INPUT_SOURCES = EXTERNAL_INPUT_SOURCES + DEPLOY_ONLY_INPUT_SOURCES
DEPLOY_INPUT_TYPES = DEPLOY_ONLY_INPUT_SOURCES
BVH_SOURCES = ("bvh_g1", "bvh_stream", "bvh")
SONY_POSE_LINES = ("v1", "v3")
SONY_V3_SMPL_JOINTS_SOURCES = ("g1_fk", "skeleton")
DEFAULT_SESSION = "sony_sonic"
FALLBACK_REPO_ROOT = Path.home() / "GR00T-WholeBodyControl"
FALLBACK_ISAACLAB_ROOT = Path.home() / "xiaoyang_IssacLab" / "IsaacLab"
DEFAULT_BVH_FILE = Path.home() / "RAYNOS_Motion1.bvh"
DEFAULT_PICO_SERVICE_SCRIPT = Path("/opt/apps/roboticsservice/runService.sh")
ISAAC_SCENE_PROFILES: dict[str, dict[str, object]] = {
    "solo": {
        "task": "Isaac-SonicSolo-Locomanipulation-G1-v0",
        "device": "cpu",
        "enable_pinocchio": False,
        "headless": False,
        "description": "1 sonic robot + ground/light; fastest closed-loop physics debug",
    },
    "fullscene": {
        "task": "Isaac-SonicFullscene-Locomanipulation-G1-v0",
        "device": "cpu",
        "enable_pinocchio": False,
        "headless": False,
        "description": "1 sonic robot + warehouse; current real-time walking scene",
    },
    "fullmid": {
        "task": "Isaac-SonicFullMid-Locomanipulation-G1-v0",
        "device": "cpu",
        "enable_pinocchio": True,
        "headless": False,
        "fullmid_companions": 1,
        "description": "sonic + fixed companion G1 robots; non-real-time scene inspection",
    },
    "fullmulti": {
        "task": "Isaac-SonicFullMulti-Locomanipulation-G1-v0",
        "device": "cuda:0",
        "enable_pinocchio": True,
        "headless": False,
        "description": "4 G1 full scene; use GPU, non-real-time visual/pose inspection",
    },
    "pickplace": {
        "task": "Isaac-PickPlace-Locomanipulation-G1-Abs-v0",
        "device": "cuda:0",
        "enable_pinocchio": True,
        "headless": False,
        "description": "original PickPlace Locomanipulation host scene",
    },
}
ISAAC_SCENE_CHOICES = tuple(ISAAC_SCENE_PROFILES) + ("custom",)


@dataclass(frozen=True)
class ProcessInfo:
    pid: int
    ppid: int
    pgid: int
    stat: str
    command: str


@dataclass(frozen=True)
class CleanupCandidate:
    process: ProcessInfo
    reason: str


def _quote(value: str | Path) -> str:
    return shlex.quote(str(value))


def _run(cmd: list[str], *, capture: bool = False) -> str:
    result = subprocess.run(
        cmd,
        check=True,
        capture_output=capture,
        text=True,
    )
    return result.stdout.strip() if capture else ""


def _tmux_session_exists(session_name: str) -> bool:
    return subprocess.run(
        ["tmux", "has-session", "-t", session_name],
        stdout=subprocess.DEVNULL,
        stderr=subprocess.DEVNULL,
    ).returncode == 0


def _looks_like_repo_root(path: Path) -> bool:
    return (
        (path / "gear_sonic_deploy" / ".justfile").exists()
        and (path / "gear_sonic" / "scripts" / "mocap_manager_server.py").exists()
    )


def _looks_like_isaaclab_root(path: Path) -> bool:
    return (path / "isaaclab.sh").exists()


def _first_existing_marker(candidates: list[Path], marker: Callable[[Path], bool]) -> Path | None:
    seen: set[str] = set()
    for raw_path in candidates:
        path = raw_path.expanduser()
        key = str(path)
        if key in seen:
            continue
        seen.add(key)
        if marker(path):
            return path
    return None


def _cwd_and_parents() -> list[Path]:
    cwd = Path.cwd()
    return [cwd, *cwd.parents]


def _env_path(env_names: tuple[str, ...]) -> Path | None:
    for name in env_names:
        value = os.environ.get(name)
        if value:
            return Path(value).expanduser()
    return None


def _default_repo_root() -> Path:
    env_root = _env_path(("GROOT_REPO_ROOT", "SONIC_REPO_ROOT", "GR00T_REPO_ROOT"))
    if env_root is not None:
        return env_root

    home = Path.home()
    candidates = [
        *_cwd_and_parents(),
        FALLBACK_REPO_ROOT,
        home / "workspace" / "GR00T-WholeBodyControl",
        home / "code" / "GR00T-WholeBodyControl",
        home / "src" / "GR00T-WholeBodyControl",
        home / "projects" / "GR00T-WholeBodyControl",
    ]
    return _first_existing_marker(candidates, _looks_like_repo_root) or FALLBACK_REPO_ROOT


def _default_isaaclab_root() -> Path:
    env_root = _env_path(("ISAACLAB_ROOT", "ISAAC_LAB_ROOT"))
    if env_root is not None:
        return env_root

    home = Path.home()
    candidates = [
        *_cwd_and_parents(),
        FALLBACK_ISAACLAB_ROOT,
        home / "IsaacLab",
        home / "isaaclab" / "IsaacLab",
        home / "workspace" / "IsaacLab",
        home / "code" / "IsaacLab",
        home / "src" / "IsaacLab",
        home / "projects" / "IsaacLab",
    ]
    return _first_existing_marker(candidates, _looks_like_isaaclab_root) or FALLBACK_ISAACLAB_ROOT


def _detect_conda_sh() -> Path:
    conda_exe = shutil.which("conda")
    if conda_exe:
        try:
            base = subprocess.run(
                [conda_exe, "info", "--base"],
                check=True,
                capture_output=True,
                text=True,
            ).stdout.strip()
            candidate = Path(base) / "etc" / "profile.d" / "conda.sh"
            if candidate.exists():
                return candidate
        except subprocess.SubprocessError:
            pass
    return Path.home() / "miniconda3" / "etc" / "profile.d" / "conda.sh"


def _default_sdk_root(repo_root: Path) -> Path:
    if os.environ.get("SDK"):
        return Path(os.environ["SDK"]).expanduser()
    return repo_root / "gear_sonic_deploy" / "thirdparty" / "unitree_sdk2"


def _resolve_path(
    path: Path,
    *,
    base: Path | None = None,
    repo_root: Path | None = None,
) -> Path:
    path = path.expanduser()
    if path.is_absolute():
        if path.exists():
            return path.resolve()
        if base is not None and repo_root is not None:
            try:
                rel = path.relative_to(repo_root)
            except ValueError:
                return path.resolve()
            legacy_fallback = base / rel
            if legacy_fallback.exists():
                return legacy_fallback.resolve()
        return path.resolve()
    if path.exists():
        return path.resolve()
    if base is not None and (base / path).exists():
        return (base / path).resolve()
    return path.resolve()


def _build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        description="Launch a teleop input + SONIC deploy stack with either IsaacLab or MuJoCo in tmux.",
        formatter_class=argparse.ArgumentDefaultsHelpFormatter,
    )
    parser.add_argument(
        "--backend",
        choices=BACKENDS,
        help="simulator backend; omit in an interactive shell to choose at startup",
    )
    parser.add_argument(
        "--input-source",
        choices=INPUT_SOURCES,
        help=(
            "teleop input source; sony/pico start input pane(s), while keyboard/gamepad/gamepad_manager/"
            "manager/zmq/zmq_manager select a deploy-side --input-type"
        ),
    )
    parser.add_argument(
        "--deploy-input-type",
        choices=DEPLOY_INPUT_TYPES,
        help="advanced override for deploy --input-type; defaults from --input-source",
    )
    parser.add_argument("--session", default=DEFAULT_SESSION, help="tmux session name")
    parser.add_argument("--replace", action="store_true", help="kill an existing session with the same name first")
    parser.add_argument("--no-attach", action="store_true", help="create the session but do not attach")
    parser.add_argument("--dry-run", action="store_true", help="print pane commands without creating tmux session")
    parser.add_argument(
        "--cleanup",
        action="store_true",
        help="kill the tmux session and launcher-related leftover processes, then exit",
    )
    parser.add_argument(
        "--cleanup-dry-run",
        action="store_true",
        help="show what --cleanup would kill, then exit",
    )

    parser.add_argument(
        "--repo-root",
        type=Path,
        default=_default_repo_root(),
        help="GR00T-WholeBodyControl repo root; default auto-detects cwd/parents, then common paths",
    )
    parser.add_argument("--interface", default="enp4s0", help="Unitree DDS interface for IsaacLab/proxy mode")
    parser.add_argument("--mujoco-interface", default="lo", help="Unitree DDS interface for MuJoCo sim mode")
    parser.add_argument("--domain-id", type=int, default=0, help="Unitree DDS domain id")
    parser.add_argument(
        "--isaaclab-root",
        type=Path,
        default=_default_isaaclab_root(),
        help="IsaacLab repo root; default auto-detects cwd/parents, then common paths",
    )
    parser.add_argument("--conda-env", default="env_isaaclab", help="conda env used by IsaacLab")
    parser.add_argument("--conda-sh", type=Path, default=_detect_conda_sh(), help="path to conda.sh")

    parser.add_argument(
        "--bvh-source",
        choices=BVH_SOURCES,
        default="bvh_g1",
        help="BVH source mode used when --input-source sony",
    )
    parser.add_argument(
        "--sony-pose-line",
        choices=SONY_POSE_LINES,
        help=(
            "Sony/BVH POSE line. v1 uses protocol v1 + encoder_mode=g1; "
            "v3 uses protocol v3 + encoder_mode=smpl. Defaults to v1 for "
            "bvh_g1/bvh_stream and v3 for bvh."
        ),
    )
    parser.add_argument(
        "--bvh-g1-smpl-joints-source",
        choices=SONY_V3_SMPL_JOINTS_SOURCES,
        default="g1_fk",
        help=(
            "SMPL-like joints source for Sony POSE v3 with bvh_g1/bvh_stream. "
            "g1_fk projects the validated v1 G1 FK keypoints; skeleton uses the old raw BVH skeleton path."
        ),
    )
    parser.add_argument("--bvh-file", type=Path, default=DEFAULT_BVH_FILE)
    parser.add_argument("--no-bvh-loop", action="store_true", help="do not loop BVH playback/streaming")
    parser.add_argument("--bvh-fps", type=float, help="target BVH FPS for bvh/bvh_g1 playback or bvh_stream sender")
    parser.add_argument("--bvh-stream-host", default="0.0.0.0", help="bind host for mocap manager --source bvh_stream")
    parser.add_argument("--bvh-stream-port", type=int, default=12352, help="UDP port for BVH stream manager/sender")
    parser.add_argument(
        "--bvh-stream-sender-host",
        default="127.0.0.1",
        help="destination host used by bvh_stream_sender.py",
    )
    parser.add_argument(
        "--no-bvh-stream-sender",
        action="store_true",
        help="with --bvh-source bvh_stream, start only the manager and wait for an external sender",
    )
    parser.add_argument("--pose-window-size", type=int, default=80)
    parser.add_argument("--zmq-port", type=int, default=5556, help="input source -> deploy ZMQ port")
    parser.add_argument("--mocap-log-interval-s", type=float, default=1.0)
    parser.add_argument("--pico-target-fps", type=int, default=50, help="PICO streamer target FPS")
    parser.add_argument("--pico-buffer-size", type=int, default=15, help="PICO streamer sliding buffer size")
    parser.add_argument("--pico-vis-vr3pt", action="store_true", help="enable PICO VR3PT visualization")
    parser.add_argument("--pico-vis-smpl", action="store_true", help="enable PICO SMPL visualization")
    parser.add_argument("--pico-waist-tracking", action="store_true", help="enable PICO waist tracking")
    parser.add_argument(
        "--pico-service-script",
        type=Path,
        default=DEFAULT_PICO_SERVICE_SCRIPT,
        help="XRoboToolkit PC Service launcher required by PICO input",
    )
    parser.add_argument(
        "--require-pico-service-running",
        action="store_true",
        help="fail preflight unless RoboticsServiceProcess is already running",
    )
    parser.add_argument("--zmq-topic", default="pose", help="deploy ZMQ topic/prefix for zmq-style input types")
    parser.add_argument("--zmq-conflate", action="store_true", help="enable deploy-side ZMQ CONFLATE")

    parser.add_argument("--proxy-bin", type=Path, default=Path.home() / "bin" / "sonic_unitree_lowstate_cpp_proxy")
    parser.add_argument(
        "--sdk-root",
        type=Path,
        help="Unitree SDK root for proxy libraries; defaults to $SDK or <repo-root>/gear_sonic_deploy/thirdparty/unitree_sdk2",
    )
    parser.add_argument("--lowstate-hz", type=float, default=500.0)
    parser.add_argument("--follow-alpha", type=float, default=0.35)
    parser.add_argument("--isaac-state-endpoint", default="tcp://127.0.0.1:5560")
    parser.add_argument("--isaac-state-topic", default="sonic_state")

    parser.add_argument(
        "--isaac-scene",
        choices=ISAAC_SCENE_CHOICES,
        help="predefined IsaacLab scene profile; choose custom with --isaac-task for an arbitrary task",
    )
    parser.add_argument("--isaac-task", help="override IsaacLab task id; also used for custom scene")
    parser.add_argument("--isaac-device", help="IsaacLab --device; defaults come from the selected scene profile")
    parser.add_argument("--headless", dest="isaac_headless", action="store_true", default=None)
    parser.add_argument("--no-headless", dest="isaac_headless", action="store_false")
    parser.add_argument("--enable-pinocchio", dest="enable_pinocchio", action="store_true", default=None)
    parser.add_argument("--no-enable-pinocchio", dest="enable_pinocchio", action="store_false")
    parser.add_argument(
        "--fullmid-companions",
        type=int,
        help="SONIC_FULLMID_COMPANIONS for the fullmid IsaacLab scene",
    )
    parser.add_argument("--physics-mode", type=int, choices=[0, 1], default=1)
    parser.add_argument("--visual-servo-mode", type=int, choices=[0, 1], default=0)
    parser.add_argument("--self-collisions", type=int, choices=[0, 1], default=0)
    parser.add_argument("--stabilize-root", type=int, choices=[0, 1], default=1)
    parser.add_argument("--target-rate-limit", type=float, default=0.04)
    parser.add_argument(
        "--kit-arg",
        dest="extra_kit_args",
        action="append",
        default=[],
        help="extra Isaac/Kit argument appended inside --kit_args; may be repeated",
    )

    parser.add_argument("--decoder", type=Path, default=Path("policy") / "release" / "model_decoder.onnx")
    parser.add_argument("--encoder", type=Path, default=Path("policy") / "release" / "model_encoder.onnx")
    parser.add_argument("--planner-file", type=Path, default=Path("planner") / "target_vel" / "V2" / "planner_sonic.onnx")
    parser.add_argument("--obs-config", type=Path, default=Path("policy") / "release" / "observation_config.yaml")
    parser.add_argument("--motion-data", type=Path, default=Path("reference") / "example")

    parser.add_argument("--wait-after-isaaclab", type=float, default=2.0)
    parser.add_argument("--wait-after-mujoco", type=float, default=3.0)
    parser.add_argument("--wait-after-proxy", type=float, default=2.0)
    parser.add_argument("--wait-after-deploy", type=float, default=2.0)
    return parser


def _choose_backend(backend: str | None) -> str:
    if backend:
        return backend

    if not sys.stdin.isatty():
        print("ERROR: --backend is required when stdin is not interactive.")
        print("  Choose one of: isaaclab, mujoco, none")
        sys.exit(2)

    print("Select simulator backend:")
    print("  1) IsaacLab  - IsaacLab SonicSolo + C++ LowState proxy")
    print("  2) MuJoCo    - MuJoCo sim loop, no IsaacLab/proxy")
    print("  3) None      - proxy + deploy only; you start IsaacLab manually")
    while True:
        choice = input("Choice [1/2/3] (default: 1): ").strip().lower()
        if choice in ("", "1", "isaaclab", "i"):
            return "isaaclab"
        if choice in ("2", "mujoco", "m"):
            return "mujoco"
        if choice in ("3", "none", "n"):
            return "none"
        print("Please enter 1/isaaclab, 2/mujoco, or 3/none.")


def _choose_input_source(input_source: str | None) -> str:
    if input_source:
        return input_source

    if not sys.stdin.isatty():
        return "sony"

    print("Select input source:")
    print("  1) Sony/BVH    - mocap_manager_server.py; choose bvh_g1/bvh_stream/bvh")
    print("  2) PICO        - pico_manager_thread_server.py --manager")
    print("  3) Keyboard    - deploy --input-type keyboard; no input pane")
    print("  4) Gamepad     - deploy --input-type gamepad; no input pane")
    print("  5) Manager     - deploy --input-type manager; switch with Shift+1/2/3/4")
    print("  6) ZMQ         - deploy --input-type zmq; external ZMQ pose source")
    print("  7) ZMQ manager - deploy --input-type zmq_manager; external manager source")
    while True:
        choice = input("Choice [1-7] (default: 1): ").strip().lower()
        if choice in ("", "1", "sony", "s"):
            return "sony"
        if choice in ("2", "pico", "p"):
            return "pico"
        if choice in ("3", "keyboard", "key", "k"):
            return "keyboard"
        if choice in ("4", "gamepad", "g"):
            return "gamepad"
        if choice in ("5", "manager", "m"):
            return "manager"
        if choice in ("6", "zmq", "z"):
            return "zmq"
        if choice in ("7", "zmq_manager", "zmq-manager", "zm"):
            return "zmq_manager"
        print("Please enter a listed number or source name.")


def _prompt_choice(title: str, choices: list[tuple[str, str]], default: str) -> str:
    by_index = {str(index): key for index, (key, _) in enumerate(choices, start=1)}
    valid_keys = {key for key, _ in choices}
    print(title)
    for index, (key, label) in enumerate(choices, start=1):
        default_marker = " [default]" if key == default else ""
        print(f"  {index}) {label}{default_marker}")
    while True:
        choice = input(f"Choice [default: {default}]: ").strip().lower()
        if not choice:
            return default
        if choice in by_index:
            return by_index[choice]
        if choice in valid_keys:
            return choice
        print("Please enter a listed number or key.")


def _prompt_bool(label: str, default: bool) -> bool:
    suffix = "Y/n" if default else "y/N"
    while True:
        raw = input(f"{label} [{suffix}]: ").strip().lower()
        if not raw:
            return default
        if raw in ("y", "yes", "1", "true", "on"):
            return True
        if raw in ("n", "no", "0", "false", "off"):
            return False
        print("Please enter y or n.")


def _prompt_text(label: str, default: str | None) -> str:
    shown = default if default is not None else ""
    raw = input(f"{label} [{shown}]: ").strip()
    return raw or shown


def _prompt_int(label: str, default: int) -> int:
    while True:
        raw = input(f"{label} [{default}]: ").strip()
        if not raw:
            return default
        try:
            return int(raw)
        except ValueError:
            print("Please enter an integer.")


def _prompt_float(label: str, default: float) -> float:
    while True:
        raw = input(f"{label} [{default:g}]: ").strip()
        if not raw:
            return default
        try:
            return float(raw)
        except ValueError:
            print("Please enter a number.")


def _configure_isaaclab_options(args: argparse.Namespace, *, prompt_scene: bool) -> None:
    if args.backend != "isaaclab":
        return

    scene = args.isaac_scene
    if args.isaac_task and not scene:
        scene = "custom"

    if prompt_scene and not scene:
        scene = _prompt_choice(
            "Select IsaacLab scene:",
            [
                (key, f"{key}: {profile['task']} - {profile['description']}")
                for key, profile in ISAAC_SCENE_PROFILES.items()
            ]
            + [("custom", "custom: type an arbitrary IsaacLab task id")],
            "solo",
        )

    if not scene:
        scene = "solo"
    args.isaac_scene = scene

    profile = ISAAC_SCENE_PROFILES.get(scene, {})
    if not args.isaac_task:
        if scene == "custom":
            if not sys.stdin.isatty():
                print("ERROR: --isaac-task is required with --isaac-scene custom in a non-interactive shell.")
                sys.exit(2)
            args.isaac_task = _prompt_text("IsaacLab task id", None)
        else:
            args.isaac_task = str(profile["task"])

    if not args.isaac_device:
        args.isaac_device = str(profile.get("device", "cpu"))
    if args.enable_pinocchio is None:
        args.enable_pinocchio = bool(profile.get("enable_pinocchio", False))
    if args.isaac_headless is None:
        args.isaac_headless = bool(profile.get("headless", False))
    if args.fullmid_companions is None and scene == "fullmid":
        args.fullmid_companions = int(profile.get("fullmid_companions", 1))

    if not prompt_scene:
        return

    if _prompt_bool("Adjust advanced IsaacLab options?", False):
        args.isaac_device = _prompt_text("IsaacLab --device", args.isaac_device)
        args.isaac_headless = _prompt_bool("Run IsaacLab headless", bool(args.isaac_headless))
        args.enable_pinocchio = _prompt_bool("Pass --enable_pinocchio", bool(args.enable_pinocchio))
        args.physics_mode = 1 if _prompt_bool("SONIC_G1_PHYSICS_MODE", bool(args.physics_mode)) else 0
        args.visual_servo_mode = 1 if _prompt_bool("SONIC_G1_VISUAL_SERVO_MODE", bool(args.visual_servo_mode)) else 0
        args.self_collisions = 1 if _prompt_bool("SONIC_G1_SELF_COLLISIONS", bool(args.self_collisions)) else 0
        args.stabilize_root = 1 if _prompt_bool("SONIC_DEPLOY_STABILIZE_ROOT", bool(args.stabilize_root)) else 0
        args.target_rate_limit = _prompt_float("SONIC_DEPLOY_TARGET_RATE_LIMIT", args.target_rate_limit)
        if scene == "fullmid":
            args.fullmid_companions = _prompt_int(
                "SONIC_FULLMID_COMPANIONS", int(args.fullmid_companions or 1)
            )


def _configure_input_source_options(args: argparse.Namespace, *, prompt_input_options: bool) -> None:
    if not prompt_input_options:
        return

    if args.input_source == "sony":
        args.bvh_source = _prompt_choice(
            "Select BVH source mode:",
            [
                ("bvh_g1", "bvh_g1: single-process BVH-to-G1 POSE v1 regression"),
                ("bvh_stream", "bvh_stream: manager listens on UDP; sender streams the BVH file"),
                ("bvh", "bvh: legacy local BVH playback / VR3PT / SMPL debug"),
            ],
            args.bvh_source,
        )
        if args.bvh_source == "bvh_stream":
            args.no_bvh_stream_sender = not _prompt_bool(
                "Start local bvh_stream_sender.py",
                not args.no_bvh_stream_sender,
            )
        args.sony_pose_line = _prompt_choice(
            "Select Sony POSE line:",
            [
                ("v1", "v1: G1 joint reference, protocol v1 + encoder_mode=g1"),
                ("v3", "v3: SMPL-like reference, protocol v3 + encoder_mode=smpl"),
            ],
            args.sony_pose_line or ("v3" if args.bvh_source == "bvh" else "v1"),
        )
        if _prompt_bool("Adjust Sony/BVH input options?", False):
            if _requires_bvh_file(args):
                args.bvh_file = Path(_prompt_text("BVH file", str(args.bvh_file)))
                args.no_bvh_loop = not _prompt_bool("Loop BVH playback", not args.no_bvh_loop)
                current_fps = "" if args.bvh_fps is None else f"{args.bvh_fps:g}"
                fps_text = _prompt_text("BVH target FPS (empty = source/default)", current_fps)
                args.bvh_fps = float(fps_text) if fps_text else None
            if args.bvh_source == "bvh_stream":
                args.bvh_stream_port = _prompt_int("BVH stream UDP port", args.bvh_stream_port)
                if _starts_bvh_stream_sender(args):
                    args.bvh_stream_sender_host = _prompt_text(
                        "BVH stream sender destination host", args.bvh_stream_sender_host
                    )
            if args.sony_pose_line == "v3" and args.bvh_source in {"bvh_g1", "bvh_stream"}:
                args.bvh_g1_smpl_joints_source = _prompt_choice(
                    "Select Sony v3 SMPL joints source:",
                    [
                        ("g1_fk", "g1_fk: project validated v1 G1 FK keypoints"),
                        ("skeleton", "skeleton: old raw BVH skeleton path for A/B comparison"),
                    ],
                    args.bvh_g1_smpl_joints_source,
                )
            args.pose_window_size = _prompt_int("POSE window size", args.pose_window_size)
            args.mocap_log_interval_s = _prompt_float("Mocap log interval seconds", args.mocap_log_interval_s)
    elif args.input_source == "pico":
        if _prompt_bool("Adjust PICO input options?", False):
            args.pico_target_fps = _prompt_int("PICO target FPS", args.pico_target_fps)
            args.pico_buffer_size = _prompt_int("PICO buffer size", args.pico_buffer_size)
            args.pico_vis_vr3pt = _prompt_bool("Enable --vis_vr3pt", args.pico_vis_vr3pt)
            args.pico_vis_smpl = _prompt_bool("Enable --vis_smpl", args.pico_vis_smpl)
            args.pico_waist_tracking = _prompt_bool("Enable --waist_tracking", args.pico_waist_tracking)
            args.pico_service_script = Path(_prompt_text("PICO service script", str(args.pico_service_script)))
            args.require_pico_service_running = _prompt_bool(
                "Require RoboticsServiceProcess already running", args.require_pico_service_running
            )


def _input_pane_specs(args: argparse.Namespace) -> list[tuple[str, str]]:
    if args.input_source == "sony":
        if args.bvh_source == "bvh_stream":
            panes = [("input", "mocap bvh_stream")]
            if _starts_bvh_stream_sender(args):
                panes.append(("bvh_sender", "bvh stream sender"))
            return panes
        return [("input", f"mocap {args.bvh_source}")]
    if args.input_source == "pico":
        return [("input", "input pico")]
    return []


def _starts_bvh_stream_sender(args: argparse.Namespace) -> bool:
    return (
        args.input_source == "sony"
        and args.bvh_source == "bvh_stream"
        and not args.no_bvh_stream_sender
    )


def _requires_bvh_file(args: argparse.Namespace) -> bool:
    return args.input_source == "sony" and (
        args.bvh_source in {"bvh_g1", "bvh"} or _starts_bvh_stream_sender(args)
    )


def _default_deploy_input_type(input_source: str) -> str:
    if input_source in EXTERNAL_INPUT_SOURCES:
        return "zmq_manager"
    return input_source


def _configure_deploy_input_type(args: argparse.Namespace) -> None:
    default_input_type = _default_deploy_input_type(args.input_source)
    if args.deploy_input_type is None:
        args.deploy_input_type = default_input_type
        return

    if args.input_source in EXTERNAL_INPUT_SOURCES and args.deploy_input_type != "zmq_manager":
        print(
            f"ERROR: --input-source {args.input_source} requires --deploy-input-type zmq_manager "
            f"because the input pane publishes manager ZMQ messages."
        )
        sys.exit(2)


def _configure_sony_pose_line(args: argparse.Namespace) -> None:
    if args.input_source != "sony":
        return
    if args.sony_pose_line is None:
        args.sony_pose_line = "v3" if args.bvh_source == "bvh" else "v1"
    if args.bvh_source == "bvh" and args.sony_pose_line != "v3":
        print("ERROR: --bvh-source bvh is the SMPL debug source and requires --sony-pose-line v3.")
        sys.exit(2)


def _path_under(path: Path | None, root: Path) -> bool:
    if path is None:
        return False
    try:
        path.relative_to(root)
        return True
    except ValueError:
        return False


def _process_cwd(pid: int) -> Path | None:
    try:
        return Path(os.readlink(f"/proc/{pid}/cwd")).resolve()
    except OSError:
        return None


def _list_processes() -> list[ProcessInfo]:
    result = subprocess.run(
        ["ps", "-eo", "pid=,ppid=,pgid=,stat=,args="],
        check=True,
        capture_output=True,
        text=True,
    )
    processes: list[ProcessInfo] = []
    for raw_line in result.stdout.splitlines():
        parts = raw_line.strip().split(None, 4)
        if len(parts) < 5:
            continue
        pid_raw, ppid_raw, pgid_raw, stat, command = parts
        try:
            processes.append(
                ProcessInfo(
                    pid=int(pid_raw),
                    ppid=int(ppid_raw),
                    pgid=int(pgid_raw),
                    stat=stat,
                    command=command,
                )
            )
        except ValueError:
            continue
    return processes


def _pico_service_process_running() -> bool:
    return any("RoboticsServiceProcess" in process.command for process in _list_processes())


def _python_can_import_xrobotoolkit(python: Path, *, cwd: Path) -> bool:
    try:
        result = subprocess.run(
            [
                str(python),
                "-c",
                (
                    "import importlib.util, sys; "
                    "sys.exit(0 if importlib.util.find_spec('xrobotoolkit_sdk') else 1)"
                ),
            ],
            stdout=subprocess.DEVNULL,
            stderr=subprocess.DEVNULL,
            cwd=cwd,
        )
    except OSError:
        return False
    return result.returncode == 0


def _find_cleanup_candidates(args: argparse.Namespace) -> list[CleanupCandidate]:
    current_pid = os.getpid()
    repo_root = args.repo_root
    deploy_dir = args.deploy_dir
    isaaclab_root = args.isaaclab_root
    proxy_bin = args.proxy_bin
    candidates: dict[int, CleanupCandidate] = {}

    def add(process: ProcessInfo, reason: str) -> None:
        if process.pid == current_pid:
            return
        candidates.setdefault(process.pid, CleanupCandidate(process, reason))

    for process in _list_processes():
        command = process.command
        cwd = _process_cwd(process.pid)
        in_repo = _path_under(cwd, repo_root) or str(repo_root) in command
        in_deploy = _path_under(cwd, deploy_dir) or str(deploy_dir) in command
        in_isaaclab = _path_under(cwd, isaaclab_root) or str(isaaclab_root) in command

        if "gear_sonic/scripts/run_sim_loop.py" in command and in_repo:
            add(process, "MuJoCo sim loop")
        elif "gear_sonic/scripts/mocap_manager_server.py" in command and in_repo:
            add(process, "BVH/Sony mocap manager")
        elif "gear_sonic/scripts/bvh_stream_sender.py" in command and in_repo:
            add(process, "BVH stream sender")
        elif "gear_sonic/scripts/pico_manager_thread_server.py" in command and in_repo:
            add(process, "PICO manager")
        elif "g1_deploy_onnx_ref" in command and in_deploy:
            add(process, "SONIC deploy")
        elif "teleop_se3_agent.py" in command and in_isaaclab:
            add(process, "IsaacLab teleop scene")
        elif str(proxy_bin) in command or (
            proxy_bin.name in command and "--isaac-state-endpoint" in command
        ):
            add(process, "C++ LowState proxy")

    return sorted(candidates.values(), key=lambda item: item.process.pid)


def _process_alive(pid: int) -> bool:
    try:
        os.kill(pid, 0)
    except ProcessLookupError:
        return False
    except PermissionError:
        return True
    return True


def _terminate_candidates(candidates: list[CleanupCandidate]) -> None:
    for candidate in candidates:
        process = candidate.process
        try:
            os.kill(process.pid, signal.SIGTERM)
            print(f"Sent TERM to PID {process.pid}: {candidate.reason}")
        except ProcessLookupError:
            print(f"Already exited PID {process.pid}: {candidate.reason}")
        except PermissionError:
            print(f"Permission denied sending TERM to PID {process.pid}: {candidate.reason}")

    time.sleep(1.0)
    for candidate in candidates:
        process = candidate.process
        if not _process_alive(process.pid):
            continue
        try:
            os.kill(process.pid, signal.SIGKILL)
            print(f"Sent KILL to PID {process.pid}: {candidate.reason}")
        except ProcessLookupError:
            pass
        except PermissionError:
            print(f"Permission denied sending KILL to PID {process.pid}: {candidate.reason}")


def _cleanup(args: argparse.Namespace, *, dry_run: bool) -> None:
    if _tmux_session_exists(args.session):
        if dry_run:
            print(f"Would kill tmux session: {args.session}")
        else:
            _run(["tmux", "kill-session", "-t", args.session])
            print(f"Killed tmux session: {args.session}")
            time.sleep(0.5)
    else:
        print(f"No tmux session found: {args.session}")

    candidates = _find_cleanup_candidates(args)
    if not candidates:
        print("No launcher-related leftover processes found.")
        return

    print("Launcher-related leftover processes:")
    for candidate in candidates:
        process = candidate.process
        print(
            f"  PID {process.pid} pgid={process.pgid} stat={process.stat} "
            f"{candidate.reason}: {process.command}"
        )

    if dry_run:
        print()
        print("Dry run only; nothing was killed.")
        return

    _terminate_candidates(candidates)


def _preflight(args: argparse.Namespace) -> None:
    arch = platform.machine()
    checks: list[tuple[bool, str]] = [
        (shutil.which("tmux") is not None, "tmux is not installed"),
        (shutil.which("just") is not None, "just is not installed"),
        (
            args.repo_root.exists(),
            f"repo root not found: {args.repo_root} "
            "(set --repo-root or GROOT_REPO_ROOT/SONIC_REPO_ROOT/GR00T_REPO_ROOT)",
        ),
        ((args.deploy_dir / ".justfile").exists(), f"deploy justfile not found: {args.deploy_dir / '.justfile'}"),
        ((args.deploy_dir / "scripts" / "setup_env.sh").exists(), "gear_sonic_deploy/scripts/setup_env.sh not found"),
        ((args.deploy_dir / "target" / "release" / "g1_deploy_onnx_ref").exists(), "g1_deploy_onnx_ref is not built"),
        (args.decoder.exists(), f"decoder ONNX not found: {args.decoder}"),
        (args.encoder.exists(), f"encoder ONNX not found: {args.encoder}"),
        (args.planner_file.exists(), f"planner ONNX not found: {args.planner_file}"),
        (args.obs_config.exists(), f"observation config not found: {args.obs_config}"),
        (args.motion_data.exists(), f"motion data path not found: {args.motion_data}"),
    ]
    if args.input_source == "sony":
        checks.extend(
            [
                ((args.repo_root / ".venv_teleop" / "bin" / "python").exists(), ".venv_teleop Python not found"),
                (
                    (args.repo_root / "gear_sonic" / "scripts" / "mocap_manager_server.py").exists(),
                    "gear_sonic/scripts/mocap_manager_server.py not found",
                ),
            ]
        )
        if _requires_bvh_file(args):
            checks.append((args.bvh_file.exists(), f"BVH file not found: {args.bvh_file}"))
        if _starts_bvh_stream_sender(args):
            checks.append(
                (
                    (args.repo_root / "gear_sonic" / "scripts" / "bvh_stream_sender.py").exists(),
                    "gear_sonic/scripts/bvh_stream_sender.py not found",
                )
            )
    else:
        if args.input_source == "pico":
            teleop_python = args.repo_root / ".venv_teleop" / "bin" / "python"
            pico_service_binary = args.pico_service_script.parent / "RoboticsServiceProcess"
            checks.extend(
                [
                    (teleop_python.exists(), ".venv_teleop Python not found"),
                    (
                        (args.repo_root / "gear_sonic" / "scripts" / "pico_manager_thread_server.py").exists(),
                        "gear_sonic/scripts/pico_manager_thread_server.py not found",
                    ),
                    (
                        args.pico_service_script.exists(),
                        f"PICO XRoboToolkit service script not found: {args.pico_service_script}",
                    ),
                    (
                        os.access(args.pico_service_script, os.X_OK),
                        f"PICO XRoboToolkit service script is not executable: {args.pico_service_script}",
                    ),
                    (
                        pico_service_binary.exists(),
                        f"PICO RoboticsServiceProcess not found: {pico_service_binary}",
                    ),
                    (
                        os.access(pico_service_binary, os.X_OK),
                        f"PICO RoboticsServiceProcess is not executable: {pico_service_binary}",
                    ),
                    (
                        _python_can_import_xrobotoolkit(teleop_python, cwd=args.repo_root),
                        "xrobotoolkit_sdk is not importable from .venv_teleop",
                    ),
                ]
            )
            if args.require_pico_service_running:
                checks.append(
                    (
                        _pico_service_process_running(),
                        "PICO XRoboToolkit service process is not running: RoboticsServiceProcess",
                    )
                )
    if args.backend == "isaaclab":
        checks.extend(
            [
                (args.conda_sh.exists(), f"conda.sh not found: {args.conda_sh}"),
                (
                    args.isaaclab_root.exists(),
                    f"IsaacLab root not found: {args.isaaclab_root} "
                    "(set --isaaclab-root or ISAACLAB_ROOT)",
                ),
                (
                    (args.isaaclab_root / "isaaclab.sh").exists(),
                    f"isaaclab.sh not found under {args.isaaclab_root}",
                ),
            ]
        )
    if args.backend in ("isaaclab", "none"):
        checks.extend(
            [
                (
                    args.proxy_bin.exists() and os.access(args.proxy_bin, os.X_OK),
                    f"proxy binary is not executable: {args.proxy_bin}",
                ),
                (
                    (args.sdk_root / "thirdparty" / "lib" / arch).exists(),
                    f"Unitree SDK thirdparty lib dir not found: {args.sdk_root / 'thirdparty' / 'lib' / arch}",
                ),
            ]
        )
    if args.backend == "mujoco":
        checks.extend(
            [
                ((args.repo_root / ".venv_sim" / "bin" / "python").exists(), ".venv_sim Python not found"),
                (
                    (args.repo_root / "gear_sonic" / "scripts" / "run_sim_loop.py").exists(),
                    "gear_sonic/scripts/run_sim_loop.py not found",
                ),
            ]
        )
    errors = [message for ok, message in checks if not ok]
    if errors:
        print("ERROR: preflight failed:")
        for error in errors:
            print(f"  - {error}")
        sys.exit(1)
    if args.input_source == "pico" and not _pico_service_process_running():
        print(
            "WARN: RoboticsServiceProcess is not running; "
            f"pico_manager_thread_server.py will try to start {args.pico_service_script}. "
            "Use --require-pico-service-running to make this a hard preflight error."
        )


def _pane_commands(args: argparse.Namespace) -> dict[str, str]:
    arch = platform.machine()
    kit_args = "--/app/vsync=false --/app/runLoops/main/rateLimitEnabled=false"

    isaaclab_parts = [
        f"cd {_quote(args.isaaclab_root)}",
        f"source {_quote(args.conda_sh)}",
        f"conda activate {_quote(args.conda_env)}",
        # CUDA 库冲突防护（同 IsaacLab/scripts/start_ubuntu_isaaclab_sonic.sh，勿删）：
        # ~/.bashrc 把系统 CUDA 12.5 塞进 LD_LIBRARY_PATH，其 libnvJitLink.so.12 缺
        # __nvJitLinkCreate_12_8，torch(cu128) 的 libcusparse 一加载就崩 Kit（undefined
        # symbol）。仅本次启动生效、不动 bashrc：① 剔除 cuda-12.5 ② 预加载 env 自带 12.8。
        r'''export LD_LIBRARY_PATH="$(printf '%s' "${LD_LIBRARY_PATH:-}" | tr ':' '\n' | grep -v 'cuda-12\.5' | paste -sd: || true)"''',
        r'''{ NVJITLINK="$(ls "${CONDA_PREFIX:-$HOME/miniconda3/envs/env_isaaclab}"/lib/python*/site-packages/nvidia/nvjitlink/lib/libnvJitLink.so.12 2>/dev/null | head -1)"; [ -n "$NVJITLINK" ] && export LD_PRELOAD="$NVJITLINK${LD_PRELOAD:+:$LD_PRELOAD}"; true; }''',
        "export PYTHONUNBUFFERED=1",
        f"export UNITREE_DDS_INTERFACE={_quote(args.interface)}",
        f"export UNITREE_DDS_DOMAIN_ID={args.domain_id}",
        f"export SONIC_G1_PHYSICS_MODE={args.physics_mode}",
        f"export SONIC_G1_VISUAL_SERVO_MODE={args.visual_servo_mode}",
        f"export SONIC_G1_SELF_COLLISIONS={args.self_collisions}",
        f"export SONIC_DEPLOY_STABILIZE_ROOT={args.stabilize_root}",
        f"export SONIC_DEPLOY_TARGET_RATE_LIMIT={args.target_rate_limit}",
        "export SONIC_PUBLISH_STATE_ZMQ=1",
    ]
    if args.fullmid_companions is not None:
        isaaclab_parts.append(f"export SONIC_FULLMID_COMPANIONS={args.fullmid_companions}")
    isaaclab_parts.append(_isaaclab_launch_command(args, kit_args))
    isaaclab_cmd = " && ".join(isaaclab_parts)

    mujoco_cmd = " && ".join(
        [
            f"cd {_quote(args.repo_root)}",
            "source .venv_sim/bin/activate",
            "export PYTHONUNBUFFERED=1",
            (
                "python -u gear_sonic/scripts/run_sim_loop.py "
                f"--interface {_quote(args.mujoco_interface)} "
                "|& tee /tmp/mujoco_sonic_$(date +%H%M).log"
            ),
        ]
    )

    proxy_cmd = " && ".join(
        [
            f"export SDK={_quote(args.sdk_root)}",
            f"export LD_LIBRARY_PATH=\"$SDK/thirdparty/lib/{arch}:$SDK/lib/{arch}:${{LD_LIBRARY_PATH:-}}\"",
            (
                f"{_quote(args.proxy_bin)} "
                f"--interface {_quote(args.interface)} "
                f"--domain-id {args.domain_id} "
                f"--lowstate-hz {args.lowstate_hz:g} "
                f"--follow-alpha {args.follow_alpha:g} "
                f"--isaac-state-endpoint {_quote(args.isaac_state_endpoint)} "
                f"--isaac-state-topic {_quote(args.isaac_state_topic)} "
                "|& tee /tmp/sonic_proxy_$(date +%H%M).log"
            ),
        ]
    )

    deploy_cmd = " && ".join(
        [
            f"cd {_quote(args.deploy_dir)}",
            "source scripts/setup_env.sh",
            (
                "just run g1_deploy_onnx_ref "
                f"{_quote(args.deploy_interface)} "
                f"{_quote(args.decoder.relative_to(args.deploy_dir) if args.decoder.is_relative_to(args.deploy_dir) else args.decoder)} "
                f"{_quote(args.motion_data.relative_to(args.deploy_dir) if args.motion_data.is_relative_to(args.deploy_dir) else args.motion_data)} "
                f"--obs-config {_quote(args.obs_config.relative_to(args.deploy_dir) if args.obs_config.is_relative_to(args.deploy_dir) else args.obs_config)} "
                f"--encoder-file {_quote(args.encoder.relative_to(args.deploy_dir) if args.encoder.is_relative_to(args.deploy_dir) else args.encoder)} "
                f"--planner-file {_quote(args.planner_file.relative_to(args.deploy_dir) if args.planner_file.is_relative_to(args.deploy_dir) else args.planner_file)} "
                f"--input-type {_quote(args.deploy_input_type)} "
                "--zmq-host localhost "
                f"--zmq-port {args.zmq_port} "
                f"--zmq-topic {_quote(args.zmq_topic)} "
                f"{'--zmq-conflate ' if args.zmq_conflate else ''}"
                "--output-type all "
                "--disable-crc-check "
                "|& tee /tmp/sonic_deploy_$(date +%H%M).log"
            ),
        ]
    )

    input_commands = _input_source_commands(args)

    if args.backend == "isaaclab":
        commands = {
            "isaaclab": isaaclab_cmd,
            "proxy": proxy_cmd,
            "deploy": deploy_cmd,
        }
        commands.update(input_commands)
        return commands

    if args.backend == "none":
        commands = {
            "proxy": proxy_cmd,
            "deploy": deploy_cmd,
        }
        commands.update(input_commands)
        return commands

    commands = {
        "mujoco": mujoco_cmd,
        "deploy": deploy_cmd,
    }
    commands.update(input_commands)
    return commands


def _input_source_commands(args: argparse.Namespace) -> dict[str, str]:
    if args.input_source == "pico":
        parts = [
            "PYTHONUNBUFFERED=1 .venv_teleop/bin/python -u",
            "gear_sonic/scripts/pico_manager_thread_server.py",
            "--manager",
            "--port",
            str(args.zmq_port),
            "--target_fps",
            str(args.pico_target_fps),
            "--buffer_size",
            str(args.pico_buffer_size),
        ]
        if args.pico_vis_vr3pt:
            parts.append("--vis_vr3pt")
        if args.pico_vis_smpl:
            parts.append("--vis_smpl")
        if args.pico_waist_tracking:
            parts.append("--waist_tracking")
        return {"input": " && ".join([f"cd {_quote(args.repo_root)}", " ".join(parts)])}

    if args.input_source != "sony":
        return {}

    commands = {"input": _sony_bvh_manager_command(args)}
    if _starts_bvh_stream_sender(args):
        commands["bvh_sender"] = _bvh_stream_sender_command(args)
    return commands


def _optional_bvh_fps_arg(args: argparse.Namespace) -> str:
    return "" if args.bvh_fps is None else f"--bvh-fps {args.bvh_fps:g} "


def _optional_sender_fps_arg(args: argparse.Namespace) -> str:
    return "" if args.bvh_fps is None else f"--fps {args.bvh_fps:g} "


def _sony_pose_args(args: argparse.Namespace) -> str:
    if args.sony_pose_line == "v1":
        return "--pose-encoder-mode g1 --pose-protocol-version 1 "
    if args.sony_pose_line == "v3":
        extra = "--allow-sony-pose-v3 " if args.bvh_source in {"bvh_g1", "bvh_stream"} else ""
        if args.bvh_source in {"bvh_g1", "bvh_stream"}:
            extra += f"--bvh-g1-smpl-joints-source {args.bvh_g1_smpl_joints_source} "
        return f"--pose-encoder-mode smpl --pose-protocol-version 3 {extra}"
    raise ValueError(f"unsupported Sony POSE line: {args.sony_pose_line}")


def _sony_bvh_manager_command(args: argparse.Namespace) -> str:
    bvh_loop = "" if args.no_bvh_loop else "--bvh-loop "
    bvh_fps = _optional_bvh_fps_arg(args)
    pose_args = _sony_pose_args(args)

    if args.bvh_source == "bvh_g1":
        manager_args = (
            "gear_sonic/scripts/mocap_manager_server.py "
            "--source bvh_g1 "
            f"--bvh-file {_quote(args.bvh_file)} "
            f"{bvh_loop}"
            f"{bvh_fps}"
            "--control-mode pose "
            f"--pose-window-size {args.pose_window_size} "
            f"{pose_args}"
            f"--zmq-port {args.zmq_port} "
            f"--log-interval-s {args.mocap_log_interval_s:g}"
        )
    elif args.bvh_source == "bvh_stream":
        manager_args = (
            "gear_sonic/scripts/mocap_manager_server.py "
            "--source bvh_stream "
            f"--bvh-stream-host {_quote(args.bvh_stream_host)} "
            f"--bvh-stream-port {args.bvh_stream_port} "
            "--control-mode pose "
            f"--pose-window-size {args.pose_window_size} "
            f"{pose_args}"
            f"--zmq-port {args.zmq_port} "
            f"--log-interval-s {args.mocap_log_interval_s:g}"
        )
    elif args.bvh_source == "bvh":
        manager_args = (
            "gear_sonic/scripts/mocap_manager_server.py "
            "--source bvh "
            f"--bvh-file {_quote(args.bvh_file)} "
            f"{bvh_loop}"
            f"{bvh_fps}"
            "--control-mode pose "
            f"--pose-window-size {args.pose_window_size} "
            f"{pose_args}"
            f"--zmq-port {args.zmq_port} "
            f"--log-interval-s {args.mocap_log_interval_s:g}"
        )
    else:
        raise ValueError(f"unsupported BVH source: {args.bvh_source}")

    return " && ".join(
        [
            f"cd {_quote(args.repo_root)}",
            f"PYTHONUNBUFFERED=1 .venv_teleop/bin/python -u {manager_args}",
        ]
    )


def _bvh_stream_sender_command(args: argparse.Namespace) -> str:
    loop = "" if args.no_bvh_loop else "--loop "
    fps = _optional_sender_fps_arg(args)
    return " && ".join(
        [
            f"cd {_quote(args.repo_root)}",
            (
                "PYTHONUNBUFFERED=1 .venv_teleop/bin/python -u "
                "gear_sonic/scripts/bvh_stream_sender.py "
                f"--bvh-file {_quote(args.bvh_file)} "
                f"--host {_quote(args.bvh_stream_sender_host)} "
                f"--port {args.bvh_stream_port} "
                f"{loop}"
                f"{fps}"
                f"--log-interval-s {args.mocap_log_interval_s:g}"
            ),
        ]
    )


def _isaaclab_launch_command(args: argparse.Namespace, kit_args: str) -> str:
    if args.extra_kit_args:
        kit_args = " ".join([kit_args, *args.extra_kit_args])
    parts = [
        "./isaaclab.sh",
        "-p",
        "scripts/environments/teleoperation/teleop_se3_agent.py",
        "--task",
        _quote(args.isaac_task),
    ]
    if args.enable_pinocchio:
        parts.append("--enable_pinocchio")
    if args.isaac_headless:
        parts.append("--headless")
    if args.isaac_device:
        parts.extend(["--device", _quote(args.isaac_device)])
    parts.extend(["--kit_args", _quote(kit_args)])
    return " ".join(parts) + " |& tee /tmp/isaac_sonic_$(date +%H%M).log"


def _create_tmux(args: argparse.Namespace) -> dict[str, str]:
    if _tmux_session_exists(args.session):
        if not args.replace:
            print(f"ERROR: tmux session already exists: {args.session}")
            print(f"  Reattach: tmux attach -t {args.session}")
            print(f"  Replace:  {_quote(Path(__file__).resolve())} --replace")
            sys.exit(1)
        _run(["tmux", "kill-session", "-t", args.session])

    pane_first = _run(
        ["tmux", "new-session", "-d", "-s", args.session, "-n", "closed_loop", "-P", "-F", "#{pane_id}"],
        capture=True,
    )

    _run(["tmux", "set-option", "-t", args.session, "-g", "mouse", "on"])
    _run(["tmux", "set-option", "-t", args.session, "-g", "pane-border-status", "top"])
    _run(["tmux", "bind-key", "-T", "root", "C-\\", "kill-session"])

    input_pane_specs = _input_pane_specs(args)

    if args.backend == "isaaclab":
        pane_proxy = _run(["tmux", "split-window", "-h", "-t", pane_first, "-P", "-F", "#{pane_id}"], capture=True)
        pane_deploy = _run(["tmux", "split-window", "-v", "-t", pane_first, "-P", "-F", "#{pane_id}"], capture=True)
        pane_titles = [
            (pane_first, "1 IsaacLab"),
            (pane_proxy, "2 LowState proxy"),
            (pane_deploy, "3 SONIC deploy"),
        ]
        panes = {
            "isaaclab": pane_first,
            "proxy": pane_proxy,
            "deploy": pane_deploy,
        }
        split_target = pane_proxy
        for index, (key, title) in enumerate(input_pane_specs, start=4):
            pane_input = _run(["tmux", "split-window", "-v", "-t", split_target, "-P", "-F", "#{pane_id}"], capture=True)
            pane_titles.append((pane_input, f"{index} {title}"))
            panes[key] = pane_input
            split_target = pane_input
    elif args.backend == "none":
        pane_deploy = _run(["tmux", "split-window", "-h", "-t", pane_first, "-P", "-F", "#{pane_id}"], capture=True)
        pane_titles = [
            (pane_first, "1 LowState proxy"),
            (pane_deploy, "2 SONIC deploy"),
        ]
        panes = {
            "proxy": pane_first,
            "deploy": pane_deploy,
        }
        split_target = pane_deploy
        for index, (key, title) in enumerate(input_pane_specs, start=3):
            pane_input = _run(["tmux", "split-window", "-v", "-t", split_target, "-P", "-F", "#{pane_id}"], capture=True)
            pane_titles.append((pane_input, f"{index} {title}"))
            panes[key] = pane_input
            split_target = pane_input
    else:
        pane_deploy = _run(["tmux", "split-window", "-h", "-t", pane_first, "-P", "-F", "#{pane_id}"], capture=True)
        pane_titles = [
            (pane_first, "1 MuJoCo"),
            (pane_deploy, "2 SONIC deploy"),
        ]
        panes = {
            "mujoco": pane_first,
            "deploy": pane_deploy,
        }
        split_target = pane_deploy
        for index, (key, title) in enumerate(input_pane_specs, start=3):
            pane_input = _run(["tmux", "split-window", "-v", "-t", split_target, "-P", "-F", "#{pane_id}"], capture=True)
            pane_titles.append((pane_input, f"{index} {title}"))
            panes[key] = pane_input
            split_target = pane_input

    _run(["tmux", "select-layout", "-t", f"{args.session}:0", "tiled"])

    for pane, title in pane_titles:
        _run(["tmux", "select-pane", "-t", pane, "-T", title])

    return panes


def _send(pane: str, command: str, wait_s: float) -> None:
    _run(["tmux", "send-keys", "-t", pane, f"bash -lc {_quote(command)}", "C-m"])
    if wait_s > 0:
        time.sleep(wait_s)


def main() -> None:
    parser = _build_parser()
    args = parser.parse_args()
    if args.cleanup and args.cleanup_dry_run:
        parser.error("--cleanup and --cleanup-dry-run are mutually exclusive")

    args.repo_root = _resolve_path(args.repo_root)
    args.deploy_dir = args.repo_root / "gear_sonic_deploy"
    args.isaaclab_root = _resolve_path(args.isaaclab_root)
    args.proxy_bin = _resolve_path(args.proxy_bin)
    if args.sdk_root is None:
        args.sdk_root = _default_sdk_root(args.repo_root)

    if args.cleanup or args.cleanup_dry_run:
        _cleanup(args, dry_run=args.cleanup_dry_run)
        return

    prompt_scene = sys.stdin.isatty() and not args.isaac_scene and not args.isaac_task
    prompt_input_options = sys.stdin.isatty() and not args.input_source
    args.backend = _choose_backend(args.backend)
    if args.input_source is None and args.deploy_input_type is not None:
        args.input_source = args.deploy_input_type
    args.input_source = _choose_input_source(args.input_source)
    _configure_isaaclab_options(args, prompt_scene=prompt_scene)
    _configure_input_source_options(args, prompt_input_options=prompt_input_options)
    _configure_sony_pose_line(args)
    _configure_deploy_input_type(args)
    args.deploy_interface = args.mujoco_interface if args.backend == "mujoco" else args.interface
    args.conda_sh = _resolve_path(args.conda_sh)
    args.bvh_file = _resolve_path(args.bvh_file)
    args.pico_service_script = _resolve_path(args.pico_service_script)
    args.sdk_root = _resolve_path(args.sdk_root)
    args.decoder = _resolve_path(args.decoder, base=args.deploy_dir, repo_root=args.repo_root)
    args.encoder = _resolve_path(args.encoder, base=args.deploy_dir, repo_root=args.repo_root)
    args.planner_file = _resolve_path(args.planner_file, base=args.deploy_dir, repo_root=args.repo_root)
    args.obs_config = _resolve_path(args.obs_config, base=args.deploy_dir, repo_root=args.repo_root)
    args.motion_data = _resolve_path(args.motion_data, base=args.deploy_dir, repo_root=args.repo_root)

    _preflight(args)
    commands = _pane_commands(args)

    if args.dry_run:
        for name, command in commands.items():
            print(f"\n[{name}]\n{command}")
        return

    panes = _create_tmux(args)

    print(f"Created tmux session: {args.session}")
    print(f"Backend: {args.backend}")
    print(f"Input source: {args.input_source}")
    if args.input_source == "sony":
        print(f"BVH source: {args.bvh_source}")
        print(f"Sony POSE line: {args.sony_pose_line}")
        if args.bvh_source == "bvh_stream":
            sender_state = "external; this launcher starts manager only" if args.no_bvh_stream_sender else "local"
            print(f"BVH stream sender: {sender_state}")
    if args.input_source == "pico":
        print(f"PICO service script: {args.pico_service_script}")
        service_state = (
            "running"
            if _pico_service_process_running()
            else "not running; input pane will try to start it"
        )
        print(f"PICO service process: {service_state}")
    print(f"Deploy input type: {args.deploy_input_type}")
    if args.backend == "isaaclab":
        print(f"IsaacLab scene: {args.isaac_scene} ({args.isaac_task})")
        print("Starting pane 1: IsaacLab...")
        _send(panes["isaaclab"], commands["isaaclab"], args.wait_after_isaaclab)
        print("Starting pane 2: C++ LowState proxy...")
        _send(panes["proxy"], commands["proxy"], args.wait_after_proxy)
        deploy_pane_number = 3
        next_extra_pane_number = 4
    elif args.backend == "none":
        print("Backend none: start IsaacLab manually with SONIC_PUBLISH_STATE_ZMQ=1")
        print(f"  (UNITREE_DDS_INTERFACE={args.interface}, proxy state endpoint {args.isaac_state_endpoint})")
        print("Starting pane 1: C++ LowState proxy...")
        _send(panes["proxy"], commands["proxy"], args.wait_after_proxy)
        deploy_pane_number = 2
        next_extra_pane_number = 3
    else:
        print("Starting pane 1: MuJoCo...")
        _send(panes["mujoco"], commands["mujoco"], args.wait_after_mujoco)
        deploy_pane_number = 2
        next_extra_pane_number = 3

    print(f"Starting pane {deploy_pane_number}: GR00T/SONIC deploy...")
    _send(panes["deploy"], commands["deploy"], args.wait_after_deploy)
    extra_keys = [key for key in commands if key not in {"isaaclab", "proxy", "mujoco", "deploy"}]
    for pane_number, key in enumerate(extra_keys, start=next_extra_pane_number):
        print(f"Starting pane {pane_number}: {key}...")
        _send(panes[key], commands[key], 0.0)
    if not extra_keys:
        print(f"No separate input pane; deploy handles input type {args.deploy_input_type}.")

    _run(["tmux", "select-pane", "-t", panes["deploy"]])

    print()
    print("All panes launched.")
    print(f"  Attach:  tmux attach -t {args.session}")
    print(f"  Kill:    tmux kill-session -t {args.session}")
    print("  Stop all panes from inside tmux: Ctrl+\\")
    print()
    print("Pane map:")
    if args.backend == "isaaclab":
        print("  1 IsaacLab       -> /tmp/isaac_sonic_HHMM.log")
        print("  2 LowState proxy -> /tmp/sonic_proxy_HHMM.log")
        print("  3 SONIC deploy   -> /tmp/sonic_deploy_HHMM.log")
        for pane_number, key in enumerate(extra_keys, start=4):
            print(f"  {pane_number} {key:<14} -> foreground logs in pane")
    elif args.backend == "none":
        print("  1 LowState proxy -> /tmp/sonic_proxy_HHMM.log")
        print("  2 SONIC deploy   -> /tmp/sonic_deploy_HHMM.log")
        for pane_number, key in enumerate(extra_keys, start=3):
            print(f"  {pane_number} {key:<14} -> foreground logs in pane")
        print("  IsaacLab         -> started manually by you (SONIC_PUBLISH_STATE_ZMQ=1)")
    else:
        print("  1 MuJoCo         -> /tmp/mujoco_sonic_HHMM.log")
        print("  2 SONIC deploy   -> /tmp/sonic_deploy_HHMM.log")
        for pane_number, key in enumerate(extra_keys, start=3):
            print(f"  {pane_number} {key:<14} -> foreground logs in pane")
    if not extra_keys:
        print(f"  deploy input     -> {args.deploy_input_type} inside SONIC deploy pane")
    print()

    if not args.no_attach:
        subprocess.run(["tmux", "attach", "-t", args.session], check=False)


if __name__ == "__main__":
    main()
