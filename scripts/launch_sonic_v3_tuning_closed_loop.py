#!/usr/bin/env python3
"""Launch the isolated Sony/BVH SMPL POSE v3 tuning stack.

This wrapper keeps v3 experiments out of the default v1 launcher path by using
the v3 worktree as repo root, a separate TCP/UDP port set, and a non-zero
Unitree DDS domain.
"""

from __future__ import annotations

import argparse
import subprocess
import sys
import time
from pathlib import Path


REPO_ROOT = Path(__file__).resolve().parents[1]
BASE_LAUNCHER = REPO_ROOT / "scripts" / "launch_sonic_local_isaaclab_closed_loop.py"

DEFAULT_SESSION = "sonic_v3_tuning_isolated"
DEFAULT_ZMQ_PORT = 6056
DEFAULT_DEBUG_PORT = 6057
DEFAULT_STATE_PORT = 6060
DEFAULT_BVH_STREAM_PORT = 12403
BASELINE_STACK_ZMQ_PORT = 6756
BASELINE_STACK_DEBUG_PORT = 6757
BASELINE_STACK_STATE_PORT = 6760
BASELINE_STACK_BVH_STREAM_PORT = 12413
DEFAULT_DOMAIN_ID = 4
BASELINE_STACK_DOMAIN_ID = 0
DEFAULT_BVH_FILE = Path.home() / "RAYNOS_Motion1.bvh"
ORIGINAL_REPO_ROOT = Path.home() / "GR00T-WholeBodyControl"
DEFAULT_ISAACLAB_ROOT = Path.home() / "xiaoyang_IssacLab" / "IsaacLab-v1-free-root-20260629"
BASELINE_ISAACLAB_ROOT = Path.home() / "xiaoyang_IssacLab" / "IsaacLab-baseline-20260630"
BASELINE_PROXY_BIN = (
    Path("/tmp/GR00T-WholeBodyControl-baseline-20260630")
    / "gear_sonic_deploy"
    / "build"
    / "tools"
    / "sonic_unitree_lowstate_cpp_proxy"
)


def _build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        description="Launch isolated Sony/BVH SMPL POSE v3 closed-loop tuning.",
        formatter_class=argparse.ArgumentDefaultsHelpFormatter,
    )
    parser.add_argument("--session", default=DEFAULT_SESSION)
    parser.add_argument("--replace", action="store_true")
    parser.add_argument("--no-attach", action="store_true")
    parser.add_argument("--dry-run", action="store_true")
    parser.add_argument("--headless", action="store_true")
    parser.add_argument(
        "--baseline-isaaclab-stack",
        action="store_true",
        help=(
            "use the verified IsaacLab baseline worktree plus baseline C++ proxy, "
            "and force DDS domain 0 for the current v3 deploy binary"
        ),
    )
    parser.add_argument(
        "--isaaclab-root",
        type=Path,
        help=f"override IsaacLab worktree passed to the base launcher; default is {DEFAULT_ISAACLAB_ROOT}",
    )
    parser.add_argument("--proxy-bin", type=Path, help="override C++ lowstate proxy binary")

    parser.add_argument("--zmq-port", type=int, default=DEFAULT_ZMQ_PORT)
    parser.add_argument("--debug-port", type=int, default=DEFAULT_DEBUG_PORT)
    parser.add_argument("--state-port", type=int, default=DEFAULT_STATE_PORT)
    parser.add_argument("--bvh-stream-port", type=int, default=DEFAULT_BVH_STREAM_PORT)
    parser.add_argument("--domain-id", type=int, help="proxy DDS domain; baseline stack profile defaults to 0")

    parser.add_argument("--bvh-file", type=Path, default=DEFAULT_BVH_FILE)
    parser.add_argument("--input-source", choices=["bvh", "sony_json"], default="bvh")
    parser.add_argument(
        "--json-file",
        type=Path,
        default=Path.home() / "saveBoneData_Yup20260702.json",
        help="Sony mocopi saveBoneData JSON file when --input-source sony_json.",
    )
    parser.add_argument("--sony-bonedata-packet-format", choices=["msgpack", "json"], default="msgpack")
    parser.add_argument("--sony-bonedata-joints-per-frame", type=int, default=27)
    parser.add_argument("--sony-bonedata-fps", type=float)
    parser.add_argument("--sony-bonedata-source-fps", type=float)
    parser.add_argument(
        "--bvh-stream-bonedata-coordinate-frame",
        choices=["sonic_zup", "left_handed_zup", "left_handed_yup", "zup_flip_xy"],
        default="left_handed_yup",
    )
    parser.add_argument("--bvh-stream-bonedata-position-scale", type=float, default=1.0)
    parser.add_argument("--bvh-stream-bonedata-input-quat-order", choices=["xyzw", "wxyz"], default="xyzw")
    parser.add_argument("--bvh-stream-bonedata-rotation-mode", choices=["input", "identity"], default="input")
    parser.add_argument("--bvh-stream-bonedata-local-root", action="store_true")
    parser.add_argument(
        "--pose-filter-profile",
        choices=["stable", "balanced", "responsive", "off"],
        default="stable",
    )
    parser.add_argument(
        "--no-root-yaw-only",
        action="store_true",
        help="do not pass --pose-root-yaw-only to the base launcher",
    )
    parser.add_argument("--metrics-duration-s", type=float, default=60.0)
    parser.add_argument("--metrics-summary-json", type=Path)
    parser.add_argument("--metrics-samples-jsonl", type=Path)
    parser.add_argument(
        "--metrics-root-yaw-reference",
        choices=["deploy_absolute", "base_relative"],
        help="override the metrics root yaw gate reference; follow-base diagnostics default to base_relative",
    )
    parser.add_argument(
        "--target-rate-limit",
        type=float,
        default=0.003,
        help="IsaacLab target step clamp for the v3 conservative root-locked validation profile",
    )
    parser.add_argument(
        "--isaac-target-field",
        choices=["body_q_target", "last_action"],
        default="last_action",
        help=(
            "deploy joint field consumed by IsaacLab. last_action is the physics-control target; "
            "body_q_target is a root-locked reference-tracking diagnostic."
        ),
    )
    parser.add_argument(
        "--post-unlock-target-rate-limit",
        type=float,
        help="optional target step clamp after root unlock; defaults to --target-rate-limit",
    )
    parser.add_argument("--post-unlock-rate-limit-release-steps", type=int, default=50)
    parser.add_argument(
        "--auto-unlock-after-packets",
        type=int,
        default=0,
        help="keep root locked by default; use a positive value to test free-root unlock",
    )
    parser.add_argument(
        "--unlock-blend-steps",
        type=int,
        help="IsaacLab root velocity release blend steps before the root is marked unlocked",
    )
    parser.add_argument(
        "--check-root-yaw-error",
        action="store_true",
        help="keep the root yaw error metric active even in the default root-locked profile",
    )
    parser.add_argument(
        "--ignore-root-yaw-error",
        action="store_true",
        help="force root yaw error to be ignored by metrics",
    )
    parser.add_argument(
        "--post-unlock-follow-base",
        action="store_true",
        help="after unlock, write root XY/yaw from deploy base targets for diagnostic replay",
    )
    parser.add_argument("--post-unlock-damping-steps", type=int, default=0)
    parser.add_argument("--post-unlock-xy-velocity-scale", type=float)
    parser.add_argument("--post-unlock-z-velocity-scale", type=float)
    parser.add_argument("--post-unlock-angular-velocity-scale", type=float)
    parser.add_argument(
        "--post-unlock-safety-assist",
        action="store_true",
        help="enable IsaacLab post-unlock velocity-only root height/tilt safety assist",
    )
    parser.add_argument("--post-unlock-safety-strength", type=float)
    parser.add_argument("--post-unlock-safety-min-height", type=float)
    parser.add_argument("--post-unlock-safety-height-margin", type=float)
    parser.add_argument("--post-unlock-safety-tilt-start", type=float)
    parser.add_argument("--post-unlock-safety-tilt-full", type=float)
    parser.add_argument("--post-unlock-safety-lift-velocity", type=float)
    parser.add_argument(
        "--auto-reset-on-fall",
        action="store_true",
        help="enable IsaacLab env.reset() watchdog when root height/tilt indicates a fall",
    )
    parser.add_argument("--fall-reset-min-height", type=float)
    parser.add_argument("--fall-reset-max-tilt", type=float)
    parser.add_argument("--fall-reset-grace-steps", type=int)
    parser.add_argument("--base-yaw-rate-limit", type=float)
    parser.add_argument("--base-translation-rate-limit", type=float)
    parser.add_argument("--bvh-g1-smpl-joints-source", choices=["g1_fk", "skeleton"], default="g1_fk")
    parser.add_argument("--bvh-g1-retarget-scale", type=float, default=1.0)
    parser.add_argument("--bvh-g1-lower-scale", type=float, default=0.60)
    parser.add_argument("--bvh-g1-upper-scale", type=float, default=0.85)
    parser.add_argument("--bvh-g1-wrist-scale", type=float, default=0.55)
    parser.add_argument("--bvh-g1-waist-scale", type=float, default=0.25)
    parser.add_argument("--bvh-g1-max-joint-velocity", type=float, default=5.5)
    parser.add_argument("--bvh-g1-max-joint-step", type=float, default=0.0)
    parser.add_argument("--bvh-g1-joint-filter-alpha", type=float, default=0.45)
    parser.add_argument("--bvh-g1-joint-delta-limit-scale", type=float, default=0.8)
    return parser


def _resolve_release_file(relative_path: str) -> Path:
    local = REPO_ROOT / "gear_sonic_deploy" / relative_path
    if local.exists():
        return local

    original = ORIGINAL_REPO_ROOT / "gear_sonic_deploy" / relative_path
    if original.exists():
        return original

    return local


def _resolve_proxy_bin(args: argparse.Namespace) -> Path:
    if args.proxy_bin is not None:
        return args.proxy_bin.expanduser()
    if args.baseline_isaaclab_stack and BASELINE_PROXY_BIN.exists():
        return BASELINE_PROXY_BIN

    relative_path = "build/tools/sonic_unitree_lowstate_cpp_proxy"
    local = REPO_ROOT / "gear_sonic_deploy" / relative_path
    if local.exists():
        return local

    original = ORIGINAL_REPO_ROOT / "gear_sonic_deploy" / relative_path
    if original.exists():
        return original

    return local


def _default_metric_path(kind: str) -> Path:
    stamp = time.strftime("%Y%m%d_%H%M%S")
    suffix = "summary.json" if kind == "summary" else "samples.jsonl"
    return Path(f"/tmp/sony_pose_v3_tuning_isolated_{stamp}_{suffix}")


def _launcher_command(args: argparse.Namespace, extra_args: list[str]) -> list[str]:
    if args.baseline_isaaclab_stack:
        if args.zmq_port == DEFAULT_ZMQ_PORT:
            args.zmq_port = BASELINE_STACK_ZMQ_PORT
        if args.debug_port == DEFAULT_DEBUG_PORT:
            args.debug_port = BASELINE_STACK_DEBUG_PORT
        if args.state_port == DEFAULT_STATE_PORT:
            args.state_port = BASELINE_STACK_STATE_PORT
        if args.bvh_stream_port == DEFAULT_BVH_STREAM_PORT:
            args.bvh_stream_port = BASELINE_STACK_BVH_STREAM_PORT
        if args.isaaclab_root is None:
            args.isaaclab_root = BASELINE_ISAACLAB_ROOT
    elif args.isaaclab_root is None:
        args.isaaclab_root = DEFAULT_ISAACLAB_ROOT

    domain_id = args.domain_id
    if domain_id is None:
        domain_id = BASELINE_STACK_DOMAIN_ID if args.baseline_isaaclab_stack else DEFAULT_DOMAIN_ID

    summary_json = args.metrics_summary_json or _default_metric_path("summary")
    samples_jsonl = args.metrics_samples_jsonl or _default_metric_path("samples")
    post_unlock_target_rate_limit = (
        args.target_rate_limit
        if args.post_unlock_target_rate_limit is None
        else args.post_unlock_target_rate_limit
    )
    metrics_root_yaw_reference = args.metrics_root_yaw_reference or (
        "base_relative" if args.post_unlock_follow_base else "deploy_absolute"
    )

    cmd = [
        str(REPO_ROOT / ".venv_teleop" / "bin" / "python"),
        str(BASE_LAUNCHER),
        "--repo-root",
        str(REPO_ROOT),
        "--session",
        args.session,
        "--domain-id",
        str(domain_id),
        "--zmq-port",
        str(args.zmq_port),
        "--debug-port",
        str(args.debug_port),
        "--state-port",
        str(args.state_port),
        "--bvh-stream-port",
        str(args.bvh_stream_port),
        "--bvh-stream-bonedata-coordinate-frame",
        args.bvh_stream_bonedata_coordinate_frame,
        "--bvh-stream-bonedata-position-scale",
        str(args.bvh_stream_bonedata_position_scale),
        "--bvh-stream-bonedata-input-quat-order",
        args.bvh_stream_bonedata_input_quat_order,
        "--bvh-stream-bonedata-rotation-mode",
        args.bvh_stream_bonedata_rotation_mode,
        "--sony-pose-line",
        "v3",
        "--pose-filter-profile",
        args.pose_filter_profile,
        "--bvh-g1-smpl-joints-source",
        args.bvh_g1_smpl_joints_source,
        "--bvh-g1-retarget-scale",
        str(args.bvh_g1_retarget_scale),
        "--bvh-g1-lower-scale",
        str(args.bvh_g1_lower_scale),
        "--bvh-g1-upper-scale",
        str(args.bvh_g1_upper_scale),
        "--bvh-g1-wrist-scale",
        str(args.bvh_g1_wrist_scale),
        "--bvh-g1-waist-scale",
        str(args.bvh_g1_waist_scale),
        "--bvh-g1-max-joint-velocity",
        str(args.bvh_g1_max_joint_velocity),
        "--bvh-g1-max-joint-step",
        str(args.bvh_g1_max_joint_step),
        "--bvh-g1-joint-filter-alpha",
        str(args.bvh_g1_joint_filter_alpha),
        "--bvh-g1-joint-delta-limit-scale",
        str(args.bvh_g1_joint_delta_limit_scale),
        "--bvh-file",
        str(args.bvh_file),
        "--decoder",
        str(_resolve_release_file("policy/release/model_decoder.onnx")),
        "--encoder",
        str(_resolve_release_file("policy/release/model_encoder.onnx")),
        "--planner-file",
        str(_resolve_release_file("planner/target_vel/V2/planner_sonic.onnx")),
        "--obs-config",
        str(_resolve_release_file("policy/release/observation_config.yaml")),
        "--proxy-bin",
        str(_resolve_proxy_bin(args)),
        "--target-rate-limit",
        str(args.target_rate_limit),
        "--post-unlock-target-rate-limit",
        str(post_unlock_target_rate_limit),
        "--post-unlock-rate-limit-release-steps",
        str(args.post_unlock_rate_limit_release_steps),
        "--auto-unlock-after-packets",
        str(args.auto_unlock_after_packets),
        "--metrics-duration-s",
        str(args.metrics_duration_s),
        "--metrics-summary-json",
        str(summary_json),
        "--metrics-samples-jsonl",
        str(samples_jsonl),
        "--metrics-root-yaw-reference",
        metrics_root_yaw_reference,
        "--isaac-env",
        f"SONIC_DEPLOY_TARGET_FIELD={args.isaac_target_field}",
    ]
    if args.input_source == "sony_json":
        cmd.extend(
            [
                "--bvh-stream-input-source",
                "sony_json",
                "--sony-bonedata-json-file",
                str(args.json_file.expanduser()),
                "--sony-bonedata-packet-format",
                args.sony_bonedata_packet_format,
                "--sony-bonedata-joints-per-frame",
                str(args.sony_bonedata_joints_per_frame),
            ]
        )
        if args.sony_bonedata_fps is not None:
            cmd.extend(["--sony-bonedata-fps", str(args.sony_bonedata_fps)])
        if args.sony_bonedata_source_fps is not None:
            cmd.extend(["--sony-bonedata-source-fps", str(args.sony_bonedata_source_fps)])
        if args.bvh_stream_bonedata_local_root:
            cmd.append("--bvh-stream-bonedata-local-root")
    if args.auto_reset_on_fall:
        cmd.append("--auto-reset-on-fall")
    if args.fall_reset_min_height is not None:
        cmd.extend(["--fall-reset-min-height", str(args.fall_reset_min_height)])
    if args.fall_reset_max_tilt is not None:
        cmd.extend(["--fall-reset-max-tilt", str(args.fall_reset_max_tilt)])
    if args.fall_reset_grace_steps is not None:
        cmd.extend(["--fall-reset-grace-steps", str(args.fall_reset_grace_steps)])
    if args.post_unlock_follow_base:
        cmd.extend(["--isaac-env", "SONIC_DEPLOY_POST_UNLOCK_FOLLOW_BASE=1"])
        cmd.extend(["--isaac-env", "SONIC_DEPLOY_FOLLOW_BASE_YAW=1"])
        cmd.extend(["--isaac-env", "SONIC_DEPLOY_FOLLOW_BASE_TRANSLATION=1"])
    if args.unlock_blend_steps is not None:
        cmd.extend(["--isaac-env", f"SONIC_DEPLOY_UNLOCK_BLEND_STEPS={args.unlock_blend_steps}"])
    if args.post_unlock_damping_steps > 0:
        cmd.extend(["--isaac-env", f"SONIC_DEPLOY_POST_UNLOCK_DAMPING_STEPS={args.post_unlock_damping_steps}"])
    if args.post_unlock_xy_velocity_scale is not None:
        cmd.extend([
            "--isaac-env",
            f"SONIC_DEPLOY_POST_UNLOCK_XY_VELOCITY_SCALE={args.post_unlock_xy_velocity_scale}",
        ])
    if args.post_unlock_z_velocity_scale is not None:
        cmd.extend([
            "--isaac-env",
            f"SONIC_DEPLOY_POST_UNLOCK_Z_VELOCITY_SCALE={args.post_unlock_z_velocity_scale}",
        ])
    if args.post_unlock_angular_velocity_scale is not None:
        cmd.extend([
            "--isaac-env",
            f"SONIC_DEPLOY_POST_UNLOCK_ANGULAR_VELOCITY_SCALE={args.post_unlock_angular_velocity_scale}",
        ])
    if args.post_unlock_safety_assist:
        cmd.extend(["--isaac-env", "SONIC_DEPLOY_POST_UNLOCK_SAFETY_ASSIST=1"])
    if args.post_unlock_safety_strength is not None:
        cmd.extend([
            "--isaac-env",
            f"SONIC_DEPLOY_POST_UNLOCK_SAFETY_STRENGTH={args.post_unlock_safety_strength}",
        ])
    if args.post_unlock_safety_min_height is not None:
        cmd.extend([
            "--isaac-env",
            f"SONIC_DEPLOY_POST_UNLOCK_SAFETY_MIN_HEIGHT={args.post_unlock_safety_min_height}",
        ])
    if args.post_unlock_safety_height_margin is not None:
        cmd.extend([
            "--isaac-env",
            f"SONIC_DEPLOY_POST_UNLOCK_SAFETY_HEIGHT_MARGIN={args.post_unlock_safety_height_margin}",
        ])
    if args.post_unlock_safety_tilt_start is not None:
        cmd.extend([
            "--isaac-env",
            f"SONIC_DEPLOY_POST_UNLOCK_SAFETY_TILT_START={args.post_unlock_safety_tilt_start}",
        ])
    if args.post_unlock_safety_tilt_full is not None:
        cmd.extend([
            "--isaac-env",
            f"SONIC_DEPLOY_POST_UNLOCK_SAFETY_TILT_FULL={args.post_unlock_safety_tilt_full}",
        ])
    if args.post_unlock_safety_lift_velocity is not None:
        cmd.extend([
            "--isaac-env",
            f"SONIC_DEPLOY_POST_UNLOCK_SAFETY_LIFT_VELOCITY={args.post_unlock_safety_lift_velocity}",
        ])
    if args.base_yaw_rate_limit is not None:
        cmd.extend(["--isaac-env", f"SONIC_DEPLOY_BASE_YAW_RATE_LIMIT={args.base_yaw_rate_limit}"])
    if args.base_translation_rate_limit is not None:
        cmd.extend([
            "--isaac-env",
            f"SONIC_DEPLOY_BASE_TRANSLATION_RATE_LIMIT={args.base_translation_rate_limit}",
        ])
    if args.ignore_root_yaw_error or (args.auto_unlock_after_packets == 0 and not args.check_root_yaw_error):
        cmd.append("--metrics-ignore-root-yaw-error")
    if args.replace:
        cmd.append("--replace")
    if args.no_attach:
        cmd.append("--no-attach")
    if args.dry_run:
        cmd.append("--dry-run")
    if args.headless:
        cmd.append("--headless")
    if not args.no_root_yaw_only:
        cmd.append("--pose-root-yaw-only")
    if args.isaaclab_root is not None:
        cmd.extend(["--isaaclab-root", str(args.isaaclab_root.expanduser())])
    cmd.extend(extra_args)
    return cmd


def main(argv: list[str] | None = None) -> int:
    parser = _build_parser()
    args, extra_args = parser.parse_known_args(argv)

    if not BASE_LAUNCHER.exists():
        print(f"[sonic-v3] missing base launcher: {BASE_LAUNCHER}", file=sys.stderr)
        return 2
    if not (REPO_ROOT / ".venv_teleop" / "bin" / "python").exists():
        print(f"[sonic-v3] missing .venv_teleop in worktree: {REPO_ROOT}", file=sys.stderr)
        return 2

    cmd = _launcher_command(args, extra_args)
    return subprocess.run(cmd, check=False, cwd=REPO_ROOT).returncode


if __name__ == "__main__":
    raise SystemExit(main())
