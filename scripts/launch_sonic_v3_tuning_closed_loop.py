#!/usr/bin/env python3
"""Launch the isolated Sony/BVH SMPL POSE v3 tuning stack.

This wrapper keeps v3 experiments out of the default v1 launcher path by using
the v3 worktree as repo root, a separate TCP/UDP port set, and a non-zero
Unitree DDS domain.
"""

from __future__ import annotations

import argparse
import subprocess
import time
from pathlib import Path


REPO_ROOT = Path(__file__).resolve().parents[1]
BASE_LAUNCHER = REPO_ROOT / "scripts" / "launch_sonic_local_isaaclab_closed_loop.py"

DEFAULT_SESSION = "sonic_v3_tuning_isolated"
DEFAULT_ZMQ_PORT = 6056
DEFAULT_DEBUG_PORT = 6057
DEFAULT_STATE_PORT = 6060
DEFAULT_BVH_STREAM_PORT = 12403
DEFAULT_BVH_FILE = Path.home() / "RAYNOS_Motion1.bvh"
ORIGINAL_REPO_ROOT = Path.home() / "GR00T-WholeBodyControl"


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

    parser.add_argument("--zmq-port", type=int, default=DEFAULT_ZMQ_PORT)
    parser.add_argument("--debug-port", type=int, default=DEFAULT_DEBUG_PORT)
    parser.add_argument("--state-port", type=int, default=DEFAULT_STATE_PORT)
    parser.add_argument("--bvh-stream-port", type=int, default=DEFAULT_BVH_STREAM_PORT)
    parser.add_argument("--domain-id", type=int, default=4)

    parser.add_argument("--bvh-file", type=Path, default=DEFAULT_BVH_FILE)
    parser.add_argument(
        "--pose-filter-profile",
        choices=["stable", "responsive", "off"],
        default="responsive",
    )
    parser.add_argument(
        "--no-root-yaw-only",
        action="store_true",
        help="do not pass --pose-root-yaw-only to the base launcher",
    )
    parser.add_argument("--metrics-duration-s", type=float, default=60.0)
    parser.add_argument("--metrics-summary-json", type=Path)
    parser.add_argument("--metrics-samples-jsonl", type=Path)
    return parser


def _resolve_release_file(relative_path: str) -> Path:
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
    summary_json = args.metrics_summary_json or _default_metric_path("summary")
    samples_jsonl = args.metrics_samples_jsonl or _default_metric_path("samples")

    cmd = [
        str(REPO_ROOT / ".venv_teleop" / "bin" / "python"),
        str(BASE_LAUNCHER),
        "--repo-root",
        str(REPO_ROOT),
        "--session",
        args.session,
        "--domain-id",
        str(args.domain_id),
        "--zmq-port",
        str(args.zmq_port),
        "--debug-port",
        str(args.debug_port),
        "--state-port",
        str(args.state_port),
        "--bvh-stream-port",
        str(args.bvh_stream_port),
        "--sony-pose-line",
        "v3",
        "--pose-filter-profile",
        args.pose_filter_profile,
        "--bvh-g1-smpl-joints-source",
        "g1_fk",
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
        "--metrics-duration-s",
        str(args.metrics_duration_s),
        "--metrics-summary-json",
        str(summary_json),
        "--metrics-samples-jsonl",
        str(samples_jsonl),
    ]
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
