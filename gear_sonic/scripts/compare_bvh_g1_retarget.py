"""Compare BVH-to-G1 retarget modes against a reference mode."""

from __future__ import annotations

import argparse
import time

import numpy as np

from gear_sonic.utils.teleop.sources.base import (
    G1_LOWER_BODY_JOINT_IDX_ISAACLAB,
    G1_WRIST_JOINT_IDX_ISAACLAB,
)
from gear_sonic.utils.teleop.sources.bvh_g1_source import (
    BvhG1RetargetConfig,
    G1_SEGMENT_BODY_POINTS,
    load_bvh_g1_motion,
)
from gear_sonic.utils.teleop.sources.g1_body_fk import SONIC_BODY_NAMES
from gear_sonic.utils.teleop.sources.joint_probe_source import G1_ISAACLAB_JOINT_NAMES


WAIST_JOINT_IDX_ISAACLAB = np.array([2, 5, 8], dtype=np.int64)
UPPER_BODY_JOINT_IDX_ISAACLAB = np.array(
    [11, 15, 19, 21, 12, 16, 20, 22],
    dtype=np.int64,
)


def build_arg_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        description="Compare BVH-to-G1 retarget modes against a reference mode."
    )
    parser.add_argument("--bvh-file", required=True, help="Input BVH file")
    parser.add_argument("--bvh-fps", type=float, default=50.0)
    parser.add_argument("--bvh-unit-scale", type=float, default=0.01)
    parser.add_argument("--bvh-no-y-up-to-z-up", action="store_true")
    parser.add_argument(
        "--reference-mode",
        choices=("numeric", "fast", "analytic", "off"),
        default="numeric",
        help="Reference IK mode. numeric is slow but closest to the current reference path.",
    )
    parser.add_argument(
        "--candidate-mode",
        action="append",
        choices=("fast", "analytic", "off", "numeric"),
        default=None,
        help="Candidate mode to compare. Repeat for multiple modes.",
    )
    parser.add_argument("--bvh-g1-lower-scale", type=float, default=0.60)
    parser.add_argument("--bvh-g1-upper-scale", type=float, default=0.85)
    parser.add_argument("--bvh-g1-wrist-scale", type=float, default=0.55)
    parser.add_argument("--bvh-g1-waist-scale", type=float, default=0.25)
    parser.add_argument("--bvh-g1-max-joint-velocity", type=float, default=6.0)
    parser.add_argument("--bvh-g1-joint-filter-alpha", type=float, default=0.45)
    parser.add_argument("--bvh-g1-joint-limit-margin", type=float, default=0.05)
    parser.add_argument("--bvh-g1-joint-delta-limit-scale", type=float, default=0.8)
    parser.add_argument(
        "--bvh-g1-segment-direction",
        choices=("off", "upper", "all"),
        default="all",
    )
    parser.add_argument(
        "--bvh-g1-axis-map",
        choices=("identity", "bvh_y_forward"),
        default="bvh_y_forward",
    )
    parser.add_argument(
        "--bvh-g1-root-mode",
        choices=("source", "yaw", "locked"),
        default="yaw",
    )
    parser.add_argument("--bvh-g1-no-body-fk", action="store_true")
    parser.add_argument("--top-joints", type=int, default=12)
    return parser


def main() -> None:
    args = build_arg_parser().parse_args()
    candidate_modes = args.candidate_mode or ["fast", "analytic"]
    modes = [args.reference_mode] + [mode for mode in candidate_modes if mode != args.reference_mode]

    motions = {}
    timings = {}
    for mode in modes:
        start_s = time.perf_counter()
        motions[mode] = load_bvh_g1_motion(
            bvh_file=args.bvh_file,
            target_fps=args.bvh_fps,
            unit_scale=args.bvh_unit_scale,
            y_up_to_z_up=not args.bvh_no_y_up_to_z_up,
            retarget_config=_make_config(args, mode),
        )
        timings[mode] = time.perf_counter() - start_s

    ref = motions[args.reference_mode]
    print(
        f"reference={args.reference_mode} frames={ref.frame_count} "
        f"fps={ref.playback_fps:.1f} source_fps={ref.source_fps:.1f}"
    )
    for mode in modes:
        motion = motions[mode]
        q = motion.joint_pos_isaaclab
        print(
            f"mode={mode:8s} elapsed={timings[mode]:7.3f}s "
            f"q=[{float(q.min()):+.3f},{float(q.max()):+.3f}] "
            f"dq_abs={float(np.abs(motion.joint_vel_isaaclab).max()):.3f}"
        )

    for mode in modes[1:]:
        _print_mode_delta(args, ref, motions[mode], mode)


def _make_config(args: argparse.Namespace, ik_mode: str) -> BvhG1RetargetConfig:
    return BvhG1RetargetConfig(
        method="skeleton",
        lower_body_scale=args.bvh_g1_lower_scale,
        upper_body_scale=args.bvh_g1_upper_scale,
        wrist_scale=args.bvh_g1_wrist_scale,
        waist_scale=args.bvh_g1_waist_scale,
        max_joint_velocity_radps=args.bvh_g1_max_joint_velocity,
        joint_filter_alpha=args.bvh_g1_joint_filter_alpha,
        joint_limit_margin_rad=args.bvh_g1_joint_limit_margin,
        joint_delta_limit_scale=args.bvh_g1_joint_delta_limit_scale,
        skeleton_segment_direction=args.bvh_g1_segment_direction,
        skeleton_axis_mapping=args.bvh_g1_axis_map,
        ik_mode=ik_mode,
        root_mode=args.bvh_g1_root_mode,
        enable_body_fk=not args.bvh_g1_no_body_fk,
    )


def _print_mode_delta(args, ref_motion, candidate_motion, mode: str) -> None:
    n = min(ref_motion.frame_count, candidate_motion.frame_count)
    ref_q = ref_motion.joint_pos_isaaclab[:n]
    candidate_q = candidate_motion.joint_pos_isaaclab[:n]
    abs_error = np.abs(candidate_q - ref_q)

    print("")
    print(f"delta mode={mode} vs {args.reference_mode}")
    _print_joint_group("all", abs_error, np.arange(29, dtype=np.int64))
    _print_joint_group("lower", abs_error, G1_LOWER_BODY_JOINT_IDX_ISAACLAB)
    _print_joint_group("upper", abs_error, UPPER_BODY_JOINT_IDX_ISAACLAB)
    _print_joint_group("wrist", abs_error, G1_WRIST_JOINT_IDX_ISAACLAB)
    _print_joint_group("waist", abs_error, WAIST_JOINT_IDX_ISAACLAB)

    ranked = []
    for joint_idx, joint_name in enumerate(G1_ISAACLAB_JOINT_NAMES):
        joint_error = abs_error[:, joint_idx]
        ranked.append(
            (
                float(joint_error.mean()),
                float(np.percentile(joint_error, 90)),
                float(joint_error.max()),
                joint_idx,
                joint_name,
            )
        )
    print("top joint errors:")
    for mean, p90, max_error, joint_idx, joint_name in sorted(ranked, reverse=True)[
        : max(0, int(args.top_joints))
    ]:
        print(
            f"  {joint_idx:02d} {joint_name:22s} "
            f"mean={mean:.4f} p90={p90:.4f} max={max_error:.4f}"
        )

    if ref_motion.body_pos is not None and candidate_motion.body_pos is not None:
        print("segment direction angle errors (deg):")
        for segment_key in G1_SEGMENT_BODY_POINTS:
            angles = _segment_angle_errors(ref_motion.body_pos[:n], candidate_motion.body_pos[:n], segment_key)
            if angles is None:
                continue
            print(
                f"  {segment_key:16s} "
                f"mean={float(angles.mean()):.2f} "
                f"p90={float(np.percentile(angles, 90)):.2f} "
                f"max={float(angles.max()):.2f}"
            )


def _print_joint_group(name: str, abs_error: np.ndarray, indexes: np.ndarray) -> None:
    group = abs_error[:, indexes]
    print(
        f"  {name:6s} "
        f"mean={float(group.mean()):.4f} "
        f"p50={float(np.percentile(group, 50)):.4f} "
        f"p90={float(np.percentile(group, 90)):.4f} "
        f"max={float(group.max()):.4f}"
    )


def _segment_angle_errors(
    ref_body_pos: np.ndarray,
    candidate_body_pos: np.ndarray,
    segment_key: str,
) -> np.ndarray | None:
    body_to_idx = {name: idx for idx, name in enumerate(SONIC_BODY_NAMES)}
    start_name, end_name = G1_SEGMENT_BODY_POINTS[segment_key]
    if start_name not in body_to_idx or end_name not in body_to_idx:
        return None
    start_idx = body_to_idx[start_name]
    end_idx = body_to_idx[end_name]
    ref_dir = _unit_vectors(ref_body_pos[:, end_idx] - ref_body_pos[:, start_idx])
    candidate_dir = _unit_vectors(candidate_body_pos[:, end_idx] - candidate_body_pos[:, start_idx])
    dot = np.sum(ref_dir * candidate_dir, axis=1)
    return np.degrees(np.arccos(np.clip(dot, -1.0, 1.0)))


def _unit_vectors(values: np.ndarray) -> np.ndarray:
    norms = np.linalg.norm(values, axis=1, keepdims=True)
    return values / np.maximum(norms, 1e-8)


if __name__ == "__main__":
    main()
