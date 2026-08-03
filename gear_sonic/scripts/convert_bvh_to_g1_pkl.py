"""Convert a BVH file into the G1 joint-reference PKL format used by POSE v1."""

from __future__ import annotations

import argparse
from pathlib import Path

import numpy as np

from gear_sonic.utils.teleop.sources import (
    BvhG1RetargetConfig,
    load_bvh_g1_motion,
    save_bvh_g1_motion_lib_pkl,
)


def build_arg_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        description="Convert BVH motion to a G1 29-DOF motion_lib-style PKL."
    )
    parser.add_argument("--bvh-file", required=True, help="Input BVH file")
    parser.add_argument("--output", required=True, help="Output PKL path")
    parser.add_argument(
        "--bvh-fps",
        type=float,
        default=50.0,
        help="Target/downsample playback FPS. The IsaacLab mocap path used 50 Hz.",
    )
    parser.add_argument(
        "--bvh-unit-scale",
        type=float,
        default=0.01,
        help="Scale BVH position units to meters. Use 0.01 for centimeter BVH files.",
    )
    parser.add_argument(
        "--bvh-no-y-up-to-z-up",
        action="store_true",
        help="Disable BVH Y-up to SONIC Z-up coordinate conversion.",
    )
    parser.add_argument(
        "--bvh-g1-method",
        choices=["skeleton", "heuristic"],
        default="skeleton",
        help=(
            "Retargeting method. skeleton projects BVH world rotations onto the G1 MJCF "
            "joints; heuristic keeps the older SMPL-axis-angle gains."
        ),
    )
    parser.add_argument(
        "--bvh-g1-retarget-scale",
        type=float,
        default=1.0,
        help="Global gain for BVH-to-G1 joint retargeting.",
    )
    parser.add_argument(
        "--bvh-g1-lower-scale",
        type=float,
        default=0.60,
        help="Lower-body gain for BVH-to-G1 joint retargeting.",
    )
    parser.add_argument(
        "--bvh-g1-upper-scale",
        type=float,
        default=0.85,
        help="Upper-body gain for BVH-to-G1 joint retargeting.",
    )
    parser.add_argument(
        "--bvh-g1-waist-scale",
        type=float,
        default=0.25,
        help="Waist gain for BVH-to-G1 joint retargeting.",
    )
    parser.add_argument(
        "--bvh-g1-max-joint-velocity",
        type=float,
        default=6.0,
        help="Clamp BVH-to-G1 joint velocities in rad/s. Use <=0 to disable.",
    )
    parser.add_argument(
        "--bvh-g1-max-joint-step",
        type=float,
        default=0.0,
        help=(
            "Clamp BVH-to-G1 joint position changes in rad/frame. "
            "Defaults to max_joint_velocity / FPS; use <=0 with max velocity <=0 to disable."
        ),
    )
    parser.add_argument(
        "--bvh-g1-joint-filter-alpha",
        type=float,
        default=0.45,
        help=(
            "Optional low-pass alpha for BVH-to-G1 joint targets. "
            "1.0 disables low-pass smoothing; lower values are smoother but laggier."
        ),
    )
    parser.add_argument(
        "--bvh-g1-joint-delta-limit-scale",
        type=float,
        default=0.8,
        help=(
            "Scale the per-joint default-pose delta limits used by skeleton retargeting. "
            "Lower is more stable; higher preserves larger BVH motion."
        ),
    )
    parser.add_argument(
        "--bvh-g1-no-skeleton-sign-correction",
        action="store_true",
        help=(
            "Disable the side-specific sign correction used by skeleton retargeting for "
            "mirrored roll/yaw and elbow/wrist joints."
        ),
    )
    parser.add_argument(
        "--bvh-g1-segment-direction",
        choices=("off", "upper", "all"),
        default="all",
        help=(
            "Use BVH joint positions to align G1 limb segment directions before projecting "
            "skeleton rotations. upper affects arms only; all also affects legs."
        ),
    )
    parser.add_argument(
        "--bvh-g1-axis-map",
        choices=("identity", "bvh_y_forward"),
        default="bvh_y_forward",
        help=(
            "Map BVH root-local segment axes into G1 pelvis axes before direction retargeting. "
            "bvh_y_forward maps BVH -Y forward to G1 +X forward."
        ),
    )
    parser.add_argument(
        "--bvh-g1-ik-mode",
        choices=("numeric", "fast", "analytic", "off"),
        default="numeric",
        help=(
            "Limb direction refinement. numeric is the slower least-squares reference; "
            "analytic avoids SciPy IK and matches the online default; fast is experimental local IK."
        ),
    )
    parser.add_argument(
        "--bvh-g1-no-align-root",
        action="store_true",
        help="Disable first-frame root alignment.",
    )
    parser.add_argument(
        "--bvh-g1-local-root",
        action="store_true",
        help="Use BVH root-local positions instead of preserving global root translation.",
    )
    parser.add_argument(
        "--bvh-g1-no-body-fk",
        action="store_true",
        help="Disable MJCF FK body_pos14 generation in the output PKL.",
    )
    parser.add_argument(
        "--motion-name",
        default=None,
        help="Optional motion name stored inside the PKL. Defaults to the BVH basename.",
    )
    return parser


def main() -> None:
    args = build_arg_parser().parse_args()
    retarget_config = BvhG1RetargetConfig(
        method=args.bvh_g1_method,
        retarget_scale=args.bvh_g1_retarget_scale,
        lower_body_scale=args.bvh_g1_lower_scale,
        upper_body_scale=args.bvh_g1_upper_scale,
        waist_scale=args.bvh_g1_waist_scale,
        max_joint_velocity_radps=args.bvh_g1_max_joint_velocity,
        max_joint_step_rad=args.bvh_g1_max_joint_step,
        joint_filter_alpha=args.bvh_g1_joint_filter_alpha,
        joint_delta_limit_scale=args.bvh_g1_joint_delta_limit_scale,
        skeleton_sign_correction=not args.bvh_g1_no_skeleton_sign_correction,
        skeleton_segment_direction=args.bvh_g1_segment_direction,
        skeleton_axis_mapping=args.bvh_g1_axis_map,
        ik_mode=args.bvh_g1_ik_mode,
        enable_body_fk=not args.bvh_g1_no_body_fk,
    )
    motion = load_bvh_g1_motion(
        bvh_file=args.bvh_file,
        target_fps=args.bvh_fps,
        unit_scale=args.bvh_unit_scale,
        y_up_to_z_up=not args.bvh_no_y_up_to_z_up,
        local_root=args.bvh_g1_local_root,
        align_root=not args.bvh_g1_no_align_root,
        retarget_config=retarget_config,
    )
    if args.motion_name:
        motion.motion_name = args.motion_name

    output_path = Path(args.output)
    output_path.parent.mkdir(parents=True, exist_ok=True)
    save_bvh_g1_motion_lib_pkl(motion, str(output_path))

    q = motion.joint_pos_isaaclab
    dq = motion.joint_vel_isaaclab
    root_z = motion.root_pos_w[:, 2]
    q_step_abs = float(np.max(np.abs(np.diff(q, axis=0)))) if q.shape[0] > 1 else 0.0
    body_pos_desc = "none"
    if motion.body_pos is not None:
        body_pos_desc = (
            f"{motion.body_pos.shape} abs_max={float(np.max(np.abs(motion.body_pos))):.3f}"
        )
    print(
        "[convert_bvh_to_g1_pkl] wrote "
        f"{output_path} motion={motion.motion_name!r} frames={motion.frame_count} "
        f"source_fps={motion.source_fps:.1f} playback_fps={motion.playback_fps:.1f} "
        f"q=[{float(np.min(q)):.3f},{float(np.max(q)):.3f}] "
        f"q_step_abs={q_step_abs:.3f} "
        f"dq_abs={float(np.max(np.abs(dq))):.3f} "
        f"root_z=[{float(np.min(root_z)):.3f},{float(np.max(root_z)):.3f}] "
        f"body_pos={body_pos_desc}"
    )


if __name__ == "__main__":
    main()
