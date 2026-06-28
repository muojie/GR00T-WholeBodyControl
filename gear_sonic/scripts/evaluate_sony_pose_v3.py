"""Offline evaluator for Sony/BVH POSE v3 references against the stable G1 v1 path."""

from __future__ import annotations

import argparse
import json
from pathlib import Path
from typing import Any

import numpy as np
from scipy.spatial.transform import Rotation

from gear_sonic.scripts.mocap_manager_server import POSE_FILTER_PROFILES
from gear_sonic.utils.teleop.sources.base import (
    FullBodyReference,
    G1_WRIST_JOINT_IDX_ISAACLAB,
    G1_WRIST_JOINT_LOWER_LIMIT_ISAACLAB,
    G1_WRIST_JOINT_UPPER_LIMIT_ISAACLAB,
    normalize_quat_wxyz,
    smpl_pose_to_g1_wrist_joint_pos,
)
from gear_sonic.utils.teleop.sources.bvh_g1_source import (
    BvhG1RetargetConfig,
    G1_BODY14_COMPARE_SMPL_IDX,
    g1_body_pos14_pelvis_to_smpl_joints,
    load_bvh_g1_motion,
    prepare_bvh_g1_retarget_context,
)
from gear_sonic.utils.teleop.sources.bvh_source import build_full_body_reference_from_skeleton_frame
from gear_sonic.utils.teleop.zmq.zmq_pose_sender import PoseStreamPublisher


COMPARE_NAMES = (
    "pelvis",
    "left_hip",
    "left_knee",
    "left_ankle",
    "right_hip",
    "right_knee",
    "right_ankle",
    "torso",
    "left_shoulder",
    "left_elbow",
    "left_wrist",
    "right_shoulder",
    "right_elbow",
    "right_wrist",
)

SEGMENTS = (
    ("left_thigh", 1, 2),
    ("left_shin", 2, 3),
    ("right_thigh", 4, 5),
    ("right_shin", 5, 6),
    ("torso", 0, 7),
    ("left_upper_arm", 8, 9),
    ("left_forearm", 9, 10),
    ("right_upper_arm", 11, 12),
    ("right_forearm", 12, 13),
)

WRIST_JOINT_NAMES = (
    "left_roll",
    "right_roll",
    "left_pitch",
    "right_pitch",
    "left_yaw",
    "right_yaw",
)

DEPLOY_RELEASE_WINDOW_SIZE = 10


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description=(
            "Compare Sony/BVH POSE v3 SMPL joints with the validated v1 G1 FK body reference."
        )
    )
    parser.add_argument("--bvh-file", default="/home/nolo/MCPM_20260526_190029.BVH")
    parser.add_argument("--target-fps", type=float, default=50.0)
    parser.add_argument("--unit-scale", type=float, default=0.01)
    parser.add_argument("--max-frames", type=int, default=0, help="0 means all frames")
    parser.add_argument("--frame-stride", type=int, default=1)
    parser.add_argument(
        "--pose-filter-profile",
        choices=sorted(POSE_FILTER_PROFILES.keys()),
        default="stable",
        help="POSE filter profile to evaluate against the raw Sony reference.",
    )
    parser.add_argument(
        "--pose-root-yaw-only",
        action="store_true",
        help="Evaluate the manager's --pose-root-yaw-only behavior.",
    )
    parser.add_argument(
        "--json-out",
        default=None,
        help="Optional path to write the full metric dictionary as JSON.",
    )
    return parser.parse_args()


def main() -> int:
    args = parse_args()
    bvh_path = Path(args.bvh_file).expanduser()
    if not bvh_path.exists():
        raise FileNotFoundError(bvh_path)

    cfg = BvhG1RetargetConfig(enable_body_fk=True, smpl_joints_source="g1_fk")
    motion = load_bvh_g1_motion(
        str(bvh_path),
        target_fps=args.target_fps,
        unit_scale=args.unit_scale,
        retarget_config=cfg,
    )
    context = prepare_bvh_g1_retarget_context(
        str(bvh_path),
        target_fps=args.target_fps,
        unit_scale=args.unit_scale,
        retarget_config=cfg,
    )

    frame_count = min(motion.frame_count, context.frame_count)
    frame_indices = np.arange(0, frame_count, max(1, int(args.frame_stride)), dtype=np.int64)
    if args.max_frames > 0:
        frame_indices = frame_indices[: args.max_frames]
    if frame_indices.size == 0:
        raise ValueError("no frames selected")
    frame_stride = max(1, int(args.frame_stride))
    evaluation_fps = float(motion.playback_fps) / float(frame_stride)

    g1_body_local = context.fk.compute_body_pos14_pelvis(
        motion.joint_pos_isaaclab[frame_indices],
        motion.root_pos_w[frame_indices],
        motion.root_quat_wxyz[frame_indices],
    )
    skeleton_smpl = _build_skeleton_smpl_sequence(context, frame_indices)
    g1_fk_smpl = np.stack(
        [g1_body_pos14_pelvis_to_smpl_joints(body_pos) for body_pos in g1_body_local],
        axis=0,
    )
    references = _build_g1_fk_reference_sequence(
        motion=motion,
        context=context,
        frame_indices=frame_indices,
        smpl_joints=g1_fk_smpl,
    )
    filtered_references = _apply_pose_filter(
        references,
        fps=evaluation_fps,
        profile_name=args.pose_filter_profile,
        root_yaw_only=bool(args.pose_root_yaw_only),
    )

    skeleton_compare = skeleton_smpl[:, G1_BODY14_COMPARE_SMPL_IDX, :]
    g1_fk_compare = g1_fk_smpl[:, G1_BODY14_COMPARE_SMPL_IDX, :]

    metrics: dict[str, Any] = {
        "bvh_file": str(bvh_path),
        "source_fps": float(motion.source_fps),
        "playback_fps": float(motion.playback_fps),
        "evaluation_fps": evaluation_fps,
        "frames_evaluated": int(frame_indices.size),
        "frame_stride": int(frame_stride),
        "root": _root_metrics(
            motion.root_pos_w[frame_indices],
            motion.root_quat_wxyz[frame_indices],
        ),
        "release_observations": {
            "note": (
                "Reference-only deploy observation approximation. Anchor orientation assumes "
                "identity robot base at reset and applies the same first-frame heading removal "
                "as deploy's GatherMotionAnchorOrientationMutiFrame(mode=0)."
            ),
            "pose_filter_profile": args.pose_filter_profile,
            "pose_root_yaw_only": bool(args.pose_root_yaw_only),
            "window_size": DEPLOY_RELEASE_WINDOW_SIZE,
            "raw": _release_observation_metrics(
                references,
                fps=evaluation_fps,
                window_size=DEPLOY_RELEASE_WINDOW_SIZE,
            ),
            "filtered": _release_observation_metrics(
                filtered_references,
                fps=evaluation_fps,
                window_size=DEPLOY_RELEASE_WINDOW_SIZE,
            ),
            "filter_lag": _filter_lag_metrics(references, filtered_references),
        },
        "variants": {
            "skeleton": _variant_metrics(g1_body_local, skeleton_compare),
            "g1_fk": _variant_metrics(g1_body_local, g1_fk_compare),
        },
    }

    print(_format_report(metrics))

    if args.json_out:
        out_path = Path(args.json_out).expanduser()
        out_path.parent.mkdir(parents=True, exist_ok=True)
        out_path.write_text(json.dumps(metrics, indent=2, ensure_ascii=False), encoding="utf-8")
        print(f"\nJSON written to {out_path}")
    return 0


def _build_skeleton_smpl_sequence(context: Any, frame_indices: np.ndarray) -> np.ndarray:
    out = np.empty((frame_indices.size, 24, 3), dtype=np.float32)
    for out_idx, frame_idx in enumerate(frame_indices):
        source_frame_idx = int(context.frame_indices[int(frame_idx)])
        reference = build_full_body_reference_from_skeleton_frame(
            context.bvh_motion.joint_names,
            context.bvh_motion.world_positions[source_frame_idx],
            context.bvh_motion.world_quat_wxyz[source_frame_idx],
        )
        out[out_idx] = reference.smpl_joints
    return out


def _build_g1_fk_reference_sequence(
    *,
    motion: Any,
    context: Any,
    frame_indices: np.ndarray,
    smpl_joints: np.ndarray,
) -> list[FullBodyReference]:
    refs: list[FullBodyReference] = []
    for out_idx, frame_idx in enumerate(frame_indices):
        frame = int(frame_idx)
        source_frame_idx = int(context.frame_indices[frame])
        body_pos = motion.body_pos[frame] if motion.body_pos is not None else None
        refs.append(
            build_full_body_reference_from_skeleton_frame(
                context.bvh_motion.joint_names,
                context.bvh_motion.world_positions[source_frame_idx],
                context.bvh_motion.world_quat_wxyz[source_frame_idx],
                frame_index=int(frame),
                smpl_joints=smpl_joints[out_idx],
                body_quat_w=motion.root_quat_wxyz[frame],
                body_pos_w=motion.root_pos_w[frame],
                body_pos=body_pos,
                joint_pos=motion.joint_pos_isaaclab[frame],
                joint_vel=motion.joint_vel_isaaclab[frame],
            )
        )
    return refs


def _apply_pose_filter(
    references: list[FullBodyReference],
    *,
    fps: float,
    profile_name: str,
    root_yaw_only: bool,
) -> list[FullBodyReference]:
    profile = POSE_FILTER_PROFILES[profile_name]
    publisher = PoseStreamPublisher(
        window_size=DEPLOY_RELEASE_WINDOW_SIZE,
        protocol_version=3,
        encoder_mode=2,
        enable_reference_filter=bool(profile["enable_reference_filter"]),
        reference_alpha=float(profile["reference_alpha"]),
        max_smpl_joint_speed_mps=float(profile["max_smpl_joint_speed_mps"]),
        max_smpl_pose_speed_radps=float(profile["max_smpl_pose_speed_radps"]),
        max_root_angular_speed_radps=float(profile["max_root_angular_speed_radps"]),
        max_joint_speed_radps=float(profile["max_joint_speed_radps"]),
        root_tilt_limit_rad=float(profile["root_tilt_limit_rad"]),
        root_yaw_only=root_yaw_only,
    )
    dt_s = 1.0 / max(1e-6, float(fps))
    return [
        publisher._filter_reference(reference, timestamp_s=idx * dt_s)
        for idx, reference in enumerate(references)
    ]


def _variant_metrics(target: np.ndarray, candidate: np.ndarray) -> dict[str, Any]:
    diff = candidate - target
    point_errors = np.linalg.norm(diff, axis=-1)
    frame_rmse = np.sqrt(np.mean(diff * diff, axis=(1, 2)))
    aligned, yaw_rad, scale = _yaw_scale_align_sequence(candidate, target)
    aligned_diff = aligned - target
    aligned_frame_rmse = np.sqrt(np.mean(aligned_diff * aligned_diff, axis=(1, 2)))
    segment_metrics = _segment_metrics(target, candidate)
    side_metrics = _side_metrics(target, candidate)
    per_point_mean = np.mean(point_errors, axis=0)
    worst_order = np.argsort(per_point_mean)[::-1][:5]
    return {
        "direct_rmse_m": _summary(frame_rmse),
        "yaw_scale_aligned_rmse_m": _summary(aligned_frame_rmse),
        "point_error_m": _summary(point_errors.reshape(-1)),
        "best_yaw_deg": _summary(np.rad2deg(yaw_rad)),
        "best_scale": _summary(scale),
        "segment_angle_deg": segment_metrics,
        "side": side_metrics,
        "worst_keypoints": [
            {"name": COMPARE_NAMES[int(idx)], "mean_error_m": float(per_point_mean[int(idx)])}
            for idx in worst_order
        ],
    }


def _release_observation_metrics(
    references: list[FullBodyReference],
    *,
    fps: float,
    window_size: int,
) -> dict[str, Any]:
    smpl_joints = np.stack([ref.smpl_joints for ref in references], axis=0).astype(np.float32)
    smpl_pose = np.stack([ref.smpl_pose for ref in references], axis=0).astype(np.float32)
    body_pos = np.stack([ref.body_pos_w for ref in references], axis=0).astype(np.float32)
    body_quat = np.stack([ref.body_quat_w for ref in references], axis=0).astype(np.float32)
    joint_pos = np.stack([ref.joint_pos for ref in references], axis=0).astype(np.float32)
    wrist_joint_pos = joint_pos[:, G1_WRIST_JOINT_IDX_ISAACLAB]
    smpl_projected_joint_pos = np.stack(
        [smpl_pose_to_g1_wrist_joint_pos(pose) for pose in smpl_pose],
        axis=0,
    )
    smpl_projected_wrist = smpl_projected_joint_pos[:, G1_WRIST_JOINT_IDX_ISAACLAB]

    anchor_quat = _anchor_quats_from_body_quat(body_quat)
    anchor_6d = _quat_wxyz_to_6d(anchor_quat)
    window_end = _future_index(np.arange(len(references)), len(references), window_size)
    smpl_horizon = np.linalg.norm(smpl_joints[window_end] - smpl_joints, axis=-1)
    wrist_horizon = np.abs(wrist_joint_pos[window_end] - wrist_joint_pos)
    anchor_horizon = _quat_angle_between(anchor_quat, anchor_quat[window_end])
    root_horizon = _quat_angle_between(body_quat, body_quat[window_end])

    wrist_margin = np.minimum(
        wrist_joint_pos - G1_WRIST_JOINT_LOWER_LIMIT_ISAACLAB,
        G1_WRIST_JOINT_UPPER_LIMIT_ISAACLAB - wrist_joint_pos,
    )
    wrist_projection_error = wrist_joint_pos - smpl_projected_wrist

    return {
        "smpl_joints_10frame_step1": {
            "speed_mps": _summary(_vector_speed(smpl_joints, fps).reshape(-1)),
            "horizon_delta_m": _summary(smpl_horizon.reshape(-1)),
        },
        "smpl_anchor_orientation_10frame_step1": {
            "sixd_shape": [int(window_size), 6],
            "root": _root_metrics(body_pos, body_quat),
            "anchor": _root_metrics(np.zeros((len(anchor_quat), 3), dtype=np.float32), anchor_quat),
            "anchor_6d_abs": _summary(np.abs(anchor_6d).reshape(-1)),
            "anchor_angular_speed_degps": _summary(
                np.rad2deg(_quat_angular_speed(anchor_quat, fps))
            ),
            "anchor_horizon_angle_deg": _summary(np.rad2deg(anchor_horizon)),
            "root_angular_speed_degps": _summary(np.rad2deg(_quat_angular_speed(body_quat, fps))),
            "root_horizon_angle_deg": _summary(np.rad2deg(root_horizon)),
        },
        "motion_joint_positions_wrists_10frame_step1": {
            "order": list(WRIST_JOINT_NAMES),
            "position_rad": _summary(wrist_joint_pos.reshape(-1)),
            "abs_position_rad": _summary(np.abs(wrist_joint_pos).reshape(-1)),
            "speed_radps": _summary(_scalar_speed(wrist_joint_pos, fps).reshape(-1)),
            "horizon_delta_rad": _summary(wrist_horizon.reshape(-1)),
            "limit_margin_rad": _summary(wrist_margin.reshape(-1)),
            "smpl_pose_projection_error_rad": _summary(
                np.abs(wrist_projection_error).reshape(-1)
            ),
            "per_joint_projection_error_rad": {
                name: _summary(np.abs(wrist_projection_error[:, idx]))
                for idx, name in enumerate(WRIST_JOINT_NAMES)
            },
        },
    }


def _filter_lag_metrics(
    raw_references: list[FullBodyReference],
    filtered_references: list[FullBodyReference],
) -> dict[str, Any]:
    raw_smpl_joints = np.stack([ref.smpl_joints for ref in raw_references], axis=0)
    filtered_smpl_joints = np.stack([ref.smpl_joints for ref in filtered_references], axis=0)
    raw_smpl_pose = np.stack([ref.smpl_pose for ref in raw_references], axis=0)
    filtered_smpl_pose = np.stack([ref.smpl_pose for ref in filtered_references], axis=0)
    raw_body_quat = np.stack([ref.body_quat_w for ref in raw_references], axis=0)
    filtered_body_quat = np.stack([ref.body_quat_w for ref in filtered_references], axis=0)
    raw_joint_pos = np.stack([ref.joint_pos for ref in raw_references], axis=0)
    filtered_joint_pos = np.stack([ref.joint_pos for ref in filtered_references], axis=0)
    raw_anchor_quat = _anchor_quats_from_body_quat(raw_body_quat)
    filtered_anchor_quat = _anchor_quats_from_body_quat(filtered_body_quat)

    return {
        "smpl_joint_lag_m": _summary(
            np.linalg.norm(raw_smpl_joints - filtered_smpl_joints, axis=-1).reshape(-1)
        ),
        "smpl_pose_lag_rad": _summary(
            _rotvec_sequence_error(raw_smpl_pose, filtered_smpl_pose).reshape(-1)
        ),
        "wrist_joint_pos_lag_rad": _summary(
            np.abs(
                raw_joint_pos[:, G1_WRIST_JOINT_IDX_ISAACLAB]
                - filtered_joint_pos[:, G1_WRIST_JOINT_IDX_ISAACLAB]
            ).reshape(-1)
        ),
        "root_quat_lag_deg": _summary(
            np.rad2deg(_quat_angle_between(raw_body_quat, filtered_body_quat))
        ),
        "anchor_quat_lag_deg": _summary(
            np.rad2deg(_quat_angle_between(raw_anchor_quat, filtered_anchor_quat))
        ),
    }


def _yaw_scale_align_sequence(candidate: np.ndarray, target: np.ndarray) -> tuple[np.ndarray, np.ndarray, np.ndarray]:
    aligned = np.empty_like(candidate, dtype=np.float32)
    yaws = np.empty(candidate.shape[0], dtype=np.float32)
    scales = np.empty(candidate.shape[0], dtype=np.float32)
    for frame_idx in range(candidate.shape[0]):
        src = candidate[frame_idx]
        dst = target[frame_idx]
        yaw = _best_yaw_xy(src, dst)
        rotated = Rotation.from_euler("z", yaw).apply(src.reshape(-1, 3)).reshape(src.shape)
        denom = float(np.sum(rotated * rotated))
        scale = float(np.sum(rotated * dst) / denom) if denom > 1e-8 else 1.0
        aligned[frame_idx] = (rotated * scale).astype(np.float32)
        yaws[frame_idx] = yaw
        scales[frame_idx] = scale
    return aligned, yaws, scales


def _best_yaw_xy(candidate: np.ndarray, target: np.ndarray) -> float:
    src_xy = candidate[:, :2]
    dst_xy = target[:, :2]
    numerator = float(np.sum(src_xy[:, 0] * dst_xy[:, 1] - src_xy[:, 1] * dst_xy[:, 0]))
    denominator = float(np.sum(src_xy[:, 0] * dst_xy[:, 0] + src_xy[:, 1] * dst_xy[:, 1]))
    return float(np.arctan2(numerator, denominator))


def _segment_metrics(target: np.ndarray, candidate: np.ndarray) -> dict[str, Any]:
    per_segment: dict[str, Any] = {}
    all_angles = []
    for name, start, end in SEGMENTS:
        target_vec = target[:, end] - target[:, start]
        candidate_vec = candidate[:, end] - candidate[:, start]
        target_norm = np.linalg.norm(target_vec, axis=-1)
        candidate_norm = np.linalg.norm(candidate_vec, axis=-1)
        denom = np.maximum(target_norm * candidate_norm, 1e-8)
        cos = np.sum(target_vec * candidate_vec, axis=-1) / denom
        angles = np.rad2deg(np.arccos(np.clip(cos, -1.0, 1.0)))
        all_angles.append(angles)
        per_segment[name] = {
            "angle_deg": _summary(angles),
            "target_len_m": _summary(target_norm),
            "candidate_len_m": _summary(candidate_norm),
        }
    return {
        "overall_angle_deg": _summary(np.concatenate(all_angles)),
        "segments": per_segment,
    }


def _side_metrics(target: np.ndarray, candidate: np.ndarray) -> dict[str, Any]:
    pairs = {
        "hip_y_gap": (1, 4),
        "knee_y_gap": (2, 5),
        "ankle_y_gap": (3, 6),
        "shoulder_y_gap": (8, 11),
        "elbow_y_gap": (9, 12),
        "wrist_y_gap": (10, 13),
    }
    out: dict[str, Any] = {}
    mismatch_rates = []
    for name, (left_idx, right_idx) in pairs.items():
        target_gap = target[:, left_idx, 1] - target[:, right_idx, 1]
        candidate_gap = candidate[:, left_idx, 1] - candidate[:, right_idx, 1]
        mismatch = (target_gap * candidate_gap) < 0.0
        mismatch_rates.append(mismatch.astype(np.float32))
        out[name] = {
            "target_mean_m": float(np.mean(target_gap)),
            "candidate_mean_m": float(np.mean(candidate_gap)),
            "sign_mismatch_rate": float(np.mean(mismatch)),
        }
    out["any_sign_mismatch_rate"] = float(np.mean(np.stack(mismatch_rates, axis=1).any(axis=1)))
    return out


def _anchor_quats_from_body_quat(root_quat_wxyz: np.ndarray) -> np.ndarray:
    root_quat = _normalize_quats_wxyz(root_quat_wxyz)
    root_rot = _rotation_from_wxyz_array(root_quat)
    init_heading_inv = _heading_rotation_from_wxyz(root_quat[0]).inv()
    anchor_rot = init_heading_inv * root_rot
    return anchor_rot.as_quat()[:, [3, 0, 1, 2]].astype(np.float32)


def _quat_wxyz_to_6d(quat_wxyz: np.ndarray) -> np.ndarray:
    rot = _rotation_from_wxyz_array(quat_wxyz)
    matrix = rot.as_matrix()
    return matrix[:, :, :2].reshape(quat_wxyz.shape[0], 6).astype(np.float32)


def _root_yaw_rad(root_quat_wxyz: np.ndarray) -> np.ndarray:
    root_quat = _normalize_quats_wxyz(root_quat_wxyz)
    w = root_quat[:, 0]
    x = root_quat[:, 1]
    y = root_quat[:, 2]
    z = root_quat[:, 3]
    return np.arctan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z))


def _root_tilt_rad_array(root_quat_wxyz: np.ndarray) -> np.ndarray:
    root_quat = _normalize_quats_wxyz(root_quat_wxyz)
    x = root_quat[:, 1]
    y = root_quat[:, 2]
    root_z_dot = np.clip(1.0 - 2.0 * (x * x + y * y), -1.0, 1.0)
    return np.arccos(root_z_dot)


def _root_metrics(root_pos_w: np.ndarray, root_quat_wxyz: np.ndarray) -> dict[str, Any]:
    yaw = _root_yaw_rad(root_quat_wxyz)
    tilt = _root_tilt_rad_array(root_quat_wxyz)
    return {
        "height_m": _summary(root_pos_w[:, 2]),
        "yaw_deg": _summary(np.rad2deg(yaw)),
        "tilt_deg": _summary(np.rad2deg(tilt)),
    }


def _normalize_quats_wxyz(quat_wxyz: np.ndarray) -> np.ndarray:
    quat = np.asarray(quat_wxyz, dtype=np.float32)
    if quat.ndim == 1:
        quat = quat.reshape(1, 4)
    out = np.empty_like(quat, dtype=np.float32)
    for idx, value in enumerate(quat):
        out[idx] = normalize_quat_wxyz(value)
    return out


def _rotation_from_wxyz_array(quat_wxyz: np.ndarray) -> Rotation:
    quat = _normalize_quats_wxyz(quat_wxyz)
    return Rotation.from_quat(quat[:, [1, 2, 3, 0]])


def _heading_rotation_from_wxyz(quat_wxyz: np.ndarray) -> Rotation:
    quat = normalize_quat_wxyz(quat_wxyz)
    rot = Rotation.from_quat(quat[[1, 2, 3, 0]])
    forward = rot.apply([1.0, 0.0, 0.0])
    yaw = float(np.arctan2(forward[1], forward[0]))
    return Rotation.from_euler("z", yaw)


def _quat_angle_between(a_wxyz: np.ndarray, b_wxyz: np.ndarray) -> np.ndarray:
    a = _normalize_quats_wxyz(a_wxyz)
    b = _normalize_quats_wxyz(b_wxyz)
    dots = np.abs(np.sum(a * b, axis=1))
    return 2.0 * np.arccos(np.clip(dots, -1.0, 1.0))


def _quat_angular_speed(quat_wxyz: np.ndarray, fps: float) -> np.ndarray:
    quat = _normalize_quats_wxyz(quat_wxyz)
    if quat.shape[0] < 2:
        return np.zeros(1, dtype=np.float32)
    return (_quat_angle_between(quat[:-1], quat[1:]) * float(fps)).astype(np.float32)


def _vector_speed(values: np.ndarray, fps: float) -> np.ndarray:
    arr = np.asarray(values, dtype=np.float32)
    if arr.shape[0] < 2:
        return np.zeros((1,) + arr.shape[1:-1], dtype=np.float32)
    return (np.linalg.norm(np.diff(arr, axis=0), axis=-1) * float(fps)).astype(np.float32)


def _scalar_speed(values: np.ndarray, fps: float) -> np.ndarray:
    arr = np.asarray(values, dtype=np.float32)
    if arr.shape[0] < 2:
        return np.zeros((1,) + arr.shape[1:], dtype=np.float32)
    return (np.abs(np.diff(arr, axis=0)) * float(fps)).astype(np.float32)


def _future_index(current: np.ndarray, count: int, window_size: int) -> np.ndarray:
    horizon = max(0, int(window_size) - 1)
    return np.minimum(current + horizon, max(0, int(count) - 1)).astype(np.int64)


def _rotvec_sequence_error(raw: np.ndarray, filtered: np.ndarray) -> np.ndarray:
    raw_flat = np.asarray(raw, dtype=np.float32).reshape(-1, 3)
    filtered_flat = np.asarray(filtered, dtype=np.float32).reshape(-1, 3)
    errors = np.empty(raw_flat.shape[0], dtype=np.float32)
    for idx, (raw_rv, filtered_rv) in enumerate(zip(raw_flat, filtered_flat, strict=True)):
        error = (
            Rotation.from_rotvec(filtered_rv).inv() * Rotation.from_rotvec(raw_rv)
        ).as_rotvec()
        errors[idx] = float(np.linalg.norm(error))
    return errors.reshape(np.asarray(raw).shape[:-1])


def _summary(values: np.ndarray) -> dict[str, float]:
    arr = np.asarray(values, dtype=np.float64).reshape(-1)
    arr = arr[np.isfinite(arr)]
    if arr.size == 0:
        return {"mean": 0.0, "p50": 0.0, "p95": 0.0, "max": 0.0}
    return {
        "mean": float(np.mean(arr)),
        "p50": float(np.percentile(arr, 50.0)),
        "p95": float(np.percentile(arr, 95.0)),
        "max": float(np.max(arr)),
    }


def _format_report(metrics: dict[str, Any]) -> str:
    lines = [
        "Sony POSE v3 offline evaluation",
        f"  BVH: {metrics['bvh_file']}",
        (
            f"  Frames: {metrics['frames_evaluated']} "
            f"(source_fps={metrics['source_fps']:.2f}, "
            f"playback_fps={metrics['playback_fps']:.2f}, "
            f"eval_fps={metrics['evaluation_fps']:.2f})"
        ),
        "  Root: "
        f"height mean={metrics['root']['height_m']['mean']:.3f}m, "
        f"tilt p95={metrics['root']['tilt_deg']['p95']:.2f}deg, "
        f"yaw range=[{metrics['root']['yaw_deg']['p50']:.1f}deg p50, "
        f"{metrics['root']['yaw_deg']['max']:.1f}deg max]",
    ]
    release = metrics["release_observations"]
    raw = release["raw"]
    filtered = release["filtered"]
    lag = release["filter_lag"]
    raw_anchor = raw["smpl_anchor_orientation_10frame_step1"]
    filtered_anchor = filtered["smpl_anchor_orientation_10frame_step1"]
    raw_wrist = raw["motion_joint_positions_wrists_10frame_step1"]
    filtered_wrist = filtered["motion_joint_positions_wrists_10frame_step1"]
    lines.extend(
        [
            "",
            "Release V3 observations:",
            (
                f"  profile={release['pose_filter_profile']} "
                f"root_yaw_only={int(release['pose_root_yaw_only'])} "
                f"window={release['window_size']} frames"
            ),
            (
                "  anchor speed p95/max: "
                f"raw={raw_anchor['anchor_angular_speed_degps']['p95']:.1f}/"
                f"{raw_anchor['anchor_angular_speed_degps']['max']:.1f} deg/s, "
                f"filtered={filtered_anchor['anchor_angular_speed_degps']['p95']:.1f}/"
                f"{filtered_anchor['anchor_angular_speed_degps']['max']:.1f} deg/s"
            ),
            (
                "  root tilt p95/max: "
                f"raw={raw_anchor['root']['tilt_deg']['p95']:.2f}/"
                f"{raw_anchor['root']['tilt_deg']['max']:.2f} deg, "
                f"filtered={filtered_anchor['root']['tilt_deg']['p95']:.2f}/"
                f"{filtered_anchor['root']['tilt_deg']['max']:.2f} deg"
            ),
            (
                "  anchor 10-frame horizon p95/max: "
                f"raw={raw_anchor['anchor_horizon_angle_deg']['p95']:.1f}/"
                f"{raw_anchor['anchor_horizon_angle_deg']['max']:.1f} deg, "
                f"filtered={filtered_anchor['anchor_horizon_angle_deg']['p95']:.1f}/"
                f"{filtered_anchor['anchor_horizon_angle_deg']['max']:.1f} deg"
            ),
            (
                "  wrist speed p95/max: "
                f"raw={raw_wrist['speed_radps']['p95']:.2f}/"
                f"{raw_wrist['speed_radps']['max']:.2f} rad/s, "
                f"filtered={filtered_wrist['speed_radps']['p95']:.2f}/"
                f"{filtered_wrist['speed_radps']['max']:.2f} rad/s"
            ),
            (
                "  wrist vs smpl_pose projection error p95/max: "
                f"raw={raw_wrist['smpl_pose_projection_error_rad']['p95']:.3f}/"
                f"{raw_wrist['smpl_pose_projection_error_rad']['max']:.3f} rad, "
                f"filtered={filtered_wrist['smpl_pose_projection_error_rad']['p95']:.3f}/"
                f"{filtered_wrist['smpl_pose_projection_error_rad']['max']:.3f} rad"
            ),
            (
                "  filter lag p95: "
                f"smpl={lag['smpl_joint_lag_m']['p95']:.3f}m, "
                f"anchor={lag['anchor_quat_lag_deg']['p95']:.2f}deg, "
                f"wrist={lag['wrist_joint_pos_lag_rad']['p95']:.3f}rad"
            ),
        ]
    )
    for name, data in metrics["variants"].items():
        lines.extend(
            [
                "",
                f"Variant: {name}",
                _metric_line("direct RMSE", data["direct_rmse_m"], "m"),
                _metric_line("yaw+scale RMSE", data["yaw_scale_aligned_rmse_m"], "m"),
                _metric_line("point error", data["point_error_m"], "m"),
                _metric_line("segment angle", data["segment_angle_deg"]["overall_angle_deg"], "deg"),
                _metric_line("best yaw", data["best_yaw_deg"], "deg"),
                _metric_line("best scale", data["best_scale"], "x"),
                (
                    "  side mismatch: "
                    f"{100.0 * data['side']['any_sign_mismatch_rate']:.1f}% frames"
                ),
                "  worst keypoints: "
                + ", ".join(
                    f"{item['name']}={item['mean_error_m']:.3f}m"
                    for item in data["worst_keypoints"]
                ),
            ]
        )

    skeleton = metrics["variants"]["skeleton"]["direct_rmse_m"]["mean"]
    g1_fk = metrics["variants"]["g1_fk"]["direct_rmse_m"]["mean"]
    lines.extend(
        [
            "",
            "Recommendation:",
            (
                "  Use --bvh-g1-smpl-joints-source g1_fk for Sony POSE v3 "
                f"(mean direct RMSE {skeleton:.3f}m -> {g1_fk:.3f}m against v1 G1 FK)."
            ),
            (
                "  Inspect release_observations in JSON before closed-loop tests; "
                "root/anchor, wrist, and filter lag are reported separately."
            ),
            "  Use --bvh-g1-smpl-joints-source skeleton only for raw-skeleton A/B tests.",
        ]
    )
    return "\n".join(lines)


def _metric_line(label: str, summary: dict[str, float], unit: str) -> str:
    return (
        f"  {label}: mean={summary['mean']:.3f}{unit}, "
        f"p50={summary['p50']:.3f}{unit}, "
        f"p95={summary['p95']:.3f}{unit}, "
        f"max={summary['max']:.3f}{unit}"
    )


if __name__ == "__main__":
    raise SystemExit(main())
