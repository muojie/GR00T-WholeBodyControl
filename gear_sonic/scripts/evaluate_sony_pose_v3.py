"""Offline evaluator for Sony/BVH POSE v3 references against the stable G1 v1 path."""

from __future__ import annotations

import argparse
import json
from pathlib import Path
from typing import Any

import numpy as np
from scipy.spatial.transform import Rotation

from gear_sonic.utils.teleop.sources.bvh_g1_source import (
    BvhG1RetargetConfig,
    G1_BODY14_COMPARE_SMPL_IDX,
    g1_body_pos14_pelvis_to_smpl_joints,
    load_bvh_g1_motion,
    prepare_bvh_g1_retarget_context,
)
from gear_sonic.utils.teleop.sources.bvh_source import build_full_body_reference_from_skeleton_frame


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


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description=(
            "Compare Sony/BVH POSE v3 SMPL joints with the validated v1 G1 FK body reference."
        )
    )
    parser.add_argument("--bvh-file", default="/home/nolo/RAYNOS_Motion1.bvh")
    parser.add_argument("--target-fps", type=float, default=50.0)
    parser.add_argument("--unit-scale", type=float, default=0.01)
    parser.add_argument("--max-frames", type=int, default=0, help="0 means all frames")
    parser.add_argument("--frame-stride", type=int, default=1)
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

    skeleton_compare = skeleton_smpl[:, G1_BODY14_COMPARE_SMPL_IDX, :]
    g1_fk_compare = g1_fk_smpl[:, G1_BODY14_COMPARE_SMPL_IDX, :]

    metrics: dict[str, Any] = {
        "bvh_file": str(bvh_path),
        "source_fps": float(motion.source_fps),
        "playback_fps": float(motion.playback_fps),
        "frames_evaluated": int(frame_indices.size),
        "frame_stride": int(max(1, int(args.frame_stride))),
        "root": _root_metrics(
            motion.root_pos_w[frame_indices],
            motion.root_quat_wxyz[frame_indices],
        ),
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


def _root_metrics(root_pos_w: np.ndarray, root_quat_wxyz: np.ndarray) -> dict[str, Any]:
    root_quat = np.asarray(root_quat_wxyz, dtype=np.float32)
    norm = np.maximum(np.linalg.norm(root_quat, axis=1, keepdims=True), 1e-8)
    root_quat = root_quat / norm
    w = root_quat[:, 0]
    x = root_quat[:, 1]
    y = root_quat[:, 2]
    z = root_quat[:, 3]
    yaw = np.arctan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z))
    root_z_dot = np.clip(1.0 - 2.0 * (x * x + y * y), -1.0, 1.0)
    tilt = np.arccos(root_z_dot)
    return {
        "height_m": _summary(root_pos_w[:, 2]),
        "yaw_deg": _summary(np.rad2deg(yaw)),
        "tilt_deg": _summary(np.rad2deg(tilt)),
    }


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
            f"(source_fps={metrics['source_fps']:.2f}, playback_fps={metrics['playback_fps']:.2f})"
        ),
        "  Root: "
        f"height mean={metrics['root']['height_m']['mean']:.3f}m, "
        f"tilt p95={metrics['root']['tilt_deg']['p95']:.2f}deg, "
        f"yaw range=[{metrics['root']['yaw_deg']['p50']:.1f}deg p50, "
        f"{metrics['root']['yaw_deg']['max']:.1f}deg max]",
    ]
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
