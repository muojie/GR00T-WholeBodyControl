#!/usr/bin/env python3
"""Collect SONIC deploy <-> IsaacLab closed-loop validation metrics."""

from __future__ import annotations

import argparse
import json
import math
import time
import xml.etree.ElementTree as ET
from dataclasses import dataclass, field
from pathlib import Path
from typing import Any

import msgpack
import numpy as np
from scipy.spatial.transform import Rotation
import zmq

from gear_sonic.utils.teleop.sources.base import (
    G1_DEFAULT_ROOT_POS_W,
    G1_MUJOCO_TO_ISAACLAB_IDX,
    normalize_quat_wxyz,
)
from gear_sonic.utils.teleop.sources.g1_body_fk import (
    DEFAULT_G1_MJCF_PATH,
    G1BodyFk,
    SONIC_BODY_NAMES,
)


GROUP_BODY_INDEXES = {
    "hand": ("left_wrist_yaw_link", "right_wrist_yaw_link"),
    "foot": ("left_ankle_roll_link", "right_ankle_roll_link"),
    "knee": ("left_knee_link", "right_knee_link"),
    "elbow": ("left_elbow_link", "right_elbow_link"),
}


@dataclass
class TopicSubscriber:
    endpoint: str
    topic: str
    ctx: zmq.Context
    received: int = 0
    latest: dict[str, Any] | None = None
    first_receive_time: float | None = None
    last_receive_time: float | None = None

    def __post_init__(self) -> None:
        self.socket = self.ctx.socket(zmq.SUB)
        self.socket.setsockopt_string(zmq.SUBSCRIBE, self.topic)
        self.socket.setsockopt(zmq.CONFLATE, 1)
        self.socket.setsockopt(zmq.RCVTIMEO, 0)
        self.socket.connect(self.endpoint)

    def poll(self) -> bool:
        updated = False
        topic_bytes = self.topic.encode("utf-8")
        while True:
            try:
                raw = self.socket.recv(zmq.NOBLOCK)
            except zmq.Again:
                return updated

            if not raw.startswith(topic_bytes):
                continue
            payload = raw[len(topic_bytes) :]
            try:
                msg = msgpack.unpackb(payload, raw=False, strict_map_key=False)
            except Exception as exc:
                print(f"[SonicIsaacMetrics] WARN failed to decode {self.topic}: {exc}", flush=True)
                continue
            now = time.monotonic()
            self.received += 1
            self.latest = msg
            self.last_receive_time = now
            if self.first_receive_time is None:
                self.first_receive_time = now
            updated = True

    def close(self) -> None:
        self.socket.close(0)


@dataclass
class MetricsAccumulator:
    samples: list[dict[str, Any]] = field(default_factory=list)
    nonfinite_samples: int = 0
    first_deploy_index: int | None = None
    last_deploy_index: int | None = None
    first_isaac_sequence: int | None = None
    last_isaac_sequence: int | None = None
    started_at: float | None = None
    ended_at: float | None = None

    def add(self, sample: dict[str, Any]) -> None:
        if self.started_at is None:
            self.started_at = float(sample["monotonic_time"])
        self.ended_at = float(sample["monotonic_time"])
        if sample.pop("_nonfinite", False):
            self.nonfinite_samples += 1
        deploy_index = _optional_int(sample.get("deploy_index"))
        isaac_sequence = _optional_int(sample.get("isaac_sequence"))
        if deploy_index is not None:
            if self.first_deploy_index is None:
                self.first_deploy_index = deploy_index
            self.last_deploy_index = deploy_index
        if isaac_sequence is not None:
            if self.first_isaac_sequence is None:
                self.first_isaac_sequence = isaac_sequence
            self.last_isaac_sequence = isaac_sequence
        self.samples.append(sample)

    @property
    def elapsed_s(self) -> float:
        if self.started_at is None or self.ended_at is None:
            return 0.0
        return max(0.0, self.ended_at - self.started_at)


def _build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        description="Collect closed-loop metrics from deploy g1_debug and IsaacLab sonic_state.",
        formatter_class=argparse.ArgumentDefaultsHelpFormatter,
    )
    parser.add_argument("--deploy-endpoint", default="tcp://127.0.0.1:5557")
    parser.add_argument("--deploy-topic", default="g1_debug")
    parser.add_argument("--isaac-endpoint", default="tcp://127.0.0.1:5560")
    parser.add_argument("--isaac-topic", default="sonic_state")
    parser.add_argument("--duration-s", type=float, default=60.0)
    parser.add_argument("--startup-timeout-s", type=float, default=240.0)
    parser.add_argument("--sample-hz", type=float, default=20.0)
    parser.add_argument("--report-interval-s", type=float, default=2.0)
    parser.add_argument("--summary-json", type=Path)
    parser.add_argument("--samples-jsonl", type=Path)
    parser.add_argument("--mjcf-path", type=Path, default=DEFAULT_G1_MJCF_PATH)

    parser.add_argument("--min-samples", type=int, default=30)
    parser.add_argument("--min-deploy-fps", type=float, default=15.0)
    parser.add_argument("--min-isaac-fps", type=float, default=15.0)
    parser.add_argument("--min-base-height-m", type=float, default=0.45)
    parser.add_argument("--max-root-tilt-rad", type=float, default=0.85)
    parser.add_argument("--max-root-yaw-error-rad", type=float, default=0.80)
    parser.add_argument("--max-root-tilt-error-rad", type=float, default=0.45)
    parser.add_argument("--max-joint-rmse-rad", type=float, default=0.35)
    parser.add_argument("--max-body-rmse-m", type=float, default=0.25)
    parser.add_argument("--max-keypoint-error-m", type=float, default=0.35)
    parser.add_argument("--max-joint-velocity-radps", type=float, default=30.0)
    parser.add_argument("--max-target-step-rad", type=float, default=1.00)
    parser.add_argument("--min-joint-limit-margin-rad", type=float, default=-0.02)
    return parser


def _array(value: Any, shape: tuple[int, ...] | None = None) -> np.ndarray | None:
    if value is None:
        return None
    arr = np.asarray(value, dtype=np.float32)
    if shape is not None and arr.shape != shape:
        return None
    if not np.all(np.isfinite(arr)):
        return None
    return arr


def _optional_float(value: Any) -> float | None:
    try:
        out = float(value)
    except (TypeError, ValueError):
        return None
    return out if math.isfinite(out) else None


def _optional_int(value: Any) -> int | None:
    try:
        return int(value)
    except (TypeError, ValueError):
        return None


def _to_mujoco(q: np.ndarray, order: str) -> np.ndarray:
    if order == "mujoco":
        return q.astype(np.float32)
    return q[G1_MUJOCO_TO_ISAACLAB_IDX].astype(np.float32)


def _to_isaaclab(q: np.ndarray, order: str) -> np.ndarray:
    if order == "isaaclab":
        return q.astype(np.float32)
    out = np.zeros(29, dtype=np.float32)
    out[G1_MUJOCO_TO_ISAACLAB_IDX] = q
    return out


def _quat_rotation(quat_wxyz: np.ndarray) -> Rotation:
    quat = normalize_quat_wxyz(quat_wxyz)
    return Rotation.from_quat(quat[[1, 2, 3, 0]])


def _root_tilt_rad(quat_wxyz: np.ndarray) -> float:
    z_axis = _quat_rotation(quat_wxyz).apply(np.array([0.0, 0.0, 1.0], dtype=np.float32))
    return float(np.arccos(np.clip(z_axis[2], -1.0, 1.0)))


def _root_yaw_rad(quat_wxyz: np.ndarray) -> float:
    q = normalize_quat_wxyz(quat_wxyz)
    w, x, y, z = [float(v) for v in q]
    return math.atan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z))


def _wrap_pi(angle: float) -> float:
    return (angle + math.pi) % (2.0 * math.pi) - math.pi


def _load_joint_limits_mujoco(mjcf_path: Path) -> tuple[np.ndarray, np.ndarray]:
    root = ET.parse(mjcf_path).getroot()
    motor_joint_order = [
        motor.attrib["joint"] for motor in root.findall("./actuator/motor") if "joint" in motor.attrib
    ]
    ranges: dict[str, tuple[float, float]] = {}
    for joint in root.findall(".//joint"):
        name = joint.attrib.get("name")
        range_text = joint.attrib.get("range")
        if not name or not range_text:
            continue
        parts = [float(part) for part in range_text.split()]
        if len(parts) == 2:
            ranges[name] = (parts[0], parts[1])
    lower = np.full(len(motor_joint_order), -np.inf, dtype=np.float32)
    upper = np.full(len(motor_joint_order), np.inf, dtype=np.float32)
    for idx, name in enumerate(motor_joint_order):
        if name in ranges:
            lower[idx], upper[idx] = ranges[name]
    return lower, upper


def _compute_body_pos_pelvis(
    fk: G1BodyFk,
    joint_pos_isaaclab: np.ndarray,
    root_pos_w: np.ndarray,
    root_quat_wxyz: np.ndarray,
) -> np.ndarray:
    return fk.compute_body_pos14_pelvis(
        joint_pos_isaaclab.reshape(1, 29),
        root_pos_w.reshape(1, 3),
        root_quat_wxyz.reshape(1, 4),
    )[0]


def _world_body_to_pelvis(
    body_pos_w: np.ndarray,
    root_pos_w: np.ndarray,
    root_quat_wxyz: np.ndarray,
) -> np.ndarray:
    inv = _quat_rotation(root_quat_wxyz).inv()
    return inv.apply(body_pos_w - root_pos_w.reshape(1, 3)).astype(np.float32)


def _body_group_error(diff: np.ndarray, names: tuple[str, ...]) -> float:
    indexes = [SONIC_BODY_NAMES.index(name) for name in names]
    return float(np.mean(np.linalg.norm(diff[indexes], axis=1)))


def _compute_sample(
    deploy: dict[str, Any],
    isaac: dict[str, Any],
    *,
    fk: G1BodyFk,
    joint_lower_mujoco: np.ndarray,
    joint_upper_mujoco: np.ndarray,
    previous_target_q_mujoco: np.ndarray | None,
    args: argparse.Namespace,
) -> dict[str, Any]:
    target_q_mujoco = _array(deploy.get("body_q_target"), (29,))
    action_q_mujoco = _array(deploy.get("last_action"), (29,))
    actual_order = str(isaac.get("target_order", "mujoco")).lower()
    if actual_order not in {"mujoco", "isaaclab"}:
        actual_order = "mujoco"
    actual_q_raw = _array(isaac.get("joint_pos"), (29,))
    actual_dq_raw = _array(isaac.get("joint_vel"), (29,))
    actual_root_pos = _array(isaac.get("root_pos_w"), (3,))
    actual_root_quat = _array(isaac.get("root_quat_w"), (4,))

    required_missing = [
        name
        for name, value in {
            "deploy.body_q_target": target_q_mujoco,
            "isaac.joint_pos": actual_q_raw,
            "isaac.joint_vel": actual_dq_raw,
            "isaac.root_pos_w": actual_root_pos,
            "isaac.root_quat_w": actual_root_quat,
        }.items()
        if value is None
    ]
    if required_missing:
        return {
            "monotonic_time": time.monotonic(),
            "wall_time": time.time(),
            "deploy_index": _optional_int(deploy.get("index")),
            "isaac_sequence": _optional_int(isaac.get("sequence")),
            "missing_fields": required_missing,
            "_nonfinite": True,
        }

    assert target_q_mujoco is not None
    assert actual_q_raw is not None
    assert actual_dq_raw is not None
    assert actual_root_pos is not None
    assert actual_root_quat is not None

    actual_q_mujoco = _to_mujoco(actual_q_raw, actual_order)
    actual_q_isaaclab = _to_isaaclab(actual_q_raw, actual_order)
    actual_dq_mujoco = _to_mujoco(actual_dq_raw, actual_order)
    target_q_isaaclab = _to_isaaclab(target_q_mujoco, "mujoco")

    target_root_pos = _array(deploy.get("base_trans_target"), (3,))
    if target_root_pos is None:
        target_root_pos = G1_DEFAULT_ROOT_POS_W.copy()
    target_root_quat = _array(deploy.get("base_quat_target"), (4,))
    if target_root_quat is None:
        target_root_quat = np.array([1.0, 0.0, 0.0, 0.0], dtype=np.float32)

    actual_body_pos_w = _array(isaac.get("body_pos14_w"), (len(SONIC_BODY_NAMES), 3))
    if actual_body_pos_w is not None:
        actual_body_pelvis = _world_body_to_pelvis(actual_body_pos_w, actual_root_pos, actual_root_quat)
    else:
        actual_body_pelvis = _compute_body_pos_pelvis(fk, actual_q_isaaclab, actual_root_pos, actual_root_quat)
    target_body_pelvis = _compute_body_pos_pelvis(fk, target_q_isaaclab, target_root_pos, target_root_quat)
    body_diff = actual_body_pelvis - target_body_pelvis

    joint_error = actual_q_mujoco - target_q_mujoco
    action_error = None if action_q_mujoco is None else actual_q_mujoco - action_q_mujoco
    margins = np.minimum(actual_q_mujoco - joint_lower_mujoco, joint_upper_mujoco - actual_q_mujoco)
    finite_margins = margins[np.isfinite(margins)]
    raw_target_step = (
        0.0
        if previous_target_q_mujoco is None
        else float(np.max(np.abs(target_q_mujoco - previous_target_q_mujoco)))
    )
    applied_target_step = _optional_float(isaac.get("target_step_delta_absmax"))
    target_step = raw_target_step if applied_target_step is None else applied_target_step

    actual_tilt = _root_tilt_rad(actual_root_quat)
    target_tilt = _root_tilt_rad(target_root_quat)
    actual_yaw = _root_yaw_rad(actual_root_quat)
    target_yaw = _root_yaw_rad(target_root_quat)
    fall_detected = bool(
        float(actual_root_pos[2]) < args.min_base_height_m
        or actual_tilt > args.max_root_tilt_rad
    )

    sample: dict[str, Any] = {
        "monotonic_time": time.monotonic(),
        "wall_time": time.time(),
        "deploy_index": _optional_int(deploy.get("index")),
        "isaac_sequence": _optional_int(isaac.get("sequence")),
        "base_height_m": float(actual_root_pos[2]),
        "root_tilt_rad": actual_tilt,
        "root_yaw_error_rad": abs(_wrap_pi(actual_yaw - target_yaw)),
        "root_tilt_error_rad": abs(actual_tilt - target_tilt),
        "root_height_error_m": float(abs(actual_root_pos[2] - target_root_pos[2])),
        "joint_tracking_rmse_rad": float(np.sqrt(np.mean(np.square(joint_error)))),
        "joint_tracking_absmax_rad": float(np.max(np.abs(joint_error))),
        "action_tracking_rmse_rad": (
            None if action_error is None else float(np.sqrt(np.mean(np.square(action_error))))
        ),
        "joint_velocity_absmax_radps": float(np.max(np.abs(actual_dq_mujoco))),
        "target_joint_absmax_rad": float(np.max(np.abs(target_q_mujoco))),
        "target_step_absmax_rad": target_step,
        "raw_target_step_absmax_rad": raw_target_step,
        "joint_limit_margin_min_rad": float(np.min(finite_margins)) if finite_margins.size else None,
        "body_keypoint_rmse_m": float(np.sqrt(np.mean(np.square(body_diff)))),
        "body_keypoint_absmax_m": float(np.max(np.linalg.norm(body_diff, axis=1))),
        "fall_detected": fall_detected,
        "missing_fields": [],
    }
    if applied_target_step is not None:
        sample["applied_target_step_absmax_rad"] = applied_target_step
    for group, names in GROUP_BODY_INDEXES.items():
        sample[f"{group}_keypoint_error_m"] = _body_group_error(body_diff, names)

    nonfinite = False
    for value in sample.values():
        if isinstance(value, float) and not math.isfinite(value):
            nonfinite = True
            break
    sample["_nonfinite"] = nonfinite
    return sample


def _values(samples: list[dict[str, Any]], key: str) -> np.ndarray:
    values = [_optional_float(sample.get(key)) for sample in samples]
    arr = np.asarray([value for value in values if value is not None], dtype=np.float64)
    return arr[np.isfinite(arr)]


def _stats(samples: list[dict[str, Any]], key: str) -> dict[str, float | None]:
    arr = _values(samples, key)
    if arr.size == 0:
        return {"mean": None, "p95": None, "min": None, "max": None}
    return {
        "mean": float(np.mean(arr)),
        "p95": float(np.percentile(arr, 95)),
        "min": float(np.min(arr)),
        "max": float(np.max(arr)),
    }


def _fps(first_index: int | None, last_index: int | None, elapsed_s: float) -> float:
    if first_index is None or last_index is None or elapsed_s <= 1e-6:
        return 0.0
    return max(0.0, float(last_index - first_index) / elapsed_s)


def _pass_fail(acc: MetricsAccumulator, args: argparse.Namespace) -> dict[str, Any]:
    samples = acc.samples
    elapsed_s = acc.elapsed_s
    deploy_fps = _fps(acc.first_deploy_index, acc.last_deploy_index, elapsed_s)
    isaac_fps = _fps(acc.first_isaac_sequence, acc.last_isaac_sequence, elapsed_s)

    stats = {
        key: _stats(samples, key)
        for key in [
            "base_height_m",
            "root_tilt_rad",
            "root_yaw_error_rad",
            "root_tilt_error_rad",
            "root_height_error_m",
            "joint_tracking_rmse_rad",
            "joint_tracking_absmax_rad",
            "action_tracking_rmse_rad",
            "joint_velocity_absmax_radps",
            "target_joint_absmax_rad",
            "target_step_absmax_rad",
            "raw_target_step_absmax_rad",
            "applied_target_step_absmax_rad",
            "joint_limit_margin_min_rad",
            "body_keypoint_rmse_m",
            "body_keypoint_absmax_m",
            "hand_keypoint_error_m",
            "foot_keypoint_error_m",
            "knee_keypoint_error_m",
            "elbow_keypoint_error_m",
        ]
    }
    fall_frames = sum(1 for sample in samples if sample.get("fall_detected"))
    missing_field_samples = sum(1 for sample in samples if sample.get("missing_fields"))

    checks = {
        "enough_samples": len(samples) >= args.min_samples,
        "deploy_fps": deploy_fps >= args.min_deploy_fps,
        "isaac_fps": isaac_fps >= args.min_isaac_fps,
        "no_nonfinite_samples": acc.nonfinite_samples == 0,
        "no_missing_fields": missing_field_samples == 0,
        "no_fall_detected": fall_frames == 0,
        "base_height": _stat_min(stats, "base_height_m") >= args.min_base_height_m,
        "root_tilt": _stat_max(stats, "root_tilt_rad") <= args.max_root_tilt_rad,
        "root_yaw_error": _stat_p95(stats, "root_yaw_error_rad") <= args.max_root_yaw_error_rad,
        "root_tilt_error": _stat_p95(stats, "root_tilt_error_rad") <= args.max_root_tilt_error_rad,
        "joint_tracking_rmse": _stat_mean(stats, "joint_tracking_rmse_rad") <= args.max_joint_rmse_rad,
        "body_keypoint_rmse": _stat_mean(stats, "body_keypoint_rmse_m") <= args.max_body_rmse_m,
        "hand_keypoint_error": _stat_mean(stats, "hand_keypoint_error_m") <= args.max_keypoint_error_m,
        "foot_keypoint_error": _stat_mean(stats, "foot_keypoint_error_m") <= args.max_keypoint_error_m,
        "knee_keypoint_error": _stat_mean(stats, "knee_keypoint_error_m") <= args.max_keypoint_error_m,
        "elbow_keypoint_error": _stat_mean(stats, "elbow_keypoint_error_m") <= args.max_keypoint_error_m,
        "joint_velocity_peak": _stat_max(stats, "joint_velocity_absmax_radps")
        <= args.max_joint_velocity_radps,
        "target_step_peak": _stat_max(stats, "target_step_absmax_rad") <= args.max_target_step_rad,
        "joint_limit_margin": _stat_min(stats, "joint_limit_margin_min_rad")
        >= args.min_joint_limit_margin_rad,
    }
    score_components = _score_components(stats, fall_frames, acc.nonfinite_samples, missing_field_samples, args)
    score = max(0.0, 100.0 - sum(score_components.values()))
    return {
        "pass": all(checks.values()),
        "checks": checks,
        "score": score,
        "score_components": score_components,
        "score_function": (
            "score = 100 - weighted normalized penalties for falls/nonfinite/missing, "
            "joint RMSE, body RMSE, root tilt/yaw errors, joint velocity, target step, "
            "and joint-limit margin; higher is better."
        ),
        "elapsed_s": elapsed_s,
        "samples": len(samples),
        "deploy_fps": deploy_fps,
        "isaac_fps": isaac_fps,
        "fall_frames": fall_frames,
        "nonfinite_samples": acc.nonfinite_samples,
        "missing_field_samples": missing_field_samples,
        "stats": stats,
    }


def _stat_mean(stats: dict[str, dict[str, float | None]], key: str) -> float:
    return float(stats[key]["mean"] if stats[key]["mean"] is not None else math.inf)


def _stat_p95(stats: dict[str, dict[str, float | None]], key: str) -> float:
    return float(stats[key]["p95"] if stats[key]["p95"] is not None else math.inf)


def _stat_min(stats: dict[str, dict[str, float | None]], key: str) -> float:
    return float(stats[key]["min"] if stats[key]["min"] is not None else -math.inf)


def _stat_max(stats: dict[str, dict[str, float | None]], key: str) -> float:
    return float(stats[key]["max"] if stats[key]["max"] is not None else math.inf)


def _penalty(value: float, threshold: float, weight: float) -> float:
    if not math.isfinite(value):
        return weight
    if threshold <= 0:
        return 0.0
    return min(weight, weight * max(0.0, value / threshold))


def _score_components(
    stats: dict[str, dict[str, float | None]],
    fall_frames: int,
    nonfinite_samples: int,
    missing_field_samples: int,
    args: argparse.Namespace,
) -> dict[str, float]:
    margin = _stat_min(stats, "joint_limit_margin_min_rad")
    margin_penalty = 0.0
    if margin < args.min_joint_limit_margin_rad:
        margin_penalty = min(10.0, 10.0 * (args.min_joint_limit_margin_rad - margin + 1e-6) / 0.2)
    return {
        "fall": 30.0 if fall_frames else 0.0,
        "nonfinite_or_missing": 20.0 if nonfinite_samples or missing_field_samples else 0.0,
        "joint_tracking": _penalty(_stat_mean(stats, "joint_tracking_rmse_rad"), args.max_joint_rmse_rad, 18.0),
        "body_keypoint": _penalty(_stat_mean(stats, "body_keypoint_rmse_m"), args.max_body_rmse_m, 22.0),
        "root_yaw": _penalty(_stat_p95(stats, "root_yaw_error_rad"), args.max_root_yaw_error_rad, 8.0),
        "root_tilt": _penalty(_stat_p95(stats, "root_tilt_error_rad"), args.max_root_tilt_error_rad, 8.0),
        "joint_velocity": _penalty(_stat_max(stats, "joint_velocity_absmax_radps"), args.max_joint_velocity_radps, 7.0),
        "target_step": _penalty(_stat_max(stats, "target_step_absmax_rad"), args.max_target_step_rad, 7.0),
        "joint_limit": margin_penalty,
    }


def _write_json(path: Path | None, data: dict[str, Any]) -> None:
    if path is None:
        return
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(json.dumps(data, indent=2, sort_keys=True) + "\n")


def _append_jsonl(path: Path | None, data: dict[str, Any]) -> None:
    if path is None:
        return
    path.parent.mkdir(parents=True, exist_ok=True)
    with path.open("a", encoding="utf-8") as file:
        file.write(json.dumps(data, sort_keys=True) + "\n")


def _print_summary(label: str, summary: dict[str, Any]) -> None:
    compact = {
        "label": label,
        "pass": summary["pass"],
        "score": round(float(summary["score"]), 3),
        "samples": summary["samples"],
        "deploy_fps": round(float(summary["deploy_fps"]), 2),
        "isaac_fps": round(float(summary["isaac_fps"]), 2),
        "fall_frames": summary["fall_frames"],
        "joint_rmse_mean": summary["stats"]["joint_tracking_rmse_rad"]["mean"],
        "body_rmse_mean": summary["stats"]["body_keypoint_rmse_m"]["mean"],
        "base_height_min": summary["stats"]["base_height_m"]["min"],
        "root_tilt_max": summary["stats"]["root_tilt_rad"]["max"],
    }
    print(f"[SonicIsaacMetrics] {json.dumps(compact, sort_keys=True)}", flush=True)


def main() -> int:
    parser = _build_parser()
    args = parser.parse_args()

    ctx = zmq.Context()
    deploy_sub = TopicSubscriber(args.deploy_endpoint, args.deploy_topic, ctx)
    isaac_sub = TopicSubscriber(args.isaac_endpoint, args.isaac_topic, ctx)
    fk = G1BodyFk(args.mjcf_path)
    joint_lower_mujoco, joint_upper_mujoco = _load_joint_limits_mujoco(args.mjcf_path)

    print(
        "[SonicIsaacMetrics] waiting for streams "
        f"deploy={args.deploy_endpoint}/{args.deploy_topic} "
        f"isaac={args.isaac_endpoint}/{args.isaac_topic}",
        flush=True,
    )

    deadline = time.monotonic() + max(0.0, args.startup_timeout_s)
    while deploy_sub.latest is None or isaac_sub.latest is None:
        deploy_sub.poll()
        isaac_sub.poll()
        if time.monotonic() >= deadline:
            summary = {
                "pass": False,
                "reason": "startup_timeout",
                "deploy_received": deploy_sub.received,
                "isaac_received": isaac_sub.received,
            }
            _write_json(args.summary_json, summary)
            print(f"[SonicIsaacMetrics] {json.dumps(summary, sort_keys=True)}", flush=True)
            return 2
        time.sleep(0.01)

    acc = MetricsAccumulator()
    duration_deadline = time.monotonic() + max(0.0, args.duration_s)
    sample_period = 1.0 / max(args.sample_hz, 1e-6)
    next_sample_time = 0.0
    next_report_time = time.monotonic() + max(args.report_interval_s, 0.5)
    previous_pair: tuple[int | None, int | None] | None = None
    previous_target_q_mujoco: np.ndarray | None = None

    try:
        while time.monotonic() < duration_deadline:
            deploy_sub.poll()
            isaac_sub.poll()
            now = time.monotonic()
            if now >= next_sample_time and deploy_sub.latest is not None and isaac_sub.latest is not None:
                pair = (
                    _optional_int(deploy_sub.latest.get("index")),
                    _optional_int(isaac_sub.latest.get("sequence")),
                )
                if pair != previous_pair:
                    sample = _compute_sample(
                        deploy_sub.latest,
                        isaac_sub.latest,
                        fk=fk,
                        joint_lower_mujoco=joint_lower_mujoco,
                        joint_upper_mujoco=joint_upper_mujoco,
                        previous_target_q_mujoco=previous_target_q_mujoco,
                        args=args,
                    )
                    target_q = _array(deploy_sub.latest.get("body_q_target"), (29,))
                    if target_q is not None:
                        previous_target_q_mujoco = target_q.copy()
                    acc.add(sample)
                    _append_jsonl(args.samples_jsonl, sample)
                    previous_pair = pair
                next_sample_time = now + sample_period

            if now >= next_report_time:
                _print_summary("partial", _pass_fail(acc, args))
                next_report_time = now + max(args.report_interval_s, 0.5)
            time.sleep(0.002)
    except KeyboardInterrupt:
        print("[SonicIsaacMetrics] stopped by user", flush=True)
    finally:
        deploy_sub.close()
        isaac_sub.close()
        ctx.term()

    summary = _pass_fail(acc, args)
    _write_json(args.summary_json, summary)
    _print_summary("final", summary)
    if args.summary_json:
        print(f"[SonicIsaacMetrics] summary_json={args.summary_json}", flush=True)
    if args.samples_jsonl:
        print(f"[SonicIsaacMetrics] samples_jsonl={args.samples_jsonl}", flush=True)
    return 0 if summary["pass"] else 1


if __name__ == "__main__":
    raise SystemExit(main())
