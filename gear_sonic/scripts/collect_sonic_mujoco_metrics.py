#!/usr/bin/env python3
"""Collect SONIC deploy metrics while the stack is driven by MuJoCo lowstate."""

from __future__ import annotations

import argparse
import json
import math
import time
from dataclasses import dataclass, field
from pathlib import Path
from typing import Any

import msgpack
import numpy as np
import zmq


G1_MUJOCO_JOINT_NAMES = [
    "left_hip_pitch_joint",
    "left_hip_roll_joint",
    "left_hip_yaw_joint",
    "left_knee_joint",
    "left_ankle_pitch_joint",
    "left_ankle_roll_joint",
    "right_hip_pitch_joint",
    "right_hip_roll_joint",
    "right_hip_yaw_joint",
    "right_knee_joint",
    "right_ankle_pitch_joint",
    "right_ankle_roll_joint",
    "waist_yaw_joint",
    "waist_roll_joint",
    "waist_pitch_joint",
    "left_shoulder_pitch_joint",
    "left_shoulder_roll_joint",
    "left_shoulder_yaw_joint",
    "left_elbow_joint",
    "left_wrist_roll_joint",
    "left_wrist_pitch_joint",
    "left_wrist_yaw_joint",
    "right_shoulder_pitch_joint",
    "right_shoulder_roll_joint",
    "right_shoulder_yaw_joint",
    "right_elbow_joint",
    "right_wrist_roll_joint",
    "right_wrist_pitch_joint",
    "right_wrist_yaw_joint",
]

PER_JOINT_KEYS = {
    "joint_velocity_abs_radps": "joint_velocity_abs_radps",
    "joint_tracking_abs_rad": "joint_tracking_abs_rad",
    "target_step_abs_rad": "target_step_abs_rad",
    "target_velocity_abs_radps": "target_velocity_abs_radps",
}

FOOT_SIDES = ("left", "right")


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
                print(f"[SonicMujocoMetrics] WARN failed to decode {self.topic}: {exc}", flush=True)
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
    started_at: float | None = None
    ended_at: float | None = None

    def add(self, sample: dict[str, Any]) -> None:
        if self.started_at is None:
            self.started_at = float(sample["monotonic_time"])
        self.ended_at = float(sample["monotonic_time"])
        if sample.pop("_nonfinite", False):
            self.nonfinite_samples += 1
        deploy_index = _optional_int(sample.get("deploy_index"))
        if deploy_index is not None:
            if self.first_deploy_index is None:
                self.first_deploy_index = deploy_index
            self.last_deploy_index = deploy_index
        self.samples.append(sample)

    @property
    def elapsed_s(self) -> float:
        if self.started_at is None or self.ended_at is None:
            return 0.0
        return max(0.0, self.ended_at - self.started_at)


@dataclass
class FootMetricState:
    previous_sim_time_s: float | None = None
    previous_positions: dict[str, np.ndarray] = field(default_factory=dict)
    previous_contacts: dict[str, bool] = field(default_factory=lambda: {side: False for side in FOOT_SIDES})
    support_anchors: dict[str, np.ndarray | None] = field(
        default_factory=lambda: {side: None for side in FOOT_SIDES}
    )


def _build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        description="Collect deploy-side closed-loop metrics for SONIC + MuJoCo.",
        formatter_class=argparse.ArgumentDefaultsHelpFormatter,
    )
    parser.add_argument("--deploy-endpoint", default="tcp://127.0.0.1:6157")
    parser.add_argument("--deploy-topic", default="g1_debug")
    parser.add_argument("--sim-endpoint", help="Optional MuJoCo diagnostic ZMQ endpoint.")
    parser.add_argument("--sim-topic", default="mujoco_metrics")
    parser.add_argument("--duration-s", type=float, default=45.0)
    parser.add_argument("--startup-timeout-s", type=float, default=180.0)
    parser.add_argument("--warmup-s", type=float, default=2.0)
    parser.add_argument("--sample-hz", type=float, default=60.0)
    parser.add_argument("--report-interval-s", type=float, default=2.0)
    parser.add_argument("--summary-json", type=Path)
    parser.add_argument("--samples-jsonl", type=Path)
    parser.add_argument("--top-k-joints", type=int, default=5)
    parser.add_argument(
        "--no-per-joint-jsonl",
        action="store_true",
        help="Omit per-joint arrays from JSONL samples while keeping summary top offenders.",
    )

    parser.add_argument("--min-samples", type=int, default=30)
    parser.add_argument("--min-deploy-fps", type=float, default=15.0)
    parser.add_argument("--min-base-height-m", type=float, default=0.45)
    parser.add_argument("--max-root-tilt-rad", type=float, default=0.85)
    parser.add_argument("--max-joint-rmse-rad", type=float, default=0.75)
    parser.add_argument("--max-joint-velocity-radps", type=float, default=35.0)
    parser.add_argument("--max-joint-velocity-p95-radps", type=float, default=25.0)
    parser.add_argument("--max-target-step-rad", type=float, default=1.25)
    parser.add_argument("--max-target-velocity-radps", type=float, default=45.0)
    parser.add_argument("--max-sim-sample-age-s", type=float, default=0.5)
    parser.add_argument("--max-foot-slip-speed-mps", type=float, default=8.0)
    parser.add_argument("--max-support-foot-drift-m", type=float, default=1.0)
    parser.add_argument("--foot-support-height-margin-m", type=float, default=0.04)
    parser.add_argument("--min-any-foot-contact-ratio", type=float, default=0.05)
    parser.add_argument("--max-no-foot-contact-ratio", type=float, default=0.95)
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


def _normalize_quat_wxyz(quat: np.ndarray) -> np.ndarray:
    norm = float(np.linalg.norm(quat))
    if norm <= 1e-8:
        return np.array([1.0, 0.0, 0.0, 0.0], dtype=np.float32)
    return quat.astype(np.float32) / norm


def _root_tilt_rad(quat_wxyz: np.ndarray) -> float:
    w, x, y, z = [float(v) for v in _normalize_quat_wxyz(quat_wxyz)]
    z_axis_z = 1.0 - 2.0 * (x * x + y * y)
    return float(math.acos(max(-1.0, min(1.0, z_axis_z))))


def _joint_event(prefix: str, values: np.ndarray) -> dict[str, Any]:
    if values.size == 0:
        return {
            f"{prefix}_joint_index": None,
            f"{prefix}_joint_name": None,
            f"{prefix}_value": None,
        }
    idx = int(np.argmax(np.abs(values)))
    return {
        f"{prefix}_joint_index": idx,
        f"{prefix}_joint_name": G1_MUJOCO_JOINT_NAMES[idx],
        f"{prefix}_value": float(values[idx]),
    }


def _bool_ratio(samples: list[dict[str, Any]], key: str) -> float | None:
    values = [sample.get(key) for sample in samples if key in sample]
    if not values:
        return None
    return float(sum(1 for value in values if bool(value)) / len(values))


def _foot_metrics(
    sim_msg: dict[str, Any] | None,
    sim_receive_time: float | None,
    now: float,
    state: FootMetricState,
    support_height_margin_m: float,
) -> dict[str, Any]:
    if sim_msg is None:
        return {"sim_metrics_available": False}

    feet = sim_msg.get("feet")
    if not isinstance(feet, dict):
        return {"sim_metrics_available": False, "sim_metrics_error": "missing feet"}

    sim_time_s = _optional_float(sim_msg.get("sim_time_s"))
    sim_dt_s = None
    if sim_time_s is not None and state.previous_sim_time_s is not None:
        sim_dt_s = max(0.0, sim_time_s - state.previous_sim_time_s)

    out: dict[str, Any] = {
        "sim_metrics_available": True,
        "sim_time_s": sim_time_s,
        "sim_sample_age_s": None if sim_receive_time is None else max(0.0, now - sim_receive_time),
    }
    root_pos = _array(sim_msg.get("root_pos"), (3,))
    if root_pos is not None:
        out["sim_root_height_m"] = float(root_pos[2])
        out["sim_root_xy_m"] = root_pos[:2].tolist()

    any_contact = False
    contact_count = 0
    slip_speeds: list[float] = []
    support_drifts: list[float] = []
    parsed_positions: dict[str, np.ndarray] = {}
    parsed_contacts: dict[str, bool] = {}
    parsed_floor_contacts: dict[str, bool] = {}
    parsed_velocities: dict[str, np.ndarray | None] = {}

    for side in FOOT_SIDES:
        foot = feet.get(side)
        if not isinstance(foot, dict):
            out[f"{side}_foot_metrics_available"] = False
            continue
        pos = _array(foot.get("pos"), (3,))
        if pos is None:
            out[f"{side}_foot_metrics_available"] = False
            continue
        parsed_positions[side] = pos
        parsed_floor_contacts[side] = bool(foot.get("floor_contact", False))
        parsed_velocities[side] = _array(foot.get("vel_world"), (6,))

    if parsed_positions:
        min_foot_height = min(float(pos[2]) for pos in parsed_positions.values())
        out["foot_support_height_threshold_m"] = min_foot_height + max(0.0, float(support_height_margin_m))
    else:
        min_foot_height = math.inf

    for side in FOOT_SIDES:
        pos = parsed_positions.get(side)
        if pos is None:
            continue

        floor_contact = parsed_floor_contacts.get(side, False)
        support_contact = floor_contact or (
            math.isfinite(min_foot_height)
            and float(pos[2]) <= min_foot_height + max(0.0, float(support_height_margin_m))
        )
        vel = parsed_velocities.get(side)
        parsed_contacts[side] = support_contact
        any_contact = any_contact or support_contact
        contact_count += int(support_contact)

        previous_pos = state.previous_positions.get(side)
        delta_speed = None
        if previous_pos is not None and sim_dt_s is not None and sim_dt_s > 1e-6:
            delta_speed = float(np.linalg.norm((pos[:2] - previous_pos[:2]) / sim_dt_s))
        velocity_speed = None
        if vel is not None:
            velocity_speed = float(np.linalg.norm(vel[:2]))
        horizontal_speed = velocity_speed if velocity_speed is not None else delta_speed

        if support_contact:
            if not state.previous_contacts.get(side, False) or state.support_anchors.get(side) is None:
                state.support_anchors[side] = pos[:2].copy()
            anchor = state.support_anchors.get(side)
            support_drift = 0.0 if anchor is None else float(np.linalg.norm(pos[:2] - anchor))
            slip_speed = 0.0 if horizontal_speed is None else horizontal_speed
        else:
            state.support_anchors[side] = None
            support_drift = 0.0
            slip_speed = 0.0

        slip_speeds.append(slip_speed)
        support_drifts.append(support_drift)
        out.update(
            {
                f"{side}_foot_height_m": float(pos[2]),
                f"{side}_foot_floor_contact": floor_contact,
                f"{side}_foot_support_contact": support_contact,
                f"{side}_foot_horizontal_speed_mps": 0.0
                if horizontal_speed is None
                else float(horizontal_speed),
                f"{side}_foot_slip_speed_mps": float(slip_speed),
                f"{side}_support_foot_drift_m": float(support_drift),
            }
        )

    state.previous_positions.update(parsed_positions)
    state.previous_contacts.update(parsed_contacts)
    if sim_time_s is not None:
        state.previous_sim_time_s = sim_time_s

    out.update(
        {
            "any_foot_contact": any_contact,
            "double_support": contact_count == 2,
            "no_foot_contact": contact_count == 0,
            "foot_slip_speed_mps": float(max(slip_speeds)) if slip_speeds else None,
            "support_foot_drift_m": float(max(support_drifts)) if support_drifts else None,
        }
    )
    return out


def _compute_sample(
    msg: dict[str, Any],
    sim_msg: dict[str, Any] | None,
    sim_receive_time: float | None,
    foot_state: FootMetricState,
    previous_target_q: np.ndarray | None,
    previous_sample_time: float | None,
    previous_deploy_index: int | None,
    args: argparse.Namespace,
) -> dict[str, Any]:
    now = time.monotonic()
    target_q = _array(msg.get("body_q_target"), (29,))
    measured_q = _array(msg.get("body_q_measured"), (29,))
    if measured_q is None:
        measured_q = _array(msg.get("body_q"), (29,))
    measured_dq = _array(msg.get("body_dq"), (29,))
    root_pos = _array(msg.get("base_trans_measured"), (3,))
    if root_pos is None:
        root_pos = _array(msg.get("base_trans_target"), (3,))
    root_quat = _array(msg.get("base_quat_measured"), (4,))
    if root_quat is None:
        root_quat = _array(msg.get("base_quat"), (4,))

    required_missing = [
        name
        for name, value in {
            "body_q_target": target_q,
            "body_q_measured/body_q": measured_q,
            "body_dq": measured_dq,
            "base_trans_measured/base_trans_target": root_pos,
            "base_quat_measured/base_quat": root_quat,
        }.items()
        if value is None
    ]
    if required_missing:
        return {
            "monotonic_time": now,
            "wall_time": time.time(),
            "deploy_index": _optional_int(msg.get("index")),
            "missing_fields": required_missing,
            **_foot_metrics(
                sim_msg, sim_receive_time, now, foot_state, args.foot_support_height_margin_m
            ),
            "_nonfinite": True,
        }

    assert target_q is not None
    assert measured_q is not None
    assert measured_dq is not None
    assert root_pos is not None
    assert root_quat is not None

    deploy_index = _optional_int(msg.get("index"))
    deploy_index_delta = (
        None
        if deploy_index is None or previous_deploy_index is None
        else max(0, deploy_index - previous_deploy_index)
    )
    sample_dt_s = None if previous_sample_time is None else max(0.0, now - previous_sample_time)
    joint_error = measured_q - target_q
    joint_error_abs = np.abs(joint_error)
    joint_velocity_abs = np.abs(measured_dq)
    target_step_by_joint = (
        np.zeros_like(target_q, dtype=np.float32)
        if previous_target_q is None
        else np.abs(target_q - previous_target_q)
    )
    target_velocity_by_joint = (
        np.zeros_like(target_q, dtype=np.float32)
        if not sample_dt_s or sample_dt_s <= 1e-6
        else target_step_by_joint / float(sample_dt_s)
    )
    target_step = float(np.max(target_step_by_joint))
    target_velocity = float(np.max(target_velocity_by_joint))
    root_tilt = _root_tilt_rad(root_quat)
    fall_detected = bool(float(root_pos[2]) < args.min_base_height_m or root_tilt > args.max_root_tilt_rad)

    sample: dict[str, Any] = {
        "monotonic_time": now,
        "wall_time": time.time(),
        "deploy_index": deploy_index,
        "deploy_index_delta": deploy_index_delta,
        "sample_dt_s": sample_dt_s,
        "base_height_m": float(root_pos[2]),
        "root_tilt_rad": root_tilt,
        "joint_tracking_rmse_rad": float(np.sqrt(np.mean(np.square(joint_error)))),
        "joint_tracking_absmax_rad": float(np.max(joint_error_abs)),
        "joint_velocity_absmax_radps": float(np.max(joint_velocity_abs)),
        "target_joint_absmax_rad": float(np.max(np.abs(target_q))),
        "target_step_absmax_rad": target_step,
        "target_velocity_absmax_radps": target_velocity,
        "fall_detected": fall_detected,
        "missing_fields": [],
        "peak_joints": {
            **_joint_event("joint_velocity", joint_velocity_abs),
            **_joint_event("joint_tracking", joint_error_abs),
            **_joint_event("target_step", target_step_by_joint),
            **_joint_event("target_velocity", target_velocity_by_joint),
        },
        "per_joint": {
            "joint_velocity_abs_radps": joint_velocity_abs.tolist(),
            "joint_tracking_abs_rad": joint_error_abs.tolist(),
            "target_step_abs_rad": target_step_by_joint.tolist(),
            "target_velocity_abs_radps": target_velocity_by_joint.tolist(),
        },
        **_foot_metrics(
            sim_msg, sim_receive_time, now, foot_state, args.foot_support_height_margin_m
        ),
    }
    sample["_nonfinite"] = any(
        isinstance(value, float) and not math.isfinite(value) for value in sample.values()
    )
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


def _stat_mean(stats: dict[str, dict[str, float | None]], key: str) -> float:
    return float(stats[key]["mean"] if stats[key]["mean"] is not None else math.inf)


def _stat_min(stats: dict[str, dict[str, float | None]], key: str) -> float:
    return float(stats[key]["min"] if stats[key]["min"] is not None else -math.inf)


def _stat_max(stats: dict[str, dict[str, float | None]], key: str) -> float:
    return float(stats[key]["max"] if stats[key]["max"] is not None else math.inf)


def _fps(first_index: int | None, last_index: int | None, elapsed_s: float) -> float:
    if first_index is None or last_index is None or elapsed_s <= 1e-6:
        return 0.0
    return max(0.0, float(last_index - first_index) / elapsed_s)


def _per_joint_rows(samples: list[dict[str, Any]], key: str) -> tuple[np.ndarray, list[dict[str, Any]]]:
    rows: list[list[float]] = []
    source_samples: list[dict[str, Any]] = []
    for sample in samples:
        per_joint = sample.get("per_joint")
        if not isinstance(per_joint, dict):
            continue
        values = per_joint.get(key)
        if not isinstance(values, list) or len(values) != len(G1_MUJOCO_JOINT_NAMES):
            continue
        rows.append([float(value) for value in values])
        source_samples.append(sample)
    if not rows:
        return np.empty((0, len(G1_MUJOCO_JOINT_NAMES)), dtype=np.float64), []
    arr = np.asarray(rows, dtype=np.float64)
    valid_mask = np.all(np.isfinite(arr), axis=1)
    if not np.all(valid_mask):
        arr = arr[valid_mask]
        source_samples = [sample for sample, valid in zip(source_samples, valid_mask) if bool(valid)]
    return arr, source_samples


def _per_joint_top(samples: list[dict[str, Any]], key: str, top_k: int) -> list[dict[str, Any]]:
    arr, source_samples = _per_joint_rows(samples, key)
    if arr.size == 0:
        return []
    max_by_joint = np.max(arr, axis=0)
    mean_by_joint = np.mean(arr, axis=0)
    p95_by_joint = np.percentile(arr, 95, axis=0)
    order = np.argsort(-max_by_joint)[: max(0, top_k)]
    out: list[dict[str, Any]] = []
    for idx_raw in order:
        idx = int(idx_raw)
        max_sample_idx = int(np.argmax(arr[:, idx]))
        source_sample = source_samples[max_sample_idx] if max_sample_idx < len(source_samples) else {}
        out.append(
            {
                "index": idx,
                "name": G1_MUJOCO_JOINT_NAMES[idx],
                "mean": float(mean_by_joint[idx]),
                "p95": float(p95_by_joint[idx]),
                "max": float(max_by_joint[idx]),
                "max_deploy_index": _optional_int(source_sample.get("deploy_index")),
                "max_wall_time": _optional_float(source_sample.get("wall_time")),
            }
        )
    return out


def _pass_fail(acc: MetricsAccumulator, args: argparse.Namespace) -> dict[str, Any]:
    samples = acc.samples
    warmup_s = max(0.0, float(args.warmup_s))
    eval_samples = [
        sample
        for sample in samples
        if acc.started_at is None
        or float(sample.get("monotonic_time", acc.started_at)) - acc.started_at >= warmup_s
    ]
    if not eval_samples:
        eval_samples = samples
    elapsed_s = acc.elapsed_s
    deploy_fps = _fps(acc.first_deploy_index, acc.last_deploy_index, elapsed_s)
    stats = {
        key: _stats(eval_samples, key)
        for key in [
            "base_height_m",
            "root_tilt_rad",
            "joint_tracking_rmse_rad",
            "joint_tracking_absmax_rad",
            "joint_velocity_absmax_radps",
            "target_joint_absmax_rad",
            "target_step_absmax_rad",
            "target_velocity_absmax_radps",
            "deploy_index_delta",
            "sim_sample_age_s",
            "sim_root_height_m",
            "left_foot_height_m",
            "right_foot_height_m",
            "foot_support_height_threshold_m",
            "left_foot_horizontal_speed_mps",
            "right_foot_horizontal_speed_mps",
            "left_foot_slip_speed_mps",
            "right_foot_slip_speed_mps",
            "foot_slip_speed_mps",
            "left_support_foot_drift_m",
            "right_support_foot_drift_m",
            "support_foot_drift_m",
        ]
    }
    fall_frames = sum(1 for sample in samples if sample.get("fall_detected"))
    missing_field_samples = sum(1 for sample in samples if sample.get("missing_fields"))
    sim_requested = bool(args.sim_endpoint)
    sim_eval_samples = [sample for sample in eval_samples if sample.get("sim_metrics_available")]
    contact_ratios = {
        "any_foot_contact": _bool_ratio(sim_eval_samples, "any_foot_contact"),
        "double_support": _bool_ratio(sim_eval_samples, "double_support"),
        "no_foot_contact": _bool_ratio(sim_eval_samples, "no_foot_contact"),
        "left_foot_floor_contact": _bool_ratio(sim_eval_samples, "left_foot_floor_contact"),
        "right_foot_floor_contact": _bool_ratio(sim_eval_samples, "right_foot_floor_contact"),
        "left_foot_support_contact": _bool_ratio(sim_eval_samples, "left_foot_support_contact"),
        "right_foot_support_contact": _bool_ratio(sim_eval_samples, "right_foot_support_contact"),
    }
    checks = {
        "enough_samples": len(samples) >= args.min_samples,
        "deploy_fps": deploy_fps >= args.min_deploy_fps,
        "no_nonfinite_samples": acc.nonfinite_samples == 0,
        "no_missing_fields": missing_field_samples == 0,
        "no_fall_detected": fall_frames == 0,
        "base_height": _stat_min(stats, "base_height_m") >= args.min_base_height_m,
        "root_tilt": _stat_max(stats, "root_tilt_rad") <= args.max_root_tilt_rad,
        "joint_tracking_rmse": _stat_mean(stats, "joint_tracking_rmse_rad") <= args.max_joint_rmse_rad,
        "joint_velocity_peak": _stat_max(stats, "joint_velocity_absmax_radps")
        <= args.max_joint_velocity_radps,
        "joint_velocity_p95": (
            stats["joint_velocity_absmax_radps"]["p95"] is not None
            and float(stats["joint_velocity_absmax_radps"]["p95"]) <= args.max_joint_velocity_p95_radps
        ),
        "target_step_peak": _stat_max(stats, "target_step_absmax_rad") <= args.max_target_step_rad,
        "target_velocity_peak": _stat_max(stats, "target_velocity_absmax_radps")
        <= args.max_target_velocity_radps,
        "sim_metrics_available": (not sim_requested) or len(sim_eval_samples) > 0,
        "sim_sample_fresh": (
            (not sim_requested)
            or (
                stats["sim_sample_age_s"]["max"] is not None
                and float(stats["sim_sample_age_s"]["max"]) <= args.max_sim_sample_age_s
            )
        ),
        "foot_contact_observed": (
            (not sim_requested)
            or (
                contact_ratios["any_foot_contact"] is not None
                and contact_ratios["any_foot_contact"] >= args.min_any_foot_contact_ratio
            )
        ),
        "no_foot_contact_ratio": (
            (not sim_requested)
            or (
                contact_ratios["no_foot_contact"] is not None
                and contact_ratios["no_foot_contact"] <= args.max_no_foot_contact_ratio
            )
        ),
        "foot_slip_speed": (
            (not sim_requested)
            or (
                stats["foot_slip_speed_mps"]["max"] is not None
                and float(stats["foot_slip_speed_mps"]["max"]) <= args.max_foot_slip_speed_mps
            )
        ),
        "support_foot_drift": (
            (not sim_requested)
            or (
                stats["support_foot_drift_m"]["max"] is not None
                and float(stats["support_foot_drift_m"]["max"]) <= args.max_support_foot_drift_m
            )
        ),
    }
    return {
        "pass": all(checks.values()),
        "checks": checks,
        "elapsed_s": elapsed_s,
        "samples": len(samples),
        "eval_samples": len(eval_samples),
        "warmup_s": warmup_s,
        "deploy_fps": deploy_fps,
        "fall_frames": fall_frames,
        "nonfinite_samples": acc.nonfinite_samples,
        "missing_field_samples": missing_field_samples,
        "sim_metrics_requested": sim_requested,
        "sim_metrics_samples": len(sim_eval_samples),
        "foot_contact_ratios": contact_ratios,
        "stats": stats,
        "joint_names_order": G1_MUJOCO_JOINT_NAMES,
        "top_joints": {
            label: _per_joint_top(eval_samples, key, args.top_k_joints)
            for label, key in PER_JOINT_KEYS.items()
        },
        "metric_scope": (
            "deploy g1_debug stream plus optional MuJoCo diagnostic stream; body_q/body_dq are "
            "read from MuJoCo LowState via DDS, base_trans_measured is the deploy debug field, "
            "and foot contact/slip metrics use MuJoCo body poses/contact pairs when sim metrics "
            "are enabled"
        ),
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
        "samples": summary["samples"],
        "deploy_fps": round(float(summary["deploy_fps"]), 2),
        "fall_frames": summary["fall_frames"],
        "joint_rmse_mean": summary["stats"]["joint_tracking_rmse_rad"]["mean"],
        "base_height_min": summary["stats"]["base_height_m"]["min"],
        "root_tilt_max": summary["stats"]["root_tilt_rad"]["max"],
        "target_step_max": summary["stats"]["target_step_absmax_rad"]["max"],
        "target_velocity_max": summary["stats"]["target_velocity_absmax_radps"]["max"],
    }
    if summary.get("sim_metrics_requested"):
        compact.update(
            {
                "foot_slip_speed_max": summary["stats"]["foot_slip_speed_mps"]["max"],
                "support_foot_drift_max": summary["stats"]["support_foot_drift_m"]["max"],
                "any_foot_contact_ratio": summary["foot_contact_ratios"]["any_foot_contact"],
            }
        )
    print(f"[SonicMujocoMetrics] {json.dumps(compact, sort_keys=True)}", flush=True)


def _sample_for_jsonl(sample: dict[str, Any], args: argparse.Namespace) -> dict[str, Any]:
    if not args.no_per_joint_jsonl:
        return sample
    out = dict(sample)
    out.pop("per_joint", None)
    return out


def main() -> int:
    parser = _build_parser()
    args = parser.parse_args()

    ctx = zmq.Context()
    deploy_sub = TopicSubscriber(args.deploy_endpoint, args.deploy_topic, ctx)
    sim_sub = TopicSubscriber(args.sim_endpoint, args.sim_topic, ctx) if args.sim_endpoint else None
    print(
        "[SonicMujocoMetrics] waiting for stream "
        f"deploy={args.deploy_endpoint}/{args.deploy_topic}"
        + ("" if sim_sub is None else f" sim={args.sim_endpoint}/{args.sim_topic}"),
        flush=True,
    )

    try:
        deadline = time.monotonic() + max(0.0, args.startup_timeout_s)
        while deploy_sub.latest is None or (sim_sub is not None and sim_sub.latest is None):
            deploy_sub.poll()
            if sim_sub is not None:
                sim_sub.poll()
            if time.monotonic() >= deadline:
                summary = {
                    "pass": False,
                    "reason": "startup_timeout",
                    "deploy_received": deploy_sub.received,
                    "sim_received": None if sim_sub is None else sim_sub.received,
                }
                _write_json(args.summary_json, summary)
                print(f"[SonicMujocoMetrics] {json.dumps(summary, sort_keys=True)}", flush=True)
                return 2
            time.sleep(0.01)

        acc = MetricsAccumulator()
        duration_deadline = time.monotonic() + max(0.0, args.duration_s)
        sample_period = 1.0 / max(args.sample_hz, 1e-6)
        next_sample_time = 0.0
        next_report_time = time.monotonic() + max(args.report_interval_s, 0.5)
        previous_index: int | None = None
        previous_target_q: np.ndarray | None = None
        previous_sample_time: float | None = None
        foot_state = FootMetricState()

        while time.monotonic() < duration_deadline:
            deploy_sub.poll()
            if sim_sub is not None:
                sim_sub.poll()
            now = time.monotonic()
            if now >= next_sample_time and deploy_sub.latest is not None:
                deploy_index = _optional_int(deploy_sub.latest.get("index"))
                if previous_index != deploy_index:
                    sample = _compute_sample(
                        deploy_sub.latest,
                        None if sim_sub is None else sim_sub.latest,
                        None if sim_sub is None else sim_sub.last_receive_time,
                        foot_state,
                        previous_target_q,
                        previous_sample_time,
                        previous_index,
                        args,
                    )
                    target_q = _array(deploy_sub.latest.get("body_q_target"), (29,))
                    if target_q is not None:
                        previous_target_q = target_q.copy()
                    acc.add(sample)
                    _append_jsonl(args.samples_jsonl, _sample_for_jsonl(sample, args))
                    previous_sample_time = _optional_float(sample.get("monotonic_time"))
                    previous_index = deploy_index
                next_sample_time = now + sample_period
            if now >= next_report_time:
                _print_summary("partial", _pass_fail(acc, args))
                next_report_time = now + max(args.report_interval_s, 0.5)
            time.sleep(0.005)

        summary = _pass_fail(acc, args)
        _write_json(args.summary_json, summary)
        _print_summary("final", summary)
        return 0 if summary["pass"] else 1
    finally:
        deploy_sub.close()
        if sim_sub is not None:
            sim_sub.close()
        ctx.term()


if __name__ == "__main__":
    raise SystemExit(main())
