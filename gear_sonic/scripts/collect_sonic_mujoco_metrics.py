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


def _build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        description="Collect deploy-side closed-loop metrics for SONIC + MuJoCo.",
        formatter_class=argparse.ArgumentDefaultsHelpFormatter,
    )
    parser.add_argument("--deploy-endpoint", default="tcp://127.0.0.1:6157")
    parser.add_argument("--deploy-topic", default="g1_debug")
    parser.add_argument("--duration-s", type=float, default=45.0)
    parser.add_argument("--startup-timeout-s", type=float, default=180.0)
    parser.add_argument("--sample-hz", type=float, default=20.0)
    parser.add_argument("--report-interval-s", type=float, default=2.0)
    parser.add_argument("--summary-json", type=Path)
    parser.add_argument("--samples-jsonl", type=Path)

    parser.add_argument("--min-samples", type=int, default=30)
    parser.add_argument("--min-deploy-fps", type=float, default=15.0)
    parser.add_argument("--min-base-height-m", type=float, default=0.45)
    parser.add_argument("--max-root-tilt-rad", type=float, default=0.85)
    parser.add_argument("--max-joint-rmse-rad", type=float, default=0.75)
    parser.add_argument("--max-joint-velocity-radps", type=float, default=35.0)
    parser.add_argument("--max-target-step-rad", type=float, default=1.25)
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


def _compute_sample(msg: dict[str, Any], previous_target_q: np.ndarray | None, args: argparse.Namespace) -> dict[str, Any]:
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
            "monotonic_time": time.monotonic(),
            "wall_time": time.time(),
            "deploy_index": _optional_int(msg.get("index")),
            "missing_fields": required_missing,
            "_nonfinite": True,
        }

    assert target_q is not None
    assert measured_q is not None
    assert measured_dq is not None
    assert root_pos is not None
    assert root_quat is not None

    joint_error = measured_q - target_q
    target_step = 0.0 if previous_target_q is None else float(np.max(np.abs(target_q - previous_target_q)))
    root_tilt = _root_tilt_rad(root_quat)
    fall_detected = bool(float(root_pos[2]) < args.min_base_height_m or root_tilt > args.max_root_tilt_rad)

    sample: dict[str, Any] = {
        "monotonic_time": time.monotonic(),
        "wall_time": time.time(),
        "deploy_index": _optional_int(msg.get("index")),
        "base_height_m": float(root_pos[2]),
        "root_tilt_rad": root_tilt,
        "joint_tracking_rmse_rad": float(np.sqrt(np.mean(np.square(joint_error)))),
        "joint_tracking_absmax_rad": float(np.max(np.abs(joint_error))),
        "joint_velocity_absmax_radps": float(np.max(np.abs(measured_dq))),
        "target_joint_absmax_rad": float(np.max(np.abs(target_q))),
        "target_step_absmax_rad": target_step,
        "fall_detected": fall_detected,
        "missing_fields": [],
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


def _pass_fail(acc: MetricsAccumulator, args: argparse.Namespace) -> dict[str, Any]:
    samples = acc.samples
    elapsed_s = acc.elapsed_s
    deploy_fps = _fps(acc.first_deploy_index, acc.last_deploy_index, elapsed_s)
    stats = {
        key: _stats(samples, key)
        for key in [
            "base_height_m",
            "root_tilt_rad",
            "joint_tracking_rmse_rad",
            "joint_tracking_absmax_rad",
            "joint_velocity_absmax_radps",
            "target_joint_absmax_rad",
            "target_step_absmax_rad",
        ]
    }
    fall_frames = sum(1 for sample in samples if sample.get("fall_detected"))
    missing_field_samples = sum(1 for sample in samples if sample.get("missing_fields"))
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
        "target_step_peak": _stat_max(stats, "target_step_absmax_rad") <= args.max_target_step_rad,
    }
    return {
        "pass": all(checks.values()),
        "checks": checks,
        "elapsed_s": elapsed_s,
        "samples": len(samples),
        "deploy_fps": deploy_fps,
        "fall_frames": fall_frames,
        "nonfinite_samples": acc.nonfinite_samples,
        "missing_field_samples": missing_field_samples,
        "stats": stats,
        "metric_scope": (
            "deploy g1_debug stream; body_q/body_dq are read from MuJoCo LowState via DDS, "
            "base_trans_measured is the deploy debug field"
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
    }
    print(f"[SonicMujocoMetrics] {json.dumps(compact, sort_keys=True)}", flush=True)


def main() -> int:
    parser = _build_parser()
    args = parser.parse_args()

    ctx = zmq.Context()
    deploy_sub = TopicSubscriber(args.deploy_endpoint, args.deploy_topic, ctx)
    print(
        "[SonicMujocoMetrics] waiting for stream "
        f"deploy={args.deploy_endpoint}/{args.deploy_topic}",
        flush=True,
    )

    try:
        deadline = time.monotonic() + max(0.0, args.startup_timeout_s)
        while deploy_sub.latest is None:
            deploy_sub.poll()
            if time.monotonic() >= deadline:
                summary = {
                    "pass": False,
                    "reason": "startup_timeout",
                    "deploy_received": deploy_sub.received,
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

        while time.monotonic() < duration_deadline:
            deploy_sub.poll()
            now = time.monotonic()
            if now >= next_sample_time and deploy_sub.latest is not None:
                deploy_index = _optional_int(deploy_sub.latest.get("index"))
                if previous_index != deploy_index:
                    sample = _compute_sample(deploy_sub.latest, previous_target_q, args)
                    target_q = _array(deploy_sub.latest.get("body_q_target"), (29,))
                    if target_q is not None:
                        previous_target_q = target_q.copy()
                    acc.add(sample)
                    _append_jsonl(args.samples_jsonl, sample)
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
        ctx.term()


if __name__ == "__main__":
    raise SystemExit(main())
