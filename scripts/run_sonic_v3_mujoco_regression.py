#!/usr/bin/env python3
"""Run or aggregate Sony/BVH SMPL POSE v3 MuJoCo regression metrics."""

from __future__ import annotations

import argparse
import json
import subprocess
import sys
import time
from dataclasses import dataclass
from pathlib import Path
from typing import Any


REPO_ROOT = Path(__file__).resolve().parents[1]
DEFAULT_LAUNCHER = REPO_ROOT / "scripts" / "launch_sonic_v3_mujoco_closed_loop.py"
DEFAULT_SESSION = "sonic_v3_mujoco_tuning"
DEFAULT_CASES = [
    "name=raynos_stable,bvh=/home/nolo/RAYNOS_Motion1.bvh,duration=12,profile=stable",
    "name=mcpm_stable,bvh=/home/nolo/MCPM_20260526_190029.BVH,duration=45,profile=stable",
]


@dataclass(frozen=True)
class RegressionCase:
    name: str
    bvh: Path
    duration_s: float
    profile: str = "stable"


def _build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        description=(
            "Run multiple v3 MuJoCo closed-loop cases and aggregate the metrics summaries. "
            "Unknown args are forwarded to launch_sonic_v3_mujoco_closed_loop.py."
        ),
        formatter_class=argparse.ArgumentDefaultsHelpFormatter,
    )
    parser.add_argument(
        "--case",
        action="append",
        help=(
            "Case spec: name=ID,bvh=/path/file.bvh,duration=SECONDS,profile=stable. "
            "Repeat for multiple cases. Defaults to RAYNOS 12s and MCPM 45s."
        ),
    )
    parser.add_argument(
        "--from-summary",
        action="append",
        help="Aggregate an existing summary without running: name=/tmp/summary.json.",
    )
    parser.add_argument("--output-dir", type=Path, default=Path("/tmp/sony_pose_v3_mujoco_regression"))
    parser.add_argument("--summary-json", type=Path)
    parser.add_argument("--summary-md", type=Path)
    parser.add_argument("--launcher", type=Path, default=DEFAULT_LAUNCHER)
    parser.add_argument("--session", default=DEFAULT_SESSION)
    parser.add_argument("--startup-timeout-s", type=float, default=120.0)
    parser.add_argument("--timeout-margin-s", type=float, default=180.0)
    parser.add_argument("--dry-run", action="store_true")
    parser.add_argument("--keep-sessions", action="store_true")
    return parser


def _parse_kv_spec(spec: str) -> dict[str, str]:
    out: dict[str, str] = {}
    for part in spec.split(","):
        if not part.strip():
            continue
        key, sep, value = part.partition("=")
        if not sep:
            raise ValueError(f"invalid spec segment {part!r}; expected key=value")
        out[key.strip()] = value.strip()
    return out


def _parse_case(spec: str) -> RegressionCase:
    data = _parse_kv_spec(spec)
    missing = [key for key in ["name", "bvh", "duration"] if key not in data]
    if missing:
        raise ValueError(f"case spec missing {', '.join(missing)}: {spec}")
    return RegressionCase(
        name=data["name"],
        bvh=Path(data["bvh"]).expanduser().resolve(),
        duration_s=float(data["duration"]),
        profile=data.get("profile", "stable"),
    )


def _parse_from_summary(spec: str) -> tuple[str, Path]:
    name, sep, path = spec.partition("=")
    if not sep or not name.strip() or not path.strip():
        raise ValueError(f"invalid --from-summary {spec!r}; expected name=/path/summary.json")
    return name.strip(), Path(path.strip()).expanduser().resolve()


def _safe_name(name: str) -> str:
    return "".join(ch if ch.isalnum() or ch in "-_" else "_" for ch in name)


def _kill_session(session: str) -> None:
    subprocess.run(["tmux", "kill-session", "-t", session], stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)


def _launch_command(
    args: argparse.Namespace,
    case: RegressionCase,
    summary_path: Path,
    samples_path: Path,
    extra_args: list[str],
) -> list[str]:
    return [
        str(args.launcher),
        "--session",
        args.session,
        "--replace",
        "--no-attach",
        "--bvh-file",
        str(case.bvh),
        "--pose-filter-profile",
        case.profile,
        "--metrics-duration-s",
        str(case.duration_s),
        "--metrics-startup-timeout-s",
        str(args.startup_timeout_s),
        "--metrics-no-per-joint-jsonl",
        "--metrics-summary-json",
        str(summary_path),
        "--metrics-samples-jsonl",
        str(samples_path),
        *extra_args,
    ]


def _wait_for_summary(path: Path, timeout_s: float) -> dict[str, Any]:
    deadline = time.monotonic() + max(1.0, timeout_s)
    last_error: Exception | None = None
    while time.monotonic() < deadline:
        if path.exists() and path.stat().st_size > 0:
            try:
                return json.loads(path.read_text())
            except json.JSONDecodeError as exc:
                last_error = exc
        time.sleep(1.0)
    if last_error is not None:
        raise TimeoutError(f"summary did not become valid JSON: {path}: {last_error}")
    raise TimeoutError(f"summary did not appear before timeout: {path}")


def _get(summary: dict[str, Any], path: str) -> Any:
    value: Any = summary
    for key in path.split("."):
        if not isinstance(value, dict):
            return None
        value = value.get(key)
    return value


def _fmt(value: Any, digits: int = 3) -> str:
    if value is None:
        return ""
    if isinstance(value, bool):
        return "true" if value else "false"
    if isinstance(value, (int, float)):
        return f"{float(value):.{digits}f}"
    return str(value)


def _row_from_summary(name: str, summary_path: Path, summary: dict[str, Any]) -> dict[str, Any]:
    ratios = summary.get("foot_contact_ratios") if isinstance(summary.get("foot_contact_ratios"), dict) else {}
    checks = summary.get("checks") if isinstance(summary.get("checks"), dict) else {}
    return {
        "name": name,
        "pass": bool(summary.get("pass", False)),
        "failed_checks": [key for key, value in checks.items() if not value],
        "samples": summary.get("samples"),
        "eval_samples": summary.get("eval_samples"),
        "deploy_fps": summary.get("deploy_fps"),
        "joint_velocity_max_radps": _get(summary, "stats.joint_velocity_absmax_radps.max"),
        "joint_velocity_p95_radps": _get(summary, "stats.joint_velocity_absmax_radps.p95"),
        "joint_rmse_mean_rad": _get(summary, "stats.joint_tracking_rmse_rad.mean"),
        "joint_rmse_p95_rad": _get(summary, "stats.joint_tracking_rmse_rad.p95"),
        "root_tilt_max_rad": _get(summary, "stats.root_tilt_rad.max"),
        "target_step_max_rad": _get(summary, "stats.target_step_absmax_rad.max"),
        "target_velocity_max_radps": _get(summary, "stats.target_velocity_absmax_radps.max"),
        "any_foot_contact_ratio": ratios.get("any_foot_contact"),
        "double_support_ratio": ratios.get("double_support"),
        "left_floor_contact_ratio": ratios.get("left_foot_floor_contact"),
        "right_floor_contact_ratio": ratios.get("right_foot_floor_contact"),
        "foot_slip_max_mps": _get(summary, "stats.foot_slip_speed_mps.max"),
        "foot_slip_p95_mps": _get(summary, "stats.foot_slip_speed_mps.p95"),
        "support_drift_max_m": _get(summary, "stats.support_foot_drift_m.max"),
        "support_drift_p95_m": _get(summary, "stats.support_foot_drift_m.p95"),
        "sim_sample_age_p95_s": _get(summary, "stats.sim_sample_age_s.p95"),
        "summary_json": str(summary_path),
    }


def _write_outputs(rows: list[dict[str, Any]], args: argparse.Namespace) -> None:
    args.output_dir.mkdir(parents=True, exist_ok=True)
    summary_json = args.summary_json or (args.output_dir / "regression_summary.json")
    summary_md = args.summary_md or (args.output_dir / "regression_summary.md")

    payload = {
        "created_at": time.strftime("%Y-%m-%d %H:%M:%S"),
        "rows": rows,
        "pass": all(bool(row["pass"]) for row in rows),
    }
    summary_json.parent.mkdir(parents=True, exist_ok=True)
    summary_json.write_text(json.dumps(payload, indent=2, sort_keys=True) + "\n")

    headers = [
        "case",
        "pass",
        "vel max/p95",
        "rmse mean/p95",
        "tilt max",
        "any/double support",
        "floor L/R",
        "slip max/p95",
        "drift max/p95",
        "failed checks",
    ]
    lines = [
        "| " + " | ".join(headers) + " |",
        "| " + " | ".join("---" for _ in headers) + " |",
    ]
    for row in rows:
        lines.append(
            "| "
            + " | ".join(
                [
                    str(row["name"]),
                    _fmt(row["pass"]),
                    f"{_fmt(row['joint_velocity_max_radps'])}/{_fmt(row['joint_velocity_p95_radps'])}",
                    f"{_fmt(row['joint_rmse_mean_rad'])}/{_fmt(row['joint_rmse_p95_rad'])}",
                    _fmt(row["root_tilt_max_rad"]),
                    f"{_fmt(row['any_foot_contact_ratio'])}/{_fmt(row['double_support_ratio'])}",
                    f"{_fmt(row['left_floor_contact_ratio'])}/{_fmt(row['right_floor_contact_ratio'])}",
                    f"{_fmt(row['foot_slip_max_mps'])}/{_fmt(row['foot_slip_p95_mps'])}",
                    f"{_fmt(row['support_drift_max_m'])}/{_fmt(row['support_drift_p95_m'])}",
                    ", ".join(row["failed_checks"]),
                ]
            )
            + " |"
        )
    summary_md.parent.mkdir(parents=True, exist_ok=True)
    summary_md.write_text("\n".join(lines) + "\n")
    print(f"[sonic-v3-regression] summary_json={summary_json}")
    print(f"[sonic-v3-regression] summary_md={summary_md}")


def main(argv: list[str] | None = None) -> int:
    parser = _build_parser()
    args, extra_args = parser.parse_known_args(argv)
    args.output_dir = args.output_dir.expanduser().resolve()
    args.launcher = args.launcher.expanduser().resolve()

    rows: list[dict[str, Any]] = []
    if args.from_summary:
        for spec in args.from_summary:
            name, path = _parse_from_summary(spec)
            summary = json.loads(path.read_text())
            rows.append(_row_from_summary(name, path, summary))
    else:
        cases = [_parse_case(spec) for spec in (args.case or DEFAULT_CASES)]
        for case in cases:
            if not case.bvh.exists():
                raise FileNotFoundError(f"BVH not found for case {case.name}: {case.bvh}")
            case_dir = args.output_dir / _safe_name(case.name)
            summary_path = case_dir / "summary.json"
            samples_path = case_dir / "samples.jsonl"
            cmd = _launch_command(args, case, summary_path, samples_path, extra_args)
            print("[sonic-v3-regression] " + " ".join(cmd))
            if args.dry_run:
                continue
            _kill_session(args.session)
            result = subprocess.run(cmd, cwd=REPO_ROOT, check=False)
            if result.returncode != 0:
                rows.append(
                    {
                        "name": case.name,
                        "pass": False,
                        "failed_checks": [f"launcher_exit_{result.returncode}"],
                        "summary_json": str(summary_path),
                    }
                )
                if not args.keep_sessions:
                    _kill_session(args.session)
                continue
            try:
                timeout_s = args.startup_timeout_s + case.duration_s + args.timeout_margin_s
                summary = _wait_for_summary(summary_path, timeout_s)
                rows.append(_row_from_summary(case.name, summary_path, summary))
            finally:
                if not args.keep_sessions:
                    _kill_session(args.session)

    if args.dry_run and not rows:
        return 0
    _write_outputs(rows, args)
    return 0 if rows and all(bool(row["pass"]) for row in rows) else 1


if __name__ == "__main__":
    raise SystemExit(main())
