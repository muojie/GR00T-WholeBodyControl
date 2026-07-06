"""Compare raw Sony BoneData geometry with the v3 Sony-PICO SMPL reference."""

from __future__ import annotations

import argparse
import csv
import json
import sys
from pathlib import Path
from typing import Any

import numpy as np

REPO_ROOT = Path(__file__).resolve().parents[2]
if str(REPO_ROOT) not in sys.path:
    sys.path.insert(0, str(REPO_ROOT))

from gear_sonic.utils.teleop.sources.sony_bonedata_json import (
    SONY_BONEDATA_DEFAULT_JOINTS_PER_FRAME,
    load_sony_bonedata_json_raw,
)
from gear_sonic.utils.teleop.sources.sony_pico_smpl_source import (
    SONY_PICO_BONEDATA_BASES,
    SONY_PICO_BONEDATA_BASIS_BY_NAME,
    SonyPicoSmplConverter,
    apply_rooted_similarity_transform,
    bonedata_positions_to_robot_zup_root_local,
    fit_rooted_similarity_transform,
    read_bonedata_frame_arrays,
)

SMPL_JOINT_NAMES = (
    "pelvis",
    "left_hip",
    "right_hip",
    "spine1",
    "left_knee",
    "right_knee",
    "spine2",
    "left_ankle",
    "right_ankle",
    "spine3",
    "left_foot",
    "right_foot",
    "neck",
    "left_collar",
    "right_collar",
    "head",
    "left_shoulder",
    "right_shoulder",
    "left_elbow",
    "right_elbow",
    "left_wrist",
    "right_wrist",
    "left_hand",
    "right_hand",
)
KEY_JOINTS = {
    "head": 15,
    "neck": 12,
    "left_wrist": 20,
    "right_wrist": 21,
    "left_ankle": 7,
    "right_ankle": 8,
}


def build_arg_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        description=(
            "Load a Sony mocopi saveBoneData/saveBoneAllData JSON, run the v3 "
            "sony_pico PICO-FK path, and compare it with raw BoneData positions."
        )
    )
    parser.add_argument("--json-file", type=Path, required=True)
    parser.add_argument("--out-dir", type=Path, default=Path("tmp/sony_bonedata_v3_compare"))
    parser.add_argument(
        "--bases",
        default=",".join(SONY_PICO_BONEDATA_BASES),
        help="Comma-separated BoneData bases to compare.",
    )
    parser.add_argument("--stride", type=int, default=5, help="Frame sampling stride.")
    parser.add_argument("--calib-frame", type=int, default=0)
    parser.add_argument(
        "--focus-frame",
        type=int,
        action="append",
        default=[],
        help="Extra frame to include. Can be repeated.",
    )
    parser.add_argument("--top-bend-frames", type=int, default=5)
    parser.add_argument("--fps", type=float, default=50.0)
    parser.add_argument(
        "--joints-per-frame",
        type=int,
        default=SONY_BONEDATA_DEFAULT_JOINTS_PER_FRAME,
    )
    return parser


def _read_frame_arrays(motion: Any, frame_idx: int) -> tuple[list[str], np.ndarray, np.ndarray]:
    payload = motion.frame_payload(frame_idx, stream_frame_idx=frame_idx, motion_name="compare")
    return read_bonedata_frame_arrays(payload)


def _raw_root_local(
    motion: Any,
    converter: SonyPicoSmplConverter,
    frame_idx: int,
    basis: np.ndarray,
) -> tuple[list[str], np.ndarray, np.ndarray, np.ndarray]:
    names, positions, quats_xyzw = _read_frame_arrays(motion, frame_idx)
    if converter.slot_indices is None:
        converter.convert(names, positions, quats_xyzw, frame_index=frame_idx, basis=basis)
    assert converter.slot_indices is not None
    raw = bonedata_positions_to_robot_zup_root_local(
        positions,
        converter.slot_indices,
        basis=basis,
    )
    return names, positions, quats_xyzw, raw


def _cosine(a: np.ndarray, b: np.ndarray) -> float:
    denom = float(np.linalg.norm(a) * np.linalg.norm(b))
    if denom < 1e-8:
        return 0.0
    return float(np.dot(a, b) / denom)


def _frame_rms(raw_aligned: np.ndarray, smpl_joints: np.ndarray) -> tuple[float, dict[str, float]]:
    errors = np.linalg.norm(raw_aligned - smpl_joints, axis=1)
    key_errors = {name: float(errors[idx]) for name, idx in KEY_JOINTS.items()}
    return float(np.sqrt(np.mean(errors[:22] ** 2))), key_errors


def _parse_bases(value: str) -> list[str]:
    bases = [item.strip() for item in value.split(",") if item.strip()]
    unknown = sorted(set(bases) - set(SONY_PICO_BONEDATA_BASES))
    if unknown:
        raise ValueError(f"unknown basis names: {unknown}")
    return bases


def compare_basis(
    motion: Any,
    *,
    basis_name: str,
    stride: int,
    calib_frame: int,
    focus_frames: list[int],
    top_bend_frames: int,
) -> tuple[dict[str, Any], list[dict[str, Any]]]:
    basis = SONY_PICO_BONEDATA_BASIS_BY_NAME[basis_name]
    converter = SonyPicoSmplConverter()
    converter.preload()

    frame_count = motion.frame_count
    calib_frame = min(max(0, int(calib_frame)), frame_count - 1)
    _, _, _, calib_raw = _raw_root_local(motion, converter, calib_frame, basis)
    raw_head_heights = []
    for frame_idx in range(frame_count):
        _, _, _, raw = _raw_root_local(motion, converter, frame_idx, basis)
        raw_head_heights.append(float(raw[KEY_JOINTS["head"], 2]))
    raw_head_heights_np = np.asarray(raw_head_heights)
    bend_order = np.argsort(raw_head_heights_np)
    bend_frames = [int(x) for x in bend_order[: max(0, top_bend_frames)]]

    sample_frames = set(range(0, frame_count, max(1, int(stride))))
    sample_frames.add(calib_frame)
    sample_frames.update(bend_frames)
    sample_frames.update(min(max(0, int(x)), frame_count - 1) for x in focus_frames)
    sample_frames_sorted = sorted(sample_frames)

    names, positions, quats_xyzw, raw = _raw_root_local(motion, converter, calib_frame, basis)
    calib_full_body = converter.convert(
        names,
        positions,
        quats_xyzw,
        frame_index=calib_frame,
        basis=basis,
    )
    scale, rotation = fit_rooted_similarity_transform(raw, calib_full_body.smpl_joints)
    calib_raw_aligned = apply_rooted_similarity_transform(raw, scale, rotation)
    calib_smpl = calib_full_body.smpl_joints

    rows: list[dict[str, Any]] = []
    rms_values = []
    key_error_values: dict[str, list[float]] = {name: [] for name in KEY_JOINTS}
    focus_summary: list[dict[str, Any]] = []
    for frame_idx in sample_frames_sorted:
        names, positions, quats_xyzw, raw = _raw_root_local(motion, converter, frame_idx, basis)
        full_body = converter.convert(
            names,
            positions,
            quats_xyzw,
            frame_index=frame_idx,
            basis=basis,
        )
        raw_aligned = apply_rooted_similarity_transform(raw, scale, rotation)
        rms, key_errors = _frame_rms(raw_aligned, full_body.smpl_joints)
        rms_values.append(rms)
        for joint_name, err in key_errors.items():
            key_error_values[joint_name].append(err)

        if frame_idx in bend_frames or frame_idx in focus_frames or frame_idx == calib_frame:
            raw_head_delta = raw_aligned[KEY_JOINTS["head"]] - calib_raw_aligned[KEY_JOINTS["head"]]
            smpl_head_delta = (
                full_body.smpl_joints[KEY_JOINTS["head"]] - calib_smpl[KEY_JOINTS["head"]]
            )
            focus_summary.append(
                {
                    "frame": int(frame_idx),
                    "rms_m": rms,
                    "raw_head_height_m": float(raw[KEY_JOINTS["head"], 2]),
                    "raw_head_drop_m": float(
                        calib_raw[KEY_JOINTS["head"], 2] - raw[KEY_JOINTS["head"], 2]
                    ),
                    "smpl_head_drop_m": float(
                        calib_smpl[KEY_JOINTS["head"], 2]
                        - full_body.smpl_joints[KEY_JOINTS["head"], 2]
                    ),
                    "head_delta_cosine": _cosine(raw_head_delta, smpl_head_delta),
                    "raw_head_delta_norm_m": float(np.linalg.norm(raw_head_delta)),
                    "smpl_head_delta_norm_m": float(np.linalg.norm(smpl_head_delta)),
                }
            )

        for joint_name, joint_idx in KEY_JOINTS.items():
            raw_point = raw_aligned[joint_idx]
            smpl_point = full_body.smpl_joints[joint_idx]
            rows.append(
                {
                    "basis": basis_name,
                    "frame": int(frame_idx),
                    "joint": joint_name,
                    "raw_x": float(raw_point[0]),
                    "raw_y": float(raw_point[1]),
                    "raw_z": float(raw_point[2]),
                    "smpl_x": float(smpl_point[0]),
                    "smpl_y": float(smpl_point[1]),
                    "smpl_z": float(smpl_point[2]),
                    "error_m": float(np.linalg.norm(raw_point - smpl_point)),
                }
            )

    rms_np = np.asarray(rms_values, dtype=np.float64)
    summary = {
        "basis": basis_name,
        "sampled_frames": len(sample_frames_sorted),
        "calib_frame": calib_frame,
        "calibration_scale": float(scale),
        "mean_rms_m": float(np.mean(rms_np)),
        "p95_rms_m": float(np.percentile(rms_np, 95)),
        "max_rms_m": float(np.max(rms_np)),
        "mean_key_error_m": {
            name: float(np.mean(values)) for name, values in key_error_values.items()
        },
        "bend_frames_by_lowest_raw_head": bend_frames,
        "focus": focus_summary,
    }
    return summary, rows


def write_markdown_report(path: Path, report: dict[str, Any]) -> None:
    lines = [
        "# Sony BoneData v3 Geometry Compare",
        "",
        f"- json: `{report['json_file']}`",
        f"- frames: {report['frame_count']}",
        f"- stride: {report['stride']}",
        "",
        "| basis | mean RMS m | p95 RMS m | max RMS m | scale | bend frames |",
        "|---|---:|---:|---:|---:|---|",
    ]
    for item in report["bases"]:
        lines.append(
            "| {basis} | {mean:.3f} | {p95:.3f} | {maxv:.3f} | {scale:.3f} | {frames} |".format(
                basis=item["basis"],
                mean=item["mean_rms_m"],
                p95=item["p95_rms_m"],
                maxv=item["max_rms_m"],
                scale=item["calibration_scale"],
                frames=",".join(str(x) for x in item["bend_frames_by_lowest_raw_head"]),
            )
        )
    lines.extend(
        [
            "",
            "## Focus Frames",
            "",
            "| basis | frame | RMS m | raw head drop m | v3 head drop m | head delta cosine |",
            "|---|---:|---:|---:|---:|---:|",
        ]
    )
    for item in report["bases"]:
        for focus in item["focus"]:
            lines.append(
                "| {basis} | {frame} | {rms:.3f} | {raw_drop:.3f} | "
                "{smpl_drop:.3f} | {cos:.3f} |".format(
                    basis=item["basis"],
                    frame=focus["frame"],
                    rms=focus["rms_m"],
                    raw_drop=focus["raw_head_drop_m"],
                    smpl_drop=focus["smpl_head_drop_m"],
                    cos=focus["head_delta_cosine"],
                )
            )
    path.write_text("\n".join(lines) + "\n", encoding="utf-8")


def main() -> None:
    args = build_arg_parser().parse_args()
    json_file = args.json_file.expanduser().resolve()
    out_dir = args.out_dir.expanduser().resolve()
    out_dir.mkdir(parents=True, exist_ok=True)
    motion = load_sony_bonedata_json_raw(
        json_file,
        joints_per_frame=args.joints_per_frame,
        source_fps=args.fps,
        playback_fps=args.fps,
    )

    summaries = []
    rows = []
    for basis_name in _parse_bases(args.bases):
        summary, basis_rows = compare_basis(
            motion,
            basis_name=basis_name,
            stride=args.stride,
            calib_frame=args.calib_frame,
            focus_frames=args.focus_frame,
            top_bend_frames=args.top_bend_frames,
        )
        summaries.append(summary)
        rows.extend(basis_rows)

    report = {
        "json_file": str(json_file),
        "frame_count": motion.frame_count,
        "joint_count": len(motion.joint_names),
        "stride": args.stride,
        "bases": summaries,
    }
    (out_dir / "summary.json").write_text(
        json.dumps(report, indent=2, ensure_ascii=False) + "\n",
        encoding="utf-8",
    )
    write_markdown_report(out_dir / "summary.md", report)
    with (out_dir / "key_joint_comparison.csv").open("w", encoding="utf-8", newline="") as file:
        fieldnames = [
            "basis",
            "frame",
            "joint",
            "raw_x",
            "raw_y",
            "raw_z",
            "smpl_x",
            "smpl_y",
            "smpl_z",
            "error_m",
        ]
        writer = csv.DictWriter(file, fieldnames=fieldnames)
        writer.writeheader()
        writer.writerows(rows)

    print(f"wrote {out_dir / 'summary.md'}")
    for summary in summaries:
        print(
            "{basis}: mean={mean:.3f}m p95={p95:.3f}m max={maxv:.3f}m bends={bends}".format(
                basis=summary["basis"],
                mean=summary["mean_rms_m"],
                p95=summary["p95_rms_m"],
                maxv=summary["max_rms_m"],
                bends=summary["bend_frames_by_lowest_raw_head"],
            )
        )


if __name__ == "__main__":
    main()
