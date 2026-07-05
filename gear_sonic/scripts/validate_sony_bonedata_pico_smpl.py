# Offline validation: Sony mocopi saveBoneData JSON -> PICO manager SMPL stack.
#
# Route-C feasibility check for feeding mocopi data through the *PICO* conversion
# stack (compute_from_body_poses + process_smpl_joints from
# pico_manager_thread_server.py) instead of the BVH-G1 retarget v1 route.
#
# What it does:
#   1. Loads a raw saveBoneData JSON (27 mocopi bones, Unity Y-up, global poses).
#   2. Maps the 27 bones into the 24 SMPL slots the PICO XRT reader produces.
#   3. Runs the exact PICO functions to get smpl_pose / smpl_joints / body_quat.
#   4. Scores geometry assertions on labeled key frames (from the recording's
#      action table) plus yaw-turn and continuity checks over the sequence.
#   5. Renders skeleton snapshots and time-series plots for eyeball checks.
#
# Usage:
#   python gear_sonic/scripts/validate_sony_bonedata_pico_smpl.py \
#     --json-file "D:/mocopi_recordings/saveBoneData_Yup20260702.json" \
#     --out-dir "D:/mocopi_recordings/pico_smpl_validation"

from __future__ import annotations

import argparse
import json
import math
from dataclasses import dataclass, field
from pathlib import Path

import numpy as np
from scipy.spatial.transform import Rotation as sRot
import torch

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt

from gear_sonic.scripts.pico_manager_thread_server import compute_from_body_poses
from gear_sonic.utils.teleop.sources.sony_pico_smpl_source import (
    PICO_SMPL_PARENT_INDICES as PICO_PARENT_INDICES,
    UNITY_TO_XRT_BASIS,
    resolve_smpl_slot_bone_indices,
)

# mocopi saveBoneData bone order (matches MOCOPI_BONE_NAMES ids 0..26).
MOCOPI_JOINTS_PER_FRAME = 27

SMPL_BONES = [
    (0, 1), (0, 2), (0, 3), (1, 4), (2, 5), (3, 6), (4, 7), (5, 8), (6, 9),
    (7, 10), (8, 11), (9, 12), (12, 15), (9, 13), (9, 14), (13, 16), (14, 17),
    (16, 18), (17, 19), (18, 20), (19, 21),
]

MOCOPI_BONES = [
    (0, 1), (1, 2), (2, 3), (3, 4), (4, 5), (5, 6), (6, 7), (7, 8), (8, 9), (9, 10),
    (7, 11), (11, 12), (12, 13), (13, 14),
    (7, 15), (15, 16), (16, 17), (17, 18),
    (0, 19), (19, 20), (20, 21), (21, 22),
    (0, 23), (23, 24), (24, 25), (25, 26),
]

L_WRIST, R_WRIST, HEAD, PELVIS, L_SHOULDER, R_SHOULDER, L_ANKLE, R_ANKLE = 20, 21, 15, 0, 16, 17, 7, 8


@dataclass
class BoneDataMotion:
    positions: np.ndarray  # (T, 27, 3) Unity world positions
    quats_xyzw: np.ndarray  # (T, 27, 4) Unity world rotations, scalar-last
    fps: float
    smpl_slot_indices: np.ndarray  # (24,) bone row per SMPL slot

    @property
    def frame_count(self) -> int:
        return self.positions.shape[0]


def load_bonedata(json_file: Path) -> BoneDataMotion:
    with json_file.open("r", encoding="utf-8-sig") as f:
        data = json.load(f)
    n = MOCOPI_JOINTS_PER_FRAME
    total = len(data["position"])
    if total % n != 0:
        raise ValueError(f"position length {total} not divisible by {n}")
    frames = total // n
    pos = np.array(
        [[p["x"], p["y"], p["z"]] for p in data["position"]], dtype=np.float64
    ).reshape(frames, n, 3)
    quat = np.array(
        [[q["x"], q["y"], q["z"], q["w"]] for q in data["rotation"]], dtype=np.float64
    ).reshape(frames, n, 4)
    fps = float(data.get("playbackFps", 50.0))
    slot_indices = resolve_smpl_slot_bone_indices([str(x) for x in data["name"][:n]])
    return BoneDataMotion(
        positions=pos, quats_xyzw=quat, fps=fps, smpl_slot_indices=slot_indices
    )


WORLD_FIX_BASES = {
    # Rotations of the Unity world frame.
    "y180": np.diag([-1.0, 1.0, -1.0]),
    # Handedness flips (det=-1): B @ R @ B.T stays a valid rotation. "zflip"
    # converts Unity left-handed (y-up, z-forward) into the right-handed
    # convention the XRT SDK delivers to the PICO stack; it is the fix the
    # production SonyPicoSmplUdpSource applies.
    "zflip": UNITY_TO_XRT_BASIS,
    "xflip": np.diag([-1.0, 1.0, 1.0]),
}


def apply_world_fix(motion: BoneDataMotion, fix: str) -> BoneDataMotion:
    """Optionally re-express all world poses in a transformed world basis."""
    if fix == "none":
        return motion
    basis = WORLD_FIX_BASES.get(fix)
    if basis is None:
        raise ValueError(f"unknown world fix {fix!r}")
    pos = motion.positions @ basis.T
    rots = sRot.from_quat(motion.quats_xyzw.reshape(-1, 4)).as_matrix()
    rots = basis @ rots @ basis.T
    quat = sRot.from_matrix(rots).as_quat().reshape(motion.quats_xyzw.shape)
    return BoneDataMotion(
        positions=pos,
        quats_xyzw=quat,
        fps=motion.fps,
        smpl_slot_indices=motion.smpl_slot_indices,
    )


def bonedata_frame_to_xrt(motion: BoneDataMotion, frame_idx: int) -> np.ndarray:
    """Build the (24, 7) [x,y,z,qx,qy,qz,qw] array PicoReader would deliver."""
    rows = motion.smpl_slot_indices
    return np.concatenate(
        [motion.positions[frame_idx, rows], motion.quats_xyzw[frame_idx, rows]], axis=1
    )


@dataclass
class PicoStackOutput:
    joints_world: np.ndarray  # (T, 24, 3) SMPL FK joints, z-up, root orient applied
    joints_local: np.ndarray  # (T, 24, 3) root-local joints (v3 smpl_joints field)
    body_quat_wxyz: np.ndarray  # (T, 4) global orient after base-rot removal
    smpl_pose: np.ndarray  # (T, 21, 3) local axis-angle (v3 smpl_pose field)
    frame_indices: np.ndarray  # (T,) source frame numbers
    fps: float


def run_pico_stack(motion: BoneDataMotion, frame_indices: np.ndarray) -> PicoStackOutput:
    device = torch.device("cpu")
    joints_world, joints_local, body_quat, smpl_pose = [], [], [], []
    for fi in frame_indices:
        body_poses_np = bonedata_frame_to_xrt(motion, int(fi))
        data = compute_from_body_poses(PICO_PARENT_INDICES, device, body_poses_np)
        joints_world.append(data["joints"].detach().cpu().numpy()[0])
        joints_local.append(data["smpl_joints_local"].detach().cpu().numpy()[0])
        body_quat.append(data["global_orient_quat"].detach().cpu().numpy()[0])
        smpl_pose.append(
            data["smpl_pose"].detach().cpu().numpy()[0][:63].reshape(21, 3)
        )
    return PicoStackOutput(
        joints_world=np.array(joints_world),
        joints_local=np.array(joints_local),
        body_quat_wxyz=np.array(body_quat),
        smpl_pose=np.array(smpl_pose),
        frame_indices=frame_indices.copy(),
        fps=motion.fps,
    )


def yaw_of_quat_wxyz(quat_wxyz: np.ndarray) -> float:
    """Heading yaw (rad) of the rotated x-axis projected on the xy plane."""
    x_axis = sRot.from_quat(quat_wxyz, scalar_first=True).apply([1.0, 0.0, 0.0])
    return math.atan2(x_axis[1], x_axis[0])


def heading_local_joints(output: PicoStackOutput, row: int) -> np.ndarray:
    """World FK joints with the body heading yaw removed (x fwd, y left, z up)."""
    yaw = yaw_of_quat_wxyz(output.body_quat_wxyz[row])
    r_inv = sRot.from_euler("z", -yaw)
    return r_inv.apply(output.joints_world[row])


# --- Key-frame geometry assertions --------------------------------------------
# Checks run on heading-local joints (yaw removed, pitch/roll kept; x fwd,
# y left, z up). Each key pose has a time WINDOW from the operator action table
# ("times are approximate"); the scorer picks the best-matching frame inside
# the window. Bounds are deliberately loose: they test direction semantics, not
# millimetre accuracy (SMPL FK uses standard SMPL bone lengths, not the actor's).

AXIS = {"x": 0, "y": 1, "z": 2}


@dataclass
class KeyFrameCheck:
    name: str
    window_s: tuple  # (t0, t1) scan window in seconds
    checks: list = field(default_factory=list)  # (label, fn(j) -> bool, detail_fn)
    best_frame: int = -1  # filled in by evaluate_keyframes


def build_keyframe_checks() -> list[KeyFrameCheck]:
    def between(joint, axis, lo, hi):
        a = AXIS[axis]

        def fn(j):
            return lo <= j[joint, a] <= hi

        def detail(j):
            return f"{j[joint, a]:+.3f} in [{lo:+.2f}, {hi:+.2f}]"

        return fn, detail

    def rel(joint_a, joint_b, axis, lo, hi):
        a = AXIS[axis]

        def fn(j):
            return lo <= j[joint_a, a] - j[joint_b, a] <= hi

        def detail(j):
            return f"{j[joint_a, a] - j[joint_b, a]:+.3f} in [{lo:+.2f}, {hi:+.2f}]"

        return fn, detail

    def forward_reach(wrist, shoulder, min_reach=0.35, half_sector_deg=60.0):
        """Wrist extended horizontally from the shoulder, roughly forward.

        The action table describes 'front raise' from a world viewpoint while
        heading-local x follows the pelvis, so allow a generous sector: torso
        twist during a one-arm hold easily shifts the apparent direction.
        """

        def fn(j):
            d = j[wrist] - j[shoulder]
            reach = math.hypot(d[0], d[1])
            ang = math.degrees(math.atan2(d[1], d[0]))
            return reach >= min_reach and abs(ang) <= half_sector_deg

        def detail(j):
            d = j[wrist] - j[shoulder]
            return (f"reach={math.hypot(d[0], d[1]):.3f} (>= {min_reach}), "
                    f"dir={math.degrees(math.atan2(d[1], d[0])):+.0f}deg (|.|<={half_sector_deg:.0f})")

        return fn, detail

    kf = []

    c = KeyFrameCheck("still_stand", (0.5, 3.0))
    c.checks = [
        ("head above pelvis", *rel(HEAD, PELVIS, "z", 0.40, 0.85)),
        ("ankles below pelvis", *rel(L_ANKLE, PELVIS, "z", -1.10, -0.60)),
        ("L wrist hangs (z)", *rel(L_WRIST, L_SHOULDER, "z", -0.70, -0.25)),
        ("R wrist hangs (z)", *rel(R_WRIST, R_SHOULDER, "z", -0.70, -0.25)),
        ("L wrist near body (x)", *between(L_WRIST, "x", -0.35, 0.35)),
        ("R wrist near body (x)", *between(R_WRIST, "x", -0.35, 0.35)),
    ]
    kf.append(c)

    def side_raise_checks():
        return [
            ("L wrist out left (+y)", *between(L_WRIST, "y", 0.40, 1.00)),
            ("R wrist out right (-y)", *between(R_WRIST, "y", -1.00, -0.40)),
            ("L wrist lifted (z)", *rel(L_WRIST, L_SHOULDER, "z", -0.40, 0.25)),
            ("R wrist lifted (z)", *rel(R_WRIST, R_SHOULDER, "z", -0.40, 0.25)),
        ]

    c = KeyFrameCheck("side_raise_1", (4.8, 6.8))
    c.checks = side_raise_checks()
    kf.append(c)

    c = KeyFrameCheck("side_raise_5", (62.0, 66.0))
    c.checks = side_raise_checks()
    kf.append(c)

    c = KeyFrameCheck("right_front_raise", (26.0, 31.8))
    c.checks = [
        ("R wrist reaches forward", *forward_reach(R_WRIST, R_SHOULDER)),
        ("R wrist at shoulder height", *rel(R_WRIST, R_SHOULDER, "z", -0.35, 0.25)),
        ("L wrist still hangs", *rel(L_WRIST, L_SHOULDER, "z", -0.70, -0.20)),
    ]
    kf.append(c)

    c = KeyFrameCheck("both_front_raise", (67.5, 72.0))
    c.checks = [
        ("L wrist reaches forward", *forward_reach(L_WRIST, L_SHOULDER)),
        ("R wrist reaches forward", *forward_reach(R_WRIST, R_SHOULDER)),
        ("L wrist at shoulder height", *rel(L_WRIST, L_SHOULDER, "z", -0.35, 0.30)),
        ("R wrist at shoulder height", *rel(R_WRIST, R_SHOULDER, "z", -0.35, 0.30)),
    ]
    kf.append(c)

    c = KeyFrameCheck("overhead", (83.0, 86.2))
    c.checks = [
        ("L wrist high (z)", *between(L_WRIST, "z", 0.35, 1.00)),
        ("R wrist high (z)", *between(R_WRIST, "z", 0.35, 1.00)),
        ("L wrist near head height", *rel(L_WRIST, HEAD, "z", -0.30, 0.45)),
        ("R wrist near head height", *rel(R_WRIST, HEAD, "z", -0.30, 0.45)),
    ]
    kf.append(c)

    c = KeyFrameCheck("deep_bend", (89.5, 91.5))
    c.checks = [
        ("head forward (+x)", *between(HEAD, "x", 0.30, 0.90)),
        ("head dropped (z)", *between(HEAD, "z", -0.20, 0.45)),
        ("wrists hang forward-low", *between(R_WRIST, "z", -0.90, 0.10)),
    ]
    kf.append(c)

    return kf


# Expected yaw turns from the action table: (t_start, t_end, delta_deg, tol_deg).
YAW_TURNS = [
    (49.5, 53.0, +88.0, 30.0),
    (53.5, 57.5, +92.0, 30.0),
    (91.5, 94.3, +76.0, 30.0),
    (98.4, 101.0, -73.0, 30.0),
    (118.5, 124.5, -40.0, 25.0),
    (124.4, 126.2, +30.0, 20.0),
]


def evaluate_keyframes(output: PicoStackOutput, keyframes: list[KeyFrameCheck]):
    """Scan each key-pose window and score its best-matching frame."""
    times = output.frame_indices / output.fps
    results, passed, total = [], 0, 0
    for kf in keyframes:
        rows = np.nonzero((times >= kf.window_s[0]) & (times <= kf.window_s[1]))[0]
        if len(rows) == 0:
            raise ValueError(f"no computed frames inside window of {kf.name}")
        best_row, best_score, best_j = -1, -1, None
        for row in rows:
            j = heading_local_joints(output, int(row))
            score = sum(bool(fn(j)) for _, fn, _ in kf.checks)
            if score > best_score:
                best_row, best_score, best_j = int(row), score, j
        kf.best_frame = int(output.frame_indices[best_row])
        entry = {
            "keyframe": kf.name,
            "window_s": list(kf.window_s),
            "best_frame": kf.best_frame,
            "checks": [],
        }
        for label, fn, detail in kf.checks:
            ok = bool(fn(best_j))
            entry["checks"].append({"label": label, "pass": ok, "detail": detail(best_j)})
            passed += int(ok)
            total += 1
        results.append(entry)
    return results, passed, total


def evaluate_yaw_turns(output: PicoStackOutput):
    yaws = np.unwrap([yaw_of_quat_wxyz(q) for q in output.body_quat_wxyz])
    times = output.frame_indices / output.fps
    results, passed = [], 0
    for t0, t1, expected, tol in YAW_TURNS:
        i0 = int(np.searchsorted(times, t0))
        i1 = min(int(np.searchsorted(times, t1)), len(yaws) - 1)
        delta = math.degrees(yaws[i1] - yaws[i0])
        ok = abs(delta - expected) <= tol
        passed += int(ok)
        results.append(
            {
                "window_s": [t0, t1],
                "expected_deg": expected,
                "measured_deg": round(delta, 1),
                "pass": bool(ok),
            }
        )
    return results, passed, len(YAW_TURNS), yaws, times


def evaluate_continuity(output: PicoStackOutput):
    """Per-source-frame max joint axis-angle delta; flags pops in smpl_pose."""
    stride = np.diff(output.frame_indices).astype(np.float64)
    d_pose = np.linalg.norm(np.diff(output.smpl_pose, axis=0), axis=2).max(axis=1)
    per_frame = d_pose / np.maximum(stride, 1.0)
    return {
        "max_pose_delta_rad_per_frame": float(per_frame.max()),
        "p99_pose_delta_rad_per_frame": float(np.percentile(per_frame, 99)),
        "frames_over_0p35": int((per_frame > 0.35).sum()),
    }


def evaluate_wrist_height_correlation(motion: BoneDataMotion, output: PicoStackOutput):
    """Correlate mocopi raw hand height (Unity y) with SMPL FK wrist z."""
    rows = output.frame_indices
    raw_l = motion.positions[rows, 14, 1]
    raw_r = motion.positions[rows, 18, 1]
    fk_l = output.joints_world[:, L_WRIST, 2]
    fk_r = output.joints_world[:, R_WRIST, 2]
    return {
        "left_pearson_r": float(np.corrcoef(raw_l, fk_l)[0, 1]),
        "right_pearson_r": float(np.corrcoef(raw_r, fk_r)[0, 1]),
    }


def evaluate_arm_elevation_gain(motion: BoneDataMotion, output: PicoStackOutput):
    """Rotation-chain arm amplitude vs mocopi's sensor-driven hand positions.

    mocopi measures the wrists directly (wristband sensors) but estimates the
    intermediate arm rotations via internal IK, so the rotation chain tends to
    under-swing. Gain < 1 quantifies how much amplitude a pure-rotation route
    (this PICO stack) loses relative to the measured hand trajectory.
    """
    rows = output.frame_indices
    result = {}
    for side, moco_hand, moco_shoulder, smpl_wrist, smpl_shoulder in (
        ("left", 14, 12, L_WRIST, L_SHOULDER),
        ("right", 18, 16, R_WRIST, R_SHOULDER),
    ):
        d_moco = motion.positions[rows, moco_hand] - motion.positions[rows, moco_shoulder]
        d_smpl = output.joints_world[:, smpl_wrist] - output.joints_world[:, smpl_shoulder]
        # Elevation above the hanging pose, in each system's own up axis.
        elev_moco = np.arcsin(np.clip(d_moco[:, 1] / np.linalg.norm(d_moco, axis=1), -1, 1))
        elev_smpl = np.arcsin(np.clip(d_smpl[:, 2] / np.linalg.norm(d_smpl, axis=1), -1, 1))
        elev_moco -= elev_moco.min()
        elev_smpl -= elev_smpl.min()
        # Least-squares gain through the origin: how much of the measured
        # elevation the rotation chain reproduces.
        gain = float(np.dot(elev_smpl, elev_moco) / max(np.dot(elev_moco, elev_moco), 1e-9))
        result[f"{side}_elevation_gain"] = round(gain, 3)
        result[f"{side}_elevation_r"] = round(float(np.corrcoef(elev_moco, elev_smpl)[0, 1]), 3)
    return result


# --- Plots ---------------------------------------------------------------------


def plot_keyframes(output, keyframes, out_path: Path, title: str):
    frame_to_row = {int(f): i for i, f in enumerate(output.frame_indices)}
    fig, axes = plt.subplots(2, len(keyframes), figsize=(3.1 * len(keyframes), 6.6))
    for col, kf in enumerate(keyframes):
        j = heading_local_joints(output, frame_to_row[kf.best_frame])
        for rowi, (a, b, xlabel) in enumerate([(1, 2, "y (left+)"), (0, 2, "x (fwd+)")]):
            ax = axes[rowi][col]
            for p, q in SMPL_BONES:
                ax.plot([j[p, a], j[q, a]], [j[p, b], j[q, b]], "-", lw=1.6, color="tab:blue")
            ax.scatter(j[[L_WRIST], a], j[[L_WRIST], b], c="tab:green", s=28, zorder=3, label="L wrist")
            ax.scatter(j[[R_WRIST], a], j[[R_WRIST], b], c="tab:red", s=28, zorder=3, label="R wrist")
            ax.scatter(j[[HEAD], a], j[[HEAD], b], c="tab:orange", s=28, zorder=3, label="head")
            ax.set_xlim(-1.0, 1.0)
            ax.set_ylim(-1.1, 1.1)
            ax.set_aspect("equal")
            ax.grid(alpha=0.3)
            if rowi == 0:
                ax.set_title(
                    f"{kf.name}\nf={kf.best_frame} t={kf.best_frame / output.fps:.1f}s",
                    fontsize=9,
                )
            if rowi == 0 and a == 1:
                ax.invert_xaxis()  # front view: +y (left) on the image's left side
            ax.set_xlabel(xlabel, fontsize=8)
            if col == 0:
                ax.set_ylabel("z (up)", fontsize=8)
    axes[0][0].legend(fontsize=7, loc="lower right")
    fig.suptitle(title, fontsize=11)
    fig.tight_layout()
    fig.savefig(out_path, dpi=130)
    plt.close(fig)


def plot_mocopi_reference(motion, keyframes, out_path: Path):
    """Raw mocopi skeleton (Unity y-up -> display z-up) with heading removed."""
    fig, axes = plt.subplots(2, len(keyframes), figsize=(3.1 * len(keyframes), 6.6))
    for col, kf in enumerate(keyframes):
        pos_u = motion.positions[kf.best_frame]
        root = pos_u[0]
        # Unity (x, y, z) y-up -> display (x, z, y) z-up, root-centered.
        pts = np.stack([pos_u[:, 0] - root[0], pos_u[:, 2] - root[2], pos_u[:, 1] - root[1]], axis=1)
        r_root = sRot.from_quat(motion.quats_xyzw[kf.best_frame, 0])
        fwd_u = r_root.apply([0.0, 0.0, 1.0])
        yaw = math.atan2(fwd_u[0], fwd_u[2])  # heading about Unity y
        rz = sRot.from_euler("z", yaw)  # display-frame yaw removal
        pts = rz.apply(pts)
        for rowi, (a, b, xlabel) in enumerate([(1, 2, "side"), (0, 2, "front-ish")]):
            ax = axes[rowi][col]
            for p, q in MOCOPI_BONES:
                ax.plot([pts[p, a], pts[q, a]], [pts[p, b], pts[q, b]], "-", lw=1.4, color="tab:purple")
            ax.scatter(pts[[14], a], pts[[14], b], c="tab:green", s=26, zorder=3)
            ax.scatter(pts[[18], a], pts[[18], b], c="tab:red", s=26, zorder=3)
            ax.set_xlim(-1.0, 1.0)
            ax.set_ylim(-1.1, 1.1)
            ax.set_aspect("equal")
            ax.grid(alpha=0.3)
            if rowi == 0:
                ax.set_title(f"{kf.name}\nf={kf.best_frame}", fontsize=9)
            ax.set_xlabel(xlabel, fontsize=8)
            if col == 0:
                ax.set_ylabel("up", fontsize=8)
    fig.suptitle("mocopi raw skeleton reference (green=L hand, red=R hand)", fontsize=11)
    fig.tight_layout()
    fig.savefig(out_path, dpi=130)
    plt.close(fig)


def plot_timeseries(output, motion, yaws, times, yaw_results, out_path: Path):
    fig, axes = plt.subplots(2, 1, figsize=(13, 7), sharex=True)

    ax = axes[0]
    ax.plot(times, np.degrees(yaws), lw=1.2, color="tab:blue", label="body yaw (deg)")
    for r in yaw_results:
        t0, t1 = r["window_s"]
        color = "tab:green" if r["pass"] else "tab:red"
        ax.axvspan(t0, t1, alpha=0.15, color=color)
        ax.text((t0 + t1) / 2, ax.get_ylim()[1] * 0.9,
                f"exp {r['expected_deg']:+.0f}\ngot {r['measured_deg']:+.1f}",
                ha="center", fontsize=7, color=color)
    ax.set_ylabel("yaw (deg)")
    ax.grid(alpha=0.3)
    ax.legend(loc="lower left", fontsize=8)

    ax = axes[1]
    rows = output.frame_indices
    ax.plot(times, motion.positions[rows, 14, 1], lw=1.0, color="tab:green", alpha=0.7, label="mocopi L hand height (m)")
    ax.plot(times, motion.positions[rows, 18, 1], lw=1.0, color="tab:red", alpha=0.7, label="mocopi R hand height (m)")
    ax.plot(times, output.joints_world[:, L_WRIST, 2] + 0.95, "--", lw=1.0, color="darkgreen", label="SMPL L wrist z + 0.95")
    ax.plot(times, output.joints_world[:, R_WRIST, 2] + 0.95, "--", lw=1.0, color="darkred", label="SMPL R wrist z + 0.95")
    ax.set_ylabel("height (m)")
    ax.set_xlabel("time (s)")
    ax.grid(alpha=0.3)
    ax.legend(loc="upper left", fontsize=8)

    fig.suptitle("Sony BoneData -> PICO SMPL stack: heading & wrist-height consistency", fontsize=11)
    fig.tight_layout()
    fig.savefig(out_path, dpi=130)
    plt.close(fig)


# --- Main ----------------------------------------------------------------------


def keyframe_window_frames(keyframes, fps: float, stride: int) -> np.ndarray:
    frames = []
    for kf in keyframes:
        f0, f1 = int(kf.window_s[0] * fps), int(kf.window_s[1] * fps)
        frames.append(np.arange(f0, f1 + 1, stride))
    return np.unique(np.concatenate(frames))


def score_world_fix(motion, keyframes) -> dict[str, int]:
    """Run key-pose windows under each world-fix candidate, return pass counts."""
    scores = {}
    frames = keyframe_window_frames(keyframes, motion.fps, stride=10)
    for fix in ("none", "y180", "zflip", "xflip"):
        fixed = apply_world_fix(motion, fix)
        out = run_pico_stack(fixed, frames)
        _, passed, _ = evaluate_keyframes(out, keyframes)
        scores[fix] = passed
    return scores


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--json-file", required=True)
    parser.add_argument("--out-dir", required=True)
    parser.add_argument(
        "--world-fix", default="auto", choices=["auto", "none", "y180", "zflip", "xflip"]
    )
    parser.add_argument("--stride", type=int, default=5, help="frame stride for the full-sequence pass")
    parser.add_argument("--max-seconds", type=float, default=139.0, help="cut before tracking-loss tail")
    args = parser.parse_args()

    out_dir = Path(args.out_dir)
    out_dir.mkdir(parents=True, exist_ok=True)

    motion = load_bonedata(Path(args.json_file))
    print(f"[load] {motion.frame_count} frames @ {motion.fps:.0f} fps "
          f"({motion.frame_count / motion.fps:.1f}s)")

    keyframes = build_keyframe_checks()

    if args.world_fix == "auto":
        scores = score_world_fix(motion, keyframes)
        chosen = max(scores, key=scores.get)
        print(f"[world-fix] scores {scores} -> using '{chosen}'")
    else:
        chosen = args.world_fix
        scores = {}
    motion = apply_world_fix(motion, chosen)

    max_frame = min(motion.frame_count, int(args.max_seconds * motion.fps))
    seq_frames = np.arange(0, max_frame, args.stride)
    kf_frames = keyframe_window_frames(keyframes, motion.fps, stride=2)
    all_frames = np.unique(np.concatenate([seq_frames, kf_frames]))

    print(f"[run] PICO stack on {len(all_frames)} frames (stride {args.stride}) ...")
    output = run_pico_stack(motion, all_frames)

    kf_results, kf_pass, kf_total = evaluate_keyframes(output, keyframes)
    yaw_results, yaw_pass, yaw_total, yaws, times = evaluate_yaw_turns(output)
    continuity = evaluate_continuity(output)
    wrist_corr = evaluate_wrist_height_correlation(motion, output)
    arm_gain = evaluate_arm_elevation_gain(motion, output)

    print(f"\n=== Key-frame geometry: {kf_pass}/{kf_total} passed ===")
    for entry in kf_results:
        n_ok = sum(c["pass"] for c in entry["checks"])
        print(f"  [{entry['keyframe']}] {n_ok}/{len(entry['checks'])}")
        for c in entry["checks"]:
            mark = "PASS" if c["pass"] else "FAIL"
            print(f"    {mark}  {c['label']}: {c['detail']}")

    print(f"\n=== Yaw turns: {yaw_pass}/{yaw_total} passed ===")
    for r in yaw_results:
        mark = "PASS" if r["pass"] else "FAIL"
        print(f"  {mark}  t={r['window_s'][0]:.1f}-{r['window_s'][1]:.1f}s "
              f"expected {r['expected_deg']:+.0f} got {r['measured_deg']:+.1f}")

    print(f"\n=== Continuity ===\n  {continuity}")
    print(f"=== Wrist height correlation ===\n  {wrist_corr}")
    print(f"=== Arm elevation gain (rotation chain vs measured hands) ===\n  {arm_gain}")

    plot_keyframes(output, keyframes, out_dir / "keyframes_smpl.png",
                   f"PICO SMPL stack reconstruction (world-fix={chosen}, heading-local)")
    plot_mocopi_reference(motion, keyframes, out_dir / "keyframes_mocopi.png")
    plot_timeseries(output, motion, yaws, times, yaw_results, out_dir / "timeseries.png")

    summary = {
        "json_file": str(args.json_file),
        "world_fix": chosen,
        "world_fix_scores": scores,
        "keyframe_pass": [kf_pass, kf_total],
        "keyframes": kf_results,
        "yaw_pass": [yaw_pass, yaw_total],
        "yaw_turns": yaw_results,
        "continuity": continuity,
        "wrist_height_correlation": wrist_corr,
        "arm_elevation_gain": arm_gain,
    }
    with (out_dir / "summary.json").open("w", encoding="utf-8") as f:
        json.dump(summary, f, indent=2, ensure_ascii=False)

    print(f"\n[done] plots + summary.json written to {out_dir}")
    verdict = "GO" if (kf_pass >= kf_total - 3 and yaw_pass >= yaw_total - 1) else "NEEDS-WORK"
    print(f"[verdict] {verdict}  (keyframes {kf_pass}/{kf_total}, yaw {yaw_pass}/{yaw_total})")


if __name__ == "__main__":
    main()
