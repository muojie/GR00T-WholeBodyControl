"""Mirror MuJoCo/SONIC G1 joint states in a minimal Isaac Lab scene.

This script is intentionally a visualizer first: it writes root pose and joint
state directly into an Isaac Lab Articulation, so PhysX/PD tracking differences
do not distort the pose you are trying to inspect.

Example:
    TERM=xterm /home/nolovr/IsaacLab/isaaclab.sh -p \
        gear_sonic/scripts/isaaclab_g1_sim2sim_viewer.py

    TERM=xterm /home/nolovr/IsaacLab/isaaclab.sh -p \
        gear_sonic/scripts/isaaclab_g1_sim2sim_viewer.py \
        --source zmq --zmq-port 5557 --zmq-topic g1_debug
"""

from __future__ import annotations

import argparse
import csv
import math
import time
from dataclasses import dataclass
from pathlib import Path
from typing import Any

import numpy as np
from isaaclab.app import AppLauncher


REPO_ROOT = Path(__file__).resolve().parents[2]
DEFAULT_USD = REPO_ROOT / "gear_sonic/data/robots/g1/g1_43dof.usd"
DEFAULT_TRAJECTORY_DIR = REPO_ROOT / "gear_sonic_deploy/reference/example/macarena_001__A545"

# MuJoCo / Unitree / SONIC 29-DoF motor order used by this repository.
MUJOCO_29DOF_JOINT_NAMES = [
    "left_hip_yaw_joint",
    "left_hip_roll_joint",
    "left_hip_pitch_joint",
    "left_knee_joint",
    "left_ankle_pitch_joint",
    "left_ankle_roll_joint",
    "right_hip_yaw_joint",
    "right_hip_roll_joint",
    "right_hip_pitch_joint",
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

DEFAULT_MUJOCO_29DOF_Q = np.array(
    [
        -0.312,
        0.0,
        0.0,
        0.669,
        -0.363,
        0.0,
        -0.312,
        0.0,
        0.0,
        0.669,
        -0.363,
        0.0,
        0.0,
        0.0,
        0.0,
        0.2,
        0.2,
        0.0,
        0.6,
        0.0,
        0.0,
        0.0,
        0.2,
        -0.2,
        0.0,
        0.6,
        0.0,
        0.0,
        0.0,
    ],
    dtype=np.float32,
)


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "--source",
        choices=("csv", "zmq", "sine", "idle"),
        default="csv",
        help="State source. csv replays reference/example by default; zmq subscribes real-time targets.",
    )
    parser.add_argument("--robot-usd", type=Path, default=DEFAULT_USD, help="G1 USD file to load.")
    parser.add_argument(
        "--trajectory-dir",
        type=Path,
        default=DEFAULT_TRAJECTORY_DIR,
        help="Directory containing joint_pos.csv and optional joint_vel.csv/body_pos.csv/body_quat.csv.",
    )
    parser.add_argument("--csv-fps", type=float, default=50.0, help="Frame rate of CSV trajectory data.")
    parser.add_argument("--no-loop", action="store_true", help="Stop CSV replay at the final frame instead of looping.")
    parser.add_argument("--no-follow-root", action="store_true", help="Do not apply root pose from CSV/ZMQ.")
    parser.add_argument("--root-z-offset", type=float, default=0.0, help="Additive offset applied to replayed root height.")
    parser.add_argument("--sim-dt", type=float, default=0.005, help="Isaac Lab physics step.")
    parser.add_argument("--playback-speed", type=float, default=1.0, help="Scale replay time.")
    parser.add_argument("--max-steps", type=int, default=0, help="Stop after N simulation steps. 0 means run forever.")
    parser.add_argument("--print-interval", type=int, default=120, help="Print status every N steps. 0 disables logs.")
    parser.add_argument("--no-camera-follow", action="store_true", help="Keep the camera fixed.")
    parser.add_argument("--camera-update-interval", type=int, default=20, help="Update follow camera every N steps.")
    parser.add_argument("--zmq-host", default="127.0.0.1", help="ZMQ publisher host.")
    parser.add_argument("--zmq-port", type=int, default=5557, help="ZMQ publisher port.")
    parser.add_argument("--zmq-topic", default="g1_debug", help="ZMQ topic prefix.")
    parser.add_argument("--zmq-timeout", type=float, default=0.5, help="Seconds before warning about stale ZMQ data.")
    AppLauncher.add_app_launcher_args(parser)
    return parser.parse_args()


args_cli = parse_args()
app_launcher = AppLauncher(args_cli)
simulation_app = app_launcher.app


import torch  # noqa: E402

import isaaclab.sim as sim_utils  # noqa: E402
from isaaclab.assets import Articulation  # noqa: E402
from isaaclab.assets.articulation import ArticulationCfg  # noqa: E402
from isaaclab.sim import SimulationContext  # noqa: E402


@dataclass
class StateSample:
    joint_pos_mujoco: np.ndarray
    joint_vel_mujoco: np.ndarray | None = None
    root_pos_w: np.ndarray | None = None
    root_quat_w: np.ndarray | None = None
    source_frame: int | None = None
    source_time: float | None = None
    fresh: bool = True
    done: bool = False


def _read_csv_matrix(path: Path) -> np.ndarray:
    if not path.exists():
        raise FileNotFoundError(path)
    rows: list[list[float]] = []
    with path.open("r", newline="") as f:
        reader = csv.reader(f)
        _ = next(reader, None)
        for row in reader:
            if row:
                rows.append([float(v) for v in row])
    if not rows:
        raise ValueError(f"No numeric rows found in {path}")
    return np.asarray(rows, dtype=np.float32)


def _normalize_quat_wxyz(quat: np.ndarray) -> np.ndarray:
    quat = np.asarray(quat, dtype=np.float32).reshape(4)
    norm = float(np.linalg.norm(quat))
    if norm < 1.0e-6 or not math.isfinite(norm):
        return np.array([1.0, 0.0, 0.0, 0.0], dtype=np.float32)
    return quat / norm


class CsvTrajectorySource:
    def __init__(self, trajectory_dir: Path, fps: float, loop: bool, follow_root: bool, root_z_offset: float):
        self.trajectory_dir = trajectory_dir
        self.fps = fps
        self.loop = loop
        self.follow_root = follow_root
        self.root_z_offset = root_z_offset
        self.joint_pos = _read_csv_matrix(trajectory_dir / "joint_pos.csv")
        self.joint_vel = None
        joint_vel_path = trajectory_dir / "joint_vel.csv"
        if joint_vel_path.exists():
            self.joint_vel = _read_csv_matrix(joint_vel_path)
        self.root_pos = None
        self.root_quat = None
        body_pos_path = trajectory_dir / "body_pos.csv"
        body_quat_path = trajectory_dir / "body_quat.csv"
        if follow_root and body_pos_path.exists() and body_quat_path.exists():
            body_pos = _read_csv_matrix(body_pos_path)
            body_quat = _read_csv_matrix(body_quat_path)
            self.root_pos = body_pos[:, :3].copy()
            self.root_quat = body_quat[:, :4].copy()
            self.root_pos[:, 2] += root_z_offset
        self.num_frames = int(self.joint_pos.shape[0])
        if self.joint_pos.shape[1] < len(MUJOCO_29DOF_JOINT_NAMES):
            raise ValueError(
                f"{trajectory_dir / 'joint_pos.csv'} has {self.joint_pos.shape[1]} columns, "
                f"expected at least {len(MUJOCO_29DOF_JOINT_NAMES)}"
            )

    def sample(self, sim_time: float) -> StateSample:
        frame_float = max(sim_time * self.fps, 0.0)
        frame = int(frame_float)
        done = False
        if self.loop:
            frame %= self.num_frames
        else:
            if frame >= self.num_frames:
                frame = self.num_frames - 1
                done = True
        q = self.joint_pos[frame, : len(MUJOCO_29DOF_JOINT_NAMES)]
        dq = None
        if self.joint_vel is not None:
            dq = self.joint_vel[frame, : len(MUJOCO_29DOF_JOINT_NAMES)]
        root_pos = self.root_pos[frame] if self.root_pos is not None else None
        root_quat = _normalize_quat_wxyz(self.root_quat[frame]) if self.root_quat is not None else None
        return StateSample(q, dq, root_pos, root_quat, source_frame=frame, source_time=frame / self.fps, done=done)


class ZmqStateSource:
    def __init__(self, host: str, port: int, topic: str, timeout: float, follow_root: bool, root_z_offset: float):
        import msgpack
        import zmq

        self.msgpack = msgpack
        self.zmq = zmq
        self.topic = topic.encode("utf-8")
        self.timeout = timeout
        self.follow_root = follow_root
        self.root_z_offset = root_z_offset
        self.ctx = zmq.Context()
        self.socket = self.ctx.socket(zmq.SUB)
        self.socket.setsockopt(zmq.CONFLATE, 1)
        self.socket.setsockopt(zmq.RCVHWM, 1)
        self.socket.setsockopt(zmq.LINGER, 0)
        self.socket.setsockopt_string(zmq.SUBSCRIBE, topic)
        self.endpoint = f"tcp://{host}:{port}"
        self.socket.connect(self.endpoint)
        self.last_sample = StateSample(DEFAULT_MUJOCO_29DOF_Q.copy(), np.zeros(29, dtype=np.float32), fresh=False)
        self.last_rx_time = 0.0
        print(f"[INFO] ZMQ connected: {self.endpoint}/{topic}")

    def close(self) -> None:
        self.socket.close(0)
        self.ctx.term()

    def _decode(self, parts: list[bytes]) -> dict[str, Any] | None:
        if not parts:
            return None
        if len(parts) >= 2 and parts[0] == self.topic:
            payload = parts[-1]
        else:
            raw = parts[0]
            payload = raw[len(self.topic) :] if raw.startswith(self.topic) else raw
        return self.msgpack.unpackb(payload, raw=False)

    def _poll_latest(self) -> dict[str, Any] | None:
        latest = None
        while True:
            try:
                parts = self.socket.recv_multipart(flags=self.zmq.NOBLOCK)
            except self.zmq.Again:
                return latest
            latest = self._decode(parts)

    @staticmethod
    def _first_array(msg: dict[str, Any], keys: tuple[str, ...]) -> np.ndarray | None:
        for key in keys:
            if key in msg:
                arr = np.asarray(msg[key], dtype=np.float32).reshape(-1)
                if arr.size > 0:
                    return arr
        return None

    def sample(self, sim_time: float) -> StateSample:
        msg = self._poll_latest()
        if msg is None:
            stale = self.last_rx_time == 0.0 or (time.monotonic() - self.last_rx_time) > self.timeout
            self.last_sample.fresh = not stale
            return self.last_sample

        q = self._first_array(msg, ("body_q_target", "joint_pos", "q", "dof_pos"))
        if q is None:
            q = self.last_sample.joint_pos_mujoco
        dq = self._first_array(msg, ("body_dq_target", "joint_vel", "dq", "dof_vel"))
        if q.size < len(MUJOCO_29DOF_JOINT_NAMES):
            raise ValueError(f"ZMQ joint position has {q.size} values, expected at least 29")
        root_pos = None
        root_quat = None
        if self.follow_root:
            root_pos = self._first_array(msg, ("root_pos_w", "base_pos", "root_pos"))
            root_quat = self._first_array(msg, ("root_quat_w", "base_quat", "root_quat"))
            if root_pos is not None and root_pos.size >= 3:
                root_pos = root_pos[:3].copy()
                root_pos[2] += self.root_z_offset
            else:
                root_pos = None
            if root_quat is not None and root_quat.size >= 4:
                root_quat = _normalize_quat_wxyz(root_quat[:4])
            else:
                root_quat = None
        self.last_rx_time = time.monotonic()
        self.last_sample = StateSample(
            q[: len(MUJOCO_29DOF_JOINT_NAMES)].copy(),
            dq[: len(MUJOCO_29DOF_JOINT_NAMES)].copy() if dq is not None and dq.size >= 29 else None,
            root_pos,
            root_quat,
            source_time=sim_time,
            fresh=True,
        )
        return self.last_sample


class SineSource:
    def __init__(self, follow_root: bool):
        self.follow_root = follow_root

    def sample(self, sim_time: float) -> StateSample:
        q = DEFAULT_MUJOCO_29DOF_Q.copy()
        q[15] += 0.45 * math.sin(sim_time * 2.0)
        q[16] += 0.25 * math.sin(sim_time * 1.3)
        q[18] += 0.35 * math.sin(sim_time * 1.7)
        q[22] += 0.45 * math.sin(sim_time * 2.0 + math.pi)
        q[23] -= 0.25 * math.sin(sim_time * 1.3)
        q[25] += 0.35 * math.sin(sim_time * 1.7 + math.pi)
        root_pos = np.array([0.0, 0.0, 0.78], dtype=np.float32) if self.follow_root else None
        root_quat = np.array([1.0, 0.0, 0.0, 0.0], dtype=np.float32) if self.follow_root else None
        return StateSample(q, np.zeros(29, dtype=np.float32), root_pos, root_quat, source_time=sim_time)


class IdleSource:
    def __init__(self, follow_root: bool):
        self.follow_root = follow_root

    def sample(self, sim_time: float) -> StateSample:
        root_pos = np.array([0.0, 0.0, 0.78], dtype=np.float32) if self.follow_root else None
        root_quat = np.array([1.0, 0.0, 0.0, 0.0], dtype=np.float32) if self.follow_root else None
        return StateSample(DEFAULT_MUJOCO_29DOF_Q.copy(), np.zeros(29, dtype=np.float32), root_pos, root_quat)


def build_source() -> Any:
    follow_root = not args_cli.no_follow_root
    if args_cli.source == "csv":
        return CsvTrajectorySource(
            trajectory_dir=args_cli.trajectory_dir,
            fps=args_cli.csv_fps,
            loop=not args_cli.no_loop,
            follow_root=follow_root,
            root_z_offset=args_cli.root_z_offset,
        )
    if args_cli.source == "zmq":
        return ZmqStateSource(
            host=args_cli.zmq_host,
            port=args_cli.zmq_port,
            topic=args_cli.zmq_topic,
            timeout=args_cli.zmq_timeout,
            follow_root=follow_root,
            root_z_offset=args_cli.root_z_offset,
        )
    if args_cli.source == "sine":
        return SineSource(follow_root=follow_root)
    return IdleSource(follow_root=follow_root)


def design_scene() -> Articulation:
    ground_cfg = sim_utils.GroundPlaneCfg(size=(100.0, 100.0), color=(0.08, 0.16, 0.24))
    ground_cfg.func("/World/defaultGroundPlane", ground_cfg)
    light_cfg = sim_utils.DomeLightCfg(intensity=3000.0, color=(0.75, 0.75, 0.75))
    light_cfg.func("/World/Light", light_cfg)
    marker_cfg = sim_utils.SphereCfg(
        radius=0.045,
        visual_material=sim_utils.PreviewSurfaceCfg(diffuse_color=(1.0, 0.0, 0.0)),
        collision_props=sim_utils.CollisionPropertiesCfg(collision_enabled=False),
    )
    marker_cfg.func("/World/com_marker", marker_cfg, translation=(0.1, 0.0, 0.0))

    robot_cfg = ArticulationCfg(
        prim_path="/World/G1",
        spawn=sim_utils.UsdFileCfg(
            usd_path=str(args_cli.robot_usd),
            rigid_props=sim_utils.RigidBodyPropertiesCfg(
                disable_gravity=False,
                retain_accelerations=False,
                linear_damping=0.0,
                angular_damping=0.0,
                max_linear_velocity=1000.0,
                max_angular_velocity=1000.0,
                max_depenetration_velocity=1.0,
            ),
            articulation_props=sim_utils.ArticulationRootPropertiesCfg(
                enabled_self_collisions=True,
                solver_position_iteration_count=8,
                solver_velocity_iteration_count=4,
            ),
        ),
        init_state=ArticulationCfg.InitialStateCfg(
            pos=(0.0, 0.0, 0.78),
            rot=(1.0, 0.0, 0.0, 0.0),
            joint_pos={".*": 0.0},
            joint_vel={".*": 0.0},
        ),
        actuators={},
    )
    return Articulation(cfg=robot_cfg)


def build_mujoco_to_isaac_joint_ids(robot: Articulation) -> tuple[list[int], list[int], list[str]]:
    isaac_name_to_id = {name: idx for idx, name in enumerate(robot.joint_names)}
    mujoco_ids: list[int] = []
    isaac_ids: list[int] = []
    missing: list[str] = []
    for mujoco_id, name in enumerate(MUJOCO_29DOF_JOINT_NAMES):
        isaac_id = isaac_name_to_id.get(name)
        if isaac_id is None:
            missing.append(name)
            continue
        mujoco_ids.append(mujoco_id)
        isaac_ids.append(isaac_id)
    if missing:
        raise RuntimeError(
            "The USD does not contain required 29-DoF G1 joints: "
            + ", ".join(missing)
            + f"\nLoaded joints are: {robot.joint_names}"
        )
    print("[INFO] Loaded robot:")
    print(f"  USD: {args_cli.robot_usd}")
    print(f"  bodies={robot.num_bodies} joints={robot.num_joints}")
    print(f"  mapped active MuJoCo joints={len(isaac_ids)}")
    return mujoco_ids, isaac_ids, [MUJOCO_29DOF_JOINT_NAMES[i] for i in mujoco_ids]


def apply_sample(
    robot: Articulation,
    sample: StateSample,
    mujoco_ids: list[int],
    isaac_ids: list[int],
    device: torch.device,
) -> tuple[torch.Tensor, torch.Tensor]:
    joint_pos = robot.data.default_joint_pos.clone()
    joint_vel = robot.data.default_joint_vel.clone()
    q = torch.tensor(sample.joint_pos_mujoco[mujoco_ids], dtype=torch.float32, device=device).unsqueeze(0)
    joint_pos[:, isaac_ids] = q
    if sample.joint_vel_mujoco is not None:
        dq = torch.tensor(sample.joint_vel_mujoco[mujoco_ids], dtype=torch.float32, device=device).unsqueeze(0)
        joint_vel[:, isaac_ids] = dq
    robot.write_joint_state_to_sim(joint_pos, joint_vel)

    root_pose = robot.data.default_root_state[:, :7].clone()
    if sample.root_pos_w is not None:
        root_pose[:, :3] = torch.tensor(sample.root_pos_w, dtype=torch.float32, device=device).unsqueeze(0)
    if sample.root_quat_w is not None:
        root_pose[:, 3:7] = torch.tensor(sample.root_quat_w, dtype=torch.float32, device=device).unsqueeze(0)
    robot.write_root_pose_to_sim(root_pose)
    robot.write_root_velocity_to_sim(torch.zeros((1, 6), dtype=torch.float32, device=device))
    return root_pose[0, :3].detach().clone(), root_pose[0, 3:7].detach().clone()


def update_camera(sim: SimulationContext, root_pos: torch.Tensor) -> None:
    root = root_pos.detach().cpu().numpy()
    target = [float(root[0]), float(root[1]), float(root[2] + 0.35)]
    eye = [float(root[0] + 2.4), float(root[1] - 3.2), float(root[2] + 1.35)]
    sim.set_camera_view(eye, target)


def run() -> None:
    sim_cfg = sim_utils.SimulationCfg(dt=args_cli.sim_dt, device=args_cli.device)
    sim = SimulationContext(sim_cfg)
    sim.set_camera_view([2.4, -3.2, 1.8], [0.0, 0.0, 0.85])
    robot = design_scene()

    sim.reset()
    robot.update(sim.get_physics_dt())
    mujoco_ids, isaac_ids, _ = build_mujoco_to_isaac_joint_ids(robot)
    source = build_source()

    sim_dt = sim.get_physics_dt()
    sim_time = 0.0
    step = 0
    print("[INFO] Setup complete.")
    print(f"[INFO] source={args_cli.source} dt={sim_dt:.4f}s playback_speed={args_cli.playback_speed:.3f}")
    if args_cli.source == "csv":
        print(f"[INFO] trajectory={args_cli.trajectory_dir}")

    try:
        while simulation_app.is_running():
            sample = source.sample(sim_time * args_cli.playback_speed)
            root_pos, _ = apply_sample(robot, sample, mujoco_ids, isaac_ids, sim.device)
            if not args_cli.no_camera_follow and step % max(args_cli.camera_update_interval, 1) == 0:
                update_camera(sim, root_pos)
            sim.step()
            robot.update(sim_dt)
            if args_cli.print_interval > 0 and step % args_cli.print_interval == 0:
                frame = "-" if sample.source_frame is None else str(sample.source_frame)
                fresh = "fresh" if sample.fresh else "stale"
                q_absmax = float(np.max(np.abs(sample.joint_pos_mujoco[:29])))
                print(
                    f"[INFO] step={step} sim_t={sim_time:.3f} frame={frame} "
                    f"state={fresh} q_absmax={q_absmax:.3f}"
                )
            step += 1
            sim_time += sim_dt
            if args_cli.max_steps > 0 and step >= args_cli.max_steps:
                break
            if sample.done:
                break
    finally:
        if hasattr(source, "close"):
            source.close()


def close_app() -> None:
    if getattr(args_cli, "headless", False):
        try:
            simulation_app.close(wait_for_replicator=False, skip_cleanup=True)
            return
        except TypeError:
            pass
    simulation_app.close()


if __name__ == "__main__":
    run()
    close_app()
