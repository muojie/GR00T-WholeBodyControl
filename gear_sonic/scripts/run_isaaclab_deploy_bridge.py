#!/usr/bin/env python3
"""Run Isaac Lab as a Unitree-SDK2 simulator for the C++ deploy stack.

This is the "scheme B" bridge:

    pico_manager -> C++ deploy -> rt/lowcmd -> Isaac Lab
                                      ^
                                      |
                              rt/lowstate / rt/secondary_imu

It intentionally keeps the deploy binary unchanged.  Isaac Lab publishes the
same low-level state topics that the MuJoCo sim publishes, and consumes the
same low-level command topics that the deploy side writes.
"""

from __future__ import annotations

import io
import os
from pathlib import Path
from types import SimpleNamespace
import sys
import time
from typing import Any

REPO_ROOT = Path(__file__).resolve().parents[2]
UNITREE_SDK2_PY = REPO_ROOT / "external_dependencies" / "unitree_sdk2_python"
if str(REPO_ROOT) not in sys.path:
    sys.path.insert(0, str(REPO_ROOT))
if UNITREE_SDK2_PY.exists() and str(UNITREE_SDK2_PY) not in sys.path:
    sys.path.insert(0, str(UNITREE_SDK2_PY))

try:
    import isaaclab  # noqa: F401
except ImportError:
    print(
        "\n"
        "ERROR: Isaac Lab is required for the IsaacLab deploy bridge but is not installed.\n"
        "Activate the Isaac Lab Python environment before running this script.\n"
    )
    sys.exit(1)

import hydra
from hydra.core import hydra_config
import numpy as np
from omegaconf import OmegaConf, open_dict
import torch
import yaml

from gear_sonic import train_agent_trl
from gear_sonic.envs.env_utils.joint_utils import get_body_joint_indices, get_hand_joint_indices
from gear_sonic.utils.config_utils import register_rl_resolvers
from gear_sonic.utils.mujoco_sim.configs import SimLoopConfig

register_rl_resolvers()


# Same mapping as gear_sonic.envs.manager_env.robots.g1 and C++ policy_parameters.hpp.
# Array index is the destination order index, value is the source order index.
G1_ISAACLAB_TO_MUJOCO_DOF = [
    0, 3, 6, 9, 13, 17, 1, 4, 7, 10, 14, 18, 2, 5, 8,
    11, 15, 19, 21, 23, 25, 27, 12, 16, 20, 22, 24, 26, 28,
]
G1_MUJOCO_TO_ISAACLAB_DOF = [
    0, 6, 12, 1, 7, 13, 2, 8, 14, 3, 9, 15, 22, 4, 10,
    16, 23, 5, 11, 17, 24, 18, 25, 19, 26, 20, 27, 21, 28,
]


def _load_checkpoint_config(override_config):
    """Load the training config saved beside the checkpoint, mirroring eval_agent_trl."""
    checkpoint = Path(override_config.checkpoint)
    config_path = checkpoint.parent / "config.yaml"
    if not config_path.exists():
        config_path = checkpoint.parent.parent / "config.yaml"
    if not config_path.exists():
        raise FileNotFoundError(f"Could not find checkpoint config.yaml near {checkpoint}")

    with open(config_path) as file:
        raw = file.read()

    # Backward compatibility with released checkpoints.
    raw = raw.replace("groot.rl.trl.", "gear_sonic.trl.")
    raw = raw.replace("groot.rl.envs.", "gear_sonic.envs.")
    raw = raw.replace("groot.rl.utils.", "gear_sonic.utils.")
    raw = raw.replace("groot.rl.agents.modules.modules.", "gear_sonic.trl.modules.base_module.")
    raw = raw.replace("groot.rl.agents.", "gear_sonic.trl.")
    raw = raw.replace("groot/rl/data/", "gear_sonic/data/")
    raw = raw.replace("assets/bm/unitree_description/", "assets/robot_description/")
    raw = raw.replace("1215_bones_seed_filtered", "bones_seed_smpl")

    train_config = OmegaConf.load(io.StringIO(raw))
    if train_config.eval_overrides is not None:
        train_config = OmegaConf.merge(train_config, train_config.eval_overrides)

    config = OmegaConf.merge(train_config, override_config)
    config.experiment_dir = checkpoint.parent
    return config


def _prepare_bridge_config(config) -> dict[str, Any]:
    bridge_cfg = config.get("isaaclab_deploy_bridge", {})

    sim_loop_cfg = SimLoopConfig(
        interface=bridge_cfg.get("interface", "sim"),
        sim_frequency=int(round(1.0 / float(config.manager_env.config.sim_dt))),
        enable_onscreen=False,
        enable_offscreen=False,
        verbose=bool(bridge_cfg.get("verbose", False)),
    )
    wbc_config = sim_loop_cfg.load_wbc_yaml()

    if "domain_id" in bridge_cfg:
        wbc_config["DOMAIN_ID"] = int(bridge_cfg.domain_id)
    if "interface" in bridge_cfg:
        # SimLoopConfig resolves aliases such as "sim" to the actual DDS interface.
        wbc_config["INTERFACE"] = sim_loop_cfg.interface
    if "use_sensor" in bridge_cfg:
        wbc_config["USE_SENSOR"] = bool(bridge_cfg.use_sensor)

    return wbc_config


def _init_channel(wbc_config: dict[str, Any]) -> None:
    from unitree_sdk2py.core.channel import ChannelFactoryInitialize

    if wbc_config.get("INTERFACE", None):
        ChannelFactoryInitialize(wbc_config["DOMAIN_ID"], wbc_config["INTERFACE"])
    else:
        ChannelFactoryInitialize(wbc_config["DOMAIN_ID"])


class IsaacLabDeployBridge:
    """Bridge one Isaac Lab G1 env to Unitree SDK2 low-level DDS topics."""

    def __init__(self, env_wrapper, unitree_bridge, *, verbose: bool = False):
        self.env_wrapper = env_wrapper
        self.env = env_wrapper.env
        self.robot = self.env.scene["robot"]
        self.unitree_bridge = unitree_bridge
        self.verbose = verbose

        self.device = self.env.device
        self.body_joint_indices = get_body_joint_indices(self.robot)
        self.hand_joint_indices = get_hand_joint_indices(self.robot)
        self.isaaclab_to_mujoco = torch.tensor(
            G1_ISAACLAB_TO_MUJOCO_DOF, dtype=torch.long, device=self.device
        )
        self.mujoco_to_isaaclab = torch.tensor(
            G1_MUJOCO_TO_ISAACLAB_DOF, dtype=torch.long, device=self.device
        )

        self.action_term = self.env.action_manager.get_term("joint_pos")
        self.action_joint_ids = self.action_term._joint_ids  # noqa: SLF001
        self.action_offset = self.action_term._offset  # noqa: SLF001
        self.action_scale = self.action_term._scale  # noqa: SLF001
        self.action_dim = self.env.action_space.shape[-1]

        self.torso_body_id = self._body_index("torso_link", fallback=0)
        self.default_action = torch.zeros((self.env.num_envs, self.action_dim), device=self.device)
        self.last_publish_time = 0.0
        self.step_count = 0

    def _body_index(self, name: str, fallback: int) -> int:
        try:
            return self.robot.body_names.index(name)
        except ValueError:
            return fallback

    def _tensor0(self, value: torch.Tensor) -> np.ndarray:
        return value[0].detach().cpu().numpy()

    def _body_state_hw_order(self, values: torch.Tensor) -> np.ndarray:
        body_values = values[0, self.body_joint_indices]
        return body_values[self.isaaclab_to_mujoco].detach().cpu().numpy()

    def _hand_state(self, values: torch.Tensor, side: str) -> np.ndarray:
        if self.hand_joint_indices.numel() == 0:
            return np.zeros(self.unitree_bridge.num_hand_motor, dtype=np.float32)
        hand_values = values[0, self.hand_joint_indices].detach().cpu().numpy()
        n = self.unitree_bridge.num_hand_motor
        if side == "left":
            return hand_values[:n]
        return hand_values[n : n * 2]

    def build_lowstate_obs(self) -> dict[str, np.ndarray | float]:
        data = self.robot.data
        zero_root_acc = np.zeros(6, dtype=np.float32)

        root_pose = np.concatenate(
            [
                self._tensor0(data.root_pos_w),
                self._tensor0(data.root_quat_w),
            ]
        )
        root_vel = np.concatenate(
            [
                self._tensor0(data.root_lin_vel_w),
                self._tensor0(data.root_ang_vel_w),
            ]
        )

        body_acc = getattr(data, "body_acc_w", None)
        if body_acc is not None:
            torso_vel = np.concatenate(
                [
                    self._tensor0(data.body_lin_vel_w[:, self.torso_body_id]),
                    self._tensor0(data.body_ang_vel_w[:, self.torso_body_id]),
                ]
            )
        else:
            torso_vel = root_vel.copy()

        joint_acc = getattr(data, "joint_acc", None)
        if joint_acc is None:
            body_ddq = np.zeros(self.unitree_bridge.num_body_motor, dtype=np.float32)
            left_hand_ddq = np.zeros(self.unitree_bridge.num_hand_motor, dtype=np.float32)
            right_hand_ddq = np.zeros(self.unitree_bridge.num_hand_motor, dtype=np.float32)
        else:
            body_ddq = self._body_state_hw_order(joint_acc)
            left_hand_ddq = self._hand_state(joint_acc, "left")
            right_hand_ddq = self._hand_state(joint_acc, "right")

        return {
            "floating_base_pose": root_pose,
            "floating_base_vel": root_vel,
            "floating_base_acc": zero_root_acc,
            "secondary_imu_quat": self._tensor0(data.body_quat_w[:, self.torso_body_id]),
            "secondary_imu_vel": torso_vel,
            "body_q": self._body_state_hw_order(data.joint_pos),
            "body_dq": self._body_state_hw_order(data.joint_vel),
            "body_ddq": body_ddq,
            "body_tau_est": np.zeros(self.unitree_bridge.num_body_motor, dtype=np.float32),
            "left_hand_q": self._hand_state(data.joint_pos, "left"),
            "left_hand_dq": self._hand_state(data.joint_vel, "left"),
            "left_hand_ddq": left_hand_ddq,
            "left_hand_tau_est": np.zeros(self.unitree_bridge.num_hand_motor, dtype=np.float32),
            "right_hand_q": self._hand_state(data.joint_pos, "right"),
            "right_hand_dq": self._hand_state(data.joint_vel, "right"),
            "right_hand_ddq": right_hand_ddq,
            "right_hand_tau_est": np.zeros(self.unitree_bridge.num_hand_motor, dtype=np.float32),
            "time": float(self.env.common_step_counter * self.env.step_dt),
        }

    def publish_state(self) -> None:
        self.unitree_bridge.PublishLowState(self.build_lowstate_obs())

    def _lowcmd_body_target_isaac_order(self) -> torch.Tensor | None:
        if not self.unitree_bridge.low_cmd_received:
            return None
        q_hw = torch.tensor(
            [self.unitree_bridge.low_cmd.motor_cmd[i].q for i in range(self.unitree_bridge.num_body_motor)],
            dtype=torch.float32,
            device=self.device,
        )
        return q_hw[self.mujoco_to_isaaclab]

    def _hand_targets(self) -> tuple[torch.Tensor | None, torch.Tensor | None]:
        n = self.unitree_bridge.num_hand_motor
        if n <= 0:
            return None, None
        left = None
        right = None
        if self.unitree_bridge.left_hand_cmd_received:
            left = torch.tensor(
                [self.unitree_bridge.left_hand_cmd.motor_cmd[i].q for i in range(n)],
                dtype=torch.float32,
                device=self.device,
            )
        if self.unitree_bridge.right_hand_cmd_received:
            right = torch.tensor(
                [self.unitree_bridge.right_hand_cmd.motor_cmd[i].q for i in range(n)],
                dtype=torch.float32,
                device=self.device,
            )
        return left, right

    def build_action(self) -> torch.Tensor:
        target_joint_pos = self.robot.data.joint_pos[0].clone()

        body_target = self._lowcmd_body_target_isaac_order()
        if body_target is not None:
            target_joint_pos[self.body_joint_indices] = body_target

        left_hand, right_hand = self._hand_targets()
        n = self.unitree_bridge.num_hand_motor
        if self.hand_joint_indices.numel() >= n * 2:
            if left_hand is not None:
                target_joint_pos[self.hand_joint_indices[:n]] = left_hand
            if right_hand is not None:
                target_joint_pos[self.hand_joint_indices[n : n * 2]] = right_hand

        action_target = target_joint_pos[self.action_joint_ids]
        action = (action_target.unsqueeze(0) - self.action_offset) / self.action_scale
        clip_value = self.env_wrapper.config.get("action_clip_value", None)
        if clip_value is not None and clip_value > 0:
            action = torch.clamp(action, -float(clip_value), float(clip_value))
        return action

    def step(self):
        self.publish_state()
        action = self.build_action()
        self.env_wrapper.step({"actions": action})
        self.step_count += 1
        if self.verbose and self.step_count % 50 == 0:
            print(
                f"[IsaacLabDeployBridge] step={self.step_count} "
                f"lowcmd={self.unitree_bridge.low_cmd_received}"
            )


def _make_env_config(config):
    with open_dict(config):
        config.num_envs = 1
        config.manager_env.config.num_envs = 1
        config.manager_env.config.episode_length_s = config.get(
            "isaaclab_deploy_bridge", {}
        ).get("episode_length_s", 3600.0)
        # Keep one env rooted at the origin for a deploy-style sim bridge.
        config.manager_env.config.env_spacing = 0.0
        for event in config.manager_env.config.get("train_only_events", []):
            if event in config.manager_env.events:
                config.manager_env.events.pop(event)
        config.manager_env.config.headless = config.get("headless", True)
    return config


@hydra.main(config_path="../config", config_name="base_eval", version_base="1.1")
def main(override_config):
    os.chdir(hydra.utils.get_original_cwd())
    config = _load_checkpoint_config(override_config)
    config = _make_env_config(config)
    args_cli = SimpleNamespace(headless=bool(config.get("headless", True)))

    bridge_cfg = config.get("isaaclab_deploy_bridge", {})
    verbose = bool(bridge_cfg.get("verbose", False))
    max_steps = int(bridge_cfg.get("max_steps", -1))
    wait_for_lowcmd = bool(bridge_cfg.get("wait_for_lowcmd", True))

    print(f"[IsaacLabDeployBridge] Hydra output: {hydra_config.HydraConfig.get().runtime.output_dir}")
    print("[IsaacLabDeployBridge] Starting Isaac Sim app")
    from isaaclab.app import AppLauncher

    app_launcher = AppLauncher(headless=args_cli.headless)
    simulation_app = app_launcher.app

    print("[IsaacLabDeployBridge] Creating Isaac Lab environment")
    env = None
    try:
        env = train_agent_trl.create_manager_env(config, "cuda:0", args_cli)
        env.reset(flatten_dict_obs=False)

        wbc_config = _prepare_bridge_config(config)
        print(
            "[IsaacLabDeployBridge] Initializing DDS "
            f"domain={wbc_config.get('DOMAIN_ID')} interface={wbc_config.get('INTERFACE')}"
        )
        _init_channel(wbc_config)
        from gear_sonic.utils.mujoco_sim.unitree_sdk2py_bridge import UnitreeSdk2Bridge

        unitree_bridge = UnitreeSdk2Bridge(wbc_config)
        bridge = IsaacLabDeployBridge(env, unitree_bridge, verbose=verbose)

        print(
            "[IsaacLabDeployBridge] Ready. Start C++ deploy with e.g. "
            "`./deploy.sh --input-type zmq_manager sim`."
        )
        if wait_for_lowcmd:
            print("[IsaacLabDeployBridge] Waiting for first rt/lowcmd before stepping physics")
        step = 0
        wait_log_time = 0.0
        while max_steps < 0 or step < max_steps:
            start = time.monotonic()
            if wait_for_lowcmd and not unitree_bridge.low_cmd_received:
                bridge.publish_state()
                if verbose and start - wait_log_time > 2.0:
                    print("[IsaacLabDeployBridge] waiting: lowcmd=False, publishing LowState only")
                    wait_log_time = start
                sleep_s = max(0.0, float(env.env.step_dt) - (time.monotonic() - start))
                if sleep_s > 0:
                    time.sleep(sleep_s)
                continue

            bridge.step()
            step += 1
            elapsed = time.monotonic() - start
            sleep_s = max(0.0, float(env.env.step_dt) - elapsed)
            if sleep_s > 0:
                time.sleep(sleep_s)
    except KeyboardInterrupt:
        print("[IsaacLabDeployBridge] Stopped by user")
    finally:
        if env is not None:
            env.env.close()
        simulation_app.close()


if __name__ == "__main__":
    main()
