"""Motion-capture input sources for teleoperation managers."""

from gear_sonic.utils.teleop.sources.base import (
    FullBodyReference,
    G1_DEFAULT_JOINT_POS_ISAACLAB,
    G1_DEFAULT_ROOT_POS_W,
    G1_ISAACLAB_TO_MUJOCO_IDX,
    G1_MUJOCO_TO_ISAACLAB_IDX,
    G1_LOWER_BODY_JOINT_IDX_ISAACLAB,
    G1_WRIST_JOINT_LOWER_LIMIT_ISAACLAB,
    G1_WRIST_JOINT_IDX_ISAACLAB,
    G1_WRIST_JOINT_UPPER_LIMIT_ISAACLAB,
    MocapFrame,
    MocapSource,
    Pose7D,
)
from gear_sonic.utils.teleop.sources.bvh_g1_source import (
    BvhG1PlaybackSource,
    BvhG1RetargetConfig,
    prepare_bvh_g1_retarget_context_from_motion,
    load_bvh_g1_motion,
    save_bvh_g1_motion_lib_pkl,
)
from gear_sonic.utils.teleop.sources.bvh_source import (
    BvhPlaybackSource,
    build_full_body_reference_from_skeleton_frame,
    load_bvh_motion,
)
from gear_sonic.utils.teleop.sources.bvh_stream_source import (
    BVH_STREAM_DEFAULT_PORT,
    BvhStreamUdpSource,
    parse_bvh_stream_packet,
)
from gear_sonic.utils.teleop.sources.g1_body_fk import G1BodyFk, SONIC_BODY_NAMES
from gear_sonic.utils.teleop.sources.joint_probe_source import (
    G1_ISAACLAB_JOINT_NAMES,
    JointProbePlaybackSource,
    resolve_g1_isaaclab_joint_index,
)
from gear_sonic.utils.teleop.sources.mocopi_source import (
    MOCOPI_DEFAULT_PORT,
    MocopiPacketError,
    MocopiUdpSource,
    parse_mocopi_binary_packet,
    parse_mocopi_json_packet,
    parse_mocopi_packet,
)
from gear_sonic.utils.teleop.sources.robot_pkl_source import (
    RobotPklPlaybackSource,
    load_robot_pkl_motion,
)

__all__ = [
    "BvhPlaybackSource",
    "build_full_body_reference_from_skeleton_frame",
    "BvhG1PlaybackSource",
    "BvhG1RetargetConfig",
    "BVH_STREAM_DEFAULT_PORT",
    "BvhStreamUdpSource",
    "FullBodyReference",
    "G1_DEFAULT_JOINT_POS_ISAACLAB",
    "G1_DEFAULT_ROOT_POS_W",
    "G1_ISAACLAB_JOINT_NAMES",
    "G1_ISAACLAB_TO_MUJOCO_IDX",
    "G1_MUJOCO_TO_ISAACLAB_IDX",
    "G1_LOWER_BODY_JOINT_IDX_ISAACLAB",
    "G1_WRIST_JOINT_LOWER_LIMIT_ISAACLAB",
    "G1_WRIST_JOINT_IDX_ISAACLAB",
    "G1_WRIST_JOINT_UPPER_LIMIT_ISAACLAB",
    "G1BodyFk",
    "JointProbePlaybackSource",
    "MOCOPI_DEFAULT_PORT",
    "MocapFrame",
    "MocapSource",
    "MocopiPacketError",
    "MocopiUdpSource",
    "Pose7D",
    "RobotPklPlaybackSource",
    "SONIC_BODY_NAMES",
    "parse_mocopi_binary_packet",
    "parse_mocopi_json_packet",
    "parse_mocopi_packet",
    "parse_bvh_stream_packet",
    "prepare_bvh_g1_retarget_context_from_motion",
    "load_bvh_motion",
    "load_bvh_g1_motion",
    "load_robot_pkl_motion",
    "save_bvh_g1_motion_lib_pkl",
    "resolve_g1_isaaclab_joint_index",
]
