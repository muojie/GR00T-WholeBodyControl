"""Motion-capture input sources for teleoperation managers."""

from gear_sonic.utils.teleop.sources.base import (
    FullBodyReference,
    G1_DEFAULT_JOINT_POS_ISAACLAB,
    G1_DEFAULT_ROOT_POS_W,
    G1_LOWER_BODY_JOINT_IDX_ISAACLAB,
    MocapFrame,
    MocapSource,
    Pose7D,
)
from gear_sonic.utils.teleop.sources.bvh_source import BvhPlaybackSource, load_bvh_motion
from gear_sonic.utils.teleop.sources.mocopi_source import (
    MOCOPI_DEFAULT_PORT,
    MocopiPacketError,
    MocopiUdpSource,
    parse_mocopi_binary_packet,
    parse_mocopi_json_packet,
    parse_mocopi_packet,
)

__all__ = [
    "BvhPlaybackSource",
    "FullBodyReference",
    "G1_DEFAULT_JOINT_POS_ISAACLAB",
    "G1_DEFAULT_ROOT_POS_W",
    "G1_LOWER_BODY_JOINT_IDX_ISAACLAB",
    "MOCOPI_DEFAULT_PORT",
    "MocapFrame",
    "MocapSource",
    "MocopiPacketError",
    "MocopiUdpSource",
    "Pose7D",
    "parse_mocopi_binary_packet",
    "parse_mocopi_json_packet",
    "parse_mocopi_packet",
    "load_bvh_motion",
]
