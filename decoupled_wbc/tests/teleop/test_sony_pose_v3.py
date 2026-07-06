import json

import numpy as np
import pytest

from gear_sonic.scripts.mocap_manager_server import build_arg_parser, _validate_args
from gear_sonic.utils.teleop.sources.mocopi_source import parse_mocopi_json_packet
from gear_sonic.utils.teleop.sources.sony_pico_smpl_source import (
    SONY_PICO_BONEDATA_BASIS_BY_NAME,
    apply_rooted_similarity_transform,
    bonedata_to_xrt_body_poses,
    fit_rooted_similarity_transform,
)
from gear_sonic.utils.teleop.zmq.zmq_planner_sender import HEADER_SIZE
from gear_sonic.utils.teleop.zmq.zmq_pose_sender import PoseStreamPublisher


class _FakeSocket:
    def __init__(self):
        self.messages = []

    def send(self, message):
        self.messages.append(message)


def _quat_z(angle_rad: float) -> list[float]:
    half = 0.5 * angle_rad
    return [float(np.cos(half)), 0.0, 0.0, float(np.sin(half))]


def _sony_skeleton_payload(frame_index: int = 0) -> dict:
    joint_positions = {
        "root": [0.0, 0.0, 0.0],
        "torso_2": [0.0, 0.0, 0.25],
        "torso_6": [0.0, 0.0, 0.55],
        "neck_2": [0.0, 0.0, 0.75],
        "head": [0.0, 0.0, 0.9],
        "left_shoulder": [0.0, 0.18, 0.62],
        "left_lower_arm": [0.02, 0.34, 0.50],
        "left_wrist": [0.04, 0.48, 0.42],
        "right_shoulder": [0.0, -0.18, 0.62],
        "right_lower_arm": [0.02, -0.34, 0.50],
        "right_wrist": [0.04, -0.48, 0.42],
        "left_upper_leg": [0.0, 0.10, -0.08],
        "left_lower_leg": [0.02, 0.10, -0.50],
        "left_foot": [0.08, 0.10, -0.86],
        "left_toes": [0.20, 0.10, -0.90],
        "right_upper_leg": [0.0, -0.10, -0.08],
        "right_lower_leg": [0.02, -0.10, -0.50],
        "right_foot": [0.08, -0.10, -0.86],
        "right_toes": [0.20, -0.10, -0.90],
    }
    joints = {}
    for name, position in joint_positions.items():
        quat = _quat_z(0.0)
        if name == "left_lower_arm":
            quat = _quat_z(0.35)
        joints[name] = {"pos": position, "quat_wxyz": quat}
    return {
        "source": "sony_mocopi_json",
        "frame_index": frame_index,
        "joints": joints,
    }


def _decode_header(message: bytes) -> dict:
    prefix = b"pose"
    assert message.startswith(prefix)
    header_bytes = message[len(prefix) : len(prefix) + HEADER_SIZE]
    return json.loads(header_bytes.rstrip(b"\x00").decode("utf-8"))


def _parse_and_validate(argv: list[str]):
    parser = build_arg_parser()
    args = parser.parse_args(argv)
    _validate_args(
        parser,
        args,
        user_set_target_fps="--target-fps" in argv,
        user_set_pose_filter_profile="--pose-filter-profile" in argv,
    )
    return args


def test_sony_named_skeleton_builds_pose_v3_reference():
    frame = parse_mocopi_json_packet(json.dumps(_sony_skeleton_payload()).encode("utf-8"))

    assert frame.full_body is not None
    assert frame.full_body.smpl_joints.shape == (24, 3)
    assert frame.full_body.smpl_pose.shape == (21, 3)
    assert frame.full_body.joint_pos.shape == (29,)
    assert np.max(np.abs(frame.full_body.smpl_joints)) > 0.0
    assert np.max(np.abs(frame.full_body.smpl_pose)) > 0.0


def test_sony_pose_v3_publisher_includes_joint_and_smpl_fields():
    publisher = PoseStreamPublisher(
        window_size=2,
        protocol_version=3,
        encoder_mode=2,
        enable_reference_filter=False,
    )
    socket = _FakeSocket()

    frame0 = parse_mocopi_json_packet(json.dumps(_sony_skeleton_payload(0)).encode("utf-8"))
    frame1 = parse_mocopi_json_packet(json.dumps(_sony_skeleton_payload(1)).encode("utf-8"))
    assert frame0.full_body is not None
    assert frame1.full_body is not None

    assert not publisher.publish(socket, frame0.full_body, frame_index=0, timestamp_s=0.0)
    assert publisher.publish(socket, frame1.full_body, frame_index=1, timestamp_s=0.02)

    assert len(socket.messages) == 1
    header = _decode_header(socket.messages[0])
    field_names = {field["name"] for field in header["fields"]}

    assert header["v"] == 3
    assert {"smpl_joints", "smpl_pose", "joint_pos", "joint_vel"}.issubset(field_names)
    assert {"body_pos", "body_quat_w", "frame_index", "encoder_mode"}.issubset(field_names)


def test_bvh_stream_keeps_v1_default_line_explicit():
    args = _parse_and_validate(
        [
            "--source",
            "bvh_stream",
            "--control-mode",
            "pose",
            "--pose-protocol-version",
            "1",
            "--pose-encoder-mode",
            "g1",
        ]
    )

    assert args.pose_filter_profile == "off"
    assert args.target_fps == 50.0


def test_bvh_stream_rejects_v3_without_sony_experiment_flag():
    with pytest.raises(SystemExit):
        _parse_and_validate(
            [
                "--source",
                "bvh_stream",
                "--control-mode",
                "pose",
                "--pose-protocol-version",
                "3",
                "--pose-encoder-mode",
                "smpl",
            ]
        )


def test_bvh_stream_accepts_explicit_sony_pose_v3_line():
    args = _parse_and_validate(
        [
            "--source",
            "bvh_stream",
            "--control-mode",
            "pose",
            "--pose-protocol-version",
            "3",
            "--pose-encoder-mode",
            "smpl",
            "--allow-sony-pose-v3",
        ]
    )

    assert args.pose_filter_profile == "stable"
    assert args.target_fps == 50.0
    assert args.bvh_g1_smpl_joints_source == "g1_fk"


def test_sony_pico_accepts_bonedata_basis_switch():
    args = _parse_and_validate(
        [
            "--source",
            "sony_pico",
            "--control-mode",
            "pose",
            "--pose-protocol-version",
            "3",
            "--pose-encoder-mode",
            "smpl",
            "--sony-pico-bonedata-basis",
            "none",
        ]
    )

    assert args.sony_pico_bonedata_basis == "none"


def test_sony_pico_accepts_smpl_joints_source_switch():
    args = _parse_and_validate(
        [
            "--source",
            "sony_pico",
            "--control-mode",
            "pose",
            "--pose-protocol-version",
            "3",
            "--pose-encoder-mode",
            "smpl",
            "--sony-pico-smpl-joints-source",
            "bonedata_positions",
        ]
    )

    assert args.sony_pico_smpl_joints_source == "bonedata_positions"


def test_sony_pico_bonedata_basis_flips_raw_x_rotation_sign():
    angle = 1.0
    raw_x_bend_quat = np.array(
        [[np.sin(angle * 0.5), 0.0, 0.0, np.cos(angle * 0.5)]],
        dtype=np.float64,
    )
    positions = np.zeros((1, 3), dtype=np.float64)
    slot_indices = np.array([0], dtype=np.int64)

    none_pose = bonedata_to_xrt_body_poses(
        positions,
        raw_x_bend_quat,
        slot_indices,
        basis=SONY_PICO_BONEDATA_BASIS_BY_NAME["none"],
    )
    zflip_pose = bonedata_to_xrt_body_poses(
        positions,
        raw_x_bend_quat,
        slot_indices,
        basis=SONY_PICO_BONEDATA_BASIS_BY_NAME["zflip"],
    )

    assert none_pose[0, 3] == pytest.approx(-zflip_pose[0, 3])
    assert none_pose[0, 6] == pytest.approx(zflip_pose[0, 6])


def test_rooted_similarity_transform_recovers_target_points():
    source = np.array(
        [
            [0.0, 0.0, 0.0],
            [0.0, 0.0, 1.0],
            [0.0, 0.4, 0.6],
            [0.0, -0.4, 0.6],
            [0.2, 0.2, -0.8],
            [0.2, -0.2, -0.8],
        ],
        dtype=np.float64,
    )
    angle = 0.7
    rot = np.array(
        [
            [np.cos(angle), -np.sin(angle), 0.0],
            [np.sin(angle), np.cos(angle), 0.0],
            [0.0, 0.0, 1.0],
        ],
        dtype=np.float64,
    )
    target = source @ rot * 0.75

    scale, fitted_rot = fit_rooted_similarity_transform(
        source,
        target,
        joint_indices=np.arange(source.shape[0]),
    )
    recovered = apply_rooted_similarity_transform(source, scale, fitted_rot)

    np.testing.assert_allclose(recovered, target, atol=1e-6)


def test_bvh_stream_rejects_g1_fk_smpl_joints_without_body_fk():
    with pytest.raises(SystemExit):
        _parse_and_validate(
            [
                "--source",
                "bvh_stream",
                "--control-mode",
                "pose",
                "--pose-protocol-version",
                "3",
                "--pose-encoder-mode",
                "smpl",
                "--allow-sony-pose-v3",
                "--bvh-g1-no-body-fk",
            ]
        )
