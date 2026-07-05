import msgpack
import msgpack_numpy as msgpack_numpy
import numpy as np

from gear_sonic.utils.teleop.input_readers import (
    SONY_BONEDATA_JSON_FORMAT,
    build_body_pose_sample,
    decode_msgpack_byte_multi_array,
    sony_bonedata_payload_to_body_poses,
)


SONY_BONEDATA_NAMES = [
    "root",
    "torso_1",
    "torso_2",
    "torso_3",
    "torso_4",
    "torso_5",
    "torso_6",
    "torso_7",
    "neck_1",
    "neck_2",
    "head",
    "l_shoulder",
    "l_up_arm",
    "l_low_arm",
    "l_hand",
    "r_shoulder",
    "r_up_arm",
    "r_low_arm",
    "r_hand",
    "l_up_leg",
    "l_low_leg",
    "l_foot",
    "l_toes",
    "r_up_leg",
    "r_low_leg",
    "r_foot",
    "r_toes",
]


def _sony_bonedata_payload():
    return {
        "format": SONY_BONEDATA_JSON_FORMAT,
        "name": SONY_BONEDATA_NAMES,
        "position": [
            {"x": float(i), "y": float(i) + 0.1, "z": float(i) + 0.2}
            for i in range(len(SONY_BONEDATA_NAMES))
        ],
        "rotation": [
            {"x": 0.0, "y": 0.0, "z": 0.0, "w": 1.0}
            for _ in range(len(SONY_BONEDATA_NAMES))
        ],
    }


def test_decode_msgpack_byte_multi_array_from_byte_chunks():
    payload = {
        "timestamp": 123456789,
        "joint_positions": [[1.0, 2.0, 3.0], [4.0, 5.0, 6.0]],
        "joint_orientations": [[0.0, 0.0, 0.0, 1.0], [0.5, 0.5, 0.5, 0.5]],
    }
    packed = msgpack.packb(payload, default=msgpack_numpy.encode, use_bin_type=True)
    byte_chunks = [bytes([value]) for value in packed]

    decoded = decode_msgpack_byte_multi_array(
        byte_chunks,
        msgpack_module=msgpack,
        msgpack_numpy_module=msgpack_numpy,
    )

    assert decoded["timestamp"] == payload["timestamp"]
    assert decoded["joint_positions"] == payload["joint_positions"]
    assert decoded["joint_orientations"] == payload["joint_orientations"]


def test_build_body_pose_sample_uses_existing_teleop_shape():
    payload = {
        "timestamp": 1_000_000_100,
        "joint_positions": [[0.1, 0.2, 0.3], [0.4, 0.5, 0.6]],
        "joint_orientations": [[0.0, 0.0, 0.0, 1.0], [0.0, 0.0, 1.0, 0.0]],
    }

    sample, stamp_ns, fps_ema = build_body_pose_sample(
        payload,
        prev_stamp_ns=1_000_000_000,
        fps_ema=0.0,
    )

    assert sample is not None
    assert stamp_ns == payload["timestamp"]
    assert sample["body_poses_np"].shape == (24, 7)
    np.testing.assert_allclose(sample["body_poses_np"][0], np.array([0.1, 0.2, 0.3, 0.0, 0.0, 0.0, 1.0]))
    np.testing.assert_allclose(sample["body_poses_np"][1], np.array([0.4, 0.5, 0.6, 0.0, 0.0, 1.0, 0.0]))
    np.testing.assert_allclose(sample["dt"], 1e-7)
    np.testing.assert_allclose(fps_ema, 1.0 / 1e-7)


def test_sony_bonedata_payload_maps_27_bones_to_pico_24_slots():
    payload = _sony_bonedata_payload()

    body_poses, indices = sony_bonedata_payload_to_body_poses(
        payload,
        coordinate_frame="identity",
    )

    assert body_poses.shape == (24, 7)
    assert indices[0] == SONY_BONEDATA_NAMES.index("root")
    assert indices[1] == SONY_BONEDATA_NAMES.index("l_up_leg")
    assert indices[2] == SONY_BONEDATA_NAMES.index("r_up_leg")
    assert indices[21] == SONY_BONEDATA_NAMES.index("r_hand")
    assert indices[23] == SONY_BONEDATA_NAMES.index("r_hand")
    np.testing.assert_allclose(body_poses[0, :3], np.array([0.0, 0.1, 0.2]))
    np.testing.assert_allclose(body_poses[1, :3], np.array([19.0, 19.1, 19.2]))
    np.testing.assert_allclose(body_poses[21, :3], np.array([18.0, 18.1, 18.2]))
    np.testing.assert_allclose(body_poses[:, 3:], np.tile([0.0, 0.0, 0.0, 1.0], (24, 1)))


def test_sony_bonedata_payload_default_frame_flips_z_axis():
    payload = _sony_bonedata_payload()

    body_poses, _indices = sony_bonedata_payload_to_body_poses(payload)

    np.testing.assert_allclose(body_poses[0, :3], np.array([0.0, 0.1, -0.2]))
    np.testing.assert_allclose(body_poses[0, 3:], np.array([0.0, 0.0, 0.0, 1.0]))
