#!/usr/bin/env python3
"""Headless regression test for frictional grasp creep."""
from __future__ import annotations

from pathlib import Path

import mujoco
import numpy as np

import controller


HERE = Path(__file__).resolve().parent


def _block_in_gripper_frame(data: mujoco.MjData) -> np.ndarray:
    gripper = data.body("Gripper_Base_v1_1")
    rotation = gripper.xmat.reshape(3, 3)
    return rotation.T @ (data.body("block").xpos - gripper.xpos)


def test_grasp_does_not_creep() -> None:
    """Close on the cube, move the wrist both ways, then check static creep."""
    model = mujoco.MjModel.from_xml_path(str(HERE / "elrobot_follower.xml"))
    data = mujoco.MjData(model)
    servo = controller.ServoController(model)

    # Begin with the cube between partly closed fingers. Gravity is enabled
    # after the fingers have established a symmetric contact on both sides.
    gear_angle = 1.1
    initial_joints = {
        controller.GRIPPER_JOINT: gear_angle,
        "rev_motor_08_1": -0.0115 * gear_angle,
        "rev_motor_08_2": +0.0115 * gear_angle,
    }
    for joint, value in initial_joints.items():
        data.qpos[model.joint(joint).qposadr[0]] = value
    data.qpos[:3] = (0.0, 0.35, 0.22)
    data.qpos[3:7] = (1.0, 0.0, 0.0, 0.0)
    servo.ref[controller.GRIPPER_JOINT] = gear_angle
    servo.set_gripper(2.2028)
    model.opt.gravity[:] = 0.0

    samples: dict[int, np.ndarray] = {}
    for step in range(5000):
        if step == 500:
            model.opt.gravity[:] = (0.0, 0.0, -9.81)
        elif step == 1000:
            servo.set_targets({"rev_motor_07": 1.2})
        elif step == 2000:
            servo.set_targets({"rev_motor_07": -1.2})
        elif step == 3000:
            servo.set_targets({"rev_motor_07": 0.0})

        servo.step(data)
        mujoco.mj_step(model, data)
        if step in (3999, 4999):
            samples[step] = _block_in_gripper_frame(data)

    # During the final two seconds the wrist and fingers are stationary. A
    # 0.1 mm tolerance is well above floating-point noise while detecting the
    # old millimetres-per-few-seconds friction drift.
    creep = np.linalg.norm(samples[4999] - samples[3999])
    assert creep < 1e-4, f"grasp crept by {creep * 1000:.3f} mm in 2 seconds"

    block_id = model.body("block").id
    touching = set()
    for contact in data.contact[:data.ncon]:
        body1 = int(model.geom_bodyid[contact.geom1])
        body2 = int(model.geom_bodyid[contact.geom2])
        if block_id in (body1, body2):
            other = body2 if body1 == block_id else body1
            touching.add(model.body(other).name)
    assert {"Gripper_Jaw_01_v1_1", "Gripper_Jaw_02_v1_1"} <= touching


if __name__ == "__main__":
    test_grasp_does_not_creep()
    print("grasp creep regression: PASS")
