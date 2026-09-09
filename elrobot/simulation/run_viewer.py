#!/usr/bin/env python3
"""Launch the interactive MuJoCo viewer for the ElRobot follower arm.

The arm is driven by position (servo) actuators. A small built-in controller
waves the arm and opens/closes the gripper so you can see it move; edit
``controller.demo_targets`` to command your own pose.

Usage:
    python run_viewer.py

Controls in the viewer window:
    - left-drag      : orbit camera
    - right-drag     : pan
    - scroll         : zoom
    - double-click   : track a body
    - Ctrl+right-drag: rotate the free camera
"""
from __future__ import annotations

import os

import mujoco
import mujoco.viewer

import controller

HERE = os.path.dirname(os.path.abspath(__file__))
XML = os.path.join(HERE, "elrobot_follower.xml")


def main() -> None:
    model = mujoco.MjModel.from_xml_path(XML)
    data = mujoco.MjData(model)
    mujoco.mj_forward(model, data)

    print(f"Loaded {XML}")
    print(f"  bodies={model.nbody}  dof={model.nv}  actuators={model.nu}")
    print("  joints: " + ", ".join(model.joint(i).name for i in range(model.njnt)))

    servo = controller.ServoController(model)

    with mujoco.viewer.launch_passive(model, data) as viewer:
        while viewer.is_running():
            # data.time advances with the physics step, keeping the controller
            # in sync with the simulation (wall-clock time would make the
            # targets sweep faster than the arm can track in the viewer).
            servo.set_targets(controller.demo_targets(data.time))
            servo.step(data)
            mujoco.mj_step(model, data)
            viewer.sync()


if __name__ == "__main__":
    main()
