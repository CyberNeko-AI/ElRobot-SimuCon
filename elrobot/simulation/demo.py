#!/usr/bin/env python3
"""Run a headless simulation of the ElRobot arm and render an animation.

The arm waves its joints and cycles the gripper for a few seconds; frames are
saved to ``frames/`` and an animated GIF is written to ``elrobot_demo.gif``.

Usage:
    python demo.py [duration_seconds]
"""
from __future__ import annotations

import os
import sys

import mujoco
import numpy as np
from PIL import Image

import controller

HERE = os.path.dirname(os.path.abspath(__file__))
XML = os.path.join(HERE, "elrobot_follower.xml")
FRAMES_DIR = os.path.join(HERE, "frames")
GIF_OUT = os.path.join(HERE, "elrobot_demo.gif")

FPS = 30


def main() -> None:
    duration = float(sys.argv[1]) if len(sys.argv) > 1 else 4.0
    model = mujoco.MjModel.from_xml_path(XML)
    data = mujoco.MjData(model)

    renderer = mujoco.Renderer(model, height=540, width=720)
    cam = mujoco.MjvCamera()
    cam.lookat[:] = (0.0, 0.14, 0.14)
    cam.distance = 1.1
    cam.azimuth = -35.0
    cam.elevation = -16.0

    os.makedirs(FRAMES_DIR, exist_ok=True)
    frames: list[Image.Image] = []
    n_frames = int(duration * FPS)
    dt = model.opt.timestep  # 0.002 s
    substeps = int(round(1.0 / (FPS * dt)))  # physics steps per rendered frame
    servo = controller.ServoController(model)

    for k in range(n_frames):
        servo.set_targets(controller.demo_targets(data.time))
        for _ in range(substeps):
            servo.step(data)
            mujoco.mj_step(model, data)

        mujoco.mj_forward(model, data)
        renderer.update_scene(data, camera=cam)
        pix = renderer.render()
        img = Image.fromarray(np.ascontiguousarray(pix))
        frames.append(img)
        img.save(os.path.join(FRAMES_DIR, f"frame_{k:04d}.png"))

    frames[0].save(GIF_OUT, save_all=True, append_images=frames[1:],
                   duration=int(1000 / FPS), loop=0)
    print(f"Saved {len(frames)} frames to {FRAMES_DIR}/")
    print(f"Saved animation to {GIF_OUT}")
    print(f"Final gripper position: {np.round(data.body('Gripper_Base_v1_1').xpos, 3)}")


if __name__ == "__main__":
    main()
