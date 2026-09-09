#!/usr/bin/env python3
"""Render the ElRobot MuJoCo scene to a PNG offscreen."""
from __future__ import annotations

import os
import sys

import mujoco
import numpy as np
from PIL import Image

HERE = os.path.dirname(os.path.abspath(__file__))
XML = os.path.join(HERE, "elrobot_follower.xml")
OUT = os.path.join(HERE, "elrobot_scene.png")


def render(out_path: str = OUT) -> str:
    model = mujoco.MjModel.from_xml_path(XML)
    data = mujoco.MjData(model)

    renderer = mujoco.Renderer(model, height=960, width=1280)

    cam = mujoco.MjvCamera()
    cam.lookat[:] = (0.0, 0.14, 0.12)
    cam.distance = 1.0
    cam.azimuth = -35.0
    cam.elevation = -18.0

    mujoco.mj_forward(model, data)
    renderer.update_scene(data, camera=cam)
    pixels = renderer.render()  # (H, W, 3) uint8 RGB

    Image.fromarray(np.ascontiguousarray(pixels)).save(out_path)
    print(f"Rendered {out_path}  ({pixels.shape[1]}x{pixels.shape[0]})")
    return out_path


if __name__ == "__main__":
    sys.exit(0 if render() else 1)
