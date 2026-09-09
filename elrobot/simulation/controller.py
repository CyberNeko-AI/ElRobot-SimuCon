"""Realistic ST3215-C001 servo controller for the ElRobot follower arm.

Each servo is modelled as a position servo with:
  * max joint speed    (5 rad/s -> rate-limit on the reference)
  * position deadband  (gear backlash <0.5° + controller deadband, combined)
  * stall torque limit (1.91 N·m -> enforced by the MJCF actuator ``forcerange``)

The gripper jaws are rack-pinion followers of the gear (``rev_motor_08``): they
are commanded to mirror the gear's ACTUAL angle, so gear backlash / sag is
reflected in the jaw positions too.

The ``<position>`` actuator in the MJCF produces the torque (kp / forcerange);
this class only shapes the reference fed to it (rate-limit + deadband).
"""
from __future__ import annotations

import math

import mujoco
import numpy as np

JAW_MIMIC = {"rev_motor_08_1": -0.0115, "rev_motor_08_2": +0.0115}
GRIPPER_JOINT = "rev_motor_08"
ARM_JOINT_NAMES = [
    "rev_motor_01", "rev_motor_02", "rev_motor_03", "rev_motor_04",
    "rev_motor_05", "rev_motor_06", "rev_motor_07",
]

MAX_VELOCITY = 5.0   # rad/s (C001: 0.2 s/rad)
DEADBAND_DEG = 0.5   # combined gear backlash (<0.5°) + controller deadband (待测)


def actuator_index(model: mujoco.MjModel) -> dict[str, int]:
    """Map actuator name -> actuator index."""
    return {model.actuator(i).name: i for i in range(model.nu)}


class ServoController:
    """Rate-limited, deadbanded position servo for the 8 motor joints."""

    def __init__(self, model: mujoco.MjModel,
                 max_velocity: float = MAX_VELOCITY,
                 deadband_deg: float = DEADBAND_DEG) -> None:
        self.model = model
        self.idx = actuator_index(model)
        self.dt = model.opt.timestep
        self.max_velocity = max_velocity
        self.deadband = math.radians(deadband_deg)
        self.servo_joints = ARM_JOINT_NAMES + [GRIPPER_JOINT]
        self.targets = {n: 0.0 for n in self.servo_joints}
        self.ref = {n: 0.0 for n in self.servo_joints}
        self._gear_qposadr = int(model.joint(GRIPPER_JOINT).qposadr[0])

    def reset(self) -> None:
        for n in self.servo_joints:
            self.targets[n] = 0.0
            self.ref[n] = 0.0

    def set_targets(self, targets_rad: dict[str, float]) -> None:
        """Update desired joint angles (rad); call once per control tick."""
        for n, v in targets_rad.items():
            if n in self.targets:
                self.targets[n] = float(v)

    def set_gripper(self, angle_rad: float) -> None:
        self.targets[GRIPPER_JOINT] = float(angle_rad)

    def step(self, data: mujoco.MjData) -> None:
        """Write ``data.ctrl`` from the current targets (call every physics step)."""
        m = self.model
        ctrl = np.zeros(m.nu)
        for n in self.servo_joints:
            i = self.idx[n]
            q = float(data.qpos[m.joint(n).qposadr[0]])
            ref = self.ref[n]
            # 1) rate-limit the reference to the servo's max speed
            ref += min(max(self.targets[n] - ref, -self.max_velocity * self.dt),
                       self.max_velocity * self.dt)
            self.ref[n] = ref
            # 2) deadband: no torque inside the backlash, subtract it outside
            err = ref - q
            if abs(err) <= self.deadband:
                ctrl[i] = q
            else:
                ctrl[i] = ref - math.copysign(self.deadband, err)
        # 3) jaws mirror the gear's ACTUAL angle (rigid rack-pinion)
        gq = float(data.qpos[self._gear_qposadr])
        for jaw, mult in JAW_MIMIC.items():
            ctrl[self.idx[jaw]] = mult * gq
        data.ctrl[:] = ctrl


def demo_targets(t: float) -> dict[str, float]:
    """Return desired joint angles (rad) for a gentle waving motion."""
    env = 1.0 - math.exp(-t / 0.6)  # ease-in from the home pose
    return {
        "rev_motor_01": 0.5 * math.sin(0.35 * t) * env,
        "rev_motor_02": 0.12 * math.sin(0.25 * t) * env,
        "rev_motor_03": 0.20 * math.sin(0.25 * t + 1.0) * env,
        "rev_motor_04": -0.15 * math.sin(0.25 * t) * env,
        "rev_motor_05": 0.30 * math.sin(0.40 * t) * env,
        "rev_motor_06": 0.15 * math.sin(0.30 * t + 0.5) * env,
        "rev_motor_07": 0.40 * math.sin(0.35 * t + 0.8) * env,
        GRIPPER_JOINT: 0.8 + 0.6 * math.sin(0.6 * t) * env,
    }
