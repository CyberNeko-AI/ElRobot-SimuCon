# ElRobot Follower — MuJoCo Simulation

This folder turns [`elrobot_follower.urdf`](elrobot_follower.urdf) into a runnable
[MuJoCo](https://mujoco.org/) scene and places the arm in a world with a floor,
lights, and servo actuators.

## Files

| File | Purpose |
|------|---------|
| `build_mjcf.py`         | Converts the URDF into `elrobot_follower.xml` (regenerate after editing the URDF). |
| `elrobot_follower.xml`  | The generated MuJoCo scene (robot + floor + lights + actuators). |
| `controller.py`         | Shared helpers to command the position servos and the gripper. |
| `run_viewer.py`         | Interactive viewer; waves the arm and cycles the gripper. |
| `control_ui.py`         | Browser control panel: joint sliders + live MJPEG 3-D view with mouse orbit/zoom. |
| `demo.py`               | Headless run; renders frames and `elrobot_demo.gif`. |
| `render_scene.py`       | Renders a single offscreen PNG (`elrobot_scene.png`). |
| `assets/`               | STL meshes referenced by the URDF (kept in place, not copied). |

## Setup

A Python 3.12 virtual environment with `mujoco`, `numpy`, and `pillow` already
exists at `hardware/.venv`. From this directory, run scripts with:

```bash
../../.venv/bin/python <script>.py
```

or activate it first:

```bash
source ../../.venv/bin/activate   # from elrobot/simulation
```

## Usage

```bash
# (re)generate the scene from the URDF
../../.venv/bin/python build_mjcf.py

# open the interactive viewer (needs a display)
../../.venv/bin/python run_viewer.py

# control each joint angle with a browser slider panel (opens automatically)
../../.venv/bin/python control_ui.py
# options:
../../.venv/bin/python control_ui.py --no-browser            # don't auto-open browser
../../.venv/bin/python control_ui.py --port 9000             # use a specific port
../../.venv/bin/python control_ui.py --width 1280 --height 960   # render resolution
../../.venv/bin/python control_ui.py --fps 60                # target stream frame rate
# native MuJoCo 3-D window as well (on macOS run this via mjpython):
mjpython control_ui.py --viewer

# In the browser: drag sliders to command each joint; in the 3-D view,
# left-drag orbits the camera and the scroll wheel zooms.

# run a headless 4-second demo and render an animation
../../.venv/bin/python demo.py 4.0

# render a single frame
../../.venv/bin/python render_scene.py
```

## Model notes

- **Kinematics** — 19 links, 8 revolute + 2 prismatic joints = 10 DoF. The base
  link is welded to the world with its bottom resting on the floor (`z = 0`).
  URDF joint origins are mapped to the MJCF body `pos`, with each joint axis
  passing through its child body frame.
- **Servo model (ST3215 C001)** — the 8 arm/gear joints use `position`
  actuators with `kp=200`, `forcerange=±1.91 N·m` (stall) and `armature=0.02`
  (reflected inertia ≈ J_rotor·345²). `controller.ServoController` adds the
  realistic servo behaviour on top: a **5 rad/s speed limit** (0.2 s/rad) and a
  **0.5° deadband** (gear backlash + controller deadband), so the arm sags
  slightly within the backlash instead of holding rigidly. The two prismatic
  gripper jaws are rack-pinion followers: they mirror the gear's ACTUAL angle.
- **Masses** — printed parts use PLA at 1.20 g/cm³ (computed from mesh volumes),
  the 8 ST3215 servos use 55 g each; total ≈ 0.85 kg.
- **Self-collision** — the CAD collision meshes interpenetrate at the assembly
  pose (deep overlaps between adjacent links), so robot↔robot contact is
  disabled (`contype="1" conaffinity="0"`) while the arm still collides with the
  floor. Re-enable only after replacing the collision meshes with non-overlapping
  convex hulls (planned).
- **To-be-measured parameters** — servo PID bandwidth, position stiffness,
  controller deadband, contact friction: see [`MEASUREMENTS.md`](MEASUREMENTS.md).
- **Meshes** — URDF meshes are in millimetres; the MJCF scales them by 0.001.
