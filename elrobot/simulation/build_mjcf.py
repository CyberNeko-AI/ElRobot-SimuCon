#!/usr/bin/env python3
"""Convert elrobot_follower.urdf into a MuJoCo MJCF scene.

The generated scene contains:
  * the ElRobot follower arm (7 revolute joints + gripper with mimic jaws)
  * a floor with a checker texture
  * lights and a camera
  * position (servo) actuators for the 10 motorised joints (8 arm + 2 jaws)

Usage:
    python build_mjcf.py            # writes elrobot_follower.xml
"""
from __future__ import annotations

import struct
import xml.etree.ElementTree as ET
from pathlib import Path

HERE = Path(__file__).resolve().parent
URDF_PATH = HERE / "elrobot_follower.urdf"
OUT_PATH = HERE / "elrobot_follower.xml"

MESH_SCALE = "0.001 0.001 0.001"  # URDF meshes are authored in millimetres
BASE_HEIGHT = 0.0  # base_link bottom rests on the floor (z=0)

# --------------------------------------------------------------------------
# Physical parameters (ST3215 C001 servo + PLA printed parts), see MEASUREMENTS.md
# --------------------------------------------------------------------------
STALL_TORQUE = 1.91     # N·m    (C001 stall torque: 19.5 kg·cm)
PRINT_DENSITY = 1200.0  # kg/m³  (PLA, high infill, 1.20 g/cm³)
SERVO_MASS = 0.055      # kg     (C001 self weight)
ARMATURE = 0.02         # kg·m²  (reflected rotor inertia ≈ J_rotor * 345², estimate)
SERVO_KP = 200          # N·m/rad  (servo position stiffness, 待测)
SERVO_DAMPING = 0.5     # N·m·s/rad (servo velocity feedback / D-term, 待测)
JAW_KP = 3000           # stiff rack-pinion coupling for the mimic jaws
JAW_FORCE = 166.0       # N      (stall torque / 0.0115 m rack-pinion radius)

# --------------------------------------------------------------------------
# Gripper convex collision decomposition (V-HACD), from pgripper/MuJoCo_collision.
# Each concave gripper part is split into convex OBJ pieces expressed in the
# pgripper part-local STL frame (mm). The pos/quat below map that part-local
# frame onto the elrobot link frame (aligned to the visual STL). Only the
# COLLISION geoms use these; the visual geoms keep the original STL.
# --------------------------------------------------------------------------
# link name -> (piece stem, piece count, pos(m), quat(w x y z))
GRIPPER_COLLISION = {
    "Gripper_Base_v1_1":   ("Gripper Base", 88, "0.000846 0.028286 -0.035465",  "0.707107 0 0 0.707107"),
    "Gripper_Gear_v1_1":   ("Gripper Gear", 70, "0.000068 0.003103 -0.013739",  "0.707107 0 0 -0.707107"),
    "Gripper_Jaw_01_v1_1": ("Gripper Jaw",  55, "-0.29302 0.346214 0.048671",   "0.707107 0.707107 0 0"),
    "Gripper_Jaw_02_v1_1": ("Gripper Jaw",  55, "-0.325475 0.303498 -0.032556", "0 0 -0.707107 0.707107"),
}


# --------------------------------------------------------------------------
# Grasp test block (dynamic free body). Graspable: it collides with the gripper
# and the floor. Friction is a placeholder (contact friction 待测).
# --------------------------------------------------------------------------
BLOCK_POS = "0 0.30 0.03"            # initial center (m); drops onto the floor
BLOCK_SIZE = "0.015 0.015 0.015"     # half-extents -> 3 cm cube
BLOCK_MASS = 0.05                    # kg


def _col_mesh_name(stem: str, i: int) -> str:
    return f'col_{stem.lower().replace(" ", "_")}_{i:03d}'


def _vec(attr: str, default: str = "0 0 0") -> str:
    parts = (attr or default).split()
    return " ".join(f"{float(x):.7g}" for x in parts)


def _rgba(material_name: str) -> str:
    if material_name == "silver":
        return "0.72 0.72 0.74 1.0"
    if material_name.startswith("ST3215"):
        return "0.13 0.13 0.14 1.0"  # servos are black in the URDF
    return "0.7 0.7 0.7 1.0"


def _stl_volume_m3(path: Path) -> float:
    """Return the signed volume (m³) of a binary STL mesh (vertices in mm)."""
    data = path.read_bytes()
    n = (len(data) - 84) // 50
    vol = 0.0
    off = 84
    for _ in range(n):
        v0 = struct.unpack_from("<3f", data, off + 12)
        v1 = struct.unpack_from("<3f", data, off + 24)
        v2 = struct.unpack_from("<3f", data, off + 36)
        vol += (v0[0] * (v1[1] * v2[2] - v1[2] * v2[1])
                - v0[1] * (v1[0] * v2[2] - v1[2] * v2[0])
                + v0[2] * (v1[0] * v2[1] - v1[1] * v2[0]))
        off += 50
    return abs(vol) / 6.0 * 1e-9  # mm³ -> m³


def _fullinertia(inertia_el: ET.Element, scale: float = 1.0) -> str | None:
    ixx = float(inertia_el.get("ixx", "0")) * scale
    iyy = float(inertia_el.get("iyy", "0")) * scale
    izz = float(inertia_el.get("izz", "0")) * scale
    ixy = float(inertia_el.get("ixy", "0")) * scale
    ixz = float(inertia_el.get("ixz", "0")) * scale
    iyz = float(inertia_el.get("iyz", "0")) * scale
    if ixx + iyy + izz <= 0.0:
        return None  # degenerate inertia -> handled by caller
    return f"{ixx:.7g} {iyy:.7g} {izz:.7g} {ixy:.7g} {ixz:.7g} {iyz:.7g}"


def main() -> None:
    robot = ET.parse(URDF_PATH).getroot()
    links = {l.get("name"): l for l in robot.findall("link")}
    joints = {j.get("name"): j for j in robot.findall("joint")}

    parent_of: dict[str, str] = {}
    joint_of: dict[str, ET.Element] = {}
    for j in joints.values():
        parent_of[j.find("child").get("link")] = j.find("parent").get("link")
        joint_of[j.find("child").get("link")] = j

    children: dict[str, list[str]] = {}
    for child, parent in parent_of.items():
        children.setdefault(parent, []).append(child)
    roots = [n for n in links if n not in parent_of]

    meshes: set[tuple[str, str]] = set()
    body_xml: list[str] = []
    actuator_defs: list[str] = []

    def emit_chain(name: str, depth: int) -> None:
        link = links[name]
        joint = joint_of.get(name)
        parent = parent_of.get(name)
        ind = "  " * (depth + 1)

        # mesh file (visual and collision share the same STL)
        mesh_file = None
        vis = link.find("visual")
        if vis is not None:
            mesh_file = vis.find("geometry").find("mesh").get("filename")

        attrs = [f'name="{name}"']
        if parent is None:
            attrs.append(f'pos="0 0 {BASE_HEIGHT}"')
        elif joint is not None:
            # URDF joint origin = child link frame in the parent frame, which
            # in MJCF is the body `pos` (NOT the joint `pos`).
            attrs.append(f'pos="{_vec(joint.find("origin").get("xyz"))}"')
        body_xml.append(ind + "<body " + " ".join(attrs) + ">")

        # revolute / prismatic joint
        if joint is not None and joint.get("type") != "fixed":
            axis = joint.find("axis")
            limit = joint.find("limit")
            jtype = "hinge" if joint.get("type") == "revolute" else "slide"
            lo = float(limit.get("lower"))
            hi = float(limit.get("upper"))
            j_attrs = [
                f'name="{joint.get("name")}"',
                f'type="{jtype}"',
                # the axis passes through the child frame origin
                'pos="0 0 0"',
                f'axis="{_vec(axis.get("xyz"))}"',
                f'range="{lo:.7g} {hi:.7g}"',
                'limited="true"',
                # armature = reflected rotor inertia (stabilises the low-inertia
                # gear joint); damping = servo velocity feedback / D-term (待测).
                f'armature="{ARMATURE}"',
                f'damping="{SERVO_DAMPING}"',
            ]
            body_xml.append(ind + "  " + "<joint " + " ".join(j_attrs) + "/>")

            # Position (servo) actuator. Hinge joints (arm + gripper gear) are
            # torque-limited by the C001 stall torque; the two prismatic jaws
            # are stiff rack-pinion followers driven from the controller by
            # mirroring the gear's ACTUAL angle.
            if jtype == "hinge":
                actuator_defs.append(
                    f'    <position name="{joint.get("name")}" joint="{joint.get("name")}" '
                    f'kp="{SERVO_KP}" forcerange="-{STALL_TORQUE} {STALL_TORQUE}" '
                    f'ctrlrange="{lo:.7g} {hi:.7g}"/>'
                )
            else:
                actuator_defs.append(
                    f'    <position name="{joint.get("name")}" joint="{joint.get("name")}" '
                    f'kp="{JAW_KP}" forcerange="-{JAW_FORCE} {JAW_FORCE}" '
                    f'ctrlrange="{lo:.7g} {hi:.7g}"/>'
                )

        # inertial (mass/inertia corrected for PLA density / servo self-weight)
        inertial_el = link.find("inertial")
        if inertial_el is not None:
            urdf_mass = float(inertial_el.find("mass").get("value"))
            com = inertial_el.find("origin")
            if name.startswith("ST3215"):
                new_mass = SERVO_MASS
            else:
                vol = _stl_volume_m3(HERE / mesh_file) if mesh_file else 0.0
                new_mass = PRINT_DENSITY * vol
            scale = new_mass / urdf_mass if urdf_mass > 0 else 1.0
            fullinertia = _fullinertia(inertial_el.find("inertia"), scale)
            if fullinertia is not None:
                body_xml.append(
                    ind + "  " +
                    f'<inertial pos="{_vec(com.get("xyz"))}" '
                    f'mass="{new_mass:.7g}" fullinertia="{fullinertia}"/>'
                )
            else:
                # degenerate inertia (e.g. Gripper_Gear): tiny placeholder
                small = new_mass * 1e-5
                body_xml.append(
                    ind + "  " +
                    f'<inertial pos="{_vec(com.get("xyz"))}" '
                    f'mass="{new_mass:.7g}" '
                    f'diaginertia="{small:.7g} {small:.7g} {small:.7g}"/>'
                )

        # visual + collision geoms
        for tag, cls in (("visual", "visual"), ("collision", "collision")):
            el = link.find(tag)
            if el is None:
                continue
            origin = el.find("origin")
            mat = el.find("material")
            rgba = _rgba(mat.get("name")) if mat is not None else "0.7 0.7 0.7 1.0"

            if tag == "collision" and name in GRIPPER_COLLISION:
                # convex-decomposed collision geoms for the gripper parts
                stem, count, pos, quat = GRIPPER_COLLISION[name]
                for i in range(count):
                    mname = _col_mesh_name(stem, i)
                    meshes.add((mname, f"{stem}_parts/{stem}_{i:03d}.obj"))
                    g = [
                        f'class="{cls}"',
                        'type="mesh"',
                        f'mesh="{mname}"',
                        f'pos="{pos}"',
                        f'quat="{quat}"',
                        f'rgba="{rgba}"',
                    ]
                    body_xml.append(ind + "  " + "<geom " + " ".join(g) + "/>")
            else:
                mesh_file = el.find("geometry").find("mesh").get("filename")
                mesh_name = Path(mesh_file).stem
                meshes.add((mesh_name, Path(mesh_file).name))
                g = [
                    f'class="{cls}"',
                    'type="mesh"',
                    f'mesh="{mesh_name}"',
                    f'pos="{_vec(origin.get("xyz"))}"',
                    f'rgba="{rgba}"',
                ]
                body_xml.append(ind + "  " + "<geom " + " ".join(g) + "/>")

        for child in children.get(name, []):
            emit_chain(child, depth + 1)
        body_xml.append(ind + "</body>")

    for root_name in roots:
        emit_chain(root_name, 0)

    mesh_assets = "\n".join(
        f'    <mesh name="{name}" file="{fname}" scale="{MESH_SCALE}"/>'
        for name, fname in sorted(meshes)
    )

    xml = f"""<mujoco model="elrobot_follower">
  <compiler angle="radian" meshdir="assets" autolimits="true"/>
  <option gravity="0 0 -9.81" timestep="0.002" integrator="implicit"/>

  <visual>
    <global offwidth="1280" offheight="960"/>
  </visual>

  <asset>
    <texture name="texplane" type="2d" builtin="checker" rgb1="0.82 0.82 0.84"
             rgb2="0.72 0.72 0.74" width="512" height="512"/>
    <material name="floor" texture="texplane" texrepeat="6 6" reflectance="0.05"/>
{mesh_assets}
  </asset>

  <default>
    <geom friction="0.4 0.005 0.0001"/>
    <default class="visual">
      <geom contype="0" conaffinity="0" group="0"/>
    </default>
    <default class="collision">
      <!-- contype=1/conaffinity=0: the arm collides with the floor but its
           links never collide with each other (the CAD collision meshes
           interpenetrate at the assembly pose). -->
      <geom contype="1" conaffinity="0" group="1" rgba="0.6 0.6 0.6 1"/>
    </default>
  </default>

  <worldbody>
    <light name="key" pos="0.6 -0.6 0.8" dir="-0.5 0.5 -0.6" diffuse="0.8 0.8 0.8"
           specular="0.2 0.2 0.2" directional="true" castshadow="true"/>
    <light name="fill" pos="-0.6 0.4 0.6" diffuse="0.35 0.35 0.38"/>

    <geom name="floor" type="plane" size="1.5 1.5 0.05" pos="0 0 0"
          material="floor" contype="0" conaffinity="1"/>

    <!-- Graspable test block (dynamic free body) -->
    <body name="block" pos="{BLOCK_POS}">
      <freejoint/>
      <geom name="block_geom" type="box" size="{BLOCK_SIZE}" mass="{BLOCK_MASS}"
            rgba="0.9 0.35 0.2 1" friction="0.8 0.05 0.001"
            contype="1" conaffinity="1"/>
    </body>

    <!-- ElRobot follower arm; base_link is mounted at z=BASE_HEIGHT. -->
{chr(10).join(body_xml)}
  </worldbody>

  <actuator>
{chr(10).join(actuator_defs)}
  </actuator>
</mujoco>
"""
    OUT_PATH.write_text(xml)
    print(f"Wrote {OUT_PATH}")
    print(f"  bodies    : {len(links)}")
    print(f"  meshes    : {len(meshes)}")
    print(f"  actuators : {len(actuator_defs)}")


if __name__ == "__main__":
    main()
