import math
import os
import shutil
import subprocess
import xml.etree.ElementTree as ET

from ament_index_python.packages import get_package_share_directory


INITIAL_JOINT_ANGLES_DEG = {
    "shoulder_pan_joint": 0.0,
    "shoulder_lift_joint": 0.0,
    "elbow_joint": 0.0,
    "wrist_1_joint": 0.0,
    "wrist_2_joint": 0.0,
    "wrist_3_joint": 0.0,
}


def build_stage1_robot_description(base_pose):
    simulation_share = get_package_share_directory("ur_simulation_gz")
    xacro = shutil.which("xacro")
    if xacro is None:
        raise RuntimeError("The xacro executable is not available in PATH")

    controllers_file = os.path.join(
        simulation_share, "config", "ur_controllers.yaml"
    )
    description_file = os.path.join(
        simulation_share, "urdf", "ur_gz.urdf.xacro"
    )
    result = subprocess.run(
        [
            xacro,
            description_file,
            "safety_limits:=true",
            "safety_pos_margin:=0.15",
            "safety_k_position:=20",
            "name:=ur",
            "ur_type:=ur5e",
            f"simulation_controllers:={controllers_file}",
        ],
        check=True,
        stdout=subprocess.PIPE,
        text=True,
    )
    description = ET.fromstring(result.stdout)

    movable_joints = [
        joint
        for joint in description.findall("joint")
        if joint.get("type") != "fixed"
    ]
    if len(movable_joints) != 6 or any(
        joint.get("type") not in {"revolute", "continuous", "prismatic"}
        for joint in movable_joints
    ):
        raise RuntimeError(
            f"Expected six supported UR5e movable joints, found {len(movable_joints)}"
        )
    _set_robot_base_pose(description, base_pose)
    _set_initial_joint_angles(description)
    _add_stage1_suction_attachment(description)

    for joint in movable_joints:
        if joint.get("name") == "shoulder_lift_joint":
            origin = joint.find("origin")
            if origin is None:
                raise RuntimeError("The UR5e shoulder-lift joint has no origin")
            origin.set("rpy", "0 1.570796327 -1.570796327")
        elif joint.get("name") == "wrist_1_joint":
            _rotate_fixed_joint_about_local_z(joint, math.pi / 2)

    return ET.tostring(description, encoding="unicode")


def add_stage1_suction_attachment(description):
    _add_stage1_suction_attachment(description)


def _add_stage1_suction_attachment(description):
    if any(link.get("name") == "stage1_suction_manifold" for link in description.findall("link")):
        raise RuntimeError("The UR5e description already contains the Stage 1 suction manifold")
    if not any(link.get("name") == "tool0" for link in description.findall("link")):
        raise RuntimeError("The UR5e description does not contain the expected tool0 link")

    manifold = ET.SubElement(
        description, "link", {"name": "stage1_suction_manifold"}
    )
    inertial = ET.SubElement(manifold, "inertial")
    ET.SubElement(inertial, "origin", {"xyz": "0 0 0", "rpy": "0 0 0"})
    ET.SubElement(inertial, "mass", {"value": "0.6"})
    ET.SubElement(
        inertial,
        "inertia",
        {
            "ixx": "0.001",
            "ixy": "0",
            "ixz": "0",
            "iyy": "0.022",
            "iyz": "0",
            "izz": "0.022",
        },
    )

    beam = ET.SubElement(manifold, "visual", {"name": "stage1_manifold_beam"})
    ET.SubElement(beam, "origin", {"xyz": "0 0 0.005", "rpy": "0 0 0"})
    beam_geometry = ET.SubElement(beam, "geometry")
    ET.SubElement(beam_geometry, "box", {"size": "0.68 0.07 0.025"})
    beam_material = ET.SubElement(beam, "material", {"name": "stage1_manifold_gray"})
    ET.SubElement(beam_material, "color", {"rgba": "0.22 0.25 0.27 1"})

    beam_collision = ET.SubElement(
        manifold, "collision", {"name": "stage1_manifold_beam_collision"}
    )
    ET.SubElement(beam_collision, "origin", {"xyz": "0 0 0.005", "rpy": "0 0 0"})
    beam_collision_geometry = ET.SubElement(beam_collision, "geometry")
    ET.SubElement(beam_collision_geometry, "box", {"size": "0.68 0.07 0.025"})

    pad_positions = (-0.28, -0.14, 0.0, 0.14, 0.28)
    for index, position in enumerate(pad_positions, start=1):
        pad_visual = ET.SubElement(manifold, "visual", {"name": f"stage1_suction_pad_{index}"})
        ET.SubElement(
            pad_visual,
            "origin",
            {"xyz": f"{position} 0 0.05", "rpy": "0 0 0"},
        )
        visual_geometry = ET.SubElement(pad_visual, "geometry")
        ET.SubElement(
            visual_geometry,
            "cylinder",
            {"radius": "0.035", "length": "0.04"},
        )
        pad_material = ET.SubElement(
            pad_visual, "material", {"name": f"stage1_suction_pad_{index}_off"}
        )
        ET.SubElement(pad_material, "color", {"rgba": "0.8 0.12 0.08 1"})

        pad_collision = ET.SubElement(
            manifold,
            "collision",
            {"name": f"stage1_suction_pad_{index}_collision"},
        )
        ET.SubElement(
            pad_collision,
            "origin",
            {"xyz": f"{position} 0 0.05", "rpy": "0 0 0"},
        )
        collision_geometry = ET.SubElement(pad_collision, "geometry")
        ET.SubElement(
            collision_geometry,
            "cylinder",
            {"radius": "0.035", "length": "0.04"},
        )

    attachment_joint = ET.SubElement(
        description,
        "joint",
        {"name": "stage1_suction_manifold_fixed_joint", "type": "fixed"},
    )
    ET.SubElement(attachment_joint, "parent", {"link": "tool0"})
    ET.SubElement(attachment_joint, "child", {"link": "stage1_suction_manifold"})
    preserve_joint = ET.SubElement(
        description,
        "gazebo",
        {"reference": "stage1_suction_manifold_fixed_joint"},
    )
    ET.SubElement(preserve_joint, "preserveFixedJoint").text = "true"

    for index in range(1, len(pad_positions) + 1):
        gazebo_link = ET.SubElement(
            description, "gazebo", {"reference": "stage1_suction_manifold"}
        )
        sensor = ET.SubElement(
            gazebo_link,
            "sensor",
            {"name": f"stage1_suction_pad_{index}_contact", "type": "contact"},
        )
        ET.SubElement(sensor, "always_on").text = "true"
        ET.SubElement(sensor, "update_rate").text = "50"
        contact = ET.SubElement(sensor, "contact")
        ET.SubElement(contact, "collision").text = (
            "stage1_suction_manifold_fixed_joint_lump__"
            f"stage1_suction_pad_{index}_collision_collision_{index}"
        )

def _set_robot_base_pose(description, base_pose):
    if len(base_pose) != 3:
        raise ValueError(f"Expected robot base pose (x, y, z), found {base_pose}")
    base_joint = next(
        (
            joint
            for joint in description.findall("joint")
            if joint.get("name") == "base_joint"
        ),
        None,
    )
    if base_joint is None or base_joint.get("type") != "fixed":
        raise RuntimeError("The UR5e description has no fixed base_joint")
    origin = base_joint.find("origin")
    if origin is None:
        raise RuntimeError("The UR5e base_joint has no origin")
    origin.set("xyz", " ".join(f"{coordinate:.12g}" for coordinate in base_pose))


def _set_initial_joint_angles(description):
    control_joints = {
        joint.get("name"): joint
        for control in description.findall("ros2_control")
        for joint in control.findall("joint")
    }
    missing_joints = INITIAL_JOINT_ANGLES_DEG.keys() - control_joints.keys()
    if missing_joints:
        raise RuntimeError(
            f"Robot control description is missing joints: {sorted(missing_joints)}"
        )

    for joint_name, angle_degrees in INITIAL_JOINT_ANGLES_DEG.items():
        joint = control_joints[joint_name]
        position_state = next(
            (
                state
                for state in joint.findall("state_interface")
                if state.get("name") == "position"
            ),
            None,
        )
        initial_value = (
            None
            if position_state is None
            else next(
                (
                    param
                    for param in position_state.findall("param")
                    if param.get("name") == "initial_value"
                ),
                None,
            )
        )
        if initial_value is None:
            raise RuntimeError(
                f"Robot control description has no initial position for {joint_name}"
            )
        initial_value.text = f"{math.radians(angle_degrees):.12g}"


def _rotate_fixed_joint_about_local_z(joint, angle):
    origin = joint.find("origin")
    if origin is None:
        raise RuntimeError(f"The {joint.get('name')} joint has no origin")
    rpy = [float(value) for value in origin.get("rpy", "0 0 0").split()]
    if len(rpy) != 3:
        raise RuntimeError(f"The {joint.get('name')} joint has an invalid RPY origin")
    joint_orientation = _quaternion_from_rpy(rpy)
    local_rotation = (0.0, 0.0, math.sin(angle / 2), math.cos(angle / 2))
    rotated_orientation = _multiply_quaternions(joint_orientation, local_rotation)
    origin.set(
        "rpy",
        " ".join(
            f"{value:.12g}" for value in _rpy_from_quaternion(rotated_orientation)
        ),
    )


def _quaternion_from_rpy(rpy):
    roll, pitch, yaw = rpy
    half_roll = roll / 2
    half_pitch = pitch / 2
    half_yaw = yaw / 2
    cr, sr = math.cos(half_roll), math.sin(half_roll)
    cp, sp = math.cos(half_pitch), math.sin(half_pitch)
    cy, sy = math.cos(half_yaw), math.sin(half_yaw)
    return (
        sr * cp * cy - cr * sp * sy,
        cr * sp * cy + sr * cp * sy,
        cr * cp * sy - sr * sp * cy,
        cr * cp * cy + sr * sp * sy,
    )


def _multiply_quaternions(left, right):
    lx, ly, lz, lw = left
    rx, ry, rz, rw = right
    return (
        lw * rx + lx * rw + ly * rz - lz * ry,
        lw * ry - lx * rz + ly * rw + lz * rx,
        lw * rz + lx * ry - ly * rx + lz * rw,
        lw * rw - lx * rx - ly * ry - lz * rz,
    )


def _rpy_from_quaternion(quaternion):
    x, y, z, w = quaternion
    roll = math.atan2(
        2 * (w * x + y * z),
        1 - 2 * (x * x + y * y),
    )
    sin_pitch = 2 * (w * y - z * x)
    pitch = (
        math.copysign(math.pi / 2, sin_pitch)
        if abs(sin_pitch) >= 1
        else math.asin(sin_pitch)
    )
    yaw = math.atan2(
        2 * (w * z + x * y),
        1 - 2 * (y * y + z * z),
    )
    return roll, pitch, yaw
