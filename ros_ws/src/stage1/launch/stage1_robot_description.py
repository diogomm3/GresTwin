import os
import shutil
import subprocess
import xml.etree.ElementTree as ET

from ament_index_python.packages import get_package_share_directory


def build_stationary_robot_description():
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

    control_plugin = next(
        (
            plugin
            for gazebo in description.findall("gazebo")
            for plugin in gazebo.findall("plugin")
            if plugin.get("name") == "gz_ros2_control::GazeboSimROS2ControlPlugin"
        ),
        None,
    )
    if control_plugin is None:
        raise RuntimeError("The generated UR5e description has no gz_ros2_control plugin")
    next(
        gazebo
        for gazebo in description.findall("gazebo")
        if control_plugin in list(gazebo)
    ).remove(control_plugin)

    _add_stage1_end_effector(description)
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
    for joint in movable_joints:
        joint.set("type", "fixed")

    return ET.tostring(description, encoding="unicode"), [
        joint.get("name") for joint in movable_joints
    ]


def _add_stage1_end_effector(robot):
    if not any(link.get("name") == "tool0" for link in robot.findall("link")):
        raise RuntimeError("The UR5e description does not contain the expected tool0 link")

    link = ET.SubElement(robot, "link", {"name": "stage1_suction_manifold"})
    inertial = ET.SubElement(link, "inertial")
    ET.SubElement(inertial, "origin", {"xyz": "0 0 0", "rpy": "0 0 0"})
    ET.SubElement(inertial, "mass", {"value": "2.0"})
    ET.SubElement(
        inertial,
        "inertia",
        {
            "ixx": "0.0007",
            "ixy": "0",
            "ixz": "0",
            "iyy": "0.118",
            "iyz": "0",
            "izz": "0.118",
        },
    )

    beam = ET.SubElement(link, "visual", {"name": "stage1_manifold_beam"})
    ET.SubElement(beam, "origin", {"xyz": "0 0 0", "rpy": "0 0 0"})
    beam_geometry = ET.SubElement(beam, "geometry")
    ET.SubElement(beam_geometry, "box", {"size": "0.84 0.065 0.035"})
    beam_material = ET.SubElement(beam, "material", {"name": "stage1_manifold_gray"})
    ET.SubElement(beam_material, "color", {"rgba": "0.22 0.25 0.27 1"})
    beam_collision = ET.SubElement(link, "collision", {"name": "stage1_beam_collision"})
    ET.SubElement(beam_collision, "origin", {"xyz": "0 0 0", "rpy": "0 0 0"})
    beam_collision_geometry = ET.SubElement(beam_collision, "geometry")
    ET.SubElement(beam_collision_geometry, "box", {"size": "0.84 0.065 0.035"})

    for index, position in enumerate((-0.36, -0.18, 0.0, 0.18, 0.36), start=1):
        for element_name, name in (
            ("visual", f"stage1_suction_pad_{index}"),
            ("collision", f"stage1_suction_pad_{index}_collision"),
        ):
            pad = ET.SubElement(link, element_name, {"name": name})
            ET.SubElement(
                pad,
                "origin",
                {"xyz": f"{position} 0 -0.0275", "rpy": "0 0 0"},
            )
            geometry = ET.SubElement(pad, "geometry")
            ET.SubElement(geometry, "cylinder", {"radius": "0.035", "length": "0.02"})
            if element_name == "visual":
                material = ET.SubElement(
                    pad, "material", {"name": "stage1_suction_pad_rubber"}
                )
                ET.SubElement(material, "color", {"rgba": "0.08 0.09 0.10 1"})

    joint = ET.SubElement(
        robot,
        "joint",
        {"name": "stage1_suction_manifold_fixed_joint", "type": "fixed"},
    )
    ET.SubElement(joint, "parent", {"link": "tool0"})
    ET.SubElement(joint, "child", {"link": "stage1_suction_manifold"})
    ET.SubElement(joint, "origin", {"xyz": "0 0 0.04", "rpy": "0 0 0"})
