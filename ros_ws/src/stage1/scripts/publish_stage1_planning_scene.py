#!/usr/bin/env python3

import math
import sys
import time
import xml.etree.ElementTree as ET
from pathlib import Path

import rclpy
from geometry_msgs.msg import Pose
from moveit_msgs.msg import CollisionObject, PlanningScene
from moveit_msgs.msg import PlanningSceneComponents
from moveit_msgs.srv import ApplyPlanningScene, GetPlanningScene
from rclpy.node import Node
from shape_msgs.msg import SolidPrimitive

from stage1_planning_scene_utils import set_allowed_collision_pairs


IDENTITY_POSE = ((0.0, 0.0, 0.0), (0.0, 0.0, 0.0, 1.0))


def _read_pose(element):
    if element is None or not element.text:
        return IDENTITY_POSE
    values = [float(value) for value in element.text.split()]
    if len(values) != 6:
        raise ValueError(f"Expected six SDF pose values, found {values}")

    x, y, z, roll, pitch, yaw = values
    cr, sr = math.cos(roll / 2), math.sin(roll / 2)
    cp, sp = math.cos(pitch / 2), math.sin(pitch / 2)
    cy, sy = math.cos(yaw / 2), math.sin(yaw / 2)
    return (
        (x, y, z),
        (
            sr * cp * cy - cr * sp * sy,
            cr * sp * cy + sr * cp * sy,
            cr * cp * sy - sr * sp * cy,
            cr * cp * cy + sr * sp * sy,
        ),
    )


def _compose_pose(parent, child):
    (px, py, pz), (qx, qy, qz, qw) = parent
    (cx, cy, cz), child_rotation = child
    tx = 2 * (qy * cz - qz * cy)
    ty = 2 * (qz * cx - qx * cz)
    tz = 2 * (qx * cy - qy * cx)
    rotated_child = (
        cx + qw * tx + qy * tz - qz * ty,
        cy + qw * ty + qz * tx - qx * tz,
        cz + qw * tz + qx * ty - qy * tx,
    )
    rx, ry, rz, rw = child_rotation
    return (
        (px + rotated_child[0], py + rotated_child[1], pz + rotated_child[2]),
        (
            qw * rx + qx * rw + qy * rz - qz * ry,
            qw * ry - qx * rz + qy * rw + qz * rx,
            qw * rz + qx * ry - qy * rx + qz * rw,
            qw * rw - qx * rx - qy * ry - qz * rz,
        ),
    )


def _pose_message(pose):
    result = Pose()
    (result.position.x, result.position.y, result.position.z), (
        result.orientation.x,
        result.orientation.y,
        result.orientation.z,
        result.orientation.w,
    ) = pose
    return result


def _primitive_from_sdf(geometry):
    primitive = SolidPrimitive()

    box = geometry.find("box/size")
    if box is not None:
        primitive.type = SolidPrimitive.BOX
        primitive.dimensions = [float(value) for value in box.text.split()]
        return primitive, IDENTITY_POSE

    cylinder = geometry.find("cylinder")
    if cylinder is not None:
        primitive.type = SolidPrimitive.CYLINDER
        primitive.dimensions = [
            float(cylinder.findtext("length")),
            float(cylinder.findtext("radius")),
        ]
        return primitive, IDENTITY_POSE

    sphere = geometry.find("sphere/radius")
    if sphere is not None:
        primitive.type = SolidPrimitive.SPHERE
        primitive.dimensions = [float(sphere.text)]
        return primitive, IDENTITY_POSE

    plane = geometry.find("plane")
    if plane is not None:
        primitive.type = SolidPrimitive.BOX
        primitive.dimensions = [
            *(float(value) for value in plane.findtext("size").split()),
            0.01,
        ]
        return primitive, ((0.0, 0.0, -0.005), (0.0, 0.0, 0.0, 1.0))

    raise ValueError("Unsupported SDF geometry in Stage 1 world")


def _collision_object(model, object_id, world_pose):
    result = CollisionObject()
    result.header.frame_id = "world"
    result.id = object_id
    result.pose = _pose_message(_compose_pose(world_pose, _read_pose(model.find("pose"))))

    links = model.findall("link")
    if not links:
        raise ValueError(f"SDF model {object_id} contains no links")

    for link in links:
        link_pose = _read_pose(link.find("pose"))
        shapes = link.findall("collision") or link.findall("visual")
        for shape in shapes:
            geometry = shape.find("geometry")
            if geometry is None:
                continue
            primitive, geometry_pose = _primitive_from_sdf(geometry)
            local_pose = _compose_pose(
                _compose_pose(link_pose, _read_pose(shape.find("pose"))),
                geometry_pose,
            )
            result.primitives.append(primitive)
            result.primitive_poses.append(_pose_message(local_pose))

    if not result.primitives:
        raise ValueError(f"SDF model {object_id} contains no supported shapes")
    result.operation = CollisionObject.ADD
    return result


def build_stage1_collision_objects(world_file, models_dir):
    world_root = ET.parse(world_file).getroot()
    world = world_root.find("world")
    if world is None:
        raise ValueError(f"No world element in {world_file}")

    collision_objects = []
    for model in world.findall("model"):
        collision_objects.append(
            _collision_object(model, model.get("name"), IDENTITY_POSE)
        )

    for include in world.findall("include"):
        uri = include.findtext("uri", "")
        if not uri.startswith("model://"):
            continue
        model_name = uri.removeprefix("model://")
        model_file = Path(models_dir, model_name, "model.sdf")
        model_root = ET.parse(model_file).getroot()
        model = model_root.find("model")
        if model is None:
            raise ValueError(f"No model element in {model_file}")
        object_id = include.findtext("name", model_name)
        collision_objects.append(
            _collision_object(model, object_id, _read_pose(include.find("pose")))
        )

    return collision_objects


class Stage1PlanningScenePublisher(Node):
    def __init__(self):
        super().__init__("stage1_planning_scene_publisher")
        self.declare_parameter("world_file", "")
        self.declare_parameter("models_dir", "")
        world_file = self.get_parameter("world_file").value
        models_dir = self.get_parameter("models_dir").value
        collision_objects = build_stage1_collision_objects(world_file, models_dir)

        scene = PlanningScene()
        scene.is_diff = True
        scene.world.collision_objects = collision_objects

        get_scene_client = self.create_client(
            GetPlanningScene, "/get_planning_scene"
        )
        apply_scene_client = self.create_client(
            ApplyPlanningScene, "/apply_planning_scene"
        )
        deadline = time.monotonic() + 60.0
        while rclpy.ok() and (
            not get_scene_client.wait_for_service(timeout_sec=1.0)
            or not apply_scene_client.wait_for_service(timeout_sec=1.0)
        ):
            if time.monotonic() >= deadline:
                raise TimeoutError("MoveGroup planning-scene services did not start")

        get_request = GetPlanningScene.Request()
        get_request.components.components = (
            PlanningSceneComponents.ALLOWED_COLLISION_MATRIX
        )
        get_future = get_scene_client.call_async(get_request)
        rclpy.spin_until_future_complete(self, get_future, timeout_sec=10.0)
        if not get_future.done():
            raise TimeoutError("Timed out reading MoveIt's collision matrix")
        get_response = get_future.result()
        if get_response is None:
            raise RuntimeError("MoveIt returned no allowed-collision matrix")
        scene.allowed_collision_matrix = set_allowed_collision_pairs(
            get_response.scene.allowed_collision_matrix,
            [
                (
                    "base_link_inertia",
                    "stage1_robot_pedestal",
                    True,
                ),
                ("stage1_suction_manifold", "wrist_3_link", True),
            ],
        )

        request = ApplyPlanningScene.Request()
        request.scene = scene

        future = apply_scene_client.call_async(request)
        rclpy.spin_until_future_complete(self, future, timeout_sec=30.0)
        if not future.done():
            raise TimeoutError("Timed out applying the Stage 1 planning scene")
        response = future.result()
        if response is None or not response.success:
            raise RuntimeError("MoveGroup rejected the Stage 1 planning scene")

        self.get_logger().info(
            f"Added {len(collision_objects)} Stage 1 world models and allowed "
            "the robot-to-pedestal and suction-mount contacts in MoveIt"
        )


def main():
    rclpy.init()
    node = None
    exit_code = 0
    try:
        node = Stage1PlanningScenePublisher()
    except Exception as error:
        if node is not None:
            node.get_logger().error(str(error))
        else:
            print(f"Failed to publish Stage 1 planning scene: {error}", file=sys.stderr)
        exit_code = 1
    finally:
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    return exit_code


if __name__ == "__main__":
    sys.exit(main())