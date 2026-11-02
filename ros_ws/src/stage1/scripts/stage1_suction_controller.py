#!/usr/bin/env python3

import json
import re
import shutil
import subprocess
import xml.etree.ElementTree as ET

import rclpy
from rclpy.node import Node
from ros_gz_interfaces.msg import Contacts, Entity, MaterialColor
from std_msgs.msg import Bool, ColorRGBA, Empty, String


MODEL_NAME_PATTERN = re.compile(r"stoneware_l[1-5]_[1-5]")


class Stage1SuctionController(Node):
    def __init__(self):
        super().__init__("stage1_suction_controller")
        self.declare_parameter("cylinder_names", [""])
        self.declare_parameter("world_name", "stage1_raw_stoneware")
        self.declare_parameter("robot_model_name", "ur")
        self._cylinder_names = set(
            self.get_parameter("cylinder_names")
            .get_parameter_value()
            .string_array_value
        )
        self._world_name = self.get_parameter("world_name").value
        self._robot_model_name = self.get_parameter("robot_model_name").value
        self._cylinder_names.discard("")
        if not self._cylinder_names:
            raise RuntimeError("The suction controller needs configured cylinder names")

        self._enabled = False
        self._attached_name = None
        self._pending_attach_name = None
        self._pending_detach_name = None
        self._robot_entity_id = None
        self._pad_contacts = {pad: set() for pad in range(1, 6)}
        self._loaded_joint_plugins = set()
        self._detachable_states = {}
        self._attach_publishers = {}
        self._detach_publishers = {}

        for cylinder_name in sorted(self._cylinder_names):
            self._attach_publishers[cylinder_name] = self.create_publisher(
                Empty,
                f"/stage1/suction/attach/{cylinder_name}",
                10,
            )
            self._detach_publishers[cylinder_name] = self.create_publisher(
                Empty,
                f"/stage1/suction/detach/{cylinder_name}",
                10,
            )
            self._detachable_states[cylinder_name] = None
            self.create_subscription(
                String,
                f"/model/{cylinder_name}/detachable_joint/state",
                lambda message, name=cylinder_name: (
                    self._on_detachable_joint_state(name, message)
                ),
                10,
            )
        self._enabled_publisher = self.create_publisher(
            Bool, "/stage1/suction/enabled", 10
        )
        self._attached_publisher = self.create_publisher(
            Bool, "/stage1/suction/attached", 10
        )
        self._material_publisher = self.create_publisher(
            MaterialColor,
            "/world/stage1_raw_stoneware/material_color",
            10,
        )
        self.create_subscription(
            Bool,
            "/stage1/suction/command",
            self._on_suction_command,
            10,
        )
        for pad in range(1, 6):
            self.create_subscription(
                Contacts,
                f"/world/{self._world_name}/model/{self._robot_model_name}/"
                "link/stage1_suction_manifold/"
                f"sensor/stage1_suction_pad_{pad}_contact/contact",
                lambda message, pad_index=pad: self._on_pad_contacts(
                    pad_index, message
                ),
                10,
            )

        self._publish_state()
        self._publish_pad_colors()
        self.get_logger().info(
            "Suction ready. Publish std_msgs/Bool to /stage1/suction/command."
        )

    def _on_suction_command(self, message):
        if message.data == self._enabled:
            return

        self._enabled = message.data
        if self._enabled:
            self.get_logger().info("Suction enabled.")
            self._try_attach()
        else:
            cylinder_name = (
                self._attached_name or self._pending_attach_name
            )
            if cylinder_name is not None:
                self._pending_detach_name = cylinder_name
                self._detach_publishers[cylinder_name].publish(Empty())
                self.get_logger().info(
                    f"Requested release of {cylinder_name} by disabling suction."
                )

        self._publish_state()
        self._publish_pad_colors()

    def _on_pad_contacts(self, pad, message):
        contacted_models = set()
        for contact in message.contacts:
            for entity in (contact.collision1, contact.collision2):
                match = MODEL_NAME_PATTERN.search(entity.name)
                if match is not None and match.group() in self._cylinder_names:
                    contacted_models.add(match.group())
        self._pad_contacts[pad] = contacted_models
        if self._enabled:
            self._try_attach()

    def _on_detachable_joint_state(self, cylinder_name, message):
        state = message.data.strip().lower()
        if state not in {"attached", "detached"}:
            self.get_logger().warning(
                f"Gazebo reported an unknown detachable-joint state for "
                f"{cylinder_name}: {message.data!r}"
            )
            return
        if self._detachable_states[cylinder_name] == state:
            return

        self._detachable_states[cylinder_name] = state
        self.get_logger().info(
            f"Gazebo detachable-joint state for {cylinder_name}: {state}"
        )
        if state == "attached":
            if self._pending_attach_name != cylinder_name:
                return
            self._attached_name = cylinder_name
            self._pending_attach_name = None
            self.get_logger().info(f"Vacuum attached to {cylinder_name}.")
            self._publish_state()
            if not self._enabled:
                self._pending_detach_name = cylinder_name
                self._detach_publishers[cylinder_name].publish(Empty())
            return

        changed = False
        if self._attached_name == cylinder_name:
            self._attached_name = None
            changed = True
        if self._pending_attach_name == cylinder_name:
            self._pending_attach_name = None
        if self._pending_detach_name == cylinder_name:
            self._pending_detach_name = None
            changed = True
            self.get_logger().info(f"Released {cylinder_name}.")
        if changed:
            self._publish_state()

    def _try_attach(self):
        if (
            self._attached_name is not None
            or self._pending_attach_name is not None
        ):
            return

        contacted_models = set().union(*self._pad_contacts.values())
        if not contacted_models:
            return

        cylinder_name = max(
            contacted_models,
            key=lambda name: (
                int(name.split("_")[1][1:]),
                -int(name.rsplit("_", 1)[1]),
            ),
        )
        self._pending_attach_name = cylinder_name
        if cylinder_name not in self._loaded_joint_plugins:
            try:
                self._load_detachable_joint(cylinder_name)
            except (OSError, RuntimeError, subprocess.TimeoutExpired) as exc:
                self._pending_attach_name = None
                self.get_logger().error(
                    f"Could not attach {cylinder_name}: {exc}"
                )
                return
            self._loaded_joint_plugins.add(cylinder_name)
        else:
            self._attach_publishers[cylinder_name].publish(Empty())

        self.get_logger().info(
            f"Waiting for Gazebo to confirm attachment of {cylinder_name}."
        )

    def _load_detachable_joint(self, cylinder_name):
        gz_executable = shutil.which("gz")
        if gz_executable is None:
            raise RuntimeError("The Gazebo `gz` executable is not available in PATH")
        if self._robot_entity_id is None:
            scene_graph = self._call_gazebo_service(
                gz_executable,
                "/world/stage1_raw_stoneware/scene/graph",
                "gz.msgs.Empty",
                "gz.msgs.StringMsg",
                "",
            )
            robot_match = re.search(
                r'(\d+)\s+\[label=\\?"ur \((\d+)\)\\?"\]',
                scene_graph,
            )
            if robot_match is None:
                raise RuntimeError(
                    "Could not find the `ur` model in Gazebo's scene graph"
                )
            self._robot_entity_id = int(robot_match.group(1))

        plugin_config = ET.Element("plugin")
        ET.SubElement(plugin_config, "parent_link").text = (
            "stage1_suction_manifold"
        )
        ET.SubElement(plugin_config, "child_model").text = cylinder_name
        ET.SubElement(plugin_config, "child_link").text = "stoneware_link"
        ET.SubElement(plugin_config, "attach_topic").text = (
            f"/stage1/suction/attach/{cylinder_name}"
        )
        ET.SubElement(plugin_config, "detach_topic").text = (
            f"/stage1/suction/detach/{cylinder_name}"
        )
        inner_xml = "".join(
            ET.tostring(child, encoding="unicode")
            for child in plugin_config
        )
        request = (
            f"entity {{ id: {self._robot_entity_id} type: MODEL }} "
            'plugins { name: "gz::sim::systems::DetachableJoint" '
            'filename: "gz-sim-detachable-joint-system" '
            f"innerxml: {json.dumps(inner_xml)} }}"
        )
        response = self._call_gazebo_service(
            gz_executable,
            "/world/stage1_raw_stoneware/entity/system/add",
            "gz.msgs.EntityPlugin_V",
            "gz.msgs.Boolean",
            request,
        )
        if "data: true" not in response.lower():
            raise RuntimeError(
                "Gazebo did not confirm the detachable-joint system was added: "
                f"{response.strip()}"
            )

    def _call_gazebo_service(
        self, gz_executable, service, request_type, response_type, request
    ):
        result = subprocess.run(
            [
                gz_executable,
                "service",
                "--service",
                service,
                "--reqtype",
                request_type,
                "--reptype",
                response_type,
                "--timeout",
                "3000",
                "--req",
                request,
            ],
            check=False,
            capture_output=True,
            text=True,
            timeout=5,
        )
        if result.returncode != 0:
            raise RuntimeError(
                f"Gazebo service {service} failed: {result.stderr.strip()}"
            )
        return result.stdout

    def _publish_state(self):
        enabled = Bool()
        enabled.data = self._enabled
        self._enabled_publisher.publish(enabled)

        attached = Bool()
        attached.data = self._attached_name is not None
        self._attached_publisher.publish(attached)

    def _publish_pad_colors(self):
        color = (
            ColorRGBA(r=0.1, g=0.9, b=0.15, a=1.0)
            if self._enabled
            else ColorRGBA(r=0.85, g=0.12, b=0.08, a=1.0)
        )
        for pad in range(1, 6):
            material = MaterialColor()
            material.entity = Entity(
                name=(
                    "ur::stage1_suction_manifold::"
                    "stage1_suction_manifold_fixed_joint_lump__"
                    f"stage1_suction_pad_{pad}_visual_{pad}"
                ),
                type=Entity.VISUAL,
            )
            material.entity_match = MaterialColor.FIRST
            material.ambient = color
            material.diffuse = color
            self._material_publisher.publish(material)


def main(args=None):
    rclpy.init(args=args)
    node = Stage1SuctionController()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
