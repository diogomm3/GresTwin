#!/usr/bin/env python3

import sys
import time

import rclpy
from controller_manager_msgs.srv import ListControllers
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import String


class RobotDescriptionRepublisher(Node):
    def __init__(self):
        super().__init__("robot_description_republisher")
        description = self.declare_parameter("robot_description", "").value
        if not description:
            raise ValueError("The robot_description parameter must not be empty")

        qos = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        self._message = String(data=description)
        self._publisher = self.create_publisher(String, "/robot_description", qos)
        self._controller_manager = self.create_client(
            ListControllers, "/controller_manager/list_controllers"
        )
        self._started_at = time.monotonic()
        self._finished = False
        self._success = False
        self._timer = self.create_timer(0.25, self._publish_until_ready)

    @property
    def finished(self):
        return self._finished

    @property
    def success(self):
        return self._success

    def _publish_until_ready(self):
        self._publisher.publish(self._message)
        if self._controller_manager.service_is_ready():
            self._success = True
            self._finished = True
            self._timer.cancel()
            self.get_logger().info(
                "Controller manager initialized from the published robot description"
            )
        elif time.monotonic() - self._started_at >= 60.0:
            self._finished = True
            self._timer.cancel()
            self.get_logger().error(
                "Controller manager did not initialize from robot_description "
                "within 60 seconds"
            )


def main():
    rclpy.init()
    publisher = RobotDescriptionRepublisher()
    executor = SingleThreadedExecutor()
    executor.add_node(publisher)
    try:
        while rclpy.ok() and not publisher.finished:
            executor.spin_once(timeout_sec=0.1)
    finally:
        success = publisher.success
        executor.remove_node(publisher)
        publisher.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    return 0 if success else 1


if __name__ == "__main__":
    sys.exit(main())
