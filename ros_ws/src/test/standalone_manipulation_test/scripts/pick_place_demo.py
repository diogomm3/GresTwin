#!/usr/bin/env python3

import math
import sys
import time

import rclpy
from action_msgs.msg import GoalStatus
from control_msgs.action import FollowJointTrajectory
from geometry_msgs.msg import Pose
from rclpy.action import ActionClient
from rclpy.duration import Duration
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile
from sensor_msgs.msg import JointState
from std_msgs.msg import Empty, String
from trajectory_msgs.msg import JointTrajectoryPoint

JOINTS = ("yaw_joint", "radial_joint", "lift_joint")
VELOCITY_LIMITS = (1.2, 0.40, 0.25)
JOINT_LIMITS = ((-math.pi, math.pi), (-0.35, 0.35), (-0.165, 0.435))
SOURCE = (0.65, 0.0)
DESTINATION = (0.0, 0.65)
OBJECT_CENTER_Z = 0.42
TOOL_CONTACT_Z = 0.495
TOOL_OFFSET_Z = 0.495
ATTACH_STATE_TOPIC = "/workpiece/state"


class PickPlaceDemo(Node):
    def __init__(self):
        super().__init__("pick_place_demo")
        status_qos = QoSProfile(depth=32, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self._status_publisher = self.create_publisher(
            String, "/manipulation_demo/state", status_qos
        )
        self._attach_publisher = self.create_publisher(Empty, "/workpiece/attach", 10)
        self._detach_publisher = self.create_publisher(Empty, "/workpiece/detach", 10)
        self._joint_positions = {}
        self._workpiece_position = None
        self._attachment_state = None
        self._state_subscription = self.create_subscription(
            String, ATTACH_STATE_TOPIC, self._on_attachment_state, 10
        )
        self._joint_subscription = self.create_subscription(
            JointState, "/joint_states", self._on_joint_state, 10
        )
        self._pose_subscription = self.create_subscription(
            Pose, "/model/workpiece/pose", self._on_workpiece_pose, 10
        )
        self._trajectory_client = ActionClient(
            self,
            FollowJointTrajectory,
            "/arm_controller/follow_joint_trajectory",
        )
        self._executor = SingleThreadedExecutor()
        self._executor.add_node(self)

    def _on_attachment_state(self, message):
        self._attachment_state = message.data.strip().lower()
        self.get_logger().info(f"WORKPIECE_STATE {self._attachment_state}")

    def _on_joint_state(self, message):
        self._joint_positions.update(zip(message.name, message.position))

    def _on_workpiece_pose(self, message):
        position = message.position
        self._workpiece_position = (position.x, position.y, position.z)

    def _publish_state(self, state):
        message = String()
        message.data = state
        self._status_publisher.publish(message)
        self.get_logger().info(f"DEMO_STATE {state}")

    def _wait_until(self, predicate, timeout, description):
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            if predicate():
                return
            self._executor.spin_once(timeout_sec=min(0.1, deadline - time.monotonic()))
        raise TimeoutError(f"Timed out waiting for {description}")

    @staticmethod
    def _joint_target(x, y, tool_z):
        yaw = math.atan2(y, x)
        radial = math.hypot(x, y) - 0.65
        lift = tool_z - TOOL_OFFSET_Z
        target = (yaw, radial, lift)
        for name, value, (lower, upper) in zip(JOINTS, target, JOINT_LIMITS):
            if not lower <= value <= upper:
                raise ValueError(
                    f"Cartesian target ({x:.3f}, {y:.3f}, {tool_z:.3f}) "
                    f"requires {name}={value:.3f}, outside [{lower:.3f}, {upper:.3f}]"
                )
        return target

    def _move_to(self, name, x, y, tool_z):
        target = self._joint_target(x, y, tool_z)
        self._wait_until(
            lambda: all(joint in self._joint_positions for joint in JOINTS),
            15.0,
            "joint state feedback",
        )
        current = tuple(self._joint_positions[joint] for joint in JOINTS)
        duration_seconds = max(
            2.0,
            max(
                2.0 * abs(goal - start) / max_velocity
                for start, goal, max_velocity in zip(current, target, VELOCITY_LIMITS)
            )
            + 0.5,
        )
        duration_seconds = min(duration_seconds, 8.0)
        duration_whole = int(duration_seconds)
        duration_nanos = int((duration_seconds - duration_whole) * 1_000_000_000)

        goal = FollowJointTrajectory.Goal()
        goal.trajectory.joint_names = list(JOINTS)
        point = JointTrajectoryPoint()
        point.positions = list(target)
        point.time_from_start = Duration(
            seconds=duration_whole + duration_nanos / 1_000_000_000
        ).to_msg()
        goal.trajectory.points = [point]

        self.get_logger().info(f"Moving to {name}: xyz=({x:.3f}, {y:.3f}, {tool_z:.3f})")
        goal_future = self._trajectory_client.send_goal_async(goal)
        self._wait_until(goal_future.done, 15.0, f"{name} trajectory acceptance")
        goal_handle = goal_future.result()
        if goal_handle is None or not goal_handle.accepted:
            raise RuntimeError(f"Trajectory controller rejected target pose '{name}'")

        result_future = goal_handle.get_result_async()
        self._wait_until(
            result_future.done,
            duration_seconds + 8.0,
            f"{name} trajectory completion",
        )
        wrapped_result = result_future.result()
        if wrapped_result is None or wrapped_result.status != GoalStatus.STATUS_SUCCEEDED:
            raise RuntimeError(f"Trajectory to '{name}' did not succeed")
        result = wrapped_result.result
        if result.error_code != FollowJointTrajectory.Result.SUCCESSFUL:
            raise RuntimeError(
                f"Trajectory to '{name}' failed ({result.error_code}): {result.error_string}"
            )

        self._wait_until(
            lambda: all(
                abs(self._joint_positions.get(joint, math.inf) - value) < tolerance
                for joint, value, tolerance in zip(JOINTS, target, (0.03, 0.015, 0.015))
            ),
            5.0,
            f"{name} joint position feedback",
        )

    def _wait_for_object_pose(self, target, tolerance, timeout, description):
        self._wait_until(
            lambda: (
                self._workpiece_position is not None
                and all(
                    abs(actual - expected) <= tolerance
                    for actual, expected in zip(self._workpiece_position, target)
                )
            ),
            timeout,
            description,
        )

    def _wait_for_attachment(self, expected, timeout=5.0):
        self._wait_until(
            lambda: self._attachment_state == expected,
            timeout,
            f"workpiece {expected} state",
        )

    def run_demo(self):
        self._publish_state("WAITING_FOR_SIMULATION")
        self._wait_until(
            lambda: self._trajectory_client.wait_for_server(timeout_sec=0.1),
            45.0,
            "active arm trajectory controller",
        )
        self._wait_until(
            lambda: (
                self._workpiece_position is not None
                and all(joint in self._joint_positions for joint in JOINTS)
            ),
            45.0,
            "robot and workpiece state",
        )
        self._wait_for_object_pose(
            (SOURCE[0], SOURCE[1], OBJECT_CENTER_Z),
            0.04,
            8.0,
            "cylinder at its pickup pose",
        )

        self._publish_state("INITIAL_RELEASE")
        self._detach_publisher.publish(Empty())
        self._wait_for_attachment("detached")
        self._wait_for_object_pose(
            (SOURCE[0], SOURCE[1], OBJECT_CENTER_Z),
            0.04,
            8.0,
            "cylinder settled at its pickup pose",
        )

        self._publish_state("SAFE_START")
        self._move_to("safe_start", 0.40, 0.0, 0.90)

        self._publish_state("PREGRASP")
        self._move_to("pickup_approach", SOURCE[0], SOURCE[1], 0.70)

        self._publish_state("APPROACH_PICK")
        self._move_to("pickup_contact", SOURCE[0], SOURCE[1], TOOL_CONTACT_Z)

        self._publish_state("GRASP")
        self._attach_publisher.publish(Empty())
        self._wait_for_attachment("attached")

        self._publish_state("LIFT")
        self._move_to("pickup_lift", SOURCE[0], SOURCE[1], 0.72)
        self._wait_until(
            lambda: (
                self._workpiece_position is not None
                and self._workpiece_position[2] >= OBJECT_CENTER_Z + 0.14
            ),
            5.0,
            "workpiece lift clearance "
            f"(position={self._workpiece_position}, attachment={self._attachment_state})",
        )

        self._publish_state("TRANSFER")
        self._move_to("destination_approach", DESTINATION[0], DESTINATION[1], 0.72)
        self._wait_until(
            lambda: (
                self._workpiece_position is not None
                and abs(self._workpiece_position[0] - DESTINATION[0]) < 0.05
                and abs(self._workpiece_position[1] - DESTINATION[1]) < 0.05
                and self._workpiece_position[2] >= OBJECT_CENTER_Z + 0.14
            ),
            5.0,
            "workpiece transport to the destination",
        )

        self._publish_state("PLACE")
        self._move_to(
            "destination_contact",
            DESTINATION[0],
            DESTINATION[1],
            TOOL_CONTACT_Z,
        )
        self._publish_state("RELEASE")
        self._detach_publisher.publish(Empty())
        self._wait_for_attachment("detached")
        self._wait_for_object_pose(
            (DESTINATION[0], DESTINATION[1], OBJECT_CENTER_Z),
            0.05,
            8.0,
            "released workpiece resting at the destination",
        )

        self._publish_state("RETREAT")
        self._move_to("destination_retreat", DESTINATION[0], DESTINATION[1], 0.72)
        self._move_to("safe_return", 0.40, 0.0, 0.90)
        self._publish_state("SUCCEEDED")


def main():
    rclpy.init()
    demo = PickPlaceDemo()
    success = False
    try:
        demo.run_demo()
        success = True
    except KeyboardInterrupt:
        demo.get_logger().warning("Pick-and-place demonstration was interrupted")
    except (RuntimeError, TimeoutError, ValueError) as error:
        demo.get_logger().error(f"Pick-and-place demonstration failed: {error}")
        demo._publish_state("FAILED")
    finally:
        demo._executor.remove_node(demo)
        demo.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    return 0 if success else 1


if __name__ == "__main__":
    sys.exit(main())
