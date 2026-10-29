#!/usr/bin/env python3

import math
import sys
import time
import xml.etree.ElementTree as ET

import rclpy
from action_msgs.msg import GoalStatus
from controller_manager_msgs.srv import ListControllers
from moveit_msgs.action import ExecuteTrajectory
from moveit_msgs.msg import Constraints, JointConstraint, MoveItErrorCodes
from moveit_msgs.srv import GetMotionPlan
from rclpy.action import ActionClient
from rclpy.executors import SingleThreadedExecutor
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import JointState
from std_msgs.msg import String


EXPECTED_JOINTS = (
    "shoulder_pan_joint",
    "shoulder_lift_joint",
    "elbow_joint",
    "wrist_1_joint",
    "wrist_2_joint",
    "wrist_3_joint",
)
CONTROLLER_NAMES = (
    "joint_state_broadcaster",
    "scaled_joint_trajectory_controller",
)


class UR5eValidator:
    def __init__(self):
        self.node = rclpy.create_node("ur5e_moveit_validator")
        self.executor = SingleThreadedExecutor()
        self.executor.add_node(self.node)
        self.state_samples = []
        self.robot_description = None
        self.node.create_subscription(
            JointState, "/joint_states", self._on_joint_state, 10
        )
        description_qos = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        self.node.create_subscription(
            String,
            "/robot_description",
            self._on_robot_description,
            description_qos,
        )
        self.controllers_client = self.node.create_client(
            ListControllers, "/controller_manager/list_controllers"
        )
        self.plan_client = self.node.create_client(
            GetMotionPlan, "/plan_kinematic_path"
        )
        self.execute_client = ActionClient(
            self.node, ExecuteTrajectory, "/execute_trajectory"
        )

    def _on_joint_state(self, message):
        self.state_samples.append(message)
        if len(self.state_samples) > 10:
            del self.state_samples[:-10]

    def _on_robot_description(self, message):
        self.robot_description = message.data

    def wait_for(self, condition, timeout, failure):
        deadline = time.monotonic() + timeout
        while not condition() and time.monotonic() < deadline:
            remaining = deadline - time.monotonic()
            self.executor.spin_once(timeout_sec=min(0.1, max(0.0, remaining)))
        if not condition():
            raise RuntimeError(failure)

    def current_positions(self):
        if not self.state_samples:
            raise RuntimeError("No joint-state feedback has been received")
        message = self.state_samples[-1]
        if set(message.name) != set(EXPECTED_JOINTS) or len(message.name) != 6:
            raise RuntimeError(f"Unexpected UR5e joint names: {message.name}")
        if len(message.position) != 6:
            raise RuntimeError(f"Expected six joint positions, received {len(message.position)}")
        positions = dict(zip(message.name, message.position))
        if not all(math.isfinite(positions[name]) for name in EXPECTED_JOINTS):
            raise RuntimeError(f"Joint positions contain non-finite values: {positions}")
        if (
            message.header.stamp.sec == 0
            and message.header.stamp.nanosec == 0
        ):
            raise RuntimeError(f"Joint-state timestamp is invalid: {message.header.stamp}")
        return positions

    def validate_robot_description(self):
        self.wait_for(
            lambda: self.robot_description is not None,
            15,
            "No transient-local /robot_description message received",
        )
        try:
            root = ET.fromstring(self.robot_description)
        except ET.ParseError as error:
            raise RuntimeError(f"Invalid robot description XML: {error}") from error

        joints = {
            joint.attrib["name"]: joint
            for joint in root.findall("joint")
            if joint.attrib.get("type") != "fixed"
        }
        if set(joints) != set(EXPECTED_JOINTS):
            raise RuntimeError(
                f"Robot description joints do not match UR5e: {sorted(joints)}"
            )

        limits = {}
        for name in EXPECTED_JOINTS:
            limit = joints[name].find("limit")
            if limit is None or "lower" not in limit.attrib or "upper" not in limit.attrib:
                raise RuntimeError(f"Joint {name} has no position limits in the URDF")
            limits[name] = (
                float(limit.attrib["lower"]),
                float(limit.attrib["upper"]),
            )
        print(f"PASS: robot description contains six limited UR5e joints ({root.attrib['name']})")
        return limits

    def validate_state(self):
        self.wait_for(
            lambda: len(self.state_samples) >= 2,
            15,
            "Fewer than two /joint_states messages received",
        )
        first, second = self.state_samples[-2:]
        first_stamp = (first.header.stamp.sec, first.header.stamp.nanosec)
        second_stamp = (second.header.stamp.sec, second.header.stamp.nanosec)
        if second_stamp <= first_stamp:
            raise RuntimeError(
                f"Joint-state timestamps did not advance: {first_stamp} -> {second_stamp}"
            )
        positions = self.current_positions()
        print("PASS: six finite joint positions and advancing joint-state timestamps")
        return positions

    def validate_controllers(self):
        if not self.controllers_client.wait_for_service(timeout_sec=15):
            raise RuntimeError("Controller-manager list_controllers service is unavailable")
        future = self.controllers_client.call_async(ListControllers.Request())
        self.wait_for(
            future.done,
            15,
            "Timed out querying controller-manager list_controllers",
        )
        response = future.result()
        controllers = {controller.name: controller for controller in response.controller}
        missing = set(CONTROLLER_NAMES) - set(controllers)
        if missing:
            raise RuntimeError(f"Missing expected controller(s): {sorted(missing)}")
        for name in CONTROLLER_NAMES:
            if controllers[name].state != "active":
                raise RuntimeError(
                    f"Controller {name} is {controllers[name].state}, not active"
                )

        expected_command_interfaces = {
            f"{joint}/position" for joint in EXPECTED_JOINTS
        }
        trajectory_controller = controllers["scaled_joint_trajectory_controller"]
        if set(trajectory_controller.claimed_interfaces) != expected_command_interfaces:
            raise RuntimeError(
                "Scaled trajectory controller does not claim the six UR5e position "
                f"interfaces: {trajectory_controller.claimed_interfaces}"
            )
        expected_state_interfaces = {
            interface
            for joint in EXPECTED_JOINTS
            for interface in (f"{joint}/position", f"{joint}/velocity")
        }
        if not expected_state_interfaces.issubset(
            set(trajectory_controller.required_state_interfaces)
        ):
            raise RuntimeError(
                "Scaled trajectory controller does not require the UR5e position "
                "and velocity state interfaces"
            )
        print(
            "PASS: joint_state_broadcaster and scaled_joint_trajectory_controller "
            "are active with the six expected interfaces"
        )

    def plan(self, target, limits, label):
        request = GetMotionPlan.Request()
        motion_request = request.motion_plan_request
        motion_request.group_name = "ur_manipulator"
        motion_request.pipeline_id = "ompl"
        motion_request.planner_id = "RRTConnectkConfigDefault"
        motion_request.num_planning_attempts = 3
        motion_request.allowed_planning_time = 5.0
        motion_request.max_velocity_scaling_factor = 0.1
        motion_request.max_acceleration_scaling_factor = 0.1

        current = self.current_positions()
        motion_request.start_state.joint_state.name = list(EXPECTED_JOINTS)
        motion_request.start_state.joint_state.position = [
            current[name] for name in EXPECTED_JOINTS
        ]
        motion_request.start_state.is_diff = True

        goal = Constraints()
        goal.name = label
        for name in EXPECTED_JOINTS:
            lower, upper = limits[name]
            if not lower <= target[name] <= upper:
                raise RuntimeError(f"Target for {name} violates URDF limits")
            goal.joint_constraints.append(
                JointConstraint(
                    joint_name=name,
                    position=target[name],
                    tolerance_above=0.03,
                    tolerance_below=0.03,
                    weight=1.0,
                )
            )
        motion_request.goal_constraints = [goal]

        future = self.plan_client.call_async(request)
        self.wait_for(future.done, 35, f"MoveIt planning timed out for {label}")
        response = future.result().motion_plan_response
        if response.error_code.val != MoveItErrorCodes.SUCCESS:
            raise RuntimeError(
                f"MoveIt planning failed for {label}: error "
                f"{response.error_code.val}"
            )

        trajectory = response.trajectory.joint_trajectory
        if set(trajectory.joint_names) != set(EXPECTED_JOINTS):
            raise RuntimeError(
                f"Planned trajectory has unexpected joints: {trajectory.joint_names}"
            )
        if not trajectory.points:
            raise RuntimeError(f"MoveIt returned no trajectory points for {label}")
        for point in trajectory.points:
            if len(point.positions) != len(trajectory.joint_names):
                raise RuntimeError(f"Trajectory point in {label} has invalid dimensions")
            for name, position in zip(trajectory.joint_names, point.positions):
                lower, upper = limits[name]
                if not math.isfinite(position) or not lower <= position <= upper:
                    raise RuntimeError(
                        f"Trajectory for {label} violates limits for {name}: {position}"
                    )
        print(
            f"PASS: MoveIt planned {label} with {len(trajectory.points)} points "
            f"for the six UR5e joints"
        )
        return response.trajectory

    def execute(self, trajectory, target, label):
        goal = ExecuteTrajectory.Goal()
        goal.trajectory = trajectory
        future = self.execute_client.send_goal_async(goal)
        self.wait_for(future.done, 15, f"MoveIt rejected or timed out accepting {label}")
        handle = future.result()
        if not handle.accepted:
            raise RuntimeError(f"MoveIt execution goal was rejected for {label}")

        result_future = handle.get_result_async()
        if not self._wait_for_result(result_future, 35):
            handle.cancel_goal_async()
            raise RuntimeError(f"MoveIt execution timed out for {label}; cancellation requested")
        result = result_future.result()
        if (
            result.status != GoalStatus.STATUS_SUCCEEDED
            or result.result.error_code.val != MoveItErrorCodes.SUCCESS
        ):
            raise RuntimeError(
                f"MoveIt execution failed for {label}: status={result.status}, "
                f"error={result.result.error_code.val}"
            )

        self.wait_for(
            lambda: all(
                abs(self.current_positions()[name] - target[name]) <= 0.05
                for name in EXPECTED_JOINTS
            ),
            8,
            f"Measured joint state did not reach target after {label}",
        )
        print(f"PASS: simulated motion completed for {label}")

    def _wait_for_result(self, future, timeout):
        deadline = time.monotonic() + timeout
        while not future.done() and time.monotonic() < deadline:
            remaining = deadline - time.monotonic()
            self.executor.spin_once(timeout_sec=min(0.1, max(0.0, remaining)))
        return future.done()

    def run(self):
        limits = self.validate_robot_description()
        initial = self.validate_state()
        for name, position in initial.items():
            lower, upper = limits[name]
            if not lower <= position <= upper:
                raise RuntimeError(f"Current position for {name} violates URDF limits")
        self.validate_controllers()
        self.wait_for(
            lambda: self.plan_client.service_is_ready(),
            15,
            "MoveIt planning service /plan_kinematic_path is unavailable",
        )
        if not self.execute_client.wait_for_server(timeout_sec=15):
            raise RuntimeError("MoveIt action /execute_trajectory is unavailable")

        outbound = initial.copy()
        outbound["shoulder_pan_joint"] += 0.12
        trajectory = self.plan(outbound, limits, "safe outbound target")
        self.execute(trajectory, outbound, "safe outbound target")

        trajectory = self.plan(initial, limits, "return to initial state")
        self.execute(trajectory, initial, "return to initial state")


def main():
    rclpy.init()
    validator = UR5eValidator()
    try:
        validator.run()
    except (RuntimeError, ValueError) as error:
        print(f"FAIL: {error}", file=sys.stderr)
        return 1
    finally:
        validator.node.destroy_node()
        rclpy.shutdown()
    print("PASS: UR5e controller, state, MoveIt planning, execution, and return validated")
    return 0


if __name__ == "__main__":
    sys.exit(main())
