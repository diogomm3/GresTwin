#!/usr/bin/env python3

import math
import sys
import time

import rclpy
from action_msgs.msg import GoalStatus
from geometry_msgs.msg import Pose, PoseStamped
from moveit_msgs.action import ExecuteTrajectory
from moveit_msgs.msg import (
    AttachedCollisionObject,
    CollisionObject,
    Constraints,
    JointConstraint,
    MoveItErrorCodes,
    PlanningSceneComponents,
    PlanningScene,
)
from moveit_msgs.srv import (
    ApplyPlanningScene,
    GetMotionPlan,
    GetPlanningScene,
    GetPositionIK,
)
from rclpy.action import ActionClient
from rclpy.duration import Duration
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node
from sensor_msgs.msg import JointState
from shape_msgs.msg import SolidPrimitive
from std_msgs.msg import Empty, String


JOINTS = (
    "shoulder_pan_joint",
    "shoulder_lift_joint",
    "elbow_joint",
    "wrist_1_joint",
    "wrist_2_joint",
    "wrist_3_joint",
)
GROUP = "ur_manipulator"
END_EFFECTOR_LINK = "tool0"
SOURCE = (0.5, -0.25)
DESTINATION = (0.5, 0.25)
TABLE_TOP = 0.26
OBJECT_CENTER_Z = TABLE_TOP + 0.05
TOOL_CONTACT_Z = TABLE_TOP + 0.16
APPROACH_Z = TABLE_TOP + 0.27
LIFT_Z = TABLE_TOP + 0.30
OBJECT_ID = "workpiece"
STATUS_TOPIC = "/manipulation_demo/state"
ATTACH_OFFSET_Z = TOOL_CONTACT_Z - OBJECT_CENTER_Z


class UR5ePickPlaceDemo(Node):
    def __init__(self):
        super().__init__("ur5e_pick_place_demo")
        self._executor = SingleThreadedExecutor()
        self._executor.add_node(self)
        self._joint_positions = {}
        self._joint_velocities = {}
        self._workpiece_position = None
        self._attachment_state = None

        self._status_publisher = self.create_publisher(String, STATUS_TOPIC, 10)
        self._attach_publisher = self.create_publisher(
            Empty, "/workpiece/attach", 10
        )
        self._detach_publisher = self.create_publisher(
            Empty, "/workpiece/detach", 10
        )
        self.create_subscription(JointState, "/joint_states", self._on_joint_state, 10)
        self.create_subscription(
            Pose, "/model/workpiece/pose", self._on_workpiece_pose, 10
        )
        self.create_subscription(
            String, "/workpiece/state", self._on_attachment_state, 10
        )

        self._ik_client = self.create_client(GetPositionIK, "/compute_ik")
        self._plan_client = self.create_client(
            GetMotionPlan, "/plan_kinematic_path"
        )
        self._scene_client = self.create_client(
            ApplyPlanningScene, "/apply_planning_scene"
        )
        self._get_scene_client = self.create_client(
            GetPlanningScene, "/get_planning_scene"
        )
        self._execute_client = ActionClient(
            self, ExecuteTrajectory, "/execute_trajectory"
        )

    def _on_joint_state(self, message):
        self._joint_positions.update(zip(message.name, message.position))
        if len(message.velocity) == len(message.name):
            self._joint_velocities.update(zip(message.name, message.velocity))

    def _on_workpiece_pose(self, message):
        self._workpiece_position = (
            message.position.x,
            message.position.y,
            message.position.z,
        )

    def _on_attachment_state(self, message):
        self._attachment_state = message.data.strip().lower()
        self.get_logger().info(f"WORKPIECE_STATE {self._attachment_state}")

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
            self._executor.spin_once(
                timeout_sec=min(0.1, max(0.0, deadline - time.monotonic()))
            )
        raise TimeoutError(f"Timed out waiting for {description}")

    def _wait_for_services(self):
        services = (
            (self._ik_client, "/compute_ik"),
            (self._plan_client, "/plan_kinematic_path"),
            (self._scene_client, "/apply_planning_scene"),
            (self._get_scene_client, "/get_planning_scene"),
        )
        deadline = time.monotonic() + 90.0
        pending = [name for _, name in services]
        while time.monotonic() < deadline:
            pending = [
                name
                for client, name in services
                if not client.service_is_ready()
            ]
            if not pending:
                break
            self._executor.spin_once(
                timeout_sec=min(0.1, max(0.0, deadline - time.monotonic()))
            )
        if pending:
            raise TimeoutError(
                f"Timed out waiting for MoveIt service(s): {', '.join(pending)}"
            )
        self._wait_until(
            lambda: self._execute_client.wait_for_server(timeout_sec=0.1),
            90.0,
            "MoveIt /execute_trajectory action",
        )
        self.get_logger().info("MoveIt planning and execution services are ready")

    def _current_positions(self):
        missing = set(JOINTS) - self._joint_positions.keys()
        if missing:
            raise RuntimeError(f"Missing UR5e joint states: {sorted(missing)}")
        positions = {name: self._joint_positions[name] for name in JOINTS}
        if not all(math.isfinite(value) for value in positions.values()):
            raise RuntimeError(f"UR5e joint state contains non-finite values: {positions}")
        return positions

    @staticmethod
    def _pose(x, y, z, downward=False):
        pose = Pose()
        pose.position.x = x
        pose.position.y = y
        pose.position.z = z
        if downward:
            pose.orientation.x = 1.0
        else:
            pose.orientation.w = 1.0
        return pose

    @classmethod
    def _box_collision(cls, name, x, y, z, size):
        collision = CollisionObject()
        collision.header.frame_id = "base_link"
        collision.id = name
        collision.pose = cls._pose(x, y, z)
        primitive = SolidPrimitive()
        primitive.type = SolidPrimitive.BOX
        primitive.dimensions = list(size)
        collision.primitives = [primitive]
        collision.primitive_poses = [cls._pose(0.0, 0.0, 0.0)]
        collision.operation = CollisionObject.ADD
        return collision

    @classmethod
    def _workpiece_collision(cls, x, y, z, attached=False):
        collision = CollisionObject()
        collision.header.frame_id = END_EFFECTOR_LINK if attached else "base_link"
        collision.id = OBJECT_ID
        collision.pose = (
            cls._pose(0.0, 0.0, ATTACH_OFFSET_Z, downward=True)
            if attached
            else cls._pose(x, y, z)
        )
        primitive = SolidPrimitive()
        primitive.type = SolidPrimitive.CYLINDER
        primitive.dimensions = [0.1, 0.035]
        collision.primitives = [primitive]
        collision.primitive_poses = [cls._pose(0.0, 0.0, 0.0)]
        collision.operation = CollisionObject.ADD
        return collision

    def _apply_scene(self, scene):
        scene.is_diff = True
        future = self._scene_client.call_async(
            ApplyPlanningScene.Request(scene=scene)
        )
        self._wait_until(future.done, 10.0, "planning-scene update")
        response = future.result()
        if response is None or not response.success:
            raise RuntimeError("MoveIt rejected the planning-scene update")

    def _verify_scene(self, world_ids, attached_ids):
        request = GetPlanningScene.Request()
        request.components.components = (
            PlanningSceneComponents.WORLD_OBJECT_GEOMETRY
            | PlanningSceneComponents.ROBOT_STATE_ATTACHED_OBJECTS
        )
        future = self._get_scene_client.call_async(request)
        self._wait_until(future.done, 10.0, "planning-scene verification")
        response = future.result()
        if response is None:
            raise RuntimeError("MoveIt returned no planning-scene response")
        actual_world = {
            collision.id for collision in response.scene.world.collision_objects
        }
        actual_attached = {
            attached.object.id
            for attached in response.scene.robot_state.attached_collision_objects
        }
        if not set(world_ids).issubset(actual_world):
            raise RuntimeError(
                f"Planning scene world objects are {sorted(actual_world)}, "
                f"expected at least {sorted(world_ids)}"
            )
        if not set(attached_ids).issubset(actual_attached):
            raise RuntimeError(
                f"Planning scene attached objects are {sorted(actual_attached)}, "
                f"expected at least {sorted(attached_ids)}"
            )

    def _add_initial_scene(self):
        scene = PlanningScene()
        scene.world.collision_objects = [
            self._box_collision(
                "pickup_table",
                SOURCE[0],
                SOURCE[1],
                TABLE_TOP / 2,
                (0.4, 0.2, TABLE_TOP),
            ),
            self._box_collision(
                "place_table",
                DESTINATION[0],
                DESTINATION[1],
                TABLE_TOP / 2,
                (0.4, 0.2, TABLE_TOP),
            ),
            self._workpiece_collision(
                SOURCE[0], SOURCE[1], OBJECT_CENTER_Z
            ),
        ]
        self._apply_scene(scene)
        self._verify_scene(
            ("pickup_table", "place_table", OBJECT_ID),
            (),
        )

    def _attach_collision_object(self):
        scene = PlanningScene()
        attached = AttachedCollisionObject()
        attached.link_name = END_EFFECTOR_LINK
        attached.object = self._workpiece_collision(
            0.0, 0.0, 0.0, attached=True
        )
        attached.touch_links = [END_EFFECTOR_LINK]
        scene.robot_state.attached_collision_objects = [attached]
        scene.robot_state.is_diff = True
        self._apply_scene(scene)
        self._verify_scene(("pickup_table", "place_table"), (OBJECT_ID,))

    def _detach_collision_object(self):
        scene = PlanningScene()
        detached = AttachedCollisionObject()
        detached.link_name = END_EFFECTOR_LINK
        detached.object.id = OBJECT_ID
        detached.object.operation = CollisionObject.REMOVE
        scene.robot_state.attached_collision_objects = [detached]
        scene.robot_state.is_diff = True
        scene.world.collision_objects = [
            self._workpiece_collision(
                DESTINATION[0], DESTINATION[1], OBJECT_CENTER_Z
            )
        ]
        self._apply_scene(scene)
        self._verify_scene(
            ("pickup_table", "place_table", OBJECT_ID),
            (),
        )

    def _wait_for_workpiece(self, predicate, timeout, description):
        try:
            self._wait_until(
                lambda: (
                    self._workpiece_position is not None
                    and predicate(self._workpiece_position)
                ),
                timeout,
                description,
            )
        except TimeoutError:
            self.get_logger().error(
                f"Last workpiece position while waiting for {description}: "
                f"{self._workpiece_position}"
            )
            raise
        position = self._workpiece_position
        self.get_logger().info(
            f"WORKPIECE_POSITION {position[0]:.3f} {position[1]:.3f} {position[2]:.3f}"
        )
        return position

    def _wait_for_attachment(self, expected, timeout=8.0):
        self._wait_until(
            lambda: self._attachment_state == expected,
            timeout,
            f"workpiece {expected} state",
        )

    def _solve_ik(self, x, y, z, label):
        request = GetPositionIK.Request()
        ik_request = request.ik_request
        ik_request.group_name = GROUP
        ik_request.ik_link_name = END_EFFECTOR_LINK
        ik_request.pose_stamped = PoseStamped()
        ik_request.pose_stamped.header.frame_id = "base_link"
        ik_request.pose_stamped.pose = self._pose(x, y, z, downward=True)
        ik_request.robot_state.joint_state.name = list(JOINTS)
        current = self._current_positions()
        ik_request.robot_state.joint_state.position = [
            current[name] for name in JOINTS
        ]
        ik_request.robot_state.is_diff = True
        ik_request.avoid_collisions = True
        ik_request.timeout = Duration(seconds=2).to_msg()

        future = self._ik_client.call_async(request)
        self._wait_until(future.done, 8.0, f"IK solution for {label}")
        response = future.result()
        if (
            response is None
            or response.error_code.val != MoveItErrorCodes.SUCCESS
        ):
            error = "no response" if response is None else response.error_code.val
            raise RuntimeError(f"MoveIt could not solve {label} pose: {error}")
        solution = dict(
            zip(
                response.solution.joint_state.name,
                response.solution.joint_state.position,
            )
        )
        missing = set(JOINTS) - solution.keys()
        if missing:
            raise RuntimeError(
                f"MoveIt IK solution for {label} omits joints: {sorted(missing)}"
            )
        target = {
            name: solution[name]
            + math.tau * round((current[name] - solution[name]) / math.tau)
            for name in JOINTS
        }
        self.get_logger().info(f"IK {label}: {target}")
        return target

    def _plan(self, target, label):
        request = GetMotionPlan.Request()
        motion_request = request.motion_plan_request
        motion_request.group_name = GROUP
        motion_request.pipeline_id = "ompl"
        motion_request.planner_id = "RRTConnectkConfigDefault"
        motion_request.num_planning_attempts = 3
        motion_request.allowed_planning_time = 15.0
        motion_request.max_velocity_scaling_factor = 0.15
        motion_request.max_acceleration_scaling_factor = 0.15
        current = self._current_positions()
        motion_request.start_state.joint_state.name = list(JOINTS)
        motion_request.start_state.joint_state.position = [
            current[name] for name in JOINTS
        ]
        motion_request.start_state.is_diff = True

        constraints = Constraints()
        constraints.name = label
        for name in JOINTS:
            constraints.joint_constraints.append(
                JointConstraint(
                    joint_name=name,
                    position=target[name],
                    tolerance_above=0.025,
                    tolerance_below=0.025,
                    weight=1.0,
                )
            )
        motion_request.goal_constraints = [constraints]

        future = self._plan_client.call_async(request)
        self._wait_until(future.done, 35.0, f"MoveIt plan for {label}")
        response = future.result().motion_plan_response
        if response.error_code.val != MoveItErrorCodes.SUCCESS:
            raise RuntimeError(
                f"MoveIt planning failed for {label}: "
                f"{response.error_code.val} {response.error_code.message}"
            )
        if not response.trajectory.joint_trajectory.points:
            raise RuntimeError(f"MoveIt returned an empty trajectory for {label}")
        self.get_logger().info(
            f"PLANNED {label}: "
            f"{len(response.trajectory.joint_trajectory.points)} trajectory points"
        )
        return response.trajectory, target

    def _execute(self, trajectory, target, label):
        goal = ExecuteTrajectory.Goal()
        goal.trajectory = trajectory
        future = self._execute_client.send_goal_async(goal)
        self._wait_until(future.done, 15.0, f"execution acceptance for {label}")
        goal_handle = future.result()
        if goal_handle is None or not goal_handle.accepted:
            raise RuntimeError(f"MoveIt execution was rejected for {label}")
        result_future = goal_handle.get_result_async()
        self._wait_until(result_future.done, 60.0, f"execution completion for {label}")
        result = result_future.result()
        if (
            result is None
            or result.status != GoalStatus.STATUS_SUCCEEDED
            or result.result.error_code.val != MoveItErrorCodes.SUCCESS
        ):
            code = None if result is None else result.result.error_code.val
            raise RuntimeError(f"MoveIt execution failed for {label}: {code}")

        stable_samples = 0

        def reached_and_stopped():
            nonlocal stable_samples
            positions = self._current_positions()
            velocities = self._joint_velocities
            is_stable = all(
                abs(positions[name] - value) <= 0.05
                and abs(velocities.get(name, math.inf)) <= 0.05
                for name, value in target.items()
            )
            stable_samples = stable_samples + 1 if is_stable else 0
            return stable_samples >= 5

        self._wait_until(
            reached_and_stopped,
            10.0,
            f"settled joint feedback at {label}",
        )
        self.get_logger().info(f"EXECUTED {label}")

    def _move_pose(self, label, x, y, z):
        target = self._solve_ik(x, y, z, label)
        trajectory, target = self._plan(target, label)
        self._execute(trajectory, target, label)

    def _move_joints(self, label, target):
        trajectory, target = self._plan(target, label)
        self._execute(trajectory, target, label)

    def run_demo(self):
        self._publish_state("WAITING_FOR_SIMULATION")
        self._wait_for_services()
        self._wait_until(
            lambda: (
                set(JOINTS).issubset(self._joint_positions)
                and self._workpiece_position is not None
            ),
            45.0,
            "UR5e joint and workpiece state",
        )
        initial_positions = self._current_positions()
        self._wait_for_workpiece(
            lambda position: (
                abs(position[0] - SOURCE[0]) < 0.03
                and abs(position[1] - SOURCE[1]) < 0.03
                and abs(position[2] - OBJECT_CENTER_Z) < 0.03
            ),
            10.0,
            "workpiece resting at its pickup location",
        )
        self._publish_state("INITIAL_RELEASE")
        self._detach_publisher.publish(Empty())
        self._wait_for_attachment("detached")
        self._wait_for_workpiece(
            lambda position: (
                abs(position[0] - SOURCE[0]) < 0.03
                and abs(position[1] - SOURCE[1]) < 0.03
                and abs(position[2] - OBJECT_CENTER_Z) < 0.03
            ),
            10.0,
            "workpiece settled at its pickup location",
        )
        self._add_initial_scene()

        self._publish_state("PREGRASP")
        self._move_pose("pickup approach", SOURCE[0], SOURCE[1], APPROACH_Z)
        self._publish_state("APPROACH_PICK")
        self._move_pose("pickup contact", SOURCE[0], SOURCE[1], TOOL_CONTACT_Z)

        self._publish_state("GRASP")
        self._attach_publisher.publish(Empty())
        self._wait_for_attachment("attached")
        self._attach_collision_object()

        self._publish_state("LIFT")
        self._move_pose("pickup lift", SOURCE[0], SOURCE[1], LIFT_Z)
        self._wait_for_workpiece(
            lambda position: position[2] >= OBJECT_CENTER_Z + 0.12,
            8.0,
            "workpiece lifting clear of the table",
        )

        self._publish_state("TRANSFER")
        self._move_pose(
            "destination approach", DESTINATION[0], DESTINATION[1], LIFT_Z
        )
        self._wait_for_workpiece(
            lambda position: (
                abs(position[0] - DESTINATION[0]) < 0.05
                and abs(position[1] - DESTINATION[1]) < 0.05
                and position[2] >= OBJECT_CENTER_Z + 0.12
            ),
            8.0,
            "workpiece arriving above the destination",
        )

        self._publish_state("PLACE")
        self._move_pose(
            "destination contact",
            DESTINATION[0],
            DESTINATION[1],
            TOOL_CONTACT_Z,
        )
        self._publish_state("RELEASE")
        self._detach_publisher.publish(Empty())
        self._wait_for_attachment("detached")
        self._wait_for_workpiece(
            lambda position: (
                abs(position[0] - DESTINATION[0]) < 0.03
                and abs(position[1] - DESTINATION[1]) < 0.03
                and abs(position[2] - OBJECT_CENTER_Z) < 0.03
            ),
            10.0,
            "released workpiece resting on the destination table",
        )
        self._detach_collision_object()

        self._publish_state("RETREAT")
        self._move_pose(
            "destination retreat",
            DESTINATION[0],
            DESTINATION[1],
            APPROACH_Z,
        )
        self._publish_state("SAFE_RETURN")
        self._move_joints("return to initial state", initial_positions)
        self._publish_state("SUCCEEDED")


def main():
    rclpy.init()
    demo = UR5ePickPlaceDemo()
    success = False
    try:
        demo.run_demo()
        success = True
    except KeyboardInterrupt:
        demo.get_logger().warning("UR5e pick-and-place test was interrupted")
    except (RuntimeError, TimeoutError, ValueError) as error:
        demo.get_logger().error(f"UR5e pick-and-place test failed: {error}")
        demo._publish_state("FAILED")
    finally:
        demo._executor.remove_node(demo)
        demo.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    return 0 if success else 1


if __name__ == "__main__":
    sys.exit(main())
