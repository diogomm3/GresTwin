#!/usr/bin/env python3

import copy
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
    PlanningScene,
    PlanningSceneComponents,
)
from moveit_msgs.srv import (
    ApplyPlanningScene,
    GetMotionPlan,
    GetPlanningScene,
    GetPositionFK,
    GetPositionIK,
)
from rclpy.action import ActionClient
from rclpy.duration import Duration
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node
from rclpy.time import Time
from ros_gz_interfaces.msg import Entity
from ros_gz_interfaces.srv import DeleteEntity
from sensor_msgs.msg import JointState
from stage1_planning_scene_utils import set_allowed_collision_pairs
from std_msgs.msg import Bool
from tf2_ros import Buffer, TransformException, TransformListener

JOINTS = (
    "shoulder_pan_joint",
    "shoulder_lift_joint",
    "elbow_joint",
    "wrist_1_joint",
    "wrist_2_joint",
    "wrist_3_joint",
)
JOINT_LIMIT_MARGIN = 0.15
JOINT_POSITION_LIMITS = {
    name: (
        -math.pi + JOINT_LIMIT_MARGIN,
        math.pi - JOINT_LIMIT_MARGIN,
    )
    if name == "elbow_joint"
    else (
        -math.tau + JOINT_LIMIT_MARGIN,
        math.tau - JOINT_LIMIT_MARGIN,
    )
    for name in JOINTS
}
GROUP = "ur_manipulator"
WORLD_FRAME = "world"
END_EFFECTOR_LINK = "tool0"
SUCTION_LINK = "stage1_suction_manifold"
PAD_END_OFFSET = 0.07
CONTACT_OVERLAP = 0.006
PICKUP_APPROACH_CLEARANCE = 0.12
LIFT_CLEARANCE = 0.15
PLACE_APPROACH_CLEARANCE = 0.15
PLACE_RELEASE_CLEARANCE = 0.02
PLANNING_SCENE_COMPONENTS = (
    PlanningSceneComponents.WORLD_OBJECT_GEOMETRY
    | PlanningSceneComponents.ROBOT_STATE_ATTACHED_OBJECTS
    | PlanningSceneComponents.ALLOWED_COLLISION_MATRIX
)


class Stage1PickPlaceDemo(Node):
    def __init__(self):
        super().__init__("stage1_pick_place_demo")
        self.declare_parameter("world_name", "stage1_raw_stoneware")
        self.declare_parameter("pallet_pose", [1.18, 0.5, 0.0])
        self.declare_parameter("pallet_height", 0.3)
        self.declare_parameter("level_counts_bottom_up", [5, 4, 3, 2, 1])
        self.declare_parameter("vertical_pitch", 0.086602540378)
        self.declare_parameter("table_name", "stage1_placement_table")
        self.declare_parameter("table_center", [-0.15, 0.5, 0.6])
        self.declare_parameter("cylinder_diameter", 0.1)
        self.declare_parameter("removal_delay", 5.0)
        self.declare_parameter("max_cylinders", 15)

        self._world_name = self.get_parameter("world_name").value
        self._pallet_pose = tuple(self.get_parameter("pallet_pose").value)
        self._pallet_height = float(self.get_parameter("pallet_height").value)
        self._level_counts_bottom_up = tuple(
            self.get_parameter("level_counts_bottom_up").value
        )
        self._vertical_pitch = float(
            self.get_parameter("vertical_pitch").value
        )
        self._table_name = self.get_parameter("table_name").value
        self._table_center = tuple(self.get_parameter("table_center").value)
        self._cylinder_diameter = float(
            self.get_parameter("cylinder_diameter").value
        )
        self._removal_delay = float(self.get_parameter("removal_delay").value)
        self._max_cylinders = self.get_parameter("max_cylinders").value
        if (
            len(self._pallet_pose) != 3
            or len(self._table_center) != 3
            or not all(
                math.isfinite(value)
                for value in (
                    *self._pallet_pose,
                    *self._table_center,
                    self._pallet_height,
                    self._vertical_pitch,
                    self._cylinder_diameter,
                    self._removal_delay,
                )
            )
            or not self._level_counts_bottom_up
            or any(
                not isinstance(count, int) or count <= 0
                for count in self._level_counts_bottom_up
            )
            or any(
                upper_count != lower_count - 1
                for lower_count, upper_count in zip(
                    self._level_counts_bottom_up,
                    self._level_counts_bottom_up[1:],
                )
            )
            or self._pallet_height <= 0.0
            or self._vertical_pitch <= 0.0
            or not math.isfinite(self._cylinder_diameter)
            or self._cylinder_diameter <= 0.0
            or self._removal_delay < 0.0
            or not isinstance(self._max_cylinders, int)
            or self._max_cylinders <= 0
            or self._max_cylinders > sum(self._level_counts_bottom_up)
        ):
            raise ValueError("Stage 1 pick-and-place target configuration is invalid")

        self._joint_positions = {}
        self._joint_velocities = {}
        self._suction_enabled = None
        self._suction_attached = None
        self._source_name = None
        self._source_touching_names = ()
        self._source_center = ()
        self._source_collision_object = None
        self._pending_removals = set()
        self._removal_errors = {}

        self._executor = SingleThreadedExecutor()
        self._executor.add_node(self)
        self._tf_buffer = Buffer()
        self._tf_listener = TransformListener(self._tf_buffer, self)

        self.create_subscription(
            JointState, "/joint_states", self._on_joint_state, 10
        )
        self.create_subscription(
            Bool, "/stage1/suction/enabled", self._on_suction_enabled, 10
        )
        self.create_subscription(
            Bool, "/stage1/suction/attached", self._on_suction_attached, 10
        )
        self._suction_command_publisher = self.create_publisher(
            Bool, "/stage1/suction/command", 10
        )
        self._delete_entity_client = self.create_client(
            DeleteEntity, f"/world/{self._world_name}/remove"
        )

        self._ik_client = self.create_client(GetPositionIK, "/compute_ik")
        self._fk_client = self.create_client(GetPositionFK, "/compute_fk")
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

    def _on_suction_enabled(self, message):
        self._suction_enabled = message.data

    def _on_suction_attached(self, message):
        self._suction_attached = message.data

    def _wait_until(self, predicate, timeout, description):
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            self._raise_removal_errors()
            if predicate():
                return
            self._executor.spin_once(
                timeout_sec=min(0.1, max(0.0, deadline - time.monotonic()))
            )
        self._raise_removal_errors()
        raise TimeoutError(f"Timed out waiting for {description}")

    def _raise_removal_errors(self):
        if self._removal_errors:
            cylinder_name, error = next(iter(self._removal_errors.items()))
            raise RuntimeError(
                f"Could not remove placed cylinder {cylinder_name}: {error}"
            )

    def _wait_for_services(self):
        services = (
            (self._ik_client, "/compute_ik"),
            (self._fk_client, "/compute_fk"),
            (self._plan_client, "/plan_kinematic_path"),
            (self._scene_client, "/apply_planning_scene"),
            (self._get_scene_client, "/get_planning_scene"),
            (
                self._delete_entity_client,
                f"/world/{self._world_name}/remove",
            ),
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

    def _current_positions(self):
        missing = set(JOINTS) - self._joint_positions.keys()
        if missing:
            raise RuntimeError(f"Missing UR5e joint states: {sorted(missing)}")
        positions = {name: self._joint_positions[name] for name in JOINTS}
        if not all(math.isfinite(value) for value in positions.values()):
            raise RuntimeError(
                f"UR5e joint state contains non-finite values: {positions}"
            )
        return positions

    def _get_scene(self):
        request = GetPlanningScene.Request()
        request.components.components = PLANNING_SCENE_COMPONENTS
        future = self._get_scene_client.call_async(request)
        self._wait_until(future.done, 10.0, "MoveIt planning scene")
        response = future.result()
        if response is None:
            raise RuntimeError("MoveIt returned no planning-scene response")
        return response.scene

    def _verify_scene(
        self,
        world_present=(),
        world_absent=(),
        attached_present=(),
        attached_absent=(),
        allowed_collisions=(),
    ):
        scene = self._get_scene()
        world_ids = {item.id for item in scene.world.collision_objects}
        attached_ids = {
            item.object.id
            for item in scene.robot_state.attached_collision_objects
        }
        missing_world = set(world_present) - world_ids
        unexpected_world = set(world_absent) & world_ids
        missing_attached = set(attached_present) - attached_ids
        unexpected_attached = set(attached_absent) & attached_ids
        if (
            missing_world
            or unexpected_world
            or missing_attached
            or unexpected_attached
        ):
            raise RuntimeError(
                "MoveIt scene mismatch: "
                f"world missing={sorted(missing_world)}, "
                f"world unexpected={sorted(unexpected_world)}, "
                f"attached missing={sorted(missing_attached)}, "
                f"attached unexpected={sorted(unexpected_attached)}"
            )

        matrix = scene.allowed_collision_matrix
        names = list(matrix.entry_names)
        rows = [list(entry.enabled) for entry in matrix.entry_values]
        if len(rows) != len(names) or any(
            len(row) != len(names) for row in rows
        ):
            raise RuntimeError(
                "MoveIt returned an inconsistent allowed-collision matrix"
            )
        for first, second, expected in allowed_collisions:
            if first not in names or second not in names:
                if not expected:
                    continue
                raise RuntimeError(
                    f"MoveIt collision matrix omits {first!r} or {second!r}"
                )
            actual = rows[names.index(first)][names.index(second)]
            if actual != expected:
                raise RuntimeError(
                    f"MoveIt collision allowance for {first} / {second} "
                    f"is {actual}, expected {expected}"
                )
        return scene

    def _apply_scene(self, scene, label, **expected):
        scene.is_diff = True
        future = self._scene_client.call_async(
            ApplyPlanningScene.Request(scene=scene)
        )
        self._wait_until(future.done, 10.0, f"{label} scene update")
        response = future.result()
        if response is None:
            raise RuntimeError(f"MoveIt returned no response for {label}")
        if not response.success:
            self.get_logger().warning(
                f"MoveIt returned success=false for {label}; verifying the "
                "resulting planning scene before continuing"
            )
        return self._verify_scene(**expected)

    def _update_allowed_collisions(self, pairs):
        scene = self._get_scene()
        matrix = copy.deepcopy(scene.allowed_collision_matrix)
        set_allowed_collision_pairs(matrix, pairs)
        update = PlanningScene()
        update.allowed_collision_matrix = matrix
        expected = tuple(
            (first, second, allowed)
            for first, second, allowed in pairs
        )
        self._apply_scene(
            update,
            "allowed-collision matrix",
            allowed_collisions=expected,
        )

    @staticmethod
    def _pose(x, y, z):
        pose = Pose()
        pose.position.x = x
        pose.position.y = y
        pose.position.z = z
        pose.orientation.x = 1.0
        pose.orientation.y = 0.0
        pose.orientation.z = 0.0
        pose.orientation.w = 0.0
        return pose

    @staticmethod
    def _nearest_valid_ik_target(solution, seed, label):
        target = {}
        for name in JOINTS:
            lower_limit, upper_limit = JOINT_POSITION_LIMITS[name]
            first_turn = math.ceil(
                (lower_limit - solution[name]) / math.tau
            )
            last_turn = math.floor(
                (upper_limit - solution[name]) / math.tau
            )
            candidates = [
                solution[name] + math.tau * turns
                for turns in range(first_turn, last_turn + 1)
            ]
            if not candidates:
                raise RuntimeError(
                    f"MoveIt returned an out-of-range IK solution for "
                    f"{label} on {name}: {solution[name]}"
                )
            target[name] = min(
                candidates,
                key=lambda angle: abs(seed[name] - angle),
            )
        return target

    def _solve_ik(self, x, y, z, label, seed, avoid_collisions=True):
        request = GetPositionIK.Request()
        ik_request = request.ik_request
        ik_request.group_name = GROUP
        ik_request.ik_link_name = END_EFFECTOR_LINK
        ik_request.pose_stamped = PoseStamped()
        ik_request.pose_stamped.header.frame_id = WORLD_FRAME
        ik_request.pose_stamped.pose = self._pose(x, y, z)
        ik_request.robot_state.joint_state.name = list(JOINTS)
        ik_request.robot_state.joint_state.position = [
            seed[name] for name in JOINTS
        ]
        ik_request.robot_state.is_diff = True
        ik_request.avoid_collisions = avoid_collisions
        ik_request.timeout = Duration(seconds=5).to_msg()

        future = self._ik_client.call_async(request)
        self._wait_until(future.done, 15.0, f"IK solution for {label}")
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
        fk_request = GetPositionFK.Request()
        fk_request.header.frame_id = WORLD_FRAME
        fk_request.fk_link_names = [END_EFFECTOR_LINK]
        fk_request.robot_state.joint_state.name = list(JOINTS)
        fk_request.robot_state.joint_state.position = [
            solution[name] for name in JOINTS
        ]
        fk_request.robot_state.is_diff = True
        fk_future = self._fk_client.call_async(fk_request)
        self._wait_until(fk_future.done, 10.0, f"FK verification for {label}")
        fk_response = fk_future.result()
        if (
            fk_response is None
            or fk_response.error_code.val != MoveItErrorCodes.SUCCESS
            or len(fk_response.pose_stamped) != 1
        ):
            error = (
                "no response"
                if fk_response is None
                else fk_response.error_code.val
            )
            raise RuntimeError(
                f"MoveIt could not verify the {label} IK solution: {error}"
            )
        actual_pose = fk_response.pose_stamped[0].pose
        actual_position = (
            actual_pose.position.x,
            actual_pose.position.y,
            actual_pose.position.z,
        )
        position_error = math.dist(actual_position, (x, y, z))
        target_orientation = ik_request.pose_stamped.pose.orientation
        actual_orientation = actual_pose.orientation
        orientation_dot = abs(
            target_orientation.x * actual_orientation.x
            + target_orientation.y * actual_orientation.y
            + target_orientation.z * actual_orientation.z
            + target_orientation.w * actual_orientation.w
        )
        actual_norm = math.sqrt(
            actual_orientation.x**2
            + actual_orientation.y**2
            + actual_orientation.z**2
            + actual_orientation.w**2
        )
        if not math.isfinite(actual_norm) or actual_norm < 1e-12:
            raise RuntimeError(
                f"MoveIt returned an invalid FK orientation for {label}"
            )
        orientation_error = 2.0 * math.acos(
            min(1.0, orientation_dot / actual_norm)
        )
        if position_error > 0.005 or orientation_error > math.radians(1.0):
            raise RuntimeError(
                f"MoveIt IK/FK mismatch for {label}: "
                f"{position_error:.4f} m, "
                f"{math.degrees(orientation_error):.2f} degrees"
            )
        self.get_logger().info(
            f"Verified IK/FK {label}: {position_error:.4f} m, "
            f"{math.degrees(orientation_error):.2f} degrees"
        )
        target = self._nearest_valid_ik_target(solution, seed, label)
        self.get_logger().info(f"IK {label}: {target}")
        return target

    def _plan(self, target, label):
        last_error = "no planning response"
        for attempt in range(1, 4):
            request = GetMotionPlan.Request()
            motion_request = request.motion_plan_request
            motion_request.group_name = GROUP
            motion_request.pipeline_id = "ompl"
            motion_request.planner_id = "RRTConnectkConfigDefault"
            motion_request.num_planning_attempts = 5
            motion_request.allowed_planning_time = 15.0
            motion_request.max_velocity_scaling_factor = 0.5
            motion_request.max_acceleration_scaling_factor = 0.5
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
                        tolerance_above=0.005,
                        tolerance_below=0.005,
                        weight=1.0,
                    )
                )
            motion_request.goal_constraints = [constraints]

            future = self._plan_client.call_async(request)
            self._wait_until(future.done, 35.0, f"MoveIt plan for {label}")
            response = future.result()
            if response is None:
                last_error = "MoveIt returned no planning response"
            else:
                result = response.motion_plan_response
                if (
                    result.error_code.val == MoveItErrorCodes.SUCCESS
                    and result.trajectory.joint_trajectory.points
                ):
                    final_time = (
                        result.trajectory.joint_trajectory.points[-1].time_from_start
                    )
                    duration = final_time.sec + final_time.nanosec * 1e-9
                    self.get_logger().info(
                        f"Planned {label}: "
                        f"{len(result.trajectory.joint_trajectory.points)} "
                        f"trajectory points, {duration:.1f} s"
                    )
                    return result.trajectory
                last_error = (
                    f"{result.error_code.val} {result.error_code.message}"
                )
            self.get_logger().warning(
                f"Planning attempt {attempt} for {label} failed: {last_error}"
            )
        raise RuntimeError(f"MoveIt could not plan {label}: {last_error}")

    def _execute(self, trajectory, target, label):
        goal = ExecuteTrajectory.Goal()
        goal.trajectory = trajectory
        future = self._execute_client.send_goal_async(goal)
        self._wait_until(future.done, 10.0, f"trajectory goal for {label}")
        goal_handle = future.result()
        if goal_handle is None or not goal_handle.accepted:
            raise RuntimeError(f"MoveIt rejected the trajectory for {label}")

        future = goal_handle.get_result_async()
        final_time = trajectory.joint_trajectory.points[-1].time_from_start
        duration = final_time.sec + final_time.nanosec * 1e-9
        execution_timeout = max(180.0, duration * 8.0 + 30.0)
        self._wait_until(
            future.done,
            execution_timeout,
            f"trajectory execution for {label}",
        )
        result = future.result()
        if result is None:
            raise RuntimeError(f"MoveIt returned no execution result for {label}")
        if (
            result.status != GoalStatus.STATUS_SUCCEEDED
            or result.result.error_code.val != MoveItErrorCodes.SUCCESS
        ):
            raise RuntimeError(
                f"Trajectory execution failed for {label}: "
                f"action status {result.status}, "
                f"MoveIt code {result.result.error_code.val}"
            )

        stable_samples = 0

        def reached_and_stopped():
            nonlocal stable_samples
            positions = self._current_positions()
            is_reached = all(
                abs(positions[name] - target[name]) < 0.01
                for name in JOINTS
            )
            is_stopped = all(
                name in self._joint_velocities
                and abs(self._joint_velocities[name]) < 0.03
                for name in JOINTS
            )
            stable_samples = stable_samples + 1 if is_reached and is_stopped else 0
            return stable_samples >= 5

        self._wait_until(
            reached_and_stopped,
            30.0,
            f"settled joint feedback at {label}",
        )
        self.get_logger().info(f"Reached {label}")

    def _move_joints(self, target, label):
        trajectory = self._plan(target, label)
        self._execute(trajectory, target, label)

    @staticmethod
    def _quaternion_multiply(first, second):
        ax, ay, az, aw = first
        bx, by, bz, bw = second
        return (
            aw * bx + ax * bw + ay * bz - az * by,
            aw * by - ax * bz + ay * bw + az * bx,
            aw * bz + ax * by - ay * bx + az * bw,
            aw * bw - ax * bx - ay * by - az * bz,
        )

    @staticmethod
    def _rotate_vector(rotation, vector):
        qx, qy, qz, qw = rotation
        vx, vy, vz = vector
        tx = 2.0 * (qy * vz - qz * vy)
        ty = 2.0 * (qz * vx - qx * vz)
        tz = 2.0 * (qx * vy - qy * vx)
        return (
            vx + qw * tx + qy * tz - qz * ty,
            vy + qw * ty + qz * tx - qx * tz,
            vz + qw * tz + qx * ty - qy * tx,
        )

    def _attached_object_pose(self):
        try:
            transform = self._tf_buffer.lookup_transform(
                WORLD_FRAME,
                SUCTION_LINK,
                Time(),
            )
        except TransformException as error:
            raise RuntimeError(
                f"Could not read the suction-manifold transform: {error}"
            ) from error

        translation = transform.transform.translation
        rotation = transform.transform.rotation
        inverse_rotation = (
            -rotation.x,
            -rotation.y,
            -rotation.z,
            rotation.w,
        )
        source_pose = self._source_collision_object.pose
        relative_position = self._rotate_vector(
            inverse_rotation,
            (
                source_pose.position.x - translation.x,
                source_pose.position.y - translation.y,
                source_pose.position.z - translation.z,
            ),
        )
        source_rotation = source_pose.orientation
        relative_rotation = self._quaternion_multiply(
            inverse_rotation,
            (
                source_rotation.x,
                source_rotation.y,
                source_rotation.z,
                source_rotation.w,
            ),
        )
        norm = math.sqrt(sum(value * value for value in relative_rotation))
        if not math.isfinite(norm) or norm < 1e-12:
            raise RuntimeError("The suction-relative object rotation is invalid")

        pose = Pose()
        pose.position.x, pose.position.y, pose.position.z = relative_position
        (
            pose.orientation.x,
            pose.orientation.y,
            pose.orientation.z,
            pose.orientation.w,
        ) = tuple(value / norm for value in relative_rotation)
        return pose

    def _attach_collision_object(self):
        self._wait_until(
            lambda: self._lookup_suction_transform() is not None,
            10.0,
            "suction-manifold transform",
        )
        attached = AttachedCollisionObject()
        attached.link_name = SUCTION_LINK
        attached.touch_links = [SUCTION_LINK, END_EFFECTOR_LINK]
        attached.object = copy.deepcopy(self._source_collision_object)
        attached.object.header.frame_id = SUCTION_LINK
        attached.object.pose = self._attached_object_pose()
        attached.object.operation = CollisionObject.ADD

        remove = CollisionObject()
        remove.id = self._source_name
        remove.operation = CollisionObject.REMOVE
        scene = PlanningScene()
        scene.world.collision_objects = [remove]
        scene.robot_state.is_diff = True
        scene.robot_state.attached_collision_objects = [attached]
        current_scene = self._get_scene()
        touching_pairs = [
            (self._source_name, touching_name, True)
            for touching_name in self._source_touching_names
        ]
        allowed_pairs = [
            (SUCTION_LINK, self._source_name, True),
            (self._source_name, self._table_name, False),
            *touching_pairs,
        ]
        scene.allowed_collision_matrix = set_allowed_collision_pairs(
            copy.deepcopy(current_scene.allowed_collision_matrix),
            allowed_pairs,
        )
        self._apply_scene(
            scene,
            "attaching the picked cylinder",
            world_absent=(self._source_name,),
            attached_present=(self._source_name,),
            allowed_collisions=tuple(allowed_pairs),
        )

    def _release_collision_object(self):
        placed = copy.deepcopy(self._source_collision_object)
        placed.header.frame_id = WORLD_FRAME
        placed.pose.position.x = self._table_center[0]
        placed.pose.position.y = self._table_center[1]
        placed.pose.position.z = self._table_center[2]
        placed.operation = CollisionObject.ADD

        attached = AttachedCollisionObject()
        attached.link_name = SUCTION_LINK
        attached.object.id = self._source_name
        attached.object.operation = CollisionObject.REMOVE
        scene = PlanningScene()
        scene.world.collision_objects = [placed]
        scene.robot_state.is_diff = True
        scene.robot_state.attached_collision_objects = [attached]
        self._apply_scene(
            scene,
            "placing the cylinder in the planning scene",
            world_present=(self._source_name,),
            attached_absent=(self._source_name,),
        )

    def _remove_collision_object(self, cylinder_name):
        removed = CollisionObject()
        removed.id = cylinder_name
        removed.operation = CollisionObject.REMOVE
        scene = PlanningScene()
        scene.world.collision_objects = [removed]
        self._apply_scene(
            scene,
            f"removing consumed cylinder {cylinder_name} from the planning scene",
            world_absent=(cylinder_name,),
            attached_absent=(cylinder_name,),
        )

    def _schedule_cylinder_removal(self, cylinder_name):
        self._pending_removals.add(cylinder_name)
        timer = None

        def remove_after_delay():
            timer.cancel()
            self.destroy_timer(timer)
            request = DeleteEntity.Request()
            request.entity.name = cylinder_name
            request.entity.type = Entity.MODEL
            future = self._delete_entity_client.call_async(request)
            future.add_done_callback(
                lambda result: self._on_gazebo_cylinder_removed(
                    cylinder_name, result
                )
            )

        timer = self.create_timer(
            max(self._removal_delay, 0.001),
            remove_after_delay,
        )
        self.get_logger().info(
            f"Scheduled removal of {cylinder_name} in "
            f"{self._removal_delay:g} simulated seconds"
        )

    def _on_gazebo_cylinder_removed(self, cylinder_name, future):
        try:
            response = future.result()
        except Exception as error:
            self._removal_errors[cylinder_name] = str(error)
            return
        if response is None or not response.success:
            self._removal_errors[cylinder_name] = (
                "Gazebo DeleteEntity returned success=false"
            )
            return

        removed = CollisionObject()
        removed.id = cylinder_name
        removed.operation = CollisionObject.REMOVE
        scene = PlanningScene()
        scene.is_diff = True
        scene.world.collision_objects = [removed]
        scene_future = self._scene_client.call_async(
            ApplyPlanningScene.Request(scene=scene)
        )
        scene_future.add_done_callback(
            lambda result: self._on_planning_scene_cylinder_removed(
                cylinder_name, result
            )
        )

    def _on_planning_scene_cylinder_removed(self, cylinder_name, future):
        try:
            response = future.result()
        except Exception as error:
            self._removal_errors[cylinder_name] = str(error)
            return
        if response is None or not response.success:
            self._removal_errors[cylinder_name] = (
                "MoveIt ApplyPlanningScene returned success=false"
            )
            return
        self._pending_removals.discard(cylinder_name)
        self.get_logger().info(
            f"Removed {cylinder_name} from Gazebo and MoveIt after "
            f"{self._removal_delay:g} simulated seconds"
        )

    def _lookup_suction_transform(self):
        try:
            return self._tf_buffer.lookup_transform(
                WORLD_FRAME,
                SUCTION_LINK,
                Time(),
            )
        except TransformException:
            return None

    def _wait_for_duration(self, duration, description):
        started = time.monotonic()
        self._wait_until(
            lambda: time.monotonic() - started >= duration,
            duration + 2.0,
            description,
        )

    def _command_suction(self, enabled):
        message = Bool()
        message.data = enabled
        self._suction_command_publisher.publish(message)
        self._wait_until(
            lambda: self._suction_enabled is enabled,
            5.0,
            f"suction {'enabled' if enabled else 'disabled'} state",
        )
        self._wait_until(
            lambda: self._suction_attached is enabled,
            45.0 if enabled else 12.0,
            f"Gazebo-confirmed cylinder "
            f"{'attachment' if enabled else 'release'}",
        )
        self._wait_for_duration(
            0.3,
            "Gazebo to process the suction attachment change",
        )

    def _cylinders_top_down(self):
        cylinders = []
        for level in range(len(self._level_counts_bottom_up), 0, -1):
            count = self._level_counts_bottom_up[level - 1]
            for index in range(1, count + 1):
                touching_names = []
                if level > 1:
                    touching_names.extend(
                        (
                            f"stoneware_l{level - 1}_{index}",
                            f"stoneware_l{level - 1}_{index + 1}",
                        )
                    )
                if index < count:
                    touching_names.append(f"stoneware_l{level}_{index + 1}")
                center = (
                    self._pallet_pose[0],
                    self._pallet_pose[1]
                    + (index - (count + 1) / 2.0) * self._cylinder_diameter,
                    self._pallet_pose[2]
                    + self._pallet_height
                    + self._cylinder_diameter / 2.0
                    + self._vertical_pitch * (level - 1),
                )
                cylinders.append(
                    (
                        f"stoneware_l{level}_{index}",
                        center,
                        tuple(touching_names),
                    )
                )
        return cylinders

    def _validate_source_object(self, collision_object):
        actual_center = (
            collision_object.pose.position.x,
            collision_object.pose.position.y,
            collision_object.pose.position.z,
        )
        if (
            collision_object.header.frame_id != WORLD_FRAME
            or any(
                abs(actual - expected) > 0.02
                for actual, expected in zip(
                    actual_center,
                    self._source_center,
                )
            )
        ):
            raise RuntimeError(
                f"MoveIt pose for {self._source_name} does not match "
                f"configured center {self._source_center}: {actual_center}"
            )

    def _pick_and_place_cylinder(self):
        contact_z = (
            self._source_center[2]
            + self._cylinder_diameter / 2.0
            + PAD_END_OFFSET
            - CONTACT_OVERLAP
        )
        self._update_allowed_collisions(
            [(SUCTION_LINK, self._source_name, True)]
        )
        approach_target = self._solve_ik(
            self._source_center[0],
            self._source_center[1],
            contact_z + PICKUP_APPROACH_CLEARANCE,
            f"{self._source_name} pickup approach",
            self._current_positions(),
        )
        contact_target = self._solve_ik(
            self._source_center[0],
            self._source_center[1],
            contact_z,
            f"{self._source_name} pickup contact",
            approach_target,
        )
        self._move_joints(
            approach_target, f"{self._source_name} pickup approach"
        )
        self._move_joints(
            contact_target, f"{self._source_name} pickup contact"
        )

        self.get_logger().info(
            f"Enabling suction at {self._source_name}"
        )
        try:
            self._command_suction(True)
        except TimeoutError:
            if self._suction_attached is not True:
                message = Bool()
                message.data = False
                self._suction_command_publisher.publish(message)
            raise
        self._attach_collision_object()

        lift_target = self._solve_ik(
            self._source_center[0],
            self._source_center[1],
            contact_z + LIFT_CLEARANCE,
            f"{self._source_name} pickup lift",
            contact_target,
        )
        self._move_joints(lift_target, f"{self._source_name} pickup lift")
        self._update_allowed_collisions(
            [
                (self._source_name, touching_name, False)
                for touching_name in self._source_touching_names
            ]
        )

        place_tool_z = (
            self._table_center[2] + contact_z - self._source_center[2]
        )
        table_side_target = dict(lift_target)
        table_side_target["shoulder_pan_joint"] -= math.pi
        table_side_target["wrist_3_joint"] += math.pi
        table_side_target = self._nearest_valid_ik_target(
            table_side_target,
            lift_target,
            f"{self._source_name} rotate toward table",
        )
        self._move_joints(
            table_side_target,
            f"{self._source_name} rotate toward table",
        )
        table_approach_target = self._solve_ik(
            self._table_center[0],
            self._table_center[1],
            place_tool_z + PLACE_APPROACH_CLEARANCE,
            f"{self._source_name} table approach",
            table_side_target,
        )
        self._move_joints(
            table_approach_target,
            f"{self._source_name} transfer to table",
        )
        table_descent_target = self._solve_ik(
            self._table_center[0],
            self._table_center[1],
            place_tool_z + PLACE_APPROACH_CLEARANCE / 2.0,
            f"{self._source_name} table descent",
            table_approach_target,
        )
        self._move_joints(
            table_descent_target,
            f"{self._source_name} table descent",
        )
        self._update_allowed_collisions(
            [(self._source_name, self._table_name, True)]
        )
        table_release_target = self._solve_ik(
            self._table_center[0],
            self._table_center[1],
            place_tool_z + PLACE_RELEASE_CLEARANCE,
            f"{self._source_name} table release hover",
            table_descent_target,
        )
        self._move_joints(
            table_release_target,
            f"{self._source_name} position over table to release",
        )

        self.get_logger().info(
            f"Releasing {self._source_name} on the table"
        )
        self._command_suction(False)
        self._release_collision_object()
        self._schedule_cylinder_removal(self._source_name)

        retreat_target = self._solve_ik(
            self._table_center[0],
            self._table_center[1],
            place_tool_z + PLACE_APPROACH_CLEARANCE,
            f"{self._source_name} table retreat",
            table_release_target,
        )
        self._move_joints(
            retreat_target,
            f"{self._source_name} retreat from table",
        )
        self._update_allowed_collisions(
            [
                (SUCTION_LINK, self._source_name, False),
                (self._source_name, self._table_name, False),
            ]
        )

    def run_demo(self):
        cylinders = self._cylinders_top_down()[: self._max_cylinders]
        cylinder_names = {name for name, _, _ in cylinders}
        self.get_logger().info(
            f"Starting top-down Stage 1 pick-and-place for "
            f"{len(cylinders)} cylinders"
        )
        self._wait_for_services()
        self._wait_until(
            lambda: set(JOINTS).issubset(self._joint_positions),
            45.0,
            "UR5e joint state",
        )
        self._wait_until(
            lambda: cylinder_names.issubset(
                {
                    item.id
                    for item in self._get_scene().world.collision_objects
                }
            ),
            30.0,
            "all stack cylinders in the planning scene",
        )
        self._wait_until(
            lambda: self.get_clock().now().nanoseconds > 0,
            30.0,
            "active Gazebo simulation clock",
        )
        scene = self._get_scene()
        world_objects = {
            item.id: item for item in scene.world.collision_objects
        }
        if self._table_name not in world_objects:
            raise RuntimeError(
                f"Placement table {self._table_name} is missing from MoveIt"
            )
        if any(
            item.object.id in cylinder_names
            for item in scene.robot_state.attached_collision_objects
        ):
            raise RuntimeError(
                "A Stage 1 cylinder is already attached in MoveIt"
            )

        initial_positions = self._current_positions()
        for source_name, source_center, touching_names in cylinders:
            self._source_name = source_name
            self._source_center = source_center
            self._source_touching_names = touching_names
            scene = self._get_scene()
            world_objects = {
                item.id: item for item in scene.world.collision_objects
            }
            source_object = world_objects.get(self._source_name)
            if source_object is None:
                raise RuntimeError(
                    f"{self._source_name} is missing from the MoveIt "
                    "planning scene"
                )
            self._validate_source_object(source_object)
            self._source_collision_object = copy.deepcopy(source_object)
            self.get_logger().info(
                f"Picking {self._source_name} at {self._source_center}; "
                f"touching stack cylinders: "
                f"{list(self._source_touching_names)}"
            )
            self._pick_and_place_cylinder()

        self._move_joints(initial_positions, "return to initial position")
        self._wait_until(
            lambda: not self._pending_removals,
            max(60.0, self._removal_delay * 10.0 + 30.0),
            "all scheduled cylinder removals",
        )
        self.get_logger().info(
            f"Top-down Stage 1 pick-and-place completed for "
            f"{len(cylinders)} cylinders"
        )


def main(args=None):
    rclpy.init(args=args)
    demo = Stage1PickPlaceDemo()
    success = False
    try:
        demo.run_demo()
        success = True
    except KeyboardInterrupt:
        demo.get_logger().warning(
            "Stage 1 pick-and-place motion was interrupted"
        )
    except (RuntimeError, TimeoutError, ValueError) as error:
        demo.get_logger().error(f"Stage 1 pick-and-place failed: {error}")
    finally:
        demo._executor.remove_node(demo)
        demo.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    return 0 if success else 1


if __name__ == "__main__":
    sys.exit(main())
