#!/usr/bin/env python3
"""
MoveIt2 Controller Base Class for Kinova 7-DOF Robotic Arm

This module provides a reusable base class that handles all MoveIt2 interactions
including standard motion planning, Cartesian path planning, gripper control,
and collision object management.

Can be inherited by both simulation and real robot implementations.
"""

from copy import deepcopy
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from geometry_msgs.msg import Pose
from moveit_msgs.msg import (MoveItErrorCodes, CollisionObject, PlanningScene, 
                              AttachedCollisionObject, RobotTrajectory)
from moveit_msgs.action import MoveGroup, ExecuteTrajectory
from moveit_msgs.srv import GetCartesianPath
from shape_msgs.msg import SolidPrimitive
from moveit_msgs.msg import JointConstraint, Constraints, PositionConstraint, OrientationConstraint
from moveit_msgs.msg import MotionPlanRequest, PlanningOptions, RobotState
from moveit_msgs.srv import GetMotionPlan
from sensor_msgs.msg import JointState
from tf_transformations import quaternion_multiply, quaternion_from_euler
import time
import math
import tf2_ros


        

def euler_to_quaternion(roll, pitch, yaw):
    """Convert Euler angles (radians) to quaternion (x, y, z, w)"""
    cr = math.cos(roll * 0.5)
    sr = math.sin(roll * 0.5)
    cp = math.cos(pitch * 0.5)
    sp = math.sin(pitch * 0.5)
    cy = math.cos(yaw * 0.5)
    sy = math.sin(yaw * 0.5)
    
    w = cr * cp * cy + sr * sp * sy
    x = sr * cp * cy - cr * sp * sy
    y = cr * sp * cy + sr * cp * sy
    z = cr * cp * sy - sr * sp * cy
    
    return (x, y, z, w)


class MoveItController(Node):
    """
    Base class for MoveIt2 control of Kinova 7-DOF arm
    
    Provides:
    - Standard motion planning
    - Cartesian path planning
    - Gripper control
    - Collision object management
    - Object attachment/detachment
    """
    
    def __init__(self, node_name='moveit_controller'):
        super().__init__(node_name)
        
        # Robot configuration
        self.arm_group_name = "manipulator"
        self.gripper_group_name = "gripper"
        self.base_frame = "base_link"
        self.end_effector_frame = "end_effector_link"
        
        # Gripper configuration (Robotiq 85)
        self.gripper_joint_names = [
            "robotiq_85_left_knuckle_joint",
            "robotiq_85_right_knuckle_joint"
        ]
        self.gripper_open_position = 0.0
        self.gripper_max_close_left = math.radians(46)
        self.gripper_max_close_right = math.radians(-46)

        self.GRIPPER_OUTER_LENGTH = 0.155
        self.GRIPPER_INNER_LENGTH = 0.088

        # Grasp orientations
        self.grasp_orientations = {
            'side': euler_to_quaternion(0.5*math.pi, 0, 0.5*math.pi)
        }

        # Motion Plan
        # Planner selection (OMPL by default)
        self.planning_pipeline_id = ""   # "" → use MoveIt default (OMPL)
        self.planner_id = ""            # "" → default planner in that pipeline

        # Collision object tracking
        self.collision_objects_added = set()
        
        # Initialize MoveGroup action client
        self.move_group_client = ActionClient(self, MoveGroup, '/move_action')
        
        # Cartesian path service client
        self.cartesian_path_client = self.create_client(
            GetCartesianPath, 
            '/compute_cartesian_path')
        
        # Execute trajectory action client
        self.execute_trajectory_client = ActionClient(
            self, 
            ExecuteTrajectory, 
            '/execute_trajectory')
        
        # Planning scene publisher
        self.planning_scene_pub = self.create_publisher(
            PlanningScene, '/planning_scene', 10)
        
        # TF for getting current pose
        self.tf_buffer = tf2_ros.Buffer(cache_time=rclpy.duration.Duration(seconds=10.0))
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)
        
        # Joint state subscriber for IK seed
        self.current_joint_state = None
        self.joint_state_sub = self.create_subscription(
            JointState,
            '/joint_states',
            self._joint_state_callback,
            10
        )
        
        # Wait for action server
        self.get_logger().info("Waiting for MoveGroup action server...")
        if not self.move_group_client.wait_for_server(timeout_sec=10.0):
            self.get_logger().error("MoveGroup action server not available!")
            return
          # Pilz planning service
        self.pilz_planning_client = self.create_client(
            GetMotionPlan,
            '/plan_kinematic_path'
        )

        # Check Pilz planning service
        self.get_logger().info("Waiting for Pilz planning service...")
        if not self.pilz_planning_client.wait_for_service(timeout_sec=5.0):
            self.get_logger().warn("Pilz planning service not available - will use standard planning")

        # Check Cartesian path service
        self.get_logger().info("Waiting for Cartesian path service...")
        if not self.cartesian_path_client.wait_for_service(timeout_sec=5.0):
            self.get_logger().warn("Cartesian path service not available - will use standard planning")
            self.cartesian_available = False
        else:
            self.cartesian_available = True
            self.get_logger().info("✓ Cartesian path planning enabled")
        
        # Check execute trajectory server
        if not self.execute_trajectory_client.wait_for_server(timeout_sec=5.0):
            self.get_logger().warn("Execute trajectory server not available")
        
        self.get_logger().info("✓ MoveIt2 Controller initialized")
    
    def _joint_state_callback(self, msg):
        """Store current joint state for IK seed"""
        self.current_joint_state = msg
    
    def _get_start_state(self):
        """Get current robot state as start state for planning"""
        if self.current_joint_state is None:
            return None
        
        robot_state = RobotState()
        robot_state.joint_state = self.current_joint_state
        return robot_state
    
    # ==================== POSE UTILITIES ====================
    
    def get_current_pose(self):
        """Get current end-effector pose via TF"""
        try:
            trans = self.tf_buffer.lookup_transform(
                self.base_frame,
                self.end_effector_frame,
                rclpy.time.Time(),
                timeout=rclpy.duration.Duration(seconds=1.0))
            
            pose = Pose()
            pose.position.x = trans.transform.translation.x
            pose.position.y = trans.transform.translation.y
            pose.position.z = trans.transform.translation.z
            pose.orientation = trans.transform.rotation
            
            return pose
        except Exception as e:
            self.get_logger().error(f"Failed to get current pose: {e}")
            return None
    
    def set_grasp_orientation(self, pose, orientation_name):
        """Set pose orientation using predefined orientation name"""
        if orientation_name not in self.grasp_orientations:
            self.get_logger().error(f"Unknown orientation: {orientation_name}")
            return False
            
        qx, qy, qz, qw = self.grasp_orientations[orientation_name]
        pose.orientation.x = qx
        pose.orientation.y = qy
        pose.orientation.z = qz
        pose.orientation.w = qw
        
        return True
    
    def compute_approach_pose(self, grasp_pose, distance=0.15, direction="side"):
        """Compute pre-grasp approach pose"""
        approach_pose = Pose()
        approach_pose.position.x = grasp_pose.position.x
        approach_pose.position.y = grasp_pose.position.y
        approach_pose.position.z = grasp_pose.position.z
        
        if direction == "side":
            approach_pose.position.x -= distance
            qx, qy, qz, qw = self.grasp_orientations['side']
            approach_pose.orientation.x = qx
            approach_pose.orientation.y = qy
            approach_pose.orientation.z = qz
            approach_pose.orientation.w = qw
        
        return approach_pose
    
    # ==================== MOTION PLANNING ====================
    
    def move_to_pose(self, target_pose, planning_time=7.0, max_retries=5):
        """Move arm to target pose; auto-retry if MoveIt returns INVALID_MOTION_PLAN."""
        # Adjust parameters for Pilz planners (they need more time and attempts)
        is_pilz = (self.planning_pipeline_id == "pilz_industrial_motion_planner")
        if is_pilz:
            planning_time = max(planning_time, 10.0)  # At least 10s for Pilz
            max_planning_attempts = 30  # More attempts for Pilz
            # Start with more relaxed constraints for Pilz
            initial_box = 0.10  # Start at 10cm instead of 5cm
            initial_ori_tol = 0.05  # Start at 0.05rad instead of 0.005rad
            max_retries = 2
        else:
            max_planning_attempts = 12
            initial_box = 0.02
            initial_ori_tol = 0.005
        
        for attempt in range(1, max_retries + 1):
                # Relax FASTER for Pilz
            if is_pilz:
                box = initial_box + 0.03 * (attempt - 1)  # Bigger steps
                ori_tol = initial_ori_tol + 0.02 * (attempt - 1)
            else:
                box = initial_box + 0.01 * (attempt - 1)
                ori_tol = initial_ori_tol + 0.005 * (attempt - 1)
            # Gradually relax constraints each attempt
            box = initial_box + 0.01 * (attempt - 1)
            # ori_tol = initial_ori_tol + 0.005 * (attempt - 1)

            goal = MoveGroup.Goal()
            goal.request.group_name = self.arm_group_name
            goal.request.num_planning_attempts = max_planning_attempts
            goal.request.allowed_planning_time = planning_time
            goal.request.max_velocity_scaling_factor = 0.15
            goal.request.max_acceleration_scaling_factor = 0.15

            # 👉 Set start state (current joint state) to help IK solver
            # This significantly improves IK success rate, especially for Pilz planners
            start_state = self._get_start_state()
            if start_state is not None:
                goal.request.start_state = start_state
                if attempt == 1:  # Log only on first attempt to avoid spam
                    self.get_logger().debug("Using current joint state as IK seed")
            else:
                if attempt == 1:
                    self.get_logger().debug("Joint state not available yet, using default start state")

            # 👉 Apply current planner selection (Pilz / OMPL)
            if self.planning_pipeline_id:
                goal.request.pipeline_id = self.planning_pipeline_id
            if self.planner_id:
                goal.request.planner_id = self.planner_id

            # Hint MoveGroup to replan internally if its validation fails
            try:
                goal.planning_options.replan = True
                goal.planning_options.replan_attempts = 2
                goal.planning_options.replan_delay = 0.1
            except Exception:
                pass

            constraints = Constraints()

            # Position box around target
            pos_constraint = PositionConstraint()
            pos_constraint.header.frame_id = self.base_frame
            pos_constraint.link_name = self.end_effector_frame
            pos_prim = SolidPrimitive()
            pos_prim.type = SolidPrimitive.BOX
            pos_prim.dimensions = [box, box, box]
            pos_constraint.constraint_region.primitives.append(pos_prim)
            pos_constraint.constraint_region.primitive_poses.append(target_pose)
            pos_constraint.weight = 1.0
            constraints.position_constraints.append(pos_constraint)

            # Orientation tolerance
            orient_constraint = OrientationConstraint()
            orient_constraint.header.frame_id = self.base_frame
            orient_constraint.link_name = self.end_effector_frame
            orient_constraint.orientation = target_pose.orientation
            orient_constraint.absolute_x_axis_tolerance = ori_tol
            orient_constraint.absolute_y_axis_tolerance = ori_tol
            orient_constraint.absolute_z_axis_tolerance = ori_tol
            orient_constraint.weight = 1.0
            constraints.orientation_constraints.append(orient_constraint)

            goal.request.goal_constraints = [constraints]

            self.get_logger().info(
                f"Sending move goal (attempt {attempt}/{max_retries}) "
                f"box={box:.3f}m ori_tol={ori_tol:.3f}rad")

            future = self.move_group_client.send_goal_async(goal)
            rclpy.spin_until_future_complete(self, future, timeout_sec=10.0)
            if not future.done():
                self.get_logger().warn("Move goal send timed out; retrying...")
                continue

            goal_handle = future.result()
            if not goal_handle or not goal_handle.accepted:
                self.get_logger().warn("Move goal rejected; retrying...")
                time.sleep(0.05)
                continue

            result_future = goal_handle.get_result_async()
            rclpy.spin_until_future_complete(self, result_future, timeout_sec=planning_time + 60.0)
            if not result_future.done():
                self.get_logger().warn("Move result timeout; retrying...")
                continue

            result = result_future.result()
            code = result.result.error_code.val
            code_str = self._error_to_string(code)

            if code == MoveItErrorCodes.SUCCESS:
                self.get_logger().info("✓ Move completed successfully")
                return True

            self.get_logger().warn(f"✗ Move failed with {code_str} (code {code}); retrying...")

            # Abort early on definitively invalid start/goal conditions
            if code in (
                MoveItErrorCodes.START_STATE_IN_COLLISION,
                MoveItErrorCodes.GOAL_IN_COLLISION,
                MoveItErrorCodes.GOAL_CONSTRAINTS_VIOLATED,
                MoveItErrorCodes.INVALID_GOAL_CONSTRAINTS,
                MoveItErrorCodes.INVALID_ROBOT_STATE,
            ):
                self.get_logger().error("Start/goal invalid; adjust pose/scene and retry later.")
                return False

            # Otherwise (INVALID_MOTION_PLAN, PLANNING_FAILED, FAILURE, environment change) -> loop
            time.sleep(0.05)

        self.get_logger().error("All planning retries exhausted.")
        return False
    
    #=====================HOME POSITION========================
    def move_to_home_position(self):
        """
        Move arm to safe home position using joint control
        
        Joint values from Kinova Web App:
        - joint_1: 0°
        - joint_2: 15°
        - joint_3: 180°
        - joint_4: -130°
        - joint_5: 0°
        - joint_6: 55°
        - joint_7: 90°
        """
        self.get_logger().info("Moving to HOME position...")
        
        # Convert degrees to radians
        home_joint_positions = [
            math.radians(0),    # joint_1
            math.radians(15),   # joint_2
            math.radians(180),  # joint_3
            math.radians(-130), # joint_4
            math.radians(0),    # joint_5
            math.radians(55),   # joint_6
            math.radians(90)    # joint_7
        ]
        
        return self.move_to_joint_positions(home_joint_positions)


    def move_to_joint_positions(self, joint_positions, planning_time=10.0):
        """
        Move arm to specific joint configuration
        
        Args:
            joint_positions: List of 7 joint angles in radians
            planning_time: Time allowed for planning
        
        Returns:
            bool: True if successful
        """
        if len(joint_positions) != 7:
            self.get_logger().error(f"Expected 7 joint values, got {len(joint_positions)}")
            return False
        
        goal = MoveGroup.Goal()
        goal.request.group_name = self.arm_group_name
        goal.request.num_planning_attempts = 15
        goal.request.allowed_planning_time = planning_time
        
        # Use slower speeds for safety
        goal.request.max_velocity_scaling_factor = 0.2
        goal.request.max_acceleration_scaling_factor = 0.2
        
        # Set start state
        start_state = self._get_start_state()
        if start_state is not None:
            goal.request.start_state = start_state
        
        # Create joint constraints for target position
        constraints = Constraints()
        
        joint_names = [
            "joint_1", "joint_2", "joint_3", "joint_4",
            "joint_5", "joint_6", "joint_7"
        ]
        
        for joint_name, position in zip(joint_names, joint_positions):
            joint_constraint = JointConstraint()
            joint_constraint.joint_name = joint_name
            joint_constraint.position = position
            joint_constraint.tolerance_above = 0.01  # ~0.57°
            joint_constraint.tolerance_below = 0.01
            joint_constraint.weight = 1.0
            constraints.joint_constraints.append(joint_constraint)
        
        goal.request.goal_constraints.append(constraints)
        
        self.get_logger().info(f"Planning to HOME joint configuration...")
        
        # Send goal
        future = self.move_group_client.send_goal_async(goal)
        rclpy.spin_until_future_complete(self, future, timeout_sec=10.0)
        
        if not future.done():
            self.get_logger().error("Move to home goal send timed out")
            return False
        
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().error("Move to home goal rejected")
            return False
        
        # Wait for result
        result_future = goal_handle.get_result_async()
        rclpy.spin_until_future_complete(self, result_future, timeout_sec=planning_time + 10.0)
        
        if not result_future.done():
            self.get_logger().error("Move to home result timed out")
            return False
        
        result = result_future.result()
        success = result.result.error_code.val == MoveItErrorCodes.SUCCESS
        
        if success:
            self.get_logger().info("✓ Reached HOME position")
        else:
            error_str = self._error_to_string(result.result.error_code.val)
            self.get_logger().error(f"✗ Failed to reach HOME: {error_str}")
        
        return success
        
    #=====================CHATGPT PILZ==============================
    def move_lin_relative(self, dx=0.0, dy=0.0, dz=0.0, planning_time=5.0):
        """
        Move linearly relative to current pose using Pilz LIN.
        Useful for side-approach (along x) and vertical moves.

        Args:
            dx, dy, dz: relative offsets in base frame (meters)
        """
        current_pose = self.get_current_pose()
        if current_pose is None:
            self.get_logger().error("move_lin_relative: no current pose")
            return False

        target = deepcopy(current_pose)
        target.position.x += dx
        target.position.y += dy
        target.position.z += dz

        distance = (dx**2 + dy**2 + dz**2) ** 0.5
        self.get_logger().info(
            f"Pilz LIN relative move: Δ({dx:.3f}, {dy:.3f}, {dz:.3f}) = {distance:.3f}m"
        )

        self.use_pilz_lin()
        success = self.move_to_pose(target, planning_time=planning_time)
        self.use_default_planner()
        return success


    def lift_with_pilz(self, lift_height=0.10, linear=True):
        """
        Lift the end-effector straight up by lift_height using Pilz.

        Args:
            lift_height: meters to lift in +Z of base frame
            linear: if True → LIN (Cartesian straight up), else PTP
        """
        pose = self.get_current_pose()
        if pose is None:
            self.get_logger().error("Cannot lift - current pose unreachable")
            return False

        target = deepcopy(pose)
        target.position.z += lift_height

        if linear:
            self.use_pilz_lin()
            self.get_logger().info(f"Using Pilz LIN to lift by {lift_height:.3f} m")
            if not self.move_linear_pilz(target):
                self.get_logger().error("Pilz LIN lift failed")
                self.use_pilz_ptp()
                self.get_logger().info("Falling back to Pilz PTP for lift")
                if not self.move_to_pose(target):
                    self.get_logger().error("Pilz PTP lift also failed")
                    self.use_default_planner()
                    if not self.move_to_pose(target):
                        self.get_logger().error("Final fallback standard planning lift failed")
                        return False
        else:
            self.use_pilz_ptp()
            self.get_logger().info(f"Using Pilz PTP to lift by {lift_height:.3f} m")
            if not self.move_to_pose(target):
                    self.get_logger().error("Pilz PTP lift also failed")
                    self.use_default_planner()
                    if not self.move_to_pose(target):
                        self.get_logger().error("Final fallback standard planning lift failed")
                        return False
            self.get_logger().info("Pilz PTP lift successful")

        success = self.move_to_pose(target)

        self.use_default_planner()
        return success

    #====================ROTATE EEF ================================
    def rotate_eef_relative(self, roll=0.0, pitch=0.0, yaw=0.0):
        """
        Rotate end-effector by Euler increments (relative rotation).
        Positive rotation follows REP-103 (X=roll, Y=pitch, Z=yaw).
        """

        current = self.get_current_pose()
        if not current:
            self.get_logger().error("Cannot rotate EEF — no current pose")
            return False

        # Current quaternion
        q_current = [
            current.orientation.x,
            current.orientation.y,
            current.orientation.z,
            current.orientation.w,
        ]

        # Desired relative rotation
        q_delta = quaternion_from_euler(roll, pitch, yaw)

        # New target orientation = q_current ⨉ q_delta
        q_new = quaternion_multiply(q_current, q_delta)

        target = deepcopy(current)
        target.orientation.x = q_new[0]
        target.orientation.y = q_new[1]
        target.orientation.z = q_new[2]
        target.orientation.w = q_new[3]

        # Use a smooth Pilz LIN OR a slower OMPL fallback
        self.use_pilz_lin()
        if not self.move_to_pose(target):
            self.get_logger().warn("Pilz LIN rotate failed; falling back to standard planning")
            self.use_default_planner()
            if not self.move_to_pose(target):
                self.get_logger().error("Final fallback standard planning rotate failed")
                return False
        self.use_default_planner()

        return True

    # ==================== PLANNER SELECTION (PILZ / OMPL) ====================

    def use_pilz_ptp(self):
        """Use Pilz PTP (joint-space) planner for the next motions."""
        self.planning_pipeline_id = "pilz_industrial_motion_planner"
        self.planner_id = "PTP"
        self.get_logger().info("Planner set: Pilz PTP")

    def use_pilz_lin(self):
        """Use Pilz LIN (Cartesian straight-line) planner for the next motions."""
        self.planning_pipeline_id = "pilz_industrial_motion_planner"
        self.planner_id = "LIN"
        self.get_logger().info("Planner set: Pilz LIN")

    def use_default_planner(self):
        """Return to default MoveIt pipeline (e.g., OMPL)."""
        self.planning_pipeline_id = ""
        self.planner_id = ""
        self.get_logger().info("Planner set: default (OMPL)")

        
    #===================== ERROR CODES ====================
    def _error_to_string(self, code: int) -> str:
        mapping = {
            MoveItErrorCodes.SUCCESS: "SUCCESS",
            MoveItErrorCodes.FAILURE: "FAILURE",
            MoveItErrorCodes.PLANNING_FAILED: "PLANNING_FAILED",
            MoveItErrorCodes.INVALID_MOTION_PLAN: "INVALID_MOTION_PLAN",
            MoveItErrorCodes.MOTION_PLAN_INVALIDATED_BY_ENVIRONMENT_CHANGE:
                "MOTION_PLAN_INVALIDATED_BY_ENVIRONMENT_CHANGE",
            MoveItErrorCodes.START_STATE_IN_COLLISION: "START_STATE_IN_COLLISION",
            MoveItErrorCodes.START_STATE_VIOLATES_PATH_CONSTRAINTS: "START_STATE_VIOLATES_PATH_CONSTRAINTS",
            MoveItErrorCodes.GOAL_IN_COLLISION: "GOAL_IN_COLLISION",
            MoveItErrorCodes.GOAL_VIOLATES_PATH_CONSTRAINTS: "GOAL_VIOLATES_PATH_CONSTRAINTS",
            MoveItErrorCodes.GOAL_CONSTRAINTS_VIOLATED: "GOAL_CONSTRAINTS_VIOLATED",
            MoveItErrorCodes.INVALID_GROUP_NAME: "INVALID_GROUP_NAME",
            MoveItErrorCodes.INVALID_GOAL_CONSTRAINTS: "INVALID_GOAL_CONSTRAINTS",
            MoveItErrorCodes.INVALID_ROBOT_STATE: "INVALID_ROBOT_STATE",
            MoveItErrorCodes.NO_IK_SOLUTION: "NO_IK_SOLUTION",
        }
        return mapping.get(code, f"UNKNOWN({code})")
    
    # ==================== CARTESIAN PLANNING ====================
    
    def move_cartesian(self, waypoints, eef_step=0.005, jump_threshold=0.0, 
                       avoid_collisions=True, max_velocity_scaling=0.1):
        """
        Execute Cartesian path through waypoints
        
        Args:
            waypoints: List of Pose objects to move through
            eef_step: Maximum step size between points (meters)
            jump_threshold: Maximum joint space jump allowed
            avoid_collisions: Whether to check for collisions
            max_velocity_scaling: Velocity scaling factor (0.0-1.0)
        
        Returns:
            bool: True if successful, False otherwise
        """
        if not self.cartesian_available:
            self.get_logger().warn("Cartesian path service not available")
            return False
        
        self.get_logger().info(f"Planning Cartesian path with {len(waypoints)} waypoints...")
        
        # Create request
        request = GetCartesianPath.Request()
        request.header.frame_id = self.base_frame
        request.header.stamp = self.get_clock().now().to_msg()
        request.group_name = self.arm_group_name
        request.waypoints = waypoints
        request.max_step = eef_step
        request.jump_threshold = jump_threshold
        request.avoid_collisions = avoid_collisions
        request.path_constraints = Constraints()
        
        # Call service
        future = self.cartesian_path_client.call_async(request)
        rclpy.spin_until_future_complete(self, future, timeout_sec=5.0)
        
        if not future.done():
            self.get_logger().error("Cartesian path service call timed out")
            return False
        
        response = future.result()
        
        if response is None:
            self.get_logger().error("Cartesian path service returned None")
            return False
        
        # Check if path was successfully computed
        fraction = response.fraction
        self.get_logger().info(f"Cartesian path computed: {fraction*100:.1f}% of path achieved")
        
        if fraction < 0.95:
            self.get_logger().warn(
                f"Only {fraction*100:.1f}% of Cartesian path achieved")
            if fraction < 0.5:
                self.get_logger().error("Less than 50% of path computed - aborting")
                return False
        
        # Execute the trajectory
        return self.execute_trajectory(response.solution, max_velocity_scaling)
    
    def execute_trajectory(self, trajectory, max_velocity_scaling=0.1):
        """
        Execute a computed trajectory
        
        Args:
            trajectory: RobotTrajectory message
            max_velocity_scaling: Velocity scaling factor
        
        Returns:
            bool: True if successful
        """
        self.get_logger().info("Executing Cartesian trajectory...")
        
        # Create goal
        goal = ExecuteTrajectory.Goal()
        goal.trajectory = trajectory
        
        # Send goal
        future = self.execute_trajectory_client.send_goal_async(goal)
        rclpy.spin_until_future_complete(self, future, timeout_sec=2.0)
        
        if not future.done():
            self.get_logger().error("Execute trajectory goal send timed out")
            return False
        
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().error("Execute trajectory goal rejected")
            return False
        
        self.get_logger().info("Trajectory execution started...")
        
        # Wait for result
        result_future = goal_handle.get_result_async()
        rclpy.spin_until_future_complete(self, result_future)
        
        result = result_future.result()
        success = result.result.error_code.val == MoveItErrorCodes.SUCCESS
        
        if success:
            self.get_logger().info("✓ Cartesian trajectory executed successfully")
        else:
            self.get_logger().error(f"✗ Trajectory execution failed: {result.result.error_code.val}")
        
        return success
    
    def move_relative_cartesian(self, delta_x=0.0, delta_y=0.0, delta_z=0.0, 
                            eef_step=0.01, jump_threshold=2.0):  # Increased from 0.005 and 0.0
        """
        Move relative to current pose using Cartesian path
        
        Args:
            delta_x/y/z: Position changes in meters
            eef_step: Step size for Cartesian interpolation (larger = fewer checks)
            jump_threshold: Allow larger joint jumps (0.0 = very strict)
        
        Returns:
            bool: True if successful
        """
        current_pose = self.get_current_pose()
        if current_pose is None:
            return False
        
        target_pose = Pose()
        target_pose.position.x = current_pose.position.x + delta_x
        target_pose.position.y = current_pose.position.y + delta_y
        target_pose.position.z = current_pose.position.z + delta_z
        target_pose.orientation = current_pose.orientation
        
        distance = (delta_x**2 + delta_y**2 + delta_z**2)**0.5
        self.get_logger().info(
            f"Cartesian move: Δ({delta_x:.3f}, {delta_y:.3f}, {delta_z:.3f}) = {distance:.3f}m")
        
        # Use larger step size for long distances
        adaptive_step = min(0.02, distance / 20)  # At least 20 waypoints
        
        return self.move_cartesian([target_pose], 
                                eef_step=adaptive_step, 
                                jump_threshold=jump_threshold)
    
    # ==================== GRIPPER CONTROL ====================
    
    def control_gripper(self, state, gripper_value=None):
        """Control gripper state (open/close)"""
        gripper_client = ActionClient(self, MoveGroup, '/move_action')
        
        if not gripper_client.wait_for_server(timeout_sec=5.0):
            self.get_logger().error("Gripper action server not available!")
            return False
        
        goal = MoveGroup.Goal()
        goal.request.group_name = self.gripper_group_name
        goal.request.num_planning_attempts = 5
        goal.request.allowed_planning_time = 1.0
        goal.request.max_velocity_scaling_factor = 0.5
        goal.request.max_acceleration_scaling_factor = 0.5
        
        constraints = Constraints()
        
        if state == "open":
            positions = [self.gripper_open_position, self.gripper_open_position]
        elif state == "close":
            if gripper_value:
                positions = [
                    self.gripper_max_close_left * gripper_value,
                    self.gripper_max_close_right * gripper_value
                ]
            else:
                positions = [
                    self.gripper_max_close_left * 0.6,
                    self.gripper_max_close_right * 0.6
                ]
        else:
            return False
        
        for i, joint_name in enumerate(self.gripper_joint_names):
            joint_constraint = JointConstraint()
            joint_constraint.joint_name = joint_name
            joint_constraint.position = positions[i]
            joint_constraint.tolerance_above = 0.005
            joint_constraint.tolerance_below = 0.005
            joint_constraint.weight = 1.0
            constraints.joint_constraints.append(joint_constraint)
        
        goal.request.goal_constraints.append(constraints)
        
        future = gripper_client.send_goal_async(goal)
        rclpy.spin_until_future_complete(self, future)
        
        goal_handle = future.result()
        if not goal_handle.accepted:
            gripper_client.destroy()
            return False
        
        result_future = goal_handle.get_result_async()
        rclpy.spin_until_future_complete(self, result_future)
        
        result = result_future.result()
        gripper_client.destroy()
        
        return result.result.error_code.val == MoveItErrorCodes.SUCCESS
    
    def open_gripper(self, openness=0.0):
        """Open gripper"""
        return self.control_gripper("open")
    
    def close_gripper(self, grip_force=0.5):
        """Close Robotiq 85 gripper with specific grip force"""
        return self.control_gripper("close", grip_force)
    
    # ==================== COLLISION OBJECTS ====================
    
    def add_box_collision_object(self, name, x, y, z, width, depth, height):
        """Add a box collision object to the planning scene"""
        self.get_logger().info(f"Adding box collision object '{name}'")
        
        collision_object = CollisionObject()
        collision_object.header.frame_id = self.base_frame
        collision_object.header.stamp = self.get_clock().now().to_msg()
        collision_object.id = name
        
        box = SolidPrimitive()
        box.type = SolidPrimitive.BOX
        box.dimensions = [width, depth, height]
        
        box_pose = Pose()
        box_pose.position.x = float(x)
        box_pose.position.y = float(y)
        box_pose.position.z = float(z)
        box_pose.orientation.w = 1.0
        
        collision_object.primitives.append(box)
        collision_object.primitive_poses.append(box_pose)
        collision_object.operation = CollisionObject.ADD
        
        planning_scene = PlanningScene()
        planning_scene.world.collision_objects.append(collision_object)
        planning_scene.is_diff = True
        
        self.planning_scene_pub.publish(planning_scene)
        time.sleep(0.5)
        
        self.collision_objects_added.add(name)
        self.get_logger().info(f"✓ Box '{name}' added")
        return True
    
    def add_cylinder_collision_object(self, name, x, y, z, radius, height):
        """Add a cylinder collision object to the planning scene"""
        self.get_logger().info(f"Adding cylinder collision object '{name}'")
        
        collision_object = CollisionObject()
        collision_object.header.frame_id = self.base_frame
        collision_object.header.stamp = self.get_clock().now().to_msg()
        collision_object.id = name
        
        cylinder = SolidPrimitive()
        cylinder.type = SolidPrimitive.CYLINDER
        cylinder.dimensions = [height, radius]
        
        cylinder_pose = Pose()
        cylinder_pose.position.x = float(x)
        cylinder_pose.position.y = float(y)
        cylinder_pose.position.z = float(z)
        cylinder_pose.orientation.w = 1.0
        
        collision_object.primitives.append(cylinder)
        collision_object.primitive_poses.append(cylinder_pose)
        collision_object.operation = CollisionObject.ADD
        
        planning_scene = PlanningScene()
        planning_scene.world.collision_objects.append(collision_object)
        planning_scene.is_diff = True
        
        self.planning_scene_pub.publish(planning_scene)
        time.sleep(0.5)
        
        self.collision_objects_added.add(name)
        self.get_logger().info(f"✓ Cylinder '{name}' added")
        return True
    
    def remove_collision_object(self, name):
        """Remove a collision object from the planning scene"""
        if name not in self.collision_objects_added:
            self.get_logger().warn(f"Object '{name}' not in tracked objects")
            return False
        
        collision_object = CollisionObject()
        collision_object.id = name
        collision_object.operation = CollisionObject.REMOVE
        
        planning_scene = PlanningScene()
        planning_scene.world.collision_objects.append(collision_object)
        planning_scene.is_diff = True
        
        self.planning_scene_pub.publish(planning_scene)
        self.collision_objects_added.remove(name)
        
        self.get_logger().info(f"✓ Removed collision object '{name}'")
        return True
    
    
    #======================ATTACH DETACH FIX=========================
    def attach_object_to_gripper(self, object_name):
        """Attach object from world to gripper safely."""
        self.get_logger().info(f"Attaching '{object_name}' to gripper...")

        # 1) Remove world object
        # self.remove_collision_object(object_name)
        # time.sleep(0.05)

        # 2) Create attached object

        aco = AttachedCollisionObject()
        aco.link_name = self.end_effector_frame
        aco.object.id = object_name
        aco.object.operation = CollisionObject.ADD
        aco.touch_links = [
            self.end_effector_frame,
            "robotiq_85_left_knuckle_link",
            "robotiq_85_right_knuckle_link",
            "robotiq_85_left_finger_link",
            "robotiq_85_right_finger_link",
            "robotiq_85_left_inner_knuckle_link",
            "robotiq_85_right_inner_knuckle_link",
            "robotiq_85_left_finger_tip_link",
            "robotiq_85_right_finger_tip_link"
        ]

        scene = PlanningScene()
        scene.is_diff = True
        scene.robot_state.is_diff = True
        scene.robot_state.attached_collision_objects.append(aco)

        for _ in range(3):
            self.planning_scene_pub.publish(scene)
            time.sleep(0.05)

        self.get_logger().info(f"✓ Attached '{object_name}'")
        return True
    
    def detach_object_from_gripper(self, object_name):
        """Detach object and keep the world consistent."""
        self.get_logger().info(f"Detaching '{object_name}' from gripper...")

        # 1) Remove the attached object
        aco = AttachedCollisionObject()
        aco.link_name = self.end_effector_frame
        aco.object.id = object_name
        aco.object.operation = CollisionObject.REMOVE

        remove_scene = PlanningScene()
        remove_scene.is_diff = True
        remove_scene.robot_state.is_diff = True
        remove_scene.robot_state.attached_collision_objects.append(aco)

        for _ in range(3):
            self.planning_scene_pub.publish(remove_scene)
            time.sleep(0.05)

        self.get_logger().info(f"✓ Detached '{object_name}'")

        return True

    # ==================== HIGH-LEVEL OPERATIONS ====================
    def HIGH_LEVEL_move_ptp(self, target_pose):
        """High-level PTP move to target pose using default planner."""
        self.get_logger().info("Starting high-level PTP move...")
        self.use_pilz_ptp()
        if not self.move_to_pose(target_pose):
            self.use_default_planner()
            self.get_logger().error("High-level PTP move failed with Pilz planner")
            if not self.move_to_pose(target_pose):
                self.get_logger().error("High-level PTP move failed with default planner")
                return False
        self.use_default_planner()
        self.get_logger().info("✓ High-level PTP move successful")
        return True

    def HIGH_LEVEL_move_lin(self, target_pose):
        """High-level LIN move to target pose using default planner."""
        self.get_logger().info("Starting high-level LIN move...")
        self.use_pilz_lin()
        if not self.move_to_pose(target_pose):
            if not self.HIGH_LEVEL_move_ptp(target_pose):
               return False
        self.use_default_planner()
        self.get_logger().info("✓ High-level LIN move successful")
        return True
    
    def HIGH_LEVEL_move_lin_relative(self, dx=0.0, dy=0.0, dz=0.0):
        """High-level LIN relative move using default planner."""
        self.get_logger().info("Starting high-level LIN relative move...")
        # self.use_pilz_lin()
        current_pose = self.get_current_pose()
        if current_pose is None:
            self.get_logger().error("Cannot perform LIN relative move - current pose unreachable")
            self.use_default_planner()
            return False
        target_pose = deepcopy(current_pose)
        target_pose.position.x += dx
        target_pose.position.y += dy
        target_pose.position.z += dz
        if not self.HIGH_LEVEL_move_lin(target_pose):
            return False
        self.get_logger().info("✓ High-level LIN relative move successful")
        return True
    
    def HIGH_LEVEL_pour(self, pour_angle_degree=45):
        """
        Execute a pouring motion by rotating the end-effector.

        Args:
            pour_angle_degree: Angle to rotate around Y-axis in degrees
        """
        pour_angle_rad = math.radians(pour_angle_degree)
        self.get_logger().info(f"Starting pouring motion by {pour_angle_rad:.2f} rad...")

        # Rotate end-effector around Y-axis
        if not self.rotate_eef_relative(yaw=pour_angle_rad):
            self.get_logger().error("Pouring motion failed")
            return False
        
        if not self.rotate_eef_relative(yaw=-pour_angle_rad):
            self.get_logger().error("Returning from pouring motion failed")
            return False

        self.get_logger().info("✓ Pouring motion completed successfully")
        return True

    def pick_object(self, object_name, grasp_pose, approach_direction="side",
                    approach_distance=0.02, grasp_distance=0.015,
                    lift_distance=0.2, grip_force=0.5):
        """
        Execute complete pick operation using Pilz planners.

        For side grasp: we approach along -X (base frame) from a pre-grasp,
        then slide in along +X to grasp the body.

        Args:
            object_name: Name of object to pick (collision id)
            grasp_pose: Pose of object / grasp point (in base frame)
            approach_direction: currently only "side" supported
            approach_distance: distance BEFORE contact (m)
            grasp_distance: how deep to go past pre-grasp (m)
            lift_distance: vertical lift after grasp (m)
            grip_force: gripper closing effort (0.0-1.0)

        Returns:
            bool: True if successful
        """
        self.get_logger().info(f"Starting pick operation for '{object_name}'")
        approach_distance += self.GRIPPER_OUTER_LENGTH

        # Step 1: Compute and move to pre-grasp pose with PTP
        approach_pose = self.compute_approach_pose(
            grasp_pose, distance=approach_distance, direction=approach_direction)

        self.get_logger().info(
            f"Step 1: Moving to approach pose (Pilz PTP) -> "
            f"{approach_pose.position.x:.3f}, {approach_pose.position.y:.3f}, {approach_pose.position.z:.3f}"
        )
        if not self.HIGH_LEVEL_move_lin(approach_pose):
            self.get_logger().error("Failed to reach approach pose")
            return False

        # Step 2: Open gripper
        self.get_logger().info("Step 2: Opening gripper...")
        if not self.open_gripper():
            self.get_logger().error("Failed to open gripper")
            return False
        time.sleep(1.0)

        # Step 3: Pilz LIN side-approach towards object
        delta_x = approach_distance - (self.GRIPPER_INNER_LENGTH + grasp_distance)
        delta_z = 0.0

        current_pose = self.get_current_pose()
        if current_pose:
            self.get_logger().info(
                f"Step 3: Approaching object using Pilz LIN "
                f"(delta_x={delta_x:.3f}m, delta_z={delta_z:.3f}m)"
            )
            self.get_logger().info(
                f"  Current gripper tip approx: "
                f"{current_pose.position.x + self.GRIPPER_INNER_LENGTH:.3f}, "
                f"{current_pose.position.y:.3f}, {current_pose.position.z:.3f}"
            )
        else:
            self.get_logger().warn("Failed to get current pose before LIN approach")

        if not self.HIGH_LEVEL_move_lin_relative(dx=delta_x, dz=delta_z):
            self.get_logger().error("Failed to approach object")
            return False
        time.sleep(0.5)

        # Step 4: Remove and attach object to gripper
        self.get_logger().info("Step 4: Attaching object to gripper for collision avoidance...")
        if not self.attach_object_to_gripper(object_name):
            self.get_logger().error("Failed to attach object")
            return False
        time.sleep(1.0)

        # Step 5: Close gripper to grasp
        self.get_logger().info("Step 5: Closing gripper...")
        if not self.close_gripper(grip_force):
            self.get_logger().error("Failed to close gripper")
            return False
        time.sleep(1.0)

        # Step 6: Lift vertically using Pilz LIN
        self.get_logger().info(f"Step 6: Lifting object using Pilz LIN ({lift_distance} m)...")
        if not self.HIGH_LEVEL_move_lin_relative(dz=lift_distance):
            self.get_logger().error("Failed to lift object")
            return False
        time.sleep(0.5)
        
        self.get_logger().info("✓ Pick operation completed successfully")
        return True


    def place_object_with_start(self, object_name, start_pose, place_pose, retreat_distance=0.15):
        """
        Execute complete place operation using Pilz planners.

        Strategy:
          1) Move above place pose with Pilz PTP
          2) Descend with Pilz LIN
          3) Open gripper (release)
          4) Retreat up with Pilz LIN
          5) Detach object from gripper

        Args:
            object_name: Name of object to place (collision id)
            start_pose: Starting pose of object before placing (unused except for debug)
            place_pose: Target place pose
            retreat_distance: Distance to retreat upward after placing (m)

        Returns:
            bool: True if successful
        """
        self.get_logger().info(f"Starting place operation for '{object_name}'")

        # Step 1: Move to place pre-pose (above) using PTP
        dx = place_pose.position.x - start_pose.position.x
        dy = place_pose.position.y - start_pose.position.y
        # dz = place_pose.position.z - start_pose.position.z

        # current_pose = self.get_current_pose()
        place_pre_pose_eef = self.get_current_pose()
        place_pre_pose_eef.position.x += dx
        place_pre_pose_eef.position.y += dy

        # place_pre_pose_eef.position.z += retreat_distance  # go above by retreat_distance

        self.get_logger().info(
            f"Step 1: Moving to place pre-pose x, y (Pilz PTP) -> "
            f"{place_pre_pose_eef.position.x:.3f}, {place_pre_pose_eef.position.y:.3f}, {place_pre_pose_eef.position.z:.3f}"
        )

        if not self.HIGH_LEVEL_move_lin_relative(dx=dx, dy=dy):
            self.get_logger().error("Failed to reach place pre-pose")
            return False
        time.sleep(0.5)

        # Step 2: Descend with Pilz LIN to place pose
        dz = place_pose.position.z - place_pre_pose_eef.position.z
        self.get_logger().info(
            f"Step 2: Descending to place pose (Pilz LIN) -> with dz={dz:.3f} m -> "
        )
        if not self.HIGH_LEVEL_move_lin_relative(dz=dz):
            self.get_logger().error("Failed to descend to place pose")
            return False
        time.sleep(0.5)

        # Step 3: Open gripper (release object)
        self.get_logger().info("Step 3: Opening gripper to release object...")
        if self.get_current_pose():
            current_pose = self.get_current_pose()
            self.get_logger().info(
                f"  Current gripper tip approx: "
                f"{current_pose.position.x+self.GRIPPER_INNER_LENGTH:.3f}, "
                f"{current_pose.position.y:.3f}, {current_pose.position.z:.3f}"
            )
        else:
            self.get_logger().info("  Failed to get current pose for debug")
        if not self.open_gripper():
            self.get_logger().error("Failed to open gripper")
            return False
        time.sleep(1.0)

        # Step 4: Retreat upward using Pilz LIN
        self.get_logger().info(f"Step 4: Retreating upward by {retreat_distance} m (Pilz LIN)...")
        if not self.HIGH_LEVEL_move_lin_relative(dz=retreat_distance):
            self.get_logger().error("Failed to retreat upward")
            return False
        
        # Step 5: Detach object from gripper
        self.get_logger().info("Step 5: Detaching object from gripper...")
        self.detach_object_from_gripper(object_name)

        self.get_logger().info("✓ Place operation completed successfully")
        return True

    def place_pour_object(self, object_name, start_pose, place_pose, pour_angle_degree=45,
                          retreat_distance=0.15):
        """
        Place and pour operation: place the object and perform a pouring motion.

        Args:
            object_name: Name of object to place (collision id)
            start_pose: Starting pose of object before placing
            place_pose: Target place pose
            pour_angle_degree: Angle to rotate for pouring (degrees)
            retreat_distance: Distance to retreat upward after placing (m)
        Returns:
            bool: True if successful
        
        """

        self.get_logger().info(f"Starting place operation for '{object_name}'")

        # Step 1: Move to place pre-pose (above) using PTP
        dx = place_pose.position.x - start_pose.position.x
        dy = place_pose.position.y - start_pose.position.y
        # dz = place_pose.position.z - start_pose.position.z

        # current_pose = self.get_current_pose()
        place_pre_pose_eef = self.get_current_pose()
        place_pre_pose_eef.position.x += dx
        place_pre_pose_eef.position.y += dy

        # place_pre_pose_eef.position.z += retreat_distance  # go above by retreat_distance

        self.get_logger().info(
            f"Step 1: Moving to place pre-pose x, y (Pilz PTP) -> "
            f"{place_pre_pose_eef.position.x:.3f}, {place_pre_pose_eef.position.y:.3f}, {place_pre_pose_eef.position.z:.3f}"
        )

        if not self.HIGH_LEVEL_move_lin_relative(dx=dx, dy=dy):
            self.get_logger().error("Failed to reach place pre-pose")
            return False
        time.sleep(0.5)


        # Step 2: Descend with Pilz LIN to pour height
        z_pour = place_pose.position.z + 0.15
        dz = z_pour - place_pre_pose_eef.position.z
        self.get_logger().info(
            f"Step 2: Descending to pour height (Pilz LIN) -> with dz={dz:.3f} m -> "
        )
        if not self.HIGH_LEVEL_move_lin_relative(dz=dz):
            self.get_logger().error("Failed to descend to pour height")
            return False
        time.sleep(0.5)

        #Step 3: Pouring motion
        if not self.HIGH_LEVEL_pour(pour_angle_degree):
            self.get_logger().error("Pouring motion failed")
            return False
        
        #Step 3: Descend with Pilz LIN to place pose
        dz = place_pose.position.z - self.get_current_pose().position.z
        self.get_logger().info(
            f"Step 3: Descending to place pose (Pilz LIN) -> with dz={dz:.3f} m -> "
        )
        if not self.HIGH_LEVEL_move_lin_relative(dz=dz):
            self.get_logger().error("Failed to descend to place pose")
            return False
        time.sleep(0.5)

        # Step 4: Open gripper (release object)
        self.get_logger().info("Step 4: Opening gripper to release object...")
        if self.get_current_pose():
            current_pose = self.get_current_pose()
            self.get_logger().info(
                f"  Current gripper tip approx: "
                f"{current_pose.position.x+self.GRIPPER_INNER_LENGTH:.3f}, "
                f"{current_pose.position.y:.3f}, {current_pose.position.z:.3f}"
            )
        else:
            self.get_logger().info("  Failed to get current pose for debug")
        if not self.open_gripper():
            self.get_logger().error("Failed to open gripper")
            return False
        time.sleep(1.0)

        # Step 5: Retreat upward using Pilz LIN
        self.get_logger().info(f"Step 5: Retreating upward by {retreat_distance} m (Pilz LIN)...")
        if not self.HIGH_LEVEL_move_lin_relative(dz=retreat_distance):
            self.get_logger().error("Failed to retreat upward")
            return False

        # Step 6: Detach object from gripper
        self.get_logger().info("Step 6: Detaching object from gripper...")
        self.detach_object_from_gripper(object_name)

        self.get_logger().info("✓ Place operation completed successfully")
        return True

