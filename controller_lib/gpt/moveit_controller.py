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
from moveit_msgs.msg import MotionPlanRequest, PlanningOptions
from moveit_msgs.srv import GetMotionPlan
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
        for attempt in range(1, max_retries + 1):
            # Gradually relax constraints each attempt
            box = 0.02 + 0.01 * (attempt - 1)      # 2cm → 5cm
            ori_tol = 0.005 + 0.005 * (attempt - 1)  # 0.005rad → 0.02rad

            goal = MoveGroup.Goal()
            goal.request.group_name = self.arm_group_name
            goal.request.num_planning_attempts = 12
            goal.request.allowed_planning_time = planning_time
            goal.request.max_velocity_scaling_factor = 0.15
            goal.request.max_acceleration_scaling_factor = 0.15

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
    
    # ==================== PILZ LIN PLANNER (STRAIGHT LINE) ====================
    

    def move_linear_pilz(self, target_pose, velocity_scaling=0.1, acceleration_scaling=0.1):
        """
        Use Pilz LIN planner for straight-line Cartesian motion
        
        Args:
            target_pose: Target pose
            velocity_scaling: Velocity scaling (0.0-1.0)
            acceleration_scaling: Acceleration scaling (0.0-1.0)
        
        Returns:
            bool: True if successful
        """
        self.get_logger().info("Planning linear motion with Pilz LIN planner...")
        
        # Create motion plan request
        request = GetMotionPlan.Request()
        
        # Configure request
        request.motion_plan_request.workspace_parameters.header.frame_id = self.base_frame
        request.motion_plan_request.workspace_parameters.header.stamp = self.get_clock().now().to_msg()
        
        request.motion_plan_request.group_name = self.arm_group_name
        request.motion_plan_request.num_planning_attempts = 10
        request.motion_plan_request.allowed_planning_time = 5.0
        request.motion_plan_request.max_velocity_scaling_factor = velocity_scaling
        request.motion_plan_request.max_acceleration_scaling_factor = acceleration_scaling
        
        # CRITICAL: Specify Pilz planner
        request.motion_plan_request.planner_id = "LIN"  # Linear motion
        request.motion_plan_request.pipeline_id = "pilz_industrial_motion_planner"
        
        # Set goal constraint
        constraints = Constraints()
        
        # Position constraint
        pos_constraint = PositionConstraint()
        pos_constraint.header.frame_id = self.base_frame
        pos_constraint.link_name = self.end_effector_frame
        pos_constraint.constraint_region.primitive_poses.append(target_pose)
        
        pos_prim = SolidPrimitive()
        pos_prim.type = SolidPrimitive.BOX
        pos_prim.dimensions = [0.001, 0.001, 0.001]  # Tight tolerance for Pilz
        pos_constraint.constraint_region.primitives.append(pos_prim)
        pos_constraint.weight = 1.0
        constraints.position_constraints.append(pos_constraint)
        
        # Orientation constraint
        orient_constraint = OrientationConstraint()
        orient_constraint.header.frame_id = self.base_frame
        orient_constraint.link_name = self.end_effector_frame
        orient_constraint.orientation = target_pose.orientation
        orient_constraint.absolute_x_axis_tolerance = 0.001
        orient_constraint.absolute_y_axis_tolerance = 0.001
        orient_constraint.absolute_z_axis_tolerance = 0.001
        orient_constraint.weight = 1.0
        constraints.orientation_constraints.append(orient_constraint)
        
        request.motion_plan_request.goal_constraints.append(constraints)
        
        # Call planning service
        if not self.pilz_planning_client.wait_for_service(timeout_sec=2.0):
            self.get_logger().error("Pilz planning service not available!")
            return False
        
        future = self.pilz_planning_client.call_async(request)
        rclpy.spin_until_future_complete(self, future, timeout_sec=10.0)
        
        if not future.done():
            self.get_logger().error("Pilz planning timeout")
            return False
        
        response = future.result()
        
        if response.motion_plan_response.error_code.val != MoveItErrorCodes.SUCCESS:
            error_str = self._error_to_string(response.motion_plan_response.error_code.val)
            self.get_logger().error(f"Pilz planning failed: {error_str}")
            return False
        
        self.get_logger().info("✓ Pilz LIN plan computed, executing...")
        
        # Execute the trajectory
        return self.execute_trajectory(
            response.motion_plan_response.trajectory,
            max_velocity_scaling=velocity_scaling
        )
    
    def retreat_linear_pilz(self, distance=0.4, velocity_scaling=0.15):
        """
        Retreat using Pilz LIN planner - guaranteed straight line!
        
        Args:
            distance: Distance to retreat (m)
            velocity_scaling: Speed (0.0-1.0)
        
        Returns:
            bool: True if successful
        """
        self.get_logger().info(f"🎯 Pilz LIN retreat: {distance}m at {velocity_scaling*100:.0f}% speed")
        
        current_pose = self.get_current_pose()
        if not current_pose:
            return False
        
        # Create target pose (straight up)
        target_pose = Pose()
        target_pose.position.x = current_pose.position.x
        target_pose.position.y = current_pose.position.y
        target_pose.position.z = current_pose.position.z + distance
        target_pose.orientation = current_pose.orientation
        
        self.get_logger().info(
            f"Moving from Z={current_pose.position.z:.3f}m to Z={target_pose.position.z:.3f}m"
        )
        
        # Use Pilz LIN planner
        success = self.move_linear_pilz(
            target_pose,
            velocity_scaling=velocity_scaling,
            acceleration_scaling=velocity_scaling
        )
        
        if success:
            final_pose = self.get_current_pose()
            if final_pose:
                error = abs(final_pose.position.z - target_pose.position.z)
                self.get_logger().info(
                    f"✓ Pilz retreat complete: Z={final_pose.position.z:.3f}m, "
                    f"error={error*1000:.1f}mm"
                )
        
        return success
    
    def retreat_smooth(self, distance=0.4, segment_size=0.06):
        """
        Smooth retreat using multiple small Cartesian segments
        Each segment is small enough that Cartesian planning succeeds
        
        Args:
            distance: Total distance to retreat (m)
            segment_size: Size of each segment (m) - smaller = more guaranteed success
        
        Returns:
            bool: True if successful
        """
        current_pose = self.get_current_pose()
        if not current_pose:
            return False
        
        # Calculate number of segments
        num_segments = max(1, int(distance / segment_size))
        actual_segment = distance / num_segments
        
        self.get_logger().info(
            f"Smooth retreat: {distance}m in {num_segments} segments of {actual_segment:.3f}m"
        )
        
        start_z = current_pose.position.z
        
        for i in range(1, num_segments + 1):
            # Calculate target for this segment
            target_z = start_z + (actual_segment * i)
            
            # Create waypoint
            waypoint = Pose()
            waypoint.position.x = current_pose.position.x
            waypoint.position.y = current_pose.position.y
            waypoint.position.z = target_z
            waypoint.orientation = current_pose.orientation
            
            # Try Cartesian first (should succeed with small segments)
            success = self.move_cartesian(
                waypoints=[waypoint],
                eef_step=0.01,  # 1cm steps
                jump_threshold=1.5,  # Allow some flexibility
                max_velocity_scaling=0.2
            )
            
            if not success:
                self.get_logger().warn(
                    f"Cartesian failed at segment {i}/{num_segments}, using standard planning"
                )
                # Fallback to standard planning
                if not self.move_to_pose(waypoint, planning_time=2.0):
                    self.get_logger().error(f"Failed at segment {i}/{num_segments}")
                    return False
            
            # Brief pause for stability (optional)
            if i < num_segments:
                time.sleep(0.05)
            
            # Progress update
            if i % 2 == 0 or i == num_segments:
                current = self.get_current_pose()
                if current:
                    actual_z = current.position.z
                    progress = (i / num_segments) * 100
                    self.get_logger().info(
                        f"  Progress: {i}/{num_segments} ({progress:.0f}%) - Z={actual_z:.3f}m"
                    )
        
        # Verify final position
        final_pose = self.get_current_pose()
        if final_pose:
            final_z = final_pose.position.z
            target_z = start_z + distance
            error = abs(final_z - target_z)
            self.get_logger().info(
                f"✓ Retreat complete: Z={final_z:.3f}m (error: {error*1000:.1f}mm)"
            )
            return error < 0.03  # Accept 3cm tolerance
        
        return True
    
    def move_to_pose_smooth(self, target_pose, segment_size=0.10):
        """
        Move to target pose using multi-segment Cartesian for smoother motion
        
        Args:
            target_pose: Target pose
            segment_size: Size of each segment (m)
        
        Returns:
            bool: True if successful
        """
        current_pose = self.get_current_pose()
        if not current_pose:
            return False
        
        # Calculate deltas
        dx = target_pose.position.x - current_pose.position.x
        dy = target_pose.position.y - current_pose.position.y
        dz = target_pose.position.z - current_pose.position.z
        distance = (dx**2 + dy**2 + dz**2)**0.5
        
        # Calculate segments
        num_segments = max(1, int(distance / segment_size))
        
        self.get_logger().info(
            f"Moving to place pose: {distance:.3f}m in {num_segments} segments"
        )
        
        start_x = current_pose.position.x
        start_y = current_pose.position.y
        start_z = current_pose.position.z
        
        for i in range(1, num_segments + 1):
            # Calculate intermediate waypoint
            progress = i / num_segments
            waypoint = Pose()
            waypoint.position.x = start_x + (dx * progress)
            waypoint.position.y = start_y + (dy * progress)
            waypoint.position.z = start_z + (dz * progress)
            waypoint.orientation = target_pose.orientation  # Keep target orientation
            
            # Try Cartesian first
            success = self.move_cartesian(
                waypoints=[waypoint],
                eef_step=0.01,
                jump_threshold=2.0,
                max_velocity_scaling=0.2
            )
            
            if not success:
                self.get_logger().warn(f"Cartesian failed at segment {i}, using standard planning")
                if not self.move_to_pose(waypoint, planning_time=2.0):
                    self.get_logger().error(f"Failed at segment {i}/{num_segments}")
                    return False
            
            # Progress update
            if i % 2 == 0 or i == num_segments:
                self.get_logger().info(f"  Progress: {i}/{num_segments} ({(i/num_segments)*100:.0f}%)")
        
        return True
    
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
                    self.gripper_max_close_left * 0.8,
                    self.gripper_max_close_right * 0.8
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
    
    def close_gripper(self, grip_force=0.8):
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
    
    # ==================== OBJECT ATTACHMENT ====================
    
    def attach_object_to_gripper(self, object_name):
        """Attach object to gripper for collision checking"""
        attached_object = AttachedCollisionObject()
        attached_object.link_name = self.end_effector_frame
        attached_object.object.id = object_name
        attached_object.object.operation = CollisionObject.ADD
        
        attached_object.touch_links = [
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
        
        planning_scene = PlanningScene()
        planning_scene.robot_state.attached_collision_objects.append(attached_object)
        planning_scene.is_diff = True
        planning_scene.robot_state.is_diff = True
        
        self.planning_scene_pub.publish(planning_scene)
        time.sleep(0.3)
        
        self.get_logger().info(f"✓ Attached '{object_name}' to gripper")
        return True
    
    def detach_object_from_gripper(self, object_name):
        """Detach object from gripper - FIXED VERSION"""
        self.get_logger().info(f"Detaching '{object_name}' from gripper...")
        
        # Create attached collision object message
        attached_object = AttachedCollisionObject()
        attached_object.link_name = self.end_effector_frame
        
        # The object to detach
        collision_object = CollisionObject()
        collision_object.id = object_name
        collision_object.operation = CollisionObject.REMOVE
        
        attached_object.object = collision_object
        
        # Create planning scene diff
        planning_scene = PlanningScene()
        planning_scene.is_diff = True
        planning_scene.robot_state.is_diff = True  # ← THIS WAS MISSING!
        planning_scene.robot_state.attached_collision_objects.append(attached_object)
        
        # Publish multiple times to ensure receipt
        self.get_logger().info(f"Publishing detach for '{object_name}'...")
        for i in range(5):
            self.planning_scene_pub.publish(planning_scene)
            self.get_logger().info(f"  Published detach message {i+1}/5")
            time.sleep(0.2)
        
        self.get_logger().info(f"✓ Sent detach request for '{object_name}'")
        time.sleep(1.0)
        
        return True
    
    #======================GPT ATTACH DETACH FIX=========================
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
    
    def pick_object(self, object_name, grasp_pose, approach_direction="side", 
                    approach_distance=0.02, grasp_distance=0.015, 
                    lift_distance=0.2, grip_force=0.6):
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
        self.use_pilz_ptp()
        if not self.move_to_pose(approach_pose):
            self.use_default_planner()
            self.get_logger().error("Failed to reach approach pose")
            return False
        self.use_default_planner()

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

        if not self.move_lin_relative(dx=delta_x, dz=delta_z):
            self.get_logger().error("Pilz LIN approach failed")
            return False

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
        if not self.lift_with_pilz(lift_height=lift_distance, linear=True):
            self.get_logger().error("Failed to lift object")
            return False

        self.get_logger().info("✓ Pick operation completed successfully")
        return True


    
    def place_object(self, object_name, place_pose, retreat_distance=0.2):
        """
        Execute complete place operation
        
        Args:
            object_name: Name of object to place
            place_pose: Target place pose
            retreat_distance: Distance to retreat after placing (m)
        
        Returns:
            bool: True if successful
        """
        self.get_logger().info(f"Starting place operation for '{object_name}'")
        
        # Step 1: Move to place pose
        self.get_logger().info("Step 1: Moving to place pose...")
        current_pose = self.get_current_pose()
        dx = place_pose.position.x - current_pose.position.x
        dy = place_pose.position.y - current_pose.position.y
        dz = place_pose.position.z - current_pose.position.z
        if self.cartesian_available and not self.move_relative_cartesian(delta_x=dx, delta_y=dy, delta_z=dz):
            self.get_logger().warn("Cartesian lift failed, using standard planning")
            if not self.move_to_pose(place_pose):
                return False
        elif not self.cartesian_available:
            if not self.move_to_pose(place_pose):
                return False
            
        # Step 2: Open gripper
        self.get_logger().info("Step 2: Opening gripper to release object...")
        #DEBUG: 
        if self.get_current_pose():
            current_pose = self.get_current_pose()
            self.get_logger().info(f"Current pose of gripper: {current_pose.position.x+self.GRIPPER_INNER_LENGTH:.3f}, {current_pose.position.y:.3f}, {current_pose.position.z:.3f}")
        else:
            self.get_logger().info("Failed to get current pose for debug")
        if not self.open_gripper():
            self.get_logger().error("Failed to open gripper")
            return False
        time.sleep(1.0)
        
        #Step 3: Lower object slightly to ensure release
        # self.get_logger().info("Step 3: Lowering object slightly to ensure release...")
        # if self.cartesian_available and not self.move_relative_cartesian(delta_z= -retreat_distance):
        #     self.get_logger().warn("Cartesian lower failed, using standard planning")
        #     current_pose = self.get_current_pose()
        #     if current_pose:
        #         current_pose.position.z -= retreat_distance
        #         if not self.move_to_pose(current_pose):
        #             return False
        #     else:
        #         return False
        # elif not self.cartesian_available:
        #     current_pose = self.get_current_pose()
        #     if current_pose:
        #         current_pose.position.z -= retreat_distance
        #         if not self.move_to_pose(current_pose):
        #             return False
        #     else:
        #         return False
        # Step 4: Detach object
        self.get_logger().info("Step 4: Detaching object from gripper...")
        self.detach_object_from_gripper(object_name)

        # Step 5: Retreat using Cartesian path
        self.get_logger().info(f"Step 5: Retreating ({retreat_distance}m)...")
        if not self.move_relative_cartesian(delta_z=retreat_distance):
            # Fallback
            current_pose = self.get_current_pose()
            if current_pose:
                current_pose.position.z += retreat_distance
                if not self.move_to_pose(current_pose):
                    return False
        
        self.get_logger().info("✓ Place operation completed successfully")
        return True
    
    def move_to_pose_pilz_ptp(self, target_pose, velocity_scaling=0.2):
        """
        Use Pilz PTP (point-to-point) planner for efficient movement
        PTP doesn't maintain Cartesian straight line, but is faster and more reliable
        """
        self.get_logger().info("Planning with Pilz PTP planner...")
        
        request = GetMotionPlan.Request()
        request.motion_plan_request.workspace_parameters.header.frame_id = self.base_frame
        request.motion_plan_request.workspace_parameters.header.stamp = self.get_clock().now().to_msg()
        request.motion_plan_request.group_name = self.arm_group_name
        request.motion_plan_request.allowed_planning_time = 5.0
        request.motion_plan_request.max_velocity_scaling_factor = velocity_scaling
        request.motion_plan_request.max_acceleration_scaling_factor = velocity_scaling
        
        # Use PTP instead of LIN
        request.motion_plan_request.planner_id = "PTP"  # Point-to-point
        request.motion_plan_request.pipeline_id = "pilz_industrial_motion_planner"
        
        # Add goal constraints (similar to move_linear_pilz but with PTP)
        constraints = Constraints()
        
        pos_constraint = PositionConstraint()
        pos_constraint.header.frame_id = self.base_frame
        pos_constraint.link_name = self.end_effector_frame
        pos_constraint.constraint_region.primitive_poses.append(target_pose)
        
        pos_prim = SolidPrimitive()
        pos_prim.type = SolidPrimitive.BOX
        pos_prim.dimensions = [0.01, 0.01, 0.01]
        pos_constraint.constraint_region.primitives.append(pos_prim)
        pos_constraint.weight = 1.0
        constraints.position_constraints.append(pos_constraint)
        
        orient_constraint = OrientationConstraint()
        orient_constraint.header.frame_id = self.base_frame
        orient_constraint.link_name = self.end_effector_frame
        orient_constraint.orientation = target_pose.orientation
        orient_constraint.absolute_x_axis_tolerance = 0.1
        orient_constraint.absolute_y_axis_tolerance = 0.1
        orient_constraint.absolute_z_axis_tolerance = 0.1
        orient_constraint.weight = 1.0
        constraints.orientation_constraints.append(orient_constraint)
        
        request.motion_plan_request.goal_constraints.append(constraints)
        
        if not self.pilz_planning_client.wait_for_service(timeout_sec=2.0):
            return False
        
        future = self.pilz_planning_client.call_async(request)
        rclpy.spin_until_future_complete(self, future, timeout_sec=10.0)
        
        if not future.done() or future.result().motion_plan_response.error_code.val != MoveItErrorCodes.SUCCESS:
            return False
        
        return self.execute_trajectory(
            future.result().motion_plan_response.trajectory,
            max_velocity_scaling=velocity_scaling
        )

    def place_object_with_start(self, object_name, start_pose, place_pose, retreat_distance=0.2):
        """
        Execute complete place operation
        
        Args:
            object_name: Name of object to place
            start_pose: Starting pose of object before placing
            place_pose: Target place pose
            retreat_distance: Distance to retreat after placing (m)
        
        Returns:
            bool: True if successful
        """
        self.get_logger().info(f"Starting place operation for '{object_name}'")
    
        # Step 1: Move to place pose with multi-segment
        self.get_logger().info("Step 1: Moving to place pose...")
        
        # Calculate distance to decide approach
        dx = place_pose.position.x - start_pose.position.x
        dy = place_pose.position.y - start_pose.position.y
        dz = place_pose.position.z - start_pose.position.z
        distance = (dx**2 + dy**2 + dz**2)**0.5

        current_pose = self.get_current_pose()
        target_pose = self.get_current_pose()
        target_pose.position.x += dx
        target_pose.position.y += dy
        target_pose.position.z += dz

        if not self.move_to_pose_pilz_ptp(target_pose, velocity_scaling=0.2):
            self.get_logger().warn("Pilz PTP move failed, using multi-segment approach")
            if not self.move_linear_pilz(target_pose, velocity_scaling=0.15):
                self.get_logger().warn("Pilz LIN move failed, using multi-segment Cartesian")
                if not self.move_relative_cartesian(delta_x=dx, delta_y=dy, delta_z=dz):
                    self.get_logger().error("Multi-segment move failed, using standard planning")
                    if not self.move_to_pose(place_pose):
                        return False

        # Step 2: Open gripper
        self.get_logger().info("Step 2: Opening gripper to release object...")
        #DEBUG: 
        if self.get_current_pose():
            current_pose = self.get_current_pose()
            self.get_logger().info(f"Current pose of gripper: {current_pose.position.x+self.GRIPPER_INNER_LENGTH:.3f}, {current_pose.position.y:.3f}, {current_pose.position.z:.3f}")
        else:
            self.get_logger().info("Failed to get current pose for debug")
        if not self.open_gripper():
            self.get_logger().error("Failed to open gripper")
            return False
        time.sleep(1.0)

      
        # Step 4: Retreat using Cartesian path
        self.get_logger().info(f"Step 4: Pilz LIN retreat ({retreat_distance}m)...")
        if not self.retreat_linear_pilz(distance=retreat_distance, velocity_scaling=0.15):
            self.get_logger().warn("Pilz retreat failed, using fallback")
            # Fallback to multi-segment
            if not self.retreat_smooth(distance=retreat_distance, segment_size=0.06):
                self.get_logger().error("Smooth retreat methods failed. Using standard planning")
                current_pose = self.get_current_pose()
                current_pose.position.z += retreat_distance
                if not self.move_to_pose(current_pose):
                    self.get_logger().error("Standard planning failed")
                    return False
                

          # Step 3: Detach object
        self.get_logger().info("Step 3: Detaching object from gripper...")
        self.detach_object_from_gripper(object_name)

        
        self.get_logger().info("✓ Place operation completed successfully")
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
        dz = place_pose.position.z - start_pose.position.z
        distance = (dx**2 + dy**2 + dz**2)**0.5

        # current_pose = self.get_current_pose()
        place_pre_pose_eef = self.get_current_pose()
        place_pre_pose_eef.position.x += dx
        place_pre_pose_eef.position.y += dy

        # place_pre_pose_eef.position.z += retreat_distance  # go above by retreat_distance

        self.get_logger().info(
            f"Step 1: Moving to place pre-pose (Pilz PTP) -> "
            f"{place_pre_pose_eef.position.x:.3f}, {place_pre_pose_eef.position.y:.3f}, {place_pre_pose_eef.position.z:.3f}"
        )

        if not self.move_lin_relative(dx=dx, dy=dy):
            self.get_logger().warn("Cartesian move failed, using Pilz PTP")
            self.use_pilz_ptp()
            if not self.move_to_pose(place_pre_pose_eef):
                self.use_default_planner()
                if not self.move_to_pose(place_pre_pose_eef):
                    self.get_logger().error("Failed to reach place pre-pose")
                    return False
            self.use_default_planner()

        # Step 2: Descend with Pilz LIN to place pose
        dz = place_pose.position.z - place_pre_pose_eef.position.z
        self.get_logger().info(
            f"Step 2: Descending to place pose (Pilz LIN) -> with dz={dz:.3f} m -> "
        )
        if not self.move_lin_relative(dz=dz):
            self.get_logger().warn("Cartesian descent failed, using Pilz PTP")
            self.use_pilz_ptp()
            if not self.move_to_pose(place_pose):
                self.use_default_planner()
                if not self.move_to_pose(place_pose):
                    self.get_logger().error("Failed to descend to place pose")
                    return False
            self.use_default_planner()

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
        if not self.move_lin_relative(dz=retreat_distance):
            self.get_logger().warn("Pilz LIN retreat failed, attempting fallback")
            current_pose = self.get_current_pose()
            if current_pose:
                self.use_pilz_ptp()
                current_pose.position.z += retreat_distance
                if not self.move_to_pose(current_pose):
                    self.use_default_planner()
                    if not self.move_to_pose(current_pose):
                        self.get_logger().error("Fallback retreat failed")
                        return False
                self.use_default_planner()
            else:
                self.get_logger().error("Failed to get current pose for fallback retreat")
                return False

        # Step 5: Detach object from gripper
        self.get_logger().info("Step 5: Detaching object from gripper...")
        self.detach_object_from_gripper(object_name)

        self.get_logger().info("✓ Place operation completed successfully")
        return True

