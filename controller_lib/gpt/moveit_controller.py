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
from moveit_msgs.msg import  RobotState
from moveit_msgs.srv import GetMotionPlan
from sensor_msgs.msg import JointState
from tf_transformations import quaternion_multiply, quaternion_from_euler
import time
import math
import tf2_ros
import struct
import os

from collision_objects import CollisionObjectMixin


        

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


class MoveItController(Node, CollisionObjectMixin):
    """
    Base class for MoveIt2 control of Kinova 7-DOF arm
    
    Provides:
    - Standard motion planning
    - Cartesian path planning
    - Gripper control
    - Collision object management (via CollisionObjectMixin)
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
        self.last_joint_state_time = None
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
        self.last_joint_state_time = time.time()
    
    def _get_start_state(self):
        """Get current robot state as start state for planning"""
        if self.current_joint_state is None:
            return None
        
        robot_state = RobotState()
        robot_state.joint_state = self.current_joint_state
        return robot_state
    
    def wait_for_joint_state(self, timeout=5.0, max_age=2.0, allow_stale=False, require_fresh=False):
        """
        Wait for joint state to be available and recent.
        
        Args:
            timeout: Maximum time to wait for joint state (seconds)
            max_age: Maximum age of joint state to consider valid (seconds)
            allow_stale: If True, allow stale joint state (useful for recovery scenarios)
            require_fresh: If True, MUST have fresh joint state (max_age must be met), ignore allow_stale
        
        Returns:
            bool: True if joint state is available and recent, False otherwise
        """
        if not hasattr(self, 'last_joint_state_time'):
            self.last_joint_state_time = None
        
        start_time = time.time()
        check_count = 0
        while time.time() - start_time < timeout:
            if self.current_joint_state is not None:
                if self.last_joint_state_time is not None:
                    age = time.time() - self.last_joint_state_time
                    if age <= max_age:
                        self.get_logger().debug(f"Joint state available (age: {age:.2f}s)")
                        return True
                    elif require_fresh:
                        # If require_fresh is True, don't accept stale states
                        if check_count % 5 == 0:  # Log every 0.5s
                            self.get_logger().warn(f"Waiting for fresh joint state (age: {age:.2f}s, max: {max_age}s)...")
                    elif allow_stale:
                        # Allow stale joint state if explicitly requested (for recovery)
                        self.get_logger().warn(f"Using stale joint state (age: {age:.2f}s) - hardware may be recovering")
                        return True
                    else:
                        # Only log warning, don't spam
                        if check_count % 5 == 0:  # Log every 0.5s
                            self.get_logger().warn(f"Joint state too old (age: {age:.2f}s, max: {max_age}s)")
                else:
                    # If we have joint state but no timestamp, assume it's recent
                    self.get_logger().debug("Joint state available (no timestamp)")
                    return True
            
            # Spin to receive joint state messages
            rclpy.spin_once(self, timeout_sec=0.1)
            time.sleep(0.1)
            check_count += 1
        
        # If require_fresh is True, never use stale states
        if require_fresh:
            if self.last_joint_state_time is not None:
                age = time.time() - self.last_joint_state_time
                self.get_logger().error(
                    f"Fresh joint state not available after {timeout}s - hardware driver may be down. "
                    f"Latest state age: {age:.1f}s (needs to be < {max_age}s)"
                )
            else:
                self.get_logger().error(
                    f"Fresh joint state not available after {timeout}s - hardware driver may be down. "
                    f"No joint state received yet."
                )
            return False
        
        # If we have joint state but it's stale, and allow_stale is True, use it anyway
        if allow_stale and self.current_joint_state is not None:
            self.get_logger().warn("Timeout waiting for fresh joint state, but using stale state for recovery")
            return True
        
        self.get_logger().error(f"Joint state not available after {timeout}s - hardware may be unresponsive")
        return False
    
    def wait_for_fresh_joint_state(self, timeout=10.0, max_age=1.0):
        """
        Wait for fresh joint state (strict requirement - must be very recent).
        This is required before MoveIt operations to ensure trajectory validation works.
        NOTE: MoveIt internally requires joint states within 1.0 seconds, so max_age should be <= 1.0.
        
        Args:
            timeout: Maximum time to wait (seconds)
            max_age: Maximum age of joint state (seconds) - should be < 1s for MoveIt
        
        Returns:
            bool: True if fresh joint state available, False otherwise
        """
        return self.wait_for_joint_state(timeout=timeout, max_age=max_age, allow_stale=False, require_fresh=True)
    
    def check_hardware_status(self):
        """
        Check if hardware driver is responsive by checking joint state freshness.
        
        Returns:
            tuple: (is_ok, age_seconds, status_message)
                - is_ok: True if hardware is responsive
                - age_seconds: Age of latest joint state in seconds (None if no state)
                - status_message: Human-readable status message
        """
        if self.current_joint_state is None:
            return (False, None, "No joint state received - hardware driver may not be running")
        
        if self.last_joint_state_time is None:
            return (False, None, "Joint state received but timestamp unavailable")
        
        age = time.time() - self.last_joint_state_time
        
        if age > 5.0:
            return (False, age, f"Hardware driver appears to be down - joint state is {age:.1f}s old")
        elif age > 1.0:
            return (False, age, f"Hardware driver may be having issues - joint state is {age:.1f}s old")
        else:
            return (True, age, f"Hardware driver is responsive - joint state is {age:.2f}s old")
    
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
    
    def wait_for_motion_completion(self, target_pose=None, position_tolerance=0.02, 
                                   orientation_tolerance=0.1, timeout=30.0, check_interval=0.5):
        """
        Wait for robot motion to actually complete by checking if current pose matches target.
        
        CRITICAL: This function uses TF lookups which are lightweight and don't stress the hardware.
        However, if the hardware driver is having communication issues, this will gracefully fail.
        
        Args:
            target_pose: Target pose to wait for (if None, just wait for motion to settle)
            position_tolerance: Maximum position error to consider motion complete (m)
            orientation_tolerance: Maximum orientation error to consider motion complete (rad)
            timeout: Maximum time to wait (seconds)
            check_interval: Time between pose checks (seconds) - increased to reduce load
        
        Returns:
            bool: True if motion completed, False if timeout or error
        """
        start_time = time.time()
        last_pose = None
        stable_count = 0
        required_stable_checks = 2  # Reduced from 3 to 2 for faster completion
        consecutive_errors = 0
        max_errors = 5  # Allow some TF lookup failures
        
        while time.time() - start_time < timeout:
            try:
                current_pose = self.get_current_pose()
                if current_pose is None:
                    consecutive_errors += 1
                    if consecutive_errors >= max_errors:
                        self.get_logger().warn("Too many TF lookup failures, assuming motion complete")
                        return True  # Assume complete if TF unavailable
                    time.sleep(check_interval)
                    continue
                
                consecutive_errors = 0  # Reset error count on success
                
                # If target pose provided, check if we're close enough
                if target_pose is not None:
                    pos_error = math.sqrt(
                        (current_pose.position.x - target_pose.position.x)**2 +
                        (current_pose.position.y - target_pose.position.y)**2 +
                        (current_pose.position.z - target_pose.position.z)**2
                    )
                    
                    # Check orientation error (quaternion distance)
                    q1 = [current_pose.orientation.x, current_pose.orientation.y,
                          current_pose.orientation.z, current_pose.orientation.w]
                    q2 = [target_pose.orientation.x, target_pose.orientation.y,
                          target_pose.orientation.z, target_pose.orientation.w]
                    # Quaternion dot product (closer to 1 = more similar)
                    q_dot = abs(q1[0]*q2[0] + q1[1]*q2[1] + q1[2]*q2[2] + q1[3]*q2[3])
                    ori_error = 2 * math.acos(min(1.0, q_dot))
                    
                    if pos_error <= position_tolerance and ori_error <= orientation_tolerance:
                        stable_count += 1
                        if stable_count >= required_stable_checks:
                            return True
                    else:
                        stable_count = 0
                else:
                    # No target pose - just check if pose is stable (not moving)
                    if last_pose is not None:
                        pos_change = math.sqrt(
                            (current_pose.position.x - last_pose.position.x)**2 +
                            (current_pose.position.y - last_pose.position.y)**2 +
                            (current_pose.position.z - last_pose.position.z)**2
                        )
                        if pos_change < 0.001:  # Less than 1mm movement
                            stable_count += 1
                            if stable_count >= required_stable_checks:
                                return True
                        else:
                            stable_count = 0
                    last_pose = current_pose
                
                time.sleep(check_interval)
            except Exception as e:
                # If TF lookup fails, don't crash - just log and continue
                self.get_logger().debug(f"TF lookup error in wait_for_motion_completion: {e}")
                consecutive_errors += 1
                if consecutive_errors >= max_errors:
                    self.get_logger().warn("Too many errors in motion completion check, assuming complete")
                    return True  # Assume complete to avoid blocking
                time.sleep(check_interval)
        
        self.get_logger().warn(f"Motion completion timeout after {timeout}s")
        return False
    
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
        # CRITICAL: Wait for FRESH joint state before planning
        # MoveIt requires joint states < 1 second old for trajectory validation
        # If hardware driver is down, this will fail early with clear error
        if not self.wait_for_fresh_joint_state(timeout=5.0, max_age=1.0):
            self.get_logger().error(
                "Cannot plan motion - fresh joint state not available. "
                "Hardware driver may be down or robot communication lost. "
                "Please restart the robot launch file."
            )
            return False
        
        # Adjust parameters for Pilz planners (they need more time and attempts)
        is_pilz = (self.planning_pipeline_id == "pilz_industrial_motion_planner")
        is_pilz_lin = (is_pilz and self.planner_id == "LIN")
        if is_pilz:
            planning_time = max(planning_time, 15.0)  # More time for Pilz
            max_planning_attempts = 50  # More attempts for Pilz
            # Start with more relaxed constraints for Pilz
            if is_pilz_lin:
                # Even more relaxed for LIN (linear motions need more tolerance)
                initial_box = 0.15  # Start at 15cm for LIN
                initial_ori_tol = 0.10  # Start at 0.10rad for LIN
            else:
                initial_box = 0.10  # Start at 10cm for PTP
                initial_ori_tol = 0.05  # Start at 0.05rad for PTP
            max_retries = 3  # More retries for Pilz
        else:
            max_planning_attempts = 12
            initial_box = 0.02
            initial_ori_tol = 0.005
        
        for attempt in range(1, max_retries + 1):
            # Relax constraints each attempt
            if is_pilz:
                box = initial_box + 0.05 * (attempt - 1)  # Bigger steps for Pilz
                ori_tol = initial_ori_tol + 0.03 * (attempt - 1)  # More relaxed orientation
            else:
                box = initial_box + 0.01 * (attempt - 1)
                ori_tol = initial_ori_tol + 0.005 * (attempt - 1)

            goal = MoveGroup.Goal()
            goal.request.group_name = self.arm_group_name
            goal.request.num_planning_attempts = max_planning_attempts
            goal.request.allowed_planning_time = planning_time
            # Use slower speeds for Pilz LIN to improve success rate
            if is_pilz_lin:
                goal.request.max_velocity_scaling_factor = 0.10
                goal.request.max_acceleration_scaling_factor = 0.3
            else:
                goal.request.max_velocity_scaling_factor = 0.15
                goal.request.max_acceleration_scaling_factor = 0.3

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
                # CRITICAL: Wait for actual motion completion, not just planning success
                # MoveIt returns SUCCESS when trajectory is sent, not when robot finishes
                try:
                    # Increased timeout from 15s to 20s for slow hardware driver
                    motion_complete = self.wait_for_motion_completion(target_pose=target_pose, timeout=20.0, check_interval=0.5)
                    if not motion_complete:
                        # Motion timed out - check if robot actually reached target
                        self.get_logger().warn("Motion completion verification timed out, checking final position...")
                        current_pose = self.get_current_pose()
                        if current_pose is not None and target_pose is not None:
                            pos_error = math.sqrt(
                                (current_pose.position.x - target_pose.position.x)**2 +
                                (current_pose.position.y - target_pose.position.y)**2 +
                                (current_pose.position.z - target_pose.position.z)**2
                            )
                            if pos_error > 0.05:  # More than 5cm off target
                                self.get_logger().error(
                                    f"Robot did NOT reach target! Position error: {pos_error:.3f}m. "
                                    f"Hardware driver may have crashed. Check ros2_control_node."
                                )
                                # Wait for hardware to potentially recover
                                time.sleep(3.0)
                                return False
                            else:
                                self.get_logger().info(f"Robot is close to target (error: {pos_error:.3f}m), continuing...")
                        else:
                            self.get_logger().warn("Could not verify final position, waiting for recovery...")
                            # Don't fail immediately - give hardware time to recover
                            time.sleep(3.0)
                            # Try to continue if we can get fresh joint states
                            if self.wait_for_fresh_joint_state(timeout=5.0, max_age=1.0):
                                self.get_logger().info("Hardware recovered, continuing...")
                            else:
                                return False
                except Exception as e:
                    self.get_logger().warn(f"Pose verification had issues: {e}")
                    # Try to get current pose to check if we're close
                    time.sleep(1.0)
                
                # CRITICAL: Add delay after motion to allow hardware driver to recover
                # The hardware driver may timeout during feedback refresh, so give it time
                # Increased from 0.5s to 1.0s
                time.sleep(1.0)
                
                # Wait for fresh joint states before returning - this ensures the next
                # operation doesn't fail due to stale states
                for _ in range(20):  # 2 seconds of spinning
                    rclpy.spin_once(self, timeout_sec=0.1)
                    time.sleep(0.1)
                
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
        # CRITICAL: Wait for FRESH joint state before planning
        # MoveIt requires joint states < 1 second old for trajectory validation
        if not self.wait_for_fresh_joint_state(timeout=5.0, max_age=1.0):
            self.get_logger().error(
                "Cannot plan motion - fresh joint state not available. "
                "Hardware driver may be down or robot communication lost. "
                "Please restart the robot launch file."
            )
            return False
        
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
                       avoid_collisions=True, max_velocity_scaling=0.005):
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
        
        # Be more lenient to ensure success - accept any reasonable path
        if fraction < 0.70:
            self.get_logger().warn(
                f"Only {fraction*100:.1f}% of Cartesian path achieved")
            if fraction < 0.30:
                self.get_logger().error("Less than 30% of path computed - aborting")
                return False
        
        # Even partial paths are useful for linear motion
        if fraction < 1.0:
            self.get_logger().warn(f"Executing partial Cartesian path ({fraction*100:.1f}%)")
        
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
        
        # CRITICAL: Check hardware is responsive before executing trajectory
        if not self.wait_for_fresh_joint_state(timeout=5.0, max_age=1.0):
            self.get_logger().error(
                "Cannot execute trajectory - fresh joint state not available. "
                "Hardware driver may be down."
            )
            return False
        
        # Create goal
        goal = ExecuteTrajectory.Goal()
        goal.trajectory = trajectory
        
        # Send goal
        future = self.execute_trajectory_client.send_goal_async(goal)
        rclpy.spin_until_future_complete(self, future, timeout_sec=5.0)
        
        if not future.done():
            self.get_logger().error("Execute trajectory goal send timed out")
            return False
        
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().error("Execute trajectory goal rejected")
            return False
        
        self.get_logger().info("Trajectory execution started...")
        
        # CRITICAL: Calculate expected execution time and add buffer
        # Trajectories can take longer than MoveIt expects, especially with slow velocities
        if trajectory.joint_trajectory.points:
            last_point = trajectory.joint_trajectory.points[-1]
            if last_point.time_from_start:
                expected_time = last_point.time_from_start.sec + last_point.time_from_start.nanosec / 1e9
                timeout = max(expected_time * 2.0, 30.0)  # At least 2x expected time, minimum 30s
            else:
                timeout = 30.0  # Default timeout if no time info
        else:
            timeout = 30.0
        
        self.get_logger().info(f"Waiting for trajectory execution (timeout: {timeout:.1f}s)...")
        
        # Wait for result with longer timeout
        result_future = goal_handle.get_result_async()
        rclpy.spin_until_future_complete(self, result_future, timeout_sec=timeout)
        
        if not result_future.done():
            self.get_logger().error(
                f"Trajectory execution timed out after {timeout:.1f}s. "
                "Hardware driver may be experiencing timeouts. "
                "Waiting for hardware to recover..."
            )
            # CRITICAL: Wait for hardware to recover after timeout
            time.sleep(3.0)
            if not self.wait_for_fresh_joint_state(timeout=5.0, max_age=1.0):
                self.get_logger().error("Hardware driver not recovering after trajectory timeout")
            return False
        
        result = result_future.result()
        error_code = result.result.error_code.val
        success = error_code == MoveItErrorCodes.SUCCESS
        
        if success:
            self.get_logger().info("✓ Cartesian trajectory executed successfully")
            # CRITICAL: Brief wait after successful execution to let hardware stabilize
            time.sleep(0.5)
            return True
        else:
            code_str = self._error_to_string(error_code)
            self.get_logger().error(f"Trajectory execution failed: {code_str} (code {error_code})")
            
            # CRITICAL: If trajectory was cancelled or timed out, wait for hardware recovery
            if error_code in (-4, -6):  # INVALID_ROBOT_STATE or UNKNOWN
                self.get_logger().warn(
                    "Trajectory execution failed with hardware error. "
                    "Waiting for hardware driver to recover..."
                )
                time.sleep(2.0)
                if not self.wait_for_fresh_joint_state(timeout=5.0, max_age=1.0):
                    self.get_logger().error("Hardware driver not recovering after trajectory failure")
            
            return False
    
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
        # CRITICAL: Wait for hardware driver to stabilize before gripper operation
        # The hardware driver often times out right after arm motions
        self.get_logger().info("Waiting for hardware driver to stabilize before gripper operation...")
        
        # First, check current hardware status
        is_ok, age, msg = self.check_hardware_status()
        if not is_ok:
            self.get_logger().warn(f"Hardware status before gripper: {msg}")
        
        # Give hardware driver time to recover from previous motion
        # Spin during the wait to process any pending messages - use longer wait time
        recovery_start = time.time()
        recovery_time = 5.0  # Increased from 3.0s to 5.0s for more reliable recovery
        while time.time() - recovery_start < recovery_time:
            rclpy.spin_once(self, timeout_sec=0.1)
            time.sleep(0.1)
        
        # CRITICAL: Wait for fresh joint states before gripper operation
        # Gripper operations fail with error -4 if joint states are stale
        # Use max_age of 1.0s to match MoveIt's requirement
        max_attempts = 3
        for attempt in range(max_attempts):
            self.get_logger().info(f"Checking for fresh joint states (attempt {attempt + 1}/{max_attempts})...")
            
            # Use max_age of 1.0s - MoveIt requires states within 1 second
            if self.wait_for_fresh_joint_state(timeout=10.0, max_age=1.0):
                self.get_logger().info("✓ Hardware ready, proceeding with gripper operation")
                break
            
            if attempt < max_attempts - 1:
                self.get_logger().warn(
                    f"Fresh joint state not available yet (attempt {attempt + 1}), "
                    f"waiting longer for hardware to recover..."
                )
                # Spin more aggressively to receive messages
                for _ in range(50):  # 5 seconds of spinning
                    rclpy.spin_once(self, timeout_sec=0.1)
                    time.sleep(0.1)
            else:
                self.get_logger().error(
                    "Cannot control gripper - fresh joint state not available after multiple attempts. "
                    "Hardware driver may be down or experiencing persistent timeouts. "
                    "Please check robot connection and consider restarting the robot launch file."
                )
                return False
        
        gripper_client = ActionClient(self, MoveGroup, '/move_action')
        
        if not gripper_client.wait_for_server(timeout_sec=5.0):
            self.get_logger().error("Gripper action server not available!")
            return False
        
        goal = MoveGroup.Goal()
        goal.request.group_name = self.gripper_group_name
        goal.request.num_planning_attempts = 10
        goal.request.allowed_planning_time = 2.0
        # CRITICAL: Slow down gripper to prevent timeout - gripper needs more time
        goal.request.max_velocity_scaling_factor = 0.2  # Reduced from 0.5 to 0.2
        goal.request.max_acceleration_scaling_factor = 0.2  # Reduced from 0.5 to 0.2
        
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
            # CRITICAL: Increase tolerance to make it easier to reach goal and prevent timeout
            joint_constraint.tolerance_above = 0.01  # Increased from 0.005
            joint_constraint.tolerance_below = 0.01  # Increased from 0.005
            joint_constraint.weight = 1.0
            constraints.joint_constraints.append(joint_constraint)
        
        goal.request.goal_constraints.append(constraints)
        
        # CRITICAL: Final check right before sending goal - states can become stale during planning
        self.get_logger().info("Final check: Verifying joint states are still fresh before sending gripper goal...")
        # Use max_age of 1.0s to match MoveIt's requirement
        if not self.wait_for_fresh_joint_state(timeout=3.0, max_age=1.0):
            self.get_logger().warn(
                "Joint states became stale, waiting for hardware to recover..."
            )
            # Try one more recovery
            for _ in range(30):  # 3 seconds of spinning
                rclpy.spin_once(self, timeout_sec=0.1)
                time.sleep(0.1)
            
            if not self.wait_for_fresh_joint_state(timeout=5.0, max_age=1.0):
                self.get_logger().error(
                    "Joint states still stale after recovery attempt. "
                    "Hardware driver may be experiencing timeouts. Aborting gripper operation."
                )
                gripper_client.destroy()
                return False
        
        self.get_logger().info("✓ Joint states confirmed fresh, sending gripper goal...")
        
        future = gripper_client.send_goal_async(goal)
        rclpy.spin_until_future_complete(self, future, timeout_sec=5.0)
        
        if not future.done():
            self.get_logger().error("Gripper goal send timed out")
            gripper_client.destroy()
            return False
        
        goal_handle = future.result()
        if not goal_handle or not goal_handle.accepted:
            self.get_logger().error("Gripper goal rejected")
            gripper_client.destroy()
            return False
        
        # CRITICAL: Check again right before execution - hardware can timeout during planning
        self.get_logger().info("Verifying hardware is still responsive before gripper execution...")
        time.sleep(1.0)  # Brief pause - increased from 0.5s
        # Use max_age of 1.0s to match MoveIt's requirement
        if not self.wait_for_fresh_joint_state(timeout=3.0, max_age=1.0):
            self.get_logger().warn(
                "Joint states became stale during gripper planning. "
                "Hardware driver may be timing out. Attempting recovery..."
            )
            # Try to recover
            for _ in range(30):  # 3 seconds of spinning
                rclpy.spin_once(self, timeout_sec=0.1)
                time.sleep(0.1)
        
        # CRITICAL: Increase timeout for gripper execution - grippers can take 10+ seconds
        result_future = goal_handle.get_result_async()
        rclpy.spin_until_future_complete(self, result_future, timeout_sec=15.0)
        
        if not result_future.done():
            self.get_logger().error("Gripper execution timed out after 15 seconds")
            gripper_client.destroy()
            return False
        
        result = result_future.result()
        gripper_client.destroy()
        
        error_code = result.result.error_code.val
        if error_code == MoveItErrorCodes.SUCCESS:
            # CRITICAL: Brief wait after successful gripper operation to let hardware stabilize
            self.get_logger().info("Gripper operation successful, waiting for hardware to stabilize...")
            time.sleep(1.0)
            return True
        else:
            # Error code -4 means INVALID_ROBOT_STATE - joint states were stale
            if error_code == -4:
                self.get_logger().error(
                    f"Gripper operation failed with error code -4 (INVALID_ROBOT_STATE). "
                    f"This indicates joint states became stale during execution. "
                    f"Hardware driver may have timed out. Please check robot connection."
                )
            else:
                self.get_logger().error(f"Gripper operation failed with error code: {error_code}")
            return False
    
    def open_gripper(self, openness=0.0):
        """Open gripper"""
        return self.control_gripper("open")
    
    def close_gripper(self, grip_force=0.5):
        """Close Robotiq 85 gripper with specific grip force"""
        return self.control_gripper("close", grip_force)
    
    # ==================== COLLISION OBJECTS ====================
    # Collision object methods are now provided by CollisionObjectMixin
    # See collision_objects.py for implementation
    
    # Removed methods (now in CollisionObjectMixin):
    # - add_box_collision_object
    # - add_cylinder_collision_object
    # - add_book_collision_object
    # - _load_stl_binary
    # - check_stl_dimensions
    # - add_mesh_collision_object
    # - remove_collision_object
    #======================ATTACH DETACH FIX=========================
    def attach_object_to_gripper(self, object_name):
        """Attach object from world to gripper safely."""
        self.get_logger().info(f"Attaching '{object_name}' to gripper...")

        # 1) Remove world object
        self.remove_collision_object(object_name)
        time.sleep(0.1)

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
        
        # CRITICAL: Wait for hardware to stabilize after motion
        # Hardware driver often times out right after arm motions complete
        time.sleep(2.0)
        return True

    def HIGH_LEVEL_move_lin(self, target_pose):
        """High-level LIN move to target pose with fallback chain:
        1. Pilz LIN (preferred for linear motion)
        2. Cartesian path planning (guaranteed linear trajectory)
        3. Pilz PTP (joint-space fallback)
        4. OMPL (final fallback)
        """
        self.get_logger().info("Starting high-level LIN move...")
        
        # Try 1: Pilz LIN (preferred for linear motion)
        self.use_pilz_lin()
        if self.move_to_pose(target_pose):
            self.use_default_planner()
            self.get_logger().info("✓ High-level LIN move successful (Pilz LIN)")
            
            # CRITICAL: Wait for hardware to stabilize after motion
            time.sleep(2.0)
            return True
        
        # Try 2: Cartesian path planning (guaranteed linear trajectory)
        self.get_logger().info("Pilz LIN failed, trying Cartesian path planning for linear trajectory...")
        
        # CRITICAL: Check hardware is still responsive before attempting fallback
        # Hardware may have timed out during Pilz LIN execution
        self.get_logger().info("Checking hardware status before Cartesian fallback...")
        is_ok, age, msg = self.check_hardware_status()
        if not is_ok:
            self.get_logger().warn(f"Hardware status: {msg}")
            self.get_logger().info("Waiting for hardware to recover before Cartesian fallback...")
            if not self.wait_for_fresh_joint_state(timeout=5.0, max_age=1.0):
                self.get_logger().error(
                    "Hardware driver not responding - cannot attempt Cartesian fallback. "
                    "Please restart the robot launch file."
                )
                self.use_default_planner()
                return False
        
        current_pose = self.get_current_pose()
        if current_pose is not None and self.cartesian_available:
            # Use Cartesian path with small step size for smooth linear motion
            distance = math.sqrt(
                (target_pose.position.x - current_pose.position.x)**2 +
                (target_pose.position.y - current_pose.position.y)**2 +
                (target_pose.position.z - current_pose.position.z)**2
            )
            eef_step = min(0.01, distance / 30)  # Adaptive step size
            
            # Try multiple Cartesian configurations for guaranteed success
            cartesian_configs = [
                # (eef_step, jump_threshold, avoid_collisions, description)
                (eef_step, 2.0, True, "standard with collision avoidance"),
                (eef_step * 0.5, 3.0, True, "smaller steps, relaxed jumps"),
                (eef_step, 5.0, True, "very relaxed jumps"),
            ]
            
            for step, jump, avoid_coll, desc in cartesian_configs:
                self.get_logger().info(f"Trying Cartesian with {desc}...")
                if self.move_cartesian([target_pose], eef_step=step, 
                                     jump_threshold=jump, max_velocity_scaling=0.005,
                                     avoid_collisions=avoid_coll):
                    self.use_default_planner()
                    self.get_logger().info(f"✓ High-level LIN move successful (Cartesian path - {desc})")
                    # CRITICAL: Wait for hardware to stabilize after motion
                    time.sleep(2.0)
                    return True
        
        # Try 3: Pilz PTP (joint-space fallback)
        self.get_logger().info("Cartesian path failed, trying Pilz PTP...")
        if self.HIGH_LEVEL_move_ptp(target_pose):
            self.get_logger().info("✓ High-level LIN move successful (Pilz PTP fallback)")
            # CRITICAL: Wait for hardware to stabilize after motion
            time.sleep(2.0)
            return True
        
        # All methods failed
        self.use_default_planner()
        self.get_logger().error("High-level LIN move failed with all methods")
        return False
    
    def HIGH_LEVEL_move_lin_relative(self, dx=0.0, dy=0.0, dz=0.0):
        """High-level LIN relative move using default planner."""
        self.get_logger().info("Starting high-level LIN relative move...")
        self.get_logger().info(f"dx: {dx}, dy: {dy}, dz: {dz}")
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

        # Get current pose before rotation
        start_pose = self.get_current_pose()
        if start_pose is None:
            self.get_logger().error("Cannot get current pose for pouring")
            return False

        # Rotate end-effector around Y-axis
        if not self.rotate_eef_relative(yaw=pour_angle_rad):
            self.get_logger().error("Pouring motion failed")
            return False
        
        # CRITICAL: Wait for pouring rotation to complete
        self.get_logger().info("Waiting for pouring rotation to complete...")
        time.sleep(2.0)  # Extra wait for pouring motion
        try:
            if not self.wait_for_motion_completion(timeout=15.0, check_interval=0.5):
                self.get_logger().debug("Pouring rotation verification timed out (usually OK)")
        except Exception as e:
            self.get_logger().debug(f"Pouring rotation verification had issues (non-critical): {e}")
        
        # Hold pour position briefly
        time.sleep(3.0)  # Hold pour for 3 seconds
        
        if not self.rotate_eef_relative(yaw=-pour_angle_rad):
            self.get_logger().error("Returning from pouring motion failed")
            return False
        
        # CRITICAL: Wait for return rotation to complete
        self.get_logger().info("Waiting for return rotation to complete...")
        time.sleep(2.0)  # Extra wait for return motion
        try:
            if not self.wait_for_motion_completion(target_pose=start_pose, timeout=15.0, check_interval=0.5):
                self.get_logger().debug("Return rotation verification timed out (usually OK)")
        except Exception as e:
            self.get_logger().debug(f"Return rotation verification had issues (non-critical): {e}")

        self.get_logger().info("✓ Pouring motion completed successfully")
        return True

    def pick_object(self, object_name, grasp_pose, approach_direction="side",
                    approach_distance=0.02, grasp_distance=0.015,
                    lift_distance=0.2, grip_force=0.85):
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
        self.get_logger().info(f"Starting pick operation for '{object_name} with grip force {grip_force}'")
        approach_distance += self.GRIPPER_OUTER_LENGTH

        # Step 1: Compute and move to pre-grasp pose with PTP
        approach_pose = self.compute_approach_pose(
            grasp_pose, distance=approach_distance, direction=approach_direction)

        self.get_logger().info(
            f"Step 1: Moving to approach pose (Pilz PTP) -> "
            f"{approach_pose.position.x:.3f}, {approach_pose.position.y:.3f}, {approach_pose.position.z:.3f}"
        )

        delta_x1 = approach_pose.position.x - self.get_current_pose().position.x
        delta_y1 = approach_pose.position.y - self.get_current_pose().position.y
        delta_z1 = approach_pose.position.z - self.get_current_pose().position.z

        if not self.HIGH_LEVEL_move_lin(approach_pose):
            self.get_logger().error("Failed to reach approach pose")
            return False
            
        self.get_logger().info("Successfully moved to approach pose")

        time.sleep(0.5)

        # Step 2: Open gripper
        self.get_logger().info("Step 2: Opening gripper...")
        if not self.open_gripper():
            self.get_logger().error("Failed to open gripper")
            return False
        time.sleep(2.0)  # Increased wait for gripper to fully open

        # Step 4: Remove and attach object to gripper
        self.get_logger().info("Step 4: Attaching object to gripper for collision avoidance...")
        if not self.attach_object_to_gripper(object_name):
            self.get_logger().error("Failed to attach object")
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


        # Step 5: Close gripper to grasp
        self.get_logger().info("Step 5: Closing gripper...")
        if not self.close_gripper(grip_force):
            self.get_logger().error("Failed to close gripper")
            return False
        time.sleep(2.0)  # Increased wait for gripper to fully close and grasp

        # Step 6: Lift vertically using Pilz LIN
        self.get_logger().info(f"Step 6: Lifting object using Pilz LIN ({lift_distance} m)...")
        if not self.HIGH_LEVEL_move_lin_relative(dz=lift_distance):
            self.get_logger().error("Failed to lift object")
            return False
        time.sleep(0.5)
        
        self.get_logger().info("✓ Pick operation completed successfully")
        return True


    def lin_pick_object(self, object_name, grasp_pose, approach_direction="side",
                    approach_distance=0.02, grasp_distance=0.015,
                    lift_distance=0.2, grip_force=0.85):
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

        delta_x1 = approach_pose.position.x - self.get_current_pose().position.x
        delta_y1 = approach_pose.position.y - self.get_current_pose().position.y
        delta_z1 = approach_pose.position.z - self.get_current_pose().position.z

        if not self.HIGH_LEVEL_move_lin_relative(dy=delta_y1):
            if not self.HIGH_LEVEL_move_lin(approach_pose):
                self.get_logger().error("Failed to reach approach pose")
                return False
        elif not self.HIGH_LEVEL_move_lin_relative(dx=delta_x1, dz=delta_z1):
            if not self.HIGH_LEVEL_move_lin(approach_pose):
                self.get_logger().error("Failed to reach approach pose")
                return False
            
        self.get_logger().info("Successfully moved to approach pose")

        time.sleep(0.5)

        # Step 2: Open gripper
        self.get_logger().info("Step 2: Opening gripper...")
        if not self.open_gripper():
            self.get_logger().error("Failed to open gripper")
            return False
        time.sleep(2.0)  # Increased wait for gripper to fully open

        # Step 4: Remove and attach object to gripper
        self.get_logger().info("Step 4: Attaching object to gripper for collision avoidance...")
        if not self.attach_object_to_gripper(object_name):
            self.get_logger().error("Failed to attach object")
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


        # Step 5: Close gripper to grasp
        self.get_logger().info("Step 5: Closing gripper...")
        if not self.close_gripper(grip_force):
            self.get_logger().error("Failed to close gripper")
            return False
        time.sleep(2.0)  # Increased wait for gripper to fully close and grasp

        # Step 6: Lift vertically using Pilz LIN
        self.get_logger().info(f"Step 6: Lifting object using Pilz LIN ({lift_distance} m)...")
        if not self.HIGH_LEVEL_move_lin_relative(dz=lift_distance):
            self.get_logger().error("Failed to lift object")
            return False
        time.sleep(0.5)
        
        self.get_logger().info("✓ Pick operation completed successfully")
        return True

    def place_object_with_start(self, object_name, start_pose, place_pose, retreat_distance_x=0.07, retreat_distance_z=0.15):
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
        time.sleep(2.5)  # Increased wait for gripper to fully open and release object

        # Step 4: Retreat - check if object is bowl or plate (need horizontal retreat first)
        # Extract marker_id from object_name (format: "detected_object_10")
        marker_id = None
        try:
            if object_name.startswith("detected_object_"):
                marker_id_str = object_name.replace("detected_object_", "")
                marker_id = int(marker_id_str)
        except (ValueError, AttributeError):
            pass
        
        # Check if object is bowl (11) or plate (10) - need to retreat horizontally first
        if retreat_distance_x > 0:
            horizontal_retreat = -retreat_distance_x  # Move back in -X direction
            self.get_logger().info(
                f"Step 4a: Retreating horizontally by {horizontal_retreat:.3f} m (Pilz LIN) "
                f"to clear {object_name} top surface...")
            if not self.HIGH_LEVEL_move_lin_relative(dx=horizontal_retreat):
                self.get_logger().error("Failed to retreat horizontally")
                return False
            time.sleep(0.5)

        # Step 4b: Retreat upward using Pilz LIN
        self.get_logger().info(f"Step 4b: Retreating upward by {retreat_distance_z} m (Pilz LIN)...")
        if not self.HIGH_LEVEL_move_lin_relative(dz=retreat_distance_z):
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

        if not self.HIGH_LEVEL_move_lin_relative(dx=dx):
            self.get_logger().error("Failed to reach place pre-pose along x axis")
            return False
        time.sleep(0.5)

        if not self.HIGH_LEVEL_move_lin_relative(dy=dy):
            self.get_logger().error("Failed to reach place pre-pose along y axis")
            return False
        time.sleep(0.5)

        # Step 2: Descend with Pilz LIN to pour height
        z_pour = place_pose.position.z + 0.15
        dz = z_pour - place_pre_pose_eef.position.z
        self.get_logger().info(
            f"Step 2: Descending to pour height (Pilz LIN) -> with z_pour={z_pour:.3f} m - place_pre_pose_eef.position.z={place_pre_pose_eef.position.z:.3f} m -> with dz={dz:.3f} m -> "
        )
        if not self.HIGH_LEVEL_move_lin_relative(dz=dz):
            self.get_logger().error("Failed to descend to pour height")
            return False
        time.sleep(0.5)

        self.get_logger().warn(f"Current pose after descending to pour height: {self.get_current_pose().position.x+self.GRIPPER_INNER_LENGTH+0.02:.3f}, {self.get_current_pose().position.y:.3f}, {self.get_current_pose().position.z:.3f}")
        #Step 3: Pouring motion
        if not self.HIGH_LEVEL_pour(pour_angle_degree):
            self.get_logger().error("Pouring motion failed")
            return False
        
        # CRITICAL: Wait after pouring before descending
        self.get_logger().info("Waiting after pouring motion...")
        time.sleep(1.0)
        
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
        time.sleep(2.5)  # Increased wait for gripper to fully open and release object

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


    def grasp_object(self, object_name, grasp_pose, approach_direction="side",
                    approach_distance=0.02, grasp_distance=0.015, grip_force=0.5):
        """
        Execute complete pick operation using Pilz planners.
        Args:
            object_name: Name of object to pick (collision id)              
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
        time.sleep(2.0)  # Increased wait for gripper to fully open

        # Step 3: Remove and attach object to gripper
        self.get_logger().info("Step 3: Attaching object to gripper for collision avoidance...")
        if not self.attach_object_to_gripper(object_name):
            self.get_logger().error("Failed to attach object")
            return False
        time.sleep(1.0)

        # Step 4: Pilz LIN side-approach towards object
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


        # Step 5: Close gripper to grasp
        self.get_logger().info("Step 5: Closing gripper...")
        if not self.close_gripper(grip_force):
            self.get_logger().error("Failed to close gripper")
            return False
        time.sleep(2.0)  # Increased wait for gripper to fully close and grasp

        return True


    def move_without_descend(self, object_name, start_pose, place_pose, retreat_distance=0.15):
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

        # Step 2: Open gripper (release object)
        self.get_logger().info("Step 2: Opening gripper to release object...")
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
        time.sleep(2.5)  # Increased wait for gripper to fully open and release object

        # Step 3: Retreat - check if object is bowl or plate (need horizontal retreat first)
        # Extract marker_id from object_name (format: "detected_object_10")
        marker_id = None
        try:
            if object_name.startswith("detected_object_"):
                marker_id_str = object_name.replace("detected_object_", "")
                marker_id = int(marker_id_str)
        except (ValueError, AttributeError):
            pass
        
        # Check if object is bowl (11) or plate (10) - need to retreat horizontally first
        if marker_id in [10, 11]:  # plate or bowl
            horizontal_retreat = -0.07  # Move back in -X direction
            self.get_logger().info(
                f"Step 3a: Retreating horizontally by {horizontal_retreat:.3f} m (Pilz LIN) "
                f"to clear {object_name} top surface...")
            if not self.HIGH_LEVEL_move_lin_relative(dx=horizontal_retreat):
                self.get_logger().error("Failed to retreat horizontally")
                return False
            time.sleep(0.5)

        # Step 3b: Retreat upward using Pilz LIN
        self.get_logger().info(f"Step 3b: Retreating upward by {retreat_distance} m (Pilz LIN)...")
        if not self.HIGH_LEVEL_move_lin_relative(dz=retreat_distance):
            self.get_logger().error("Failed to retreat upward")
            return False

        # Step 4: Detach object from gripper
        self.get_logger().info("Step 4: Detaching object from gripper...")
        self.detach_object_from_gripper(object_name)

        self.get_logger().info("✓ Place operation completed successfully")
        return True
    
    def grasp_move_object(self, object_name, grasp_pose, place_pose, approach_direction="side",
                    approach_distance=0.02, grasp_distance=0.015,
                    lift_distance=0.2, grip_force=0.5, retreat_distance=0.15):
        """
        Execute complete pick-and-place operation using Pilz planners.
        Args:
            object_name: Name of object to pick and place (collision id)
            grasp_pose: Pose of object / grasp point (in base frame)
            place_pose: Target place pose
            approach_direction: currently only "side" supported
            approach_distance: distance BEFORE contact (m)
            grasp_distance: how deep to go past pre-grasp (m)
            lift_distance: vertical lift after grasp (m)
            grip_force: gripper closing effort (0.0-1.0)
            retreat_distance: Distance to retreat upward after placing (m)
        """
       
        self.get_logger().info(f"Starting pick-and-place operation for '{object_name}'")
        if not self.grasp_object(object_name, grasp_pose, approach_direction,
                    approach_distance, grasp_distance,
                    grip_force):
            return False

        return self.move_without_descend(object_name, grasp_pose, place_pose, retreat_distance)
        
        
        