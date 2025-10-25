#!/usr/bin/env python3
"""
MoveIt2 Controller Base Class for Kinova 7-DOF Robotic Arm

This module provides a reusable base class that handles all MoveIt2 interactions
including standard motion planning, Cartesian path planning, gripper control,
and collision object management.

Can be inherited by both simulation and real robot implementations.
"""

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
        
        # Grasp orientations
        self.grasp_orientations = {
            'side': euler_to_quaternion(0.5*math.pi, 0, 0.5*math.pi)
        }
        
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
    
    def move_to_pose(self, target_pose, planning_time=5.0):
        """Move arm to target pose using standard motion planning"""
        goal = MoveGroup.Goal()
        goal.request.group_name = self.arm_group_name
        goal.request.num_planning_attempts = 12
        goal.request.allowed_planning_time = planning_time
        goal.request.max_velocity_scaling_factor = 0.1
        goal.request.max_acceleration_scaling_factor = 0.1
        
        constraints = Constraints()
        
        # Position constraint
        pos_constraint = PositionConstraint()
        pos_constraint.header.frame_id = self.base_frame
        pos_constraint.link_name = self.end_effector_frame
        pos_constraint.constraint_region.primitives.append(SolidPrimitive())
        pos_constraint.constraint_region.primitives[0].type = SolidPrimitive.BOX
        pos_constraint.constraint_region.primitives[0].dimensions = [0.02, 0.02, 0.02]
        pos_constraint.constraint_region.primitive_poses.append(target_pose)
        pos_constraint.weight = 1.0
        constraints.position_constraints.append(pos_constraint)
        
        # Orientation constraint
        orient_constraint = OrientationConstraint()
        orient_constraint.header.frame_id = self.base_frame
        orient_constraint.link_name = self.end_effector_frame
        orient_constraint.orientation = target_pose.orientation
        orient_constraint.absolute_x_axis_tolerance = 0.005
        orient_constraint.absolute_y_axis_tolerance = 0.005
        orient_constraint.absolute_z_axis_tolerance = 0.005
        orient_constraint.weight = 1.0
        constraints.orientation_constraints.append(orient_constraint)
        
        goal.request.goal_constraints.append(constraints)
        
        self.get_logger().info("Sending move goal...")
        future = self.move_group_client.send_goal_async(goal)
        rclpy.spin_until_future_complete(self, future)
        
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().error("Move goal rejected")
            return False
        
        self.get_logger().info("Move goal accepted, executing...")
        result_future = goal_handle.get_result_async()
        rclpy.spin_until_future_complete(self, result_future)
        
        result = result_future.result()
        success = result.result.error_code.val == MoveItErrorCodes.SUCCESS
        
        if success:
            self.get_logger().info("✓ Move completed successfully")
        else:
            self.get_logger().error(f"✗ Move failed: {result.result.error_code.val}")
        
        return success
    
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
    
    def move_relative_cartesian(self, delta_x=0.0, delta_y=0.0, delta_z=0.0):
        """
        Move relative to current pose using Cartesian path
        
        Args:
            delta_x/y/z: Position changes in meters
        
        Returns:
            bool: True if successful
        """
        current_pose = self.get_current_pose()
        if current_pose is None:
            return False
        
        # Create target pose
        target_pose = Pose()
        target_pose.position.x = current_pose.position.x + delta_x
        target_pose.position.y = current_pose.position.y + delta_y
        target_pose.position.z = current_pose.position.z + delta_z
        target_pose.orientation = current_pose.orientation  # Keep orientation
        
        self.get_logger().info(
            f"Cartesian move: Δ({delta_x:.3f}, {delta_y:.3f}, {delta_z:.3f})")
        
        # Execute Cartesian path with single waypoint
        return self.move_cartesian([target_pose], eef_step=0.005)
    
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
        
        self.planning_scene_pub.publish(planning_scene)
        time.sleep(0.3)
        
        self.get_logger().info(f"✓ Attached '{object_name}' to gripper")
        return True
    
    def detach_object_from_gripper(self, object_name):
        """Detach object from gripper"""
        self.get_logger().info(f"Detaching '{object_name}' from gripper...")
        
        attached_object = AttachedCollisionObject()
        attached_object.link_name = self.end_effector_frame
        attached_object.object.id = object_name
        attached_object.object.operation = CollisionObject.REMOVE
        
        planning_scene = PlanningScene()
        planning_scene.robot_state.attached_collision_objects.append(attached_object)
        planning_scene.is_diff = True
        
        self.planning_scene_pub.publish(planning_scene)
        time.sleep(0.5)
        
        self.get_logger().info(f"✓ Detached '{object_name}'")
        return True
    
    # ==================== HIGH-LEVEL OPERATIONS ====================
    
    def pick_object(self, object_name, grasp_pose, approach_direction="side", 
                    approach_distance=0.20, grasp_distance=0.05, 
                    lift_distance=0.10, grip_force=0.6):
        """
        Execute complete pick operation with Cartesian paths
        
        Args:
            object_name: Name of object to pick
            grasp_pose: Target grasp pose
            approach_direction: Direction to approach from
            approach_distance: Distance to stay back initially (m)
            grasp_distance: Distance to move forward to grasp (m)
            lift_distance: Distance to lift after grasp (m)
            grip_force: Gripper closing force (0.0-1.0)
        
        Returns:
            bool: True if successful
        """
        self.get_logger().info(f"Starting pick operation for '{object_name}'")
        
        # Step 1: Move to approach pose
        approach_pose = self.compute_approach_pose(
            grasp_pose, distance=approach_distance, direction=approach_direction)
        
        self.get_logger().info(f"Step 1: Moving to approach pose (standard planning) to {approach_pose.position.x:.3f}, {approach_pose.position.y:.3f}, {approach_pose.position.z:.3f}...")
        if not self.move_to_pose(approach_pose):
            self.get_logger().error("Failed to reach approach pose")
            return False
        
        # Step 2: Open gripper
        self.get_logger().info("Step 2: Opening gripper...")
        if not self.open_gripper():
            self.get_logger().error("Failed to open gripper")
            return False
        time.sleep(1.0)
        
        # Step 3: Attach object for collision avoidance
        self.get_logger().info("Step 3: Attaching object for collision avoidance...")
        if not self.attach_object_to_gripper(object_name):
            self.get_logger().error("Failed to attach object")
            return False
        time.sleep(0.3)
        
        # Step 4: Move closer using CARTESIAN PATH
        delta_z = 0.03
        self.get_logger().info(f"Step 4: Approaching object using Cartesian path ({grasp_distance}m, {delta_z}m)...")
        if self.cartesian_available and not self.move_relative_cartesian(delta_x=grasp_distance, delta_z=delta_z):
            self.get_logger().warn("Cartesian approach failed, using standard planning")
            # Fallback
            current_pose = self.get_current_pose()
            if current_pose:
                current_pose.position.x += grasp_distance
                if not self.move_to_pose(current_pose):
                    return False
            else:
                return False
        elif not self.cartesian_available:
            # Use standard planning
            current_pose = self.get_current_pose()
            if current_pose:
                current_pose.position.x += grasp_distance
                if not self.move_to_pose(current_pose):
                    return False
            else:
                return False
        
        # Step 5: Close gripper
        self.get_logger().info("Step 5: Closing gripper...")
        if not self.close_gripper(grip_force):
            self.get_logger().error("Failed to close gripper")
            return False
        time.sleep(1.0)
        
        # Step 6: Lift using CARTESIAN PATH
        self.get_logger().info(f"Step 6: Lifting object using Cartesian path ({lift_distance}m)...")
        if self.cartesian_available and not self.move_relative_cartesian(delta_z=lift_distance):
            self.get_logger().warn("Cartesian lift failed, using standard planning")
            # Fallback
            current_pose = self.get_current_pose()
            if current_pose:
                current_pose.position.z += lift_distance
                if not self.move_to_pose(current_pose):
                    return False
            else:
                return False
        elif not self.cartesian_available:
            # Use standard planning
            current_pose = self.get_current_pose()
            if current_pose:
                current_pose.position.z += lift_distance
                if not self.move_to_pose(current_pose):
                    return False
            else:
                return False
        
        self.get_logger().info("✓ Pick operation completed successfully")
        return True
    
    def place_object(self, object_name, place_pose, retreat_distance=0.10):
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
        if not self.move_to_pose(place_pose):
            self.get_logger().error("Failed to reach place pose")
            return False
        
        # Step 2: Open gripper
        self.get_logger().info("Step 2: Opening gripper to release object...")
        if not self.open_gripper():
            self.get_logger().error("Failed to open gripper")
            return False
        time.sleep(1.0)
        
        # Step 3: Detach object
        self.get_logger().info("Step 3: Detaching object from gripper...")
        self.detach_object_from_gripper(object_name)
        
        # Step 4: Retreat using Cartesian path
        self.get_logger().info(f"Step 4: Retreating ({retreat_distance}m)...")
        if not self.move_relative_cartesian(delta_z=retreat_distance):
            # Fallback
            current_pose = self.get_current_pose()
            if current_pose:
                current_pose.position.z += retreat_distance
                if not self.move_to_pose(current_pose):
                    return False
        
        self.get_logger().info("✓ Place operation completed successfully")
        return True
