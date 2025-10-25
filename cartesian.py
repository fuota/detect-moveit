#!/usr/bin/env python3
"""
Pick and Place script optimized for Kinova 7-DOF robotic arm.
Enhanced with Cartesian path planning for smooth, predictable approach and lift motions.
"""

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from moveit_msgs.msg import (Grasp, GripperTranslation, PlaceLocation, MoveItErrorCodes,
                              RobotTrajectory)
from moveit_msgs.action import MoveGroup, ExecuteTrajectory
from moveit_msgs.srv import GetCartesianPath
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from geometry_msgs.msg import Pose, PoseStamped
from std_msgs.msg import Header
from shape_msgs.msg import SolidPrimitive
from moveit_msgs.msg import (CollisionObject, PlanningScene, AttachedCollisionObject,
                              Constraints, PositionConstraint, OrientationConstraint, JointConstraint)
from geometry_msgs.msg import Point
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


def get_common_grasp_orientations():
    """Return dictionary of common grasping orientations for Kinova arm"""
    orientations = {}
    orientations['side'] = euler_to_quaternion(0.5*math.pi, 0, 0.5*math.pi)
    return orientations


class GraspPlanner(Node):
    def __init__(self):
        super().__init__('grasp_planner')
        
        # Kinova 7-DOF arm configuration
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

        self.grasp_orientations = get_common_grasp_orientations()
        
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
        
        # Check execute trajectory server
        if not self.execute_trajectory_client.wait_for_server(timeout_sec=5.0):
            self.get_logger().warn("Execute trajectory server not available")
        
        self.get_logger().info("Initialized GraspPlanner for Kinova 7-DOF arm")
        self.get_logger().info("✓ Cartesian path planning enabled")
        self.get_logger().info(f"Available grasp orientations: {list(self.grasp_orientations.keys())}")
    
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
        
        # Scale velocity if needed
        # if max_velocity_scaling < 1.0:
        #     trajectory = self.scale_trajectory_speed(trajectory, max_velocity_scaling)
        
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
    
    def scale_trajectory_speed(self, trajectory, scale_factor):
        """Scale trajectory speed by factor"""
        scaled_trajectory = RobotTrajectory()
        scaled_trajectory.joint_trajectory = trajectory.joint_trajectory
        scaled_trajectory.multi_dof_joint_trajectory = trajectory.multi_dof_joint_trajectory
        
        # Scale time_from_start for each point
        for point in scaled_trajectory.joint_trajectory.points:
            # Convert to total nanoseconds, scale, then convert back
            total_nsec = point.time_from_start.sec * 1_000_000_000 + point.time_from_start.nanosec
            scaled_nsec = int(total_nsec / scale_factor)
            
            point.time_from_start.sec = scaled_nsec // 1_000_000_000
            point.time_from_start.nanosec = scaled_nsec % 1_000_000_000
        
        return scaled_trajectory
    
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

    def compute_grasp_pose(self, grasp_pose, approach_distance=0.15, approach_direction="side"):
        """Compute pre-grasp pose offset from grasp pose"""
        pre_grasp_pose = Pose()
        pre_grasp_pose.position.x = grasp_pose.position.x
        pre_grasp_pose.position.y = grasp_pose.position.y
        pre_grasp_pose.position.z = grasp_pose.position.z
        
        if approach_direction == "side":
            pre_grasp_pose.position.x -= approach_distance
            self.set_grasp_orientation(pre_grasp_pose, "side")

        return pre_grasp_pose
    
    def move_to_pose(self, target_pose, planning_time=5.0):
        """Move arm to target pose using MoveGroup action"""
        goal = MoveGroup.Goal()
        goal.request.group_name = self.arm_group_name
        goal.request.num_planning_attempts = 12
        goal.request.allowed_planning_time = planning_time
        goal.request.max_velocity_scaling_factor = 0.1
        goal.request.max_acceleration_scaling_factor = 0.1
        goal.request.planner_id = "RRTstar"
        
        constraints = Constraints()
        
        # Position constraint
        pos_constraint = PositionConstraint()
        pos_constraint.header.frame_id = self.base_frame
        pos_constraint.link_name = self.end_effector_frame
        pos_constraint.target_point_offset.x = 0.0
        pos_constraint.target_point_offset.y = 0.0
        pos_constraint.target_point_offset.z = 0.0
        
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
            
        result_future = goal_handle.get_result_async()
        rclpy.spin_until_future_complete(self, result_future)
        
        result = result_future.result()
        if result.result.error_code.val == MoveItErrorCodes.SUCCESS:
            self.get_logger().info("Move completed successfully")
            return True
        else:
            self.get_logger().error(f"Move failed with error code: {result.result.error_code.val}")
            return False
    
    def control_gripper(self, state, gripper_value=None):
        """Control gripper using MoveGroup action for gripper group"""

        gripper_client = ActionClient(self, MoveGroup, '/move_action')
        
        if not gripper_client.wait_for_server(timeout_sec=5.0):
            self.get_logger().error("Gripper MoveGroup action server not available!")
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
            if gripper_value is not None:
                left_pos = self.gripper_max_close_left * gripper_value
                right_pos = self.gripper_max_close_right * gripper_value
                positions = [left_pos, right_pos]
            else:
                positions = [self.gripper_max_close_left * 0.8, self.gripper_max_close_right * 0.8]
        else:
            self.get_logger().error(f"Unknown gripper state: {state}")
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
    
    def pick_object(self, object_name, grasp_pose, approach_direction="side"):
        """Pick operation with Cartesian paths for approach and lift"""
        self.get_logger().info(f"Planning pick for object: {object_name}")
        
        # Step 1: Move to approach pose (long distance - use standard planning)
        approach_pose = self.compute_grasp_pose(grasp_pose, approach_distance=0.2, approach_direction=approach_direction)

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
        if not self.attach_cylinder_to_gripper("target_cylinder"):
            self.get_logger().error("Failed to attach cylinder")
            return False
        
        time.sleep(0.3)
        
        # Step 4: Move closer using CARTESIAN PATH
        self.get_logger().info(f"Step 4: Moving closer to object (Cartesian path) to pose {grasp_pose.position.x:.3f}, {grasp_pose.position.y:.3f}, {grasp_pose.position.z:.3f}...")
        if not self.move_relative_cartesian(delta_x=0.05, delta_z=0.03):
            self.get_logger().warn("Cartesian approach failed, trying standard planning")
            # Fallback to standard planning
            current_pose = self.get_current_pose()
            if current_pose:
                current_pose.position.x += 0.05
                if not self.move_to_pose(current_pose):
                    return False
            else:
                return False
        
        # Step 5: Close gripper to grasp object
        self.get_logger().info("Step 5: Closing gripper to grasp object...")
        if not self.close_gripper(0.6):
            self.get_logger().error("Failed to close gripper")
            return False
        
        time.sleep(1.0)
        
        # Step 6: Lift object using CARTESIAN PATH
        delta_z = 0.10
        self.get_logger().info(f"Step 6: Lifting object (Cartesian path) to safe height. Pose is {grasp_pose.position.x:.3f}, {grasp_pose.position.y:.3f}, {grasp_pose.position.z + 0.20:.3f}...")
        if not self.move_relative_cartesian(delta_z=delta_z):
            self.get_logger().warn("Cartesian lift failed, trying standard planning")
            # Fallback to standard planning
            current_pose = self.get_current_pose()
            if current_pose:
                current_pose.position.z += 0.20
                if not self.move_to_pose(current_pose):
                    return False
            else:
                return False
        
        self.get_logger().info("✓ Pick operation completed successfully")
        return True
    
    def place_object(self, object_name, place_pose):
        """Place operation with Cartesian paths"""
        self.get_logger().info(f"Planning place for object: {object_name}")

        current_pose = self.get_current_pose()
        delta_x = place_pose.position.x - current_pose.position.x
        delta_y = place_pose.position.y - current_pose.position.y

        

        self.get_logger().info("Moving to place approach pose by Cartesian path...")
        if not self.move_relative_cartesian(delta_x=delta_x, delta_y=delta_y):
            self.get_logger().warn("Cartesian move to place approach failed, trying standard planning")
            current_pose = self.get_current_pose()
            current_pose.position.x += delta_x
            current_pose.position.y += delta_y
            if not self.move_to_pose(current_pose):
                return False

        # Move down to place pose using Cartesian path
        self.get_logger().info("Moving to place pose (Cartesian)...")
        if not self.move_relative_cartesian(delta_z=-0.18):
            self.get_logger().warn("Cartesian move down to place pose failed, trying standard planning")
            current_pose = self.get_current_pose()
            current_pose.position.z -= 0.18
            if not self.move_to_pose(current_pose):
                return False
        
        # Open gripper
        self.get_logger().info("Opening gripper...")
        if not self.open_gripper():
            return False
        
        time.sleep(1.0)

        self.detach_cylinder_from_gripper("target_cylinder")

        # Retreat using Cartesian path
        self.get_logger().info("Retreating (Cartesian)...")
        if not self.move_relative_cartesian(delta_z=0.10):
            current_pose = self.get_current_pose()
            if current_pose:
                current_pose.position.z += 0.10
                if not self.move_to_pose(current_pose):
                    return False
        
        self.get_logger().info("✓ Place operation completed successfully")
        return True
    
    def add_table_collision_object(self, table_name="work_table", x=0.8, y=0.0, z=0.4, 
                                  width=1.2, depth=0.8, height=0.05):
        """Add a table as a collision object to the planning scene"""
        collision_object = CollisionObject()
        collision_object.header.frame_id = self.base_frame
        collision_object.header.stamp = self.get_clock().now().to_msg()
        collision_object.id = table_name
        
        box = SolidPrimitive()
        box.type = SolidPrimitive.BOX
        box.dimensions = [width, depth, height]
        
        table_pose = Pose()
        table_pose.position.x = float(x)
        table_pose.position.y = float(y)
        table_pose.position.z = float(z)
        table_pose.orientation.w = 1.0
        
        collision_object.primitives.append(box)
        collision_object.primitive_poses.append(table_pose)
        collision_object.operation = CollisionObject.ADD
        
        planning_scene = PlanningScene()
        planning_scene.world.collision_objects.append(collision_object)
        planning_scene.is_diff = True
        
        self.planning_scene_pub.publish(planning_scene)
        time.sleep(0.5)
        
        self.get_logger().info(f"Table '{table_name}' added to planning scene")
        return True
    
    def add_cylinder_collision_object(self, cylinder_name="target_cylinder", x=0.8, y=-0.336, z=0.5,
                                    radius=0.05, height=0.15):
        """Add a cylinder as a collision object to the planning scene"""
        collision_object = CollisionObject()
        collision_object.header.frame_id = self.base_frame
        collision_object.header.stamp = self.get_clock().now().to_msg()
        collision_object.id = cylinder_name
        
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
        
        self.get_logger().info(f"Cylinder '{cylinder_name}' added to planning scene")
        return True
    
    def attach_cylinder_to_gripper(self, object_name="target_cylinder"):
        """Attach cylinder to end effector"""
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
        time.sleep(0.5)
        
        return True

    
    def detach_cylinder_from_gripper(self, object_name="target_cylinder"):
        """Detach cylinder from end effector and return it to the world"""
        self.get_logger().info(f"Detaching {object_name} from gripper...")
        
        # Method 1: Use REMOVE operation with link_name specified
        attached_object = AttachedCollisionObject()
        attached_object.link_name = self.end_effector_frame  # ← IMPORTANT: Must specify link
        attached_object.object.id = object_name
        attached_object.object.operation = CollisionObject.REMOVE  # ← Remove from attached
        
        planning_scene = PlanningScene()
        planning_scene.robot_state.attached_collision_objects.append(attached_object)
        planning_scene.is_diff = True
        
        self.planning_scene_pub.publish(planning_scene)
        time.sleep(0.5)
        
        self.get_logger().info(f"✓ {object_name} detached successfully")
        return True

def main(args=None):
    rclpy.init(args=args)
    
    grasp_planner = GraspPlanner()
    
    # Add collision objects
    grasp_planner.get_logger().info("Setting up collision objects...")
    
    # Wheelchair seat
    grasp_planner.add_table_collision_object("wheel_chair_seat", x=-0.43, y=-0.1, z=0, 
                                           width=0.6, depth=0.5, height=1.26)
    
    # Wheelchair back
    chair_back_z = -0.54 + 1.26/2
    grasp_planner.add_table_collision_object("wheel_chair_back", x=-0.43, y=0.3, z=chair_back_z, 
                                           width=0.34, depth=0.35, height=1.26)
    
    # Work table
    table_x = 0.7 
    table_y = -0.5
    table_z = 0.2 
    table_height = 0.7
    grasp_planner.add_table_collision_object("work_table", x=table_x, y=table_y, z=table_z, 
                                           width=0.8, depth=1.2, height=table_height)
    
    # Target cylinder
    cylinder_x = 0.6
    cylinder_y = -0.5
    cylinder_height = 0.15
    cylinder_z = table_z + table_height/2 + cylinder_height/2 
    cylinder_radius = 0.025 
    grasp_planner.add_cylinder_collision_object("target_cylinder", x=cylinder_x, y=cylinder_y, z=cylinder_z,
                                        radius=cylinder_radius, height=cylinder_height)

    grasp_planner.get_logger().info("✓ Collision objects setup completed")
    time.sleep(1.0)
    
    # Define target grasp pose
    grasp_pose = Pose()
    grasp_pose.position.x = cylinder_x
    grasp_pose.position.y = cylinder_y
    grasp_pose.position.z = cylinder_z 

    grasp_planner.get_logger().info(f"\nTarget grasp pose: x={grasp_pose.position.x:.3f}, y={grasp_pose.position.y:.3f}, z={grasp_pose.position.z:.3f}")
    
    # Execute pick with Cartesian paths
    if grasp_planner.pick_object("cylinder_object", grasp_pose):
        grasp_planner.get_logger().info("\n🎉 Pick successful!")

        current_pose = grasp_planner.get_current_pose()
        if current_pose:
            grasp_planner.get_logger().info(f"Current pose after pick: x={current_pose.position.x:.3f}, y={current_pose.position.y:.3f}, z={current_pose.position.z:.3f}")
            # Place 
            place_pose = current_pose
            place_pose.position.y = current_pose.position.y + 0.2

            if grasp_planner.place_object("cylinder_object", place_pose):
                grasp_planner.get_logger().info("\n🎉 Place successful! Pick and place operation completed.")
            else:
                grasp_planner.get_logger().error("Place operation failed.")
        else:
            grasp_planner.get_logger().error("Failed to get current pose after pick.")
    else:
        grasp_planner.get_logger().error("\n❌ Pick operation failed.")
    
    # Keep node alive briefly
    rclpy.spin_once(grasp_planner, timeout_sec=1.0)
    grasp_planner.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()