#!/usr/bin/env python3
"""
Interactive Pick and Place Controller for Kinova 7-DOF arm with ArUco Detection
Enhanced with Cartesian path planning for smooth, predictable motions
"""

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from geometry_msgs.msg import Pose, PoseStamped, PoseArray
from std_msgs.msg import Int32MultiArray, String
from moveit_msgs.msg import (MoveItErrorCodes, CollisionObject, PlanningScene, 
                              AttachedCollisionObject, RobotTrajectory, RobotState)
from moveit_msgs.action import MoveGroup, ExecuteTrajectory
from moveit_msgs.srv import GetCartesianPath, GetPositionFK
from shape_msgs.msg import SolidPrimitive
from moveit_msgs.msg import JointConstraint, Constraints, PositionConstraint, OrientationConstraint
from sensor_msgs.msg import JointState
import time
import math
import json
import tf2_ros


# Object name mapping
OBJECT_MAP = {
    6: "water_bottle",
    7: "medicine_bottle"
}


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


class InteractivePickPlaceController(Node):
    def __init__(self):
        super().__init__('interactive_pick_place_controller')
        
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
        
        # Detected objects storage
        self.detected_objects = {}
        self.marker_ids = []
        self.collision_objects_added = set()
        
        # Movement state
        self.is_moving = False
        self.valid_pose_available = False
        
        # Subscribe to ArUco detection topics
        self.pose_sub = self.create_subscription(
            PoseArray, '/detected_objects/poses', self.pose_callback, 10)
        self.ids_sub = self.create_subscription(
            Int32MultiArray, '/detected_objects/ids', self.ids_callback, 10)
        self.names_sub = self.create_subscription(
            String, '/detected_objects/names', self.names_callback, 10)
        
        # MoveGroup action client
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
        self.planning_scene_pub = self.create_publisher(PlanningScene, '/planning_scene', 10)
        
        # TF for getting current pose
        self.tf_buffer = tf2_ros.Buffer(cache_time=rclpy.duration.Duration(seconds=10.0))
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)
        
        # Wait for services
        self.get_logger().info("Waiting for MoveGroup action server...")
        if not self.move_group_client.wait_for_server(timeout_sec=10.0):
            self.get_logger().error("MoveGroup action server not available!")
            return
        
        self.get_logger().info("Waiting for Cartesian path service...")
        if not self.cartesian_path_client.wait_for_service(timeout_sec=5.0):
            self.get_logger().warn("Cartesian path service not available - will use standard planning")
        
        if not self.execute_trajectory_client.wait_for_server(timeout_sec=5.0):
            self.get_logger().warn("Execute trajectory server not available")
        
        # Grasp orientations
        self.grasp_orientations = {
            'side': euler_to_quaternion(0.5*math.pi, 0, 0.5*math.pi)
        }
        
        self.get_logger().info("=== Interactive Pick & Place Controller Started ===")
        self.get_logger().info("✓ Cartesian path planning enabled")
        self.get_logger().info("Waiting for ArUco detections...")
    
    def pose_callback(self, msg: PoseArray):
        """Update detected object poses - IGNORED during movement"""
        if self.is_moving:
            return
        
        if len(msg.poses) != len(self.marker_ids):
            return
        
        for i, marker_id in enumerate(self.marker_ids):
            if marker_id in self.detected_objects:
                self.detected_objects[marker_id]['pose'] = msg.poses[i]
                self.update_collision_cylinder(marker_id, msg.poses[i])
        
        if len(msg.poses) > 0:
            self.valid_pose_available = True
            self.get_logger().info(f"Valid poses received for {len(msg.poses)} objects", 
                                  throttle_duration_sec=2.0)
    
    def ids_callback(self, msg: Int32MultiArray):
        """Update detected marker IDs"""
        if self.is_moving:
            return
        
        new_ids = list(msg.data)
        removed_ids = set(self.marker_ids) - set(new_ids)
        
        for marker_id in removed_ids:
            self.remove_collision_cylinder(marker_id)
            if marker_id in self.detected_objects:
                del self.detected_objects[marker_id]
        
        self.marker_ids = new_ids
        
        for marker_id in self.marker_ids:
            if marker_id not in self.detected_objects:
                self.detected_objects[marker_id] = {
                    'pose': None,
                    'name': OBJECT_MAP.get(marker_id, f'marker_{marker_id}')
                }
    
    def names_callback(self, msg: String):
        """Update object names"""
        if self.is_moving:
            return
        
        try:
            names = json.loads(msg.data)
            for i, marker_id in enumerate(self.marker_ids):
                if i < len(names) and marker_id in self.detected_objects:
                    self.detected_objects[marker_id]['name'] = names[i]
        except:
            pass
    
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
                f"Only {fraction*100:.1f}% of Cartesian path achieved - "
                f"may not reach target exactly")
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
        if max_velocity_scaling < 1.0:
            trajectory = self.scale_trajectory_speed(trajectory, max_velocity_scaling)
        
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
            point.time_from_start.sec = int(point.time_from_start.sec / scale_factor)
            point.time_from_start.nanosec = int(point.time_from_start.nanosec / scale_factor)
        
        return scaled_trajectory
    
    def move_relative_cartesian(self, delta_x=0.0, delta_y=0.0, delta_z=0.0, 
                                delta_roll=0.0, delta_pitch=0.0, delta_yaw=0.0):
        """
        Move relative to current pose using Cartesian path
        
        Args:
            delta_x/y/z: Position changes in meters
            delta_roll/pitch/yaw: Orientation changes in radians
        
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
        
        # Keep orientation (or modify if deltas provided)
        if delta_roll == 0.0 and delta_pitch == 0.0 and delta_yaw == 0.0:
            target_pose.orientation = current_pose.orientation
        else:
            # Apply orientation changes (implement if needed)
            target_pose.orientation = current_pose.orientation
        
        self.get_logger().info(
            f"Cartesian move: Δ({delta_x:.3f}, {delta_y:.3f}, {delta_z:.3f})")
        
        # Execute Cartesian path with single waypoint
        return self.move_cartesian([target_pose], eef_step=0.005)
    
    def pick_object(self, object_name, grasp_pose, marker_id, approach_direction="side"):
        """Execute pick operation with Cartesian paths for smooth motion"""
        
        # Step 1: Long-distance approach - use standard planning
        approach_pose = self.compute_approach_pose(grasp_pose, distance=0.20, direction=approach_direction)
        
        self.get_logger().info("Step 1: Moving to approach pose (standard planning)...")
        if not self.move_to_pose(approach_pose, planning_time=8.0):
            self.get_logger().error("Failed to reach approach pose")
            return False
        
        # Step 2: Open gripper
        self.get_logger().info("Step 2: Opening gripper...")
        if not self.control_gripper("open"):
            return False
        time.sleep(1.0)

        # Step 3: Attach object for collision-free approach
        self.get_logger().info("Step 3: Attaching object for collision avoidance...")
        self.attach_object_to_gripper(object_name)
        time.sleep(0.3)
        
        # Step 4: Move closer using CARTESIAN PATH
        self.get_logger().info("Step 4: Moving closer (Cartesian path)...")
        if not self.move_relative_cartesian(delta_x=0.10):
            self.get_logger().warn("Cartesian approach failed, trying standard planning")
            current_pose = self.get_current_pose()
            if current_pose:
                current_pose.position.x += 0.10
                if not self.move_to_pose(current_pose):
                    return False
            else:
                return False
        
        # Step 5: Close gripper
        self.get_logger().info("Step 5: Closing gripper...")
        if not self.control_gripper("close", 0.6):
            return False
        time.sleep(1.0)
        
        # Step 6: Lift using CARTESIAN PATH
        self.get_logger().info("Step 6: Lifting object (Cartesian path)...")
        if not self.move_relative_cartesian(delta_z=0.10):
            self.get_logger().warn("Cartesian lift failed, trying standard planning")
            current_pose = self.get_current_pose()
            if current_pose:
                current_pose.position.z += 0.10
                if not self.move_to_pose(current_pose):
                    return False
            else:
                return False
        
        return True
    
    def update_collision_cylinder(self, marker_id, pose, radius=0.03, height=0.15):
        """Create or update collision cylinder for detected object"""
        object_name = f"detected_object_{marker_id}"
        
        collision_object = CollisionObject()
        collision_object.header.frame_id = self.base_frame
        collision_object.header.stamp = self.get_clock().now().to_msg()
        collision_object.id = object_name

        height = max(0.05, (pose.position.z - 0.34) * 2)
        
        cylinder = SolidPrimitive()
        cylinder.type = SolidPrimitive.CYLINDER
        cylinder.dimensions = [height, radius]
        
        cylinder_pose = Pose()
        cylinder_pose.position.x = pose.position.x
        cylinder_pose.position.y = pose.position.y
        cylinder_pose.position.z = pose.position.z 
        cylinder_pose.orientation.w = 1.0

        collision_object.primitives.append(cylinder)
        collision_object.primitive_poses.append(cylinder_pose)
        collision_object.operation = CollisionObject.ADD
        
        planning_scene = PlanningScene()
        planning_scene.world.collision_objects.append(collision_object)
        planning_scene.is_diff = True
        
        self.planning_scene_pub.publish(planning_scene)
        
        if object_name not in self.collision_objects_added:
            self.collision_objects_added.add(object_name)
            self.get_logger().info(
                f"Added collision cylinder for marker {marker_id}")
    
    def remove_collision_cylinder(self, marker_id):
        """Remove collision cylinder"""
        object_name = f"detected_object_{marker_id}"
        
        if object_name not in self.collision_objects_added:
            return
        
        collision_object = CollisionObject()
        collision_object.id = object_name
        collision_object.operation = CollisionObject.REMOVE
        
        planning_scene = PlanningScene()
        planning_scene.world.collision_objects.append(collision_object)
        planning_scene.is_diff = True
        
        self.planning_scene_pub.publish(planning_scene)
        self.collision_objects_added.remove(object_name)
    
    def wait_for_valid_pose(self, timeout=10.0):
        """Wait for valid pose to be received"""
        self.get_logger().info("Waiting for valid pose from ArUco detection...")
        
        start_time = time.time()
        rate = self.create_rate(10)
        
        while not self.valid_pose_available:
            if time.time() - start_time > timeout:
                self.get_logger().error(f"Timeout waiting for valid pose after {timeout}s")
                return False
            
            rclpy.spin_once(self, timeout_sec=0.1)
            rate.sleep()
        
        self.get_logger().info("✓ Valid pose received!")
        return True
    
    def add_table_collision_object(self, table_name="work_table", x=0.8, y=0.0, z=0.4, 
                                  width=1.2, depth=0.8, height=0.05):
        """Add table collision object"""
        self.get_logger().info(f"Adding table '{table_name}'")
        
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
        
        return True
    
    def display_detected_objects(self):
        """Display detected objects"""
        print("\n" + "="*60)
        print("  DETECTED OBJECTS")
        print("="*60)
        
        if len(self.detected_objects) == 0:
            print("  No objects detected")
        else:
            for marker_id, obj_data in self.detected_objects.items():
                pose = obj_data.get('pose')
                name = obj_data.get('name', f'marker_{marker_id}')
                
                if pose:
                    print(f"  [{marker_id}] {name}")
                    print(f"      Position: ({pose.position.x:.3f}, "
                          f"{pose.position.y:.3f}, {pose.position.z:.3f})")
                else:
                    print(f"  [{marker_id}] {name} (no pose data)")
        
        print("="*60)
    
    def execute_pick_sequence(self, marker_id):
        """Execute pick sequence"""
        self.is_moving = True
        self.get_logger().info("🔒 Movement started - blocking pose updates")
        
        self.get_logger().info(f"\n{'='*60}")
        self.get_logger().info(f"Starting pick sequence for marker {marker_id}")
        self.get_logger().info(f"{'='*60}")
        
        obj_data = self.detected_objects.get(marker_id)
        if not obj_data or not obj_data.get('pose'):
            self.get_logger().error("No pose data available!")
            self.is_moving = False
            return False
        
        grasp_pose = obj_data['pose']
        object_name = f"detected_object_{marker_id}"
        
        success = self.pick_object(object_name, grasp_pose, marker_id)
        
        self.is_moving = False
        self.get_logger().info("🔓 Movement completed - resuming pose updates")
        
        if success:
            self.get_logger().info("✓ Pick sequence completed successfully!")
            print("\n✓ Object picked successfully!")
        else:
            self.get_logger().error("✗ Pick sequence failed!")
            print("\n✗ Pick failed!")
        
        return success
    
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
    
    def move_to_pose(self, target_pose, planning_time=5.0):
        """Move arm to target pose using standard planning"""
        goal = MoveGroup.Goal()
        goal.request.group_name = self.arm_group_name
        goal.request.num_planning_attempts = 20
        goal.request.allowed_planning_time = planning_time
        goal.request.max_velocity_scaling_factor = 0.15
        goal.request.max_acceleration_scaling_factor = 0.15
        goal.request.planner_id = "LBKPIECE1"
        
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
        
        future = self.move_group_client.send_goal_async(goal)
        rclpy.spin_until_future_complete(self, future)
        
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().error("Move goal rejected")
            return False
        
        result_future = goal_handle.get_result_async()
        rclpy.spin_until_future_complete(self, result_future)
        
        result = result_future.result()
        success = result.result.error_code.val == MoveItErrorCodes.SUCCESS
        
        if success:
            self.get_logger().info("✓ Move completed")
        else:
            self.get_logger().error(f"✗ Move failed: {result.result.error_code.val}")
        
        return success
    
    def control_gripper(self, state, gripper_value=None):
        """Control gripper"""
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
    
    def attach_object_to_gripper(self, object_name):
        """Attach object to gripper"""
        attached_object = AttachedCollisionObject()
        attached_object.link_name = self.end_effector_frame
        attached_object.object.id = object_name
        attached_object.object.operation = CollisionObject.ADD
        
        attached_object.touch_links = [
            self.end_effector_frame,
            "robotiq_85_left_knuckle_link",
            "robotiq_85_right_knuckle_link",
            "robotiq_85_left_finger_link",
            "robotiq_85_right_finger_link"
        ]
        
        planning_scene = PlanningScene()
        planning_scene.robot_state.attached_collision_objects.append(attached_object)
        planning_scene.is_diff = True
        
        self.planning_scene_pub.publish(planning_scene)
        time.sleep(0.3)


def main(args=None):
    rclpy.init(args=args)
    
    controller = InteractivePickPlaceController()
    
    # Add static environment
    chair_seat_depth = 0.5
    chair_seat_x = -0.43
    chair_seat_width = 0.6
    chair_seat_y = -0.1
    chair_seat_z = 0
    chair_seat_height = 0.72

    controller.add_table_collision_object("wheel_chair_seat", x=chair_seat_x, y=chair_seat_y,z=chair_seat_z, 
                                         width=chair_seat_width, depth=chair_seat_depth, height=chair_seat_height)
    
    chair_back_depth = 0.35
    chair_back_x = -0.43
    chair_back_width = 0.34
    chair_back_y = 0.3
    chair_back_height = 1.26
    chair_back_z = -0.54 + chair_back_height / 2
    controller.add_table_collision_object("wheel_chair_back", x=chair_back_x, y=chair_back_y, z=chair_back_z, 
                                         width=chair_back_width, depth=chair_back_depth, height=chair_back_height)
    
    table_x = 0.7
    table_y = -0.3
    table_z = -0.02
    table_height = 0.72
    controller.add_table_collision_object("work_table", x=table_x, y=table_y, z=table_z, 
                                         width=0.8, depth=1.2, height=table_height)
    
    controller.get_logger().info("✓ Static environment added")
    
    # Wait for valid pose before executing
    if controller.wait_for_valid_pose(timeout=15.0):
        controller.display_detected_objects()
        
        # Execute pick for marker ID 6
        controller.execute_pick_sequence(6)
    else:
        controller.get_logger().error("Failed to receive valid pose - aborting")
    
    # Keep spinning for any remaining operations
    try:
        rclpy.spin(controller)
    except KeyboardInterrupt:
        pass
    finally:
        controller.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()