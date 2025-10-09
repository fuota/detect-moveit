#!/usr/bin/env python3
"""
Pick and Place script optimized for Kinova 7-DOF robotic arm.

This script provides high-level pick and place functionality using MoveIt's
pick/place interface, specifically configured for:
- Kinova 7-DOF arm kinematics and workspace
- Robotiq or Kinova gripper configurations
- Top-down and side approach strategies
- Multiple grasp/place candidates for improved success rates

Configuration Notes:
- Adjust gripper_joint_names based on your specific gripper
- Modify pose coordinates to match your workspace setup
- Update approach distances based on your end-effector size
"""

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from moveit_msgs.msg import Grasp, GripperTranslation, PlaceLocation, MoveItErrorCodes
from moveit_msgs.action import MoveGroup
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from geometry_msgs.msg import Pose, PoseStamped
from std_msgs.msg import Header
from shape_msgs.msg import SolidPrimitive
from moveit_msgs.msg import CollisionObject, PlanningScene, AttachedCollisionObject
from geometry_msgs.msg import Point
import time
import math

# Quaternion utility functions for common grasping orientations
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
    

    # Roll=90°, Pitch=90°, Yaw=0
    orientations['side'] = euler_to_quaternion(0.5*math.pi, 0, 0.5*math.pi)
   
    return orientations

class GraspPlanner(Node):
    def __init__(self):
        super().__init__('grasp_planner')
        
        # Kinova 7-DOF arm configuration
        self.arm_group_name = "manipulator"  # Updated to manipulator as specified
        self.gripper_group_name = "gripper"  # Kinova gripper group
        self.base_frame = "base_link"
        self.end_effector_frame = "end_effector_link"
        
        # Kinova gripper joint names (Robotiq 85 gripper)
        # Based on your joint_states output
        self.gripper_joint_names = [
            "robotiq_85_left_knuckle_joint",
            "robotiq_85_right_knuckle_joint"
        ]
        
        # Robotiq 85 gripper position values (corrected based on your specs)
        # Maximum close: left +46°, right -46°
        # Convert degrees to radians: 46° = 46 * π/180 ≈ 0.8029 rad
        self.gripper_open_position = 0.0
        self.gripper_max_close_left = math.radians(46)   # +46° = +0.8029 rad
        self.gripper_max_close_right = math.radians(-46)  # -46° = -0.8029 rad

        self.grasp_orientations = get_common_grasp_orientations()

        
        # Initialize MoveGroup action client for ROS 2
        self.move_group_client = ActionClient(self, MoveGroup, '/move_action')
        
        # Planning scene publisher for adding/removing objects
        self.planning_scene_pub = self.create_publisher(
            PlanningScene, '/planning_scene', 10)
        
        # Wait for action server
        self.get_logger().info("Waiting for MoveGroup action server...")
        if not self.move_group_client.wait_for_server(timeout_sec=10.0):
            self.get_logger().error("MoveGroup action server not available!")
            return
        
        self.get_logger().info("Initialized GraspPlanner for Kinova 7-DOF arm with manipulator group")
        
        # Get common orientations
        self.get_logger().info(f"Available grasp orientations: {list(self.grasp_orientations.keys())}")
    

    def set_grasp_orientation(self, pose, orientation_name):
        """Set pose orientation using predefined orientation name"""
        if orientation_name not in self.grasp_orientations:
            self.get_logger().error(f"Unknown orientation: {orientation_name}")
            self.get_logger().info(f"Available orientations: {list(self.grasp_orientations.keys())}")
            return False
            
        qx, qy, qz, qw = self.grasp_orientations[orientation_name]
        pose.orientation.x = qx
        pose.orientation.y = qy
        pose.orientation.z = qz
        pose.orientation.w = qw
        
        self.get_logger().info(f"Set orientation to '{orientation_name}': qx={qx:.3f}, qy={qy:.3f}, qz={qz:.3f}, qw={qw:.3f}")
        return True

    def compute_grasp_pose(self, grasp_pose, approach_distance=0.15, approach_direction="side"):
        """Compute pre-grasp pose offset from grasp pose"""
        pre_grasp_pose = Pose()
        pre_grasp_pose.position.x = grasp_pose.position.x
        pre_grasp_pose.position.y = grasp_pose.position.y
        pre_grasp_pose.position.z = grasp_pose.position.z
        
        if approach_direction == "top_down":
            pre_grasp_pose.position.z += approach_distance
            self.set_grasp_orientation(pre_grasp_pose, "top")
        elif approach_direction == "side":
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
        
        # Set pose goal
        from moveit_msgs.msg import Constraints, PositionConstraint, OrientationConstraint
        constraints = Constraints()
        
        # Position constraint
        pos_constraint = PositionConstraint()
        pos_constraint.header.frame_id = self.base_frame
        pos_constraint.link_name = self.end_effector_frame
        pos_constraint.target_point_offset.x = 0.0
        pos_constraint.target_point_offset.y = 0.0
        pos_constraint.target_point_offset.z = 0.0
        
        # Create bounding box around target position
        from shape_msgs.msg import SolidPrimitive
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
        from moveit_msgs.msg import RobotTrajectory
        from builtin_interfaces.msg import Duration
        
        # Create action client for gripper group
        gripper_client = ActionClient(self, MoveGroup, '/move_action')
        
        if not gripper_client.wait_for_server(timeout_sec=5.0):
            self.get_logger().error("Gripper MoveGroup action server not available!")
            return False
        
        # Create goal for gripper movement
        goal = MoveGroup.Goal()
        goal.request.group_name = self.gripper_group_name  # "gripper"
        goal.request.num_planning_attempts = 5
        goal.request.allowed_planning_time = 1.0
        goal.request.max_velocity_scaling_factor = 0.5
        goal.request.max_acceleration_scaling_factor = 0.5
        
        # Set joint goal for gripper
        from moveit_msgs.msg import JointConstraint, Constraints
        constraints = Constraints()
        
        # Determine gripper positions based on state
        if state == "open":
            # Robotiq 85: Both knuckles to 0.0 for open
            positions = [self.gripper_open_position, self.gripper_open_position]
            self.get_logger().info("Opening Robotiq 85 gripper (positions: [0.0, 0.0])")
        elif state == "close":
            # Robotiq 85: Left +46°, right -46° for maximum close
            if gripper_value is not None:
                # Scale the gripper_value (0.0 to 1.0) to actual joint limits
                # gripper_value = 0.0 -> open, gripper_value = 1.0 -> max close
                left_pos = self.gripper_max_close_left * gripper_value
                right_pos = self.gripper_max_close_right * gripper_value
                positions = [left_pos, right_pos]
            else:
                # Default to 80% close for safe grasping
                positions = [self.gripper_max_close_left * 0.8, self.gripper_max_close_right * 0.8]
            self.get_logger().info(f"Closing Robotiq 85 gripper (positions: [{positions[0]:.3f}, {positions[1]:.3f}] rad)")
        elif state == "pre_grasp":
            # Partially closed (50% of max)
            left_pos = self.gripper_max_close_left * 0.5
            right_pos = self.gripper_max_close_right * 0.5
            positions = [left_pos, right_pos]
            self.get_logger().info(f"Setting Robotiq 85 to pre-grasp (positions: [{left_pos:.3f}, {right_pos:.3f}] rad)")
        else:
            self.get_logger().error(f"Unknown gripper state: {state}")
            return False
        
        # Create joint constraints for each gripper joint
        for i, joint_name in enumerate(self.gripper_joint_names):
            joint_constraint = JointConstraint()
            joint_constraint.joint_name = joint_name
            joint_constraint.position = positions[i]
            joint_constraint.tolerance_above = 0.005
            joint_constraint.tolerance_below = 0.005
            joint_constraint.weight = 1.0
            constraints.joint_constraints.append(joint_constraint)
        
        goal.request.goal_constraints.append(constraints)
        
        # Send goal and wait for result
        self.get_logger().info(f"Sending gripper {state} command...")
        future = gripper_client.send_goal_async(goal)
        rclpy.spin_until_future_complete(self, future)
        
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().error("Gripper goal rejected")
            gripper_client.destroy()
            return False
        
        # Wait for execution to complete
        result_future = goal_handle.get_result_async()
        rclpy.spin_until_future_complete(self, result_future)
        
        result = result_future.result()
        gripper_client.destroy()
        
        if result.result.error_code.val == MoveItErrorCodes.SUCCESS:
            self.get_logger().info(f"Gripper {state} completed successfully")
            return True
        else:
            self.get_logger().error(f"Gripper {state} failed with error code: {result.result.error_code.val}")
            return False

    def open_gripper(self, openness=0.0):
        """Open gripper with specific openness level
        
        Args:
            openness (float): 0.0 = fully open, higher values = less open
        """
        return self.control_gripper("open")

    def close_gripper(self, grip_force=0.8):
        """Close Robotiq 85 gripper with specific grip force
        
        Args:
            grip_force (float): 0.0 = open, 1.0 = maximum close (±46°)
                              0.8 = 80% close (±36.8°) - recommended for most objects
        """
        return self.control_gripper("close", grip_force)
    

    
    def pick_object(self, object_name, grasp_pose, approach_direction="side"):
        """Simplified pick operation for Kinova 7-DOF arm"""
        self.get_logger().info(f"Planning pick for object: {object_name} at x={grasp_pose.position.x:.3f}, y={grasp_pose.position.y:.3f}, z={grasp_pose.position.z:.3f}")
        
        # Move to approach pose (offset from grasp pose)
        approach_pose = self.compute_grasp_pose(grasp_pose, approach_distance=0.22, approach_direction=approach_direction)

        # Step 1: Move to approach pose
        self.get_logger().info(f"Moving to approach pose at x={approach_pose.position.x:.3f}, y={approach_pose.position.y:.3f}, z={approach_pose.position.z:.3f}...")
        if not self.move_to_pose(approach_pose):
            self.get_logger().error("Failed to reach approach pose")
            return False
        
        # Step 2: Open gripper before grasping
        self.get_logger().info("Opening gripper...")
        if not self.open_gripper():
            self.get_logger().error("Failed to open gripper")
            return False
        
        time.sleep(1.0)

        

        # Modify approach pose x position to be closer to the object
        approach_pose.position.x += 0.05  # Move 10cm closer to object

        # ===== NOW attach after grasping =====
        self.get_logger().info("Attaching cylinder to gripper for lifting...")
        if not self.attach_cylinder_to_gripper("target_cylinder"):
            self.get_logger().error("Failed to attach cylinder")
            return False
        # ===== END OF CORRECTION =====
        
        # Step 3: Move to the modified approach pose (closer to object)
        self.get_logger().info(f"Moving closer to object at x={approach_pose.position.x:.3f}, y={approach_pose.position.y:.3f}, z={approach_pose.position.z:.3f}...")
        if not self.move_to_pose(approach_pose):
            self.get_logger().error("Failed to reach closer position")
            return False
        
        # Step 4: Close gripper to grasp object
        self.get_logger().info("Closing gripper to grasp object...")
        if not self.close_gripper(0.6):
            self.get_logger().error("Failed to close gripper")
            return False
        
        time.sleep(1.0)
        
        
        # ===== END =====
        
        # Step 5: Lift object slightly
        approach_pose.position.z += 0.1  # Lift 10cm
        
        self.get_logger().info("Lifting object...")
        if not self.move_to_pose(approach_pose):
            self.get_logger().error("Failed to lift object")
            return False
        
        self.get_logger().info("Pick operation completed successfully")
        return True
    def place_object(self, object_name, place_pose):
        """Simplified place operation for Kinova 7-DOF arm"""
        self.get_logger().info(f"Planning place for object: {object_name}")
        
        # Move to approach pose above place location
        approach_pose = Pose()
        approach_pose.position.x = place_pose.position.x
        approach_pose.position.y = place_pose.position.y
        approach_pose.position.z = place_pose.position.z + 0.15  # 15cm above
        approach_pose.orientation = place_pose.orientation
        
        self.get_logger().info("Moving to place approach pose...")
        if not self.move_to_pose(approach_pose):
            self.get_logger().error("Failed to reach place approach pose")
            return False
        
        # Move to place pose
        self.get_logger().info("Moving to place pose...")
        if not self.move_to_pose(place_pose):
            self.get_logger().error("Failed to reach place pose")
            return False
        
        # Open gripper to release object
        self.get_logger().info("Opening gripper to release object...")
        if not self.open_gripper():
            self.get_logger().error("Failed to open gripper")
            return False
        
        # Wait for gripper to open
        time.sleep(1.0)
        
        # Retreat from object
        retreat_pose = Pose()
        retreat_pose.position.x = place_pose.position.x
        retreat_pose.position.y = place_pose.position.y
        retreat_pose.position.z = place_pose.position.z + 0.1  # Retreat 10cm
        retreat_pose.orientation = place_pose.orientation
        
        self.get_logger().info("Retreating from placed object...")
        if not self.move_to_pose(retreat_pose):
            self.get_logger().error("Failed to retreat from object")
            return False
        
        self.get_logger().info("Place operation completed successfully")
        return True
    
    def allow_collisions_with_cylinder(self, object_name="target_cylinder", allow=True):
        """Allow or disallow collisions between gripper and cylinder"""
        from moveit_msgs.msg import AllowedCollisionMatrix, AllowedCollisionEntry
        
        self.get_logger().info(f"{'Allowing' if allow else 'Disallowing'} collisions between gripper and {object_name}...")
        
        # Define all gripper links
        gripper_links = [
            "end_effector_link",
            "robotiq_85_base_link",
            "robotiq_85_left_knuckle_link",
            "robotiq_85_right_knuckle_link", 
            "robotiq_85_left_finger_link",
            "robotiq_85_right_finger_link",
            "robotiq_85_left_inner_knuckle_link",
            "robotiq_85_right_inner_knuckle_link",
            "robotiq_85_left_finger_tip_link",
            "robotiq_85_right_finger_tip_link"
        ]
        
        # Create planning scene with ACM
        planning_scene = PlanningScene()
        planning_scene.is_diff = True
        
        # Create ACM with gripper links + object
        acm = AllowedCollisionMatrix()
        acm.entry_names = gripper_links + [object_name]
        acm.default_entry_names = []
        acm.default_entry_values = []
        
        # Build collision matrix
        num_entries = len(acm.entry_names)
        cylinder_idx = num_entries - 1  # Last entry is the cylinder
        
        for i in range(num_entries):
            entry = AllowedCollisionEntry()
            entry.enabled = [False] * num_entries  # Default: no collisions allowed
            
            if i < len(gripper_links):
                # This is a gripper link - set collision with cylinder
                entry.enabled[cylinder_idx] = allow
            elif i == cylinder_idx:
                # This is the cylinder - set collision with all gripper links
                for j in range(len(gripper_links)):
                    entry.enabled[j] = allow
            
            acm.entry_values.append(entry)
        
        planning_scene.allowed_collision_matrix = acm
        self.planning_scene_pub.publish(planning_scene)
        time.sleep(0.5)
        
        self.get_logger().info(f"Collisions between gripper and {object_name} are now {'ALLOWED' if allow else 'DISALLOWED'}")
        return True
    def add_table_collision_object(self, table_name="work_table", x=0.8, y=0.0, z=0.4, 
                                  width=1.2, depth=0.8, height=0.05):
        """Add a table as a collision object to the planning scene
        
        Args:
            table_name (str): Name of the table collision object
            x, y, z (float): Position coordinates for table center
            width, depth, height (float): Table dimensions in meters
        """
        self.get_logger().info(f"Adding table collision object '{table_name}' to planning scene")
        
        # Create collision object
        collision_object = CollisionObject()
        collision_object.header.frame_id = self.base_frame
        collision_object.header.stamp = self.get_clock().now().to_msg()
        collision_object.id = table_name
        
        # Define table as a box
        box = SolidPrimitive()
        box.type = SolidPrimitive.BOX
        box.dimensions = [width, depth, height]  # [x, y, z] dimensions
        
        # Set table pose
        table_pose = Pose()
        table_pose.position.x = float(x)
        table_pose.position.y = float(y)
        table_pose.position.z = float(z)
        table_pose.orientation.w = 1.0
        
        collision_object.primitives.append(box)
        collision_object.primitive_poses.append(table_pose)
        collision_object.operation = CollisionObject.ADD
        
        # Create planning scene message
        planning_scene = PlanningScene()
        planning_scene.world.collision_objects.append(collision_object)
        planning_scene.is_diff = True
        
        # Publish to planning scene
        self.planning_scene_pub.publish(planning_scene)
        time.sleep(0.5)  # Give time for the message to be processed
        
        self.get_logger().info(f"Table '{table_name}' added to planning scene at [{x:.3f}, {y:.3f}, {z:.3f}] with dimensions [{width:.2f}x{depth:.2f}x{height:.2f}m]")
        return True
    
    def add_cylinder_collision_object(self, cylinder_name="target_cylinder", x=0.8, y=-0.336, z=0.5,
                                    radius=0.05, height=0.15):
        """Add a cylinder as a collision object to the planning scene
        
        Args:
            cylinder_name (str): Name of the cylinder collision object
            x, y, z (float): Position coordinates for cylinder center
            radius (float): Cylinder radius in meters
            height (float): Cylinder height in meters
        """
        self.get_logger().info(f"Adding cylinder collision object '{cylinder_name}' to planning scene")
        
        # Create collision object
        collision_object = CollisionObject()
        collision_object.header.frame_id = self.base_frame
        collision_object.header.stamp = self.get_clock().now().to_msg()
        collision_object.id = cylinder_name
        
        # Define cylinder
        cylinder = SolidPrimitive()
        cylinder.type = SolidPrimitive.CYLINDER
        cylinder.dimensions = [height, radius]  # [height, radius] for cylinder
        
        # Set cylinder pose
        cylinder_pose = Pose()
        cylinder_pose.position.x = float(x)
        cylinder_pose.position.y = float(y)
        cylinder_pose.position.z = float(z)
        cylinder_pose.orientation.w = 1.0
        
        collision_object.primitives.append(cylinder)
        collision_object.primitive_poses.append(cylinder_pose)
        collision_object.operation = CollisionObject.ADD
        
        # Create planning scene message
        planning_scene = PlanningScene()
        planning_scene.world.collision_objects.append(collision_object)
        planning_scene.is_diff = True
        
        # Publish to planning scene
        self.planning_scene_pub.publish(planning_scene)
        time.sleep(0.5)  # Give time for the message to be processed
        
        self.get_logger().info(f"Cylinder '{cylinder_name}' added to planning scene at [{x:.3f}, {y:.3f}, {z:.3f}] with radius={radius:.3f}m, height={height:.3f}m")
        return True
    
    def attach_cylinder_to_gripper(self, object_name="target_cylinder"):
        """Attach cylinder to end effector to allow collision-free grasping"""
        self.get_logger().info(f"Attaching {object_name} to end effector...")
        
        # Create attached collision object
        attached_object = AttachedCollisionObject()
        attached_object.link_name = self.end_effector_frame
        attached_object.object.id = object_name
        attached_object.object.operation = CollisionObject.ADD
        
        # Define touch links (gripper links that are allowed to touch the object)
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
        
        # Create planning scene message
        planning_scene = PlanningScene()
        planning_scene.robot_state.attached_collision_objects.append(attached_object)
        planning_scene.is_diff = True
        
        # Publish to planning scene
        self.planning_scene_pub.publish(planning_scene)
        time.sleep(0.5)
        
        self.get_logger().info(f"Object {object_name} attached to {self.end_effector_frame}")
        return True

    def detach_cylinder_from_gripper(self, object_name="target_cylinder"):
        """Detach cylinder from end effector"""
        self.get_logger().info(f"Detaching {object_name} from end effector...")
        
        # Create detached collision object
        attached_object = AttachedCollisionObject()
        attached_object.object.id = object_name
        attached_object.object.operation = CollisionObject.REMOVE
        
        # Create planning scene message
        planning_scene = PlanningScene()
        planning_scene.robot_state.attached_collision_objects.append(attached_object)
        planning_scene.is_diff = True
        
        # Publish to planning scene
        self.planning_scene_pub.publish(planning_scene)
        time.sleep(0.5)
        
        self.get_logger().info(f"Object {object_name} detached")
        return True
    
 
def main(args=None):
    rclpy.init(args=args)
    
    # Create instance of GraspPlanner for Kinova 7-DOF arm
    grasp_planner = GraspPlanner()
    
    # Add collision objects to planning scene
    grasp_planner.get_logger().info("Setting up collision objects in planning scene...")
    

    # TEMPORARY: Comment out collision objects to test if they're causing the issue
    # Uncomment these lines one by one to isolate the problem
    chair_seat_depth = 0.5
    chair_seat_x = -0.43
    chair_seat_width= 0.6
    chair_seat_y = -0.1
    chair_seat_z = 0
    chair_seat_height = 0.72  # Thinner table for Kinova


    grasp_planner.add_table_collision_object("wheel_chair_seat", x=chair_seat_x, y=chair_seat_y, z=chair_seat_z, 
                                           width=chair_seat_width, depth=chair_seat_depth, height=chair_seat_height)
    
    chair_back_depth = 0.35
    chair_back_x = -0.43
    chair_back_width = 0.34
    chair_back_y = 0.3
    chair_back_height = 1.26  # Thinner table for Kinova
    chair_back_z = -0.54+chair_back_height/2
    grasp_planner.add_table_collision_object("wheel_chair_back", x=chair_back_x, y=chair_back_y, z=chair_back_z, 
                                           width=chair_back_width, depth=chair_back_depth, height=chair_back_height)
    # Add table collision object (COMMENTED OUT FOR TESTING)
    table_x = 0.7 
    table_y = -0.5
    table_z = 0.2 
    table_height = 0.7  # Thinner table for Kinova
    grasp_planner.add_table_collision_object("work_table", x=table_x, y=table_y, z=table_z, 
                                           width=0.8, depth=1.2, height=table_height)
    
    # Add cylinder collision object on top of table (COMMENTED OUT FOR TESTING)
    cylinder_x = 0.5  # Adjusted X position for Kinova workspace
    cylinder_y = -0.336
    cylinder_height = 0.15  # Height of the cylinder
    cylinder_z = table_z + table_height/2 + cylinder_height/2 
    cylinder_radius = 0.025 
    grasp_planner.add_cylinder_collision_object("target_cylinder", x=cylinder_x, y=cylinder_y, z=cylinder_z,
                                        radius=cylinder_radius, height=cylinder_height)

    # grasp_planner.get_logger().info("Collision objects setup completed")
    time.sleep(1.0)  # Give time for planning scene to update
    
    # Define target grasp pose (using fixed coordinates for testing)
    cylinder_x = cylinder_x
    cylinder_y = cylinder_y
    cylinder_z = cylinder_z  # Fixed height for testing
    
    grasp_pose = Pose()
    grasp_pose.position.x = cylinder_x
    grasp_pose.position.y = cylinder_y
    grasp_pose.position.z = cylinder_z 

    grasp_planner.get_logger().info(f"Target grasp pose: x={grasp_pose.position.x:.3f}, y={grasp_pose.position.y:.3f}, z={grasp_pose.position.z:.3f}")
    
    # Execute pick using the grasp_planner
    if grasp_planner.pick_object("cylinder_object", grasp_pose):
        grasp_planner.get_logger().info("Pick successful!")
        
    #     # Define place pose (move to a different location)
    #     place_pose = Pose()
    #     place_pose.position.x = 0.4
    #     place_pose.position.y = 0.2
    #     place_pose.position.z = 0.4  # Slightly above table
    #     place_pose.orientation = grasp_pose.orientation  # Same orientation
        
    #     grasp_planner.get_logger().info(f"Target place pose: x={place_pose.position.x:.3f}, y={place_pose.position.y:.3f}, z={place_pose.position.z:.3f}")
        
    #     # Execute place
    #     if grasp_planner.place_object("cylinder_object", place_pose):
    #         grasp_planner.get_logger().info("Place successful! Pick and place operation completed.")
    #     else:
    #         grasp_planner.get_logger().error("Place operation failed.")
    # else:
    #     grasp_planner.get_logger().error("Pick operation failed.")
    
    # Keep node alive briefly to see final messages
    rclpy.spin_once(grasp_planner, timeout_sec=1.0)
    grasp_planner.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()