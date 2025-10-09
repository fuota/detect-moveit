#!/usr/bin/env python3
"""
Interactive Pick and Place Controller for Kinova 7-DOF arm with ArUco Detection

This node:
1. Subscribes to ArUco detection topics
2. Creates collision cylinders for detected objects
3. Allows user to select object to pick by ID
4. Executes pick and place operations using MoveIt
"""

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from geometry_msgs.msg import Pose, PoseStamped, PoseArray
from std_msgs.msg import Int32MultiArray, String
from moveit_msgs.msg import MoveItErrorCodes, CollisionObject, PlanningScene, AttachedCollisionObject
from moveit_msgs.action import MoveGroup
from shape_msgs.msg import SolidPrimitive
from moveit_msgs.msg import JointConstraint, Constraints, PositionConstraint, OrientationConstraint
import time
import math
import json


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
        self.detected_objects = {}  # {marker_id: {'pose': Pose, 'name': str}}
        self.marker_ids = []
        
        # Collision object tracking
        self.collision_objects_added = set()
        
        # Movement state - CRITICAL: blocks updates during movement
        self.is_moving = False
        self.pose_received = False
        self.valid_pose_available = False
        
        # Subscribe to ArUco detection topics
        self.pose_sub = self.create_subscription(
            PoseArray,
            '/detected_objects/poses',
            self.pose_callback,
            10)
        self.ids_sub = self.create_subscription(
            Int32MultiArray,
            '/detected_objects/ids',
            self.ids_callback,
            10)
        self.names_sub = self.create_subscription(
            String,
            '/detected_objects/names',
            self.names_callback,
            10)
        
        # MoveGroup action client
        self.move_group_client = ActionClient(self, MoveGroup, '/move_action')
        
        # Planning scene publisher
        self.planning_scene_pub = self.create_publisher(PlanningScene, '/planning_scene', 10)
        
        # Wait for action server
        self.get_logger().info("Waiting for MoveGroup action server...")
        if not self.move_group_client.wait_for_server(timeout_sec=10.0):
            self.get_logger().error("MoveGroup action server not available!")
            return
        
        # Grasp orientations
        self.grasp_orientations = {
            'side': euler_to_quaternion(0.5*math.pi, 0, 0.5*math.pi)
        }
        
        self.get_logger().info("=== Interactive Pick & Place Controller Started ===")
        self.get_logger().info("Waiting for ArUco detections...")
    
    def pose_callback(self, msg: PoseArray):
        """Update detected object poses - IGNORED during movement"""
        # CRITICAL: Ignore all pose updates while moving
        # print("Pose callback triggered")
        if self.is_moving:
            return
        
        if len(msg.poses) != len(self.marker_ids):
            return
        
        # Update poses
        for i, marker_id in enumerate(self.marker_ids):
            if marker_id in self.detected_objects:
                self.detected_objects[marker_id]['pose'] = msg.poses[i]
                self.update_collision_cylinder(marker_id, msg.poses[i])
        
        # Mark that we have received valid poses
        if len(msg.poses) > 0:
            self.pose_received = True
            self.valid_pose_available = True
            # self.get_logger().info(f"Valid poses received for {len(msg.poses)} objects", 
                                #   throttle_duration_sec=2.0)
    
    def ids_callback(self, msg: Int32MultiArray):
        """Update detected marker IDs - IGNORED during movement"""
        # CRITICAL: Ignore all ID updates while moving
        # print("IDs callback triggered")
       
        if self.is_moving:
            return
         
        new_ids = list(msg.data)
        
        # Remove objects that are no longer detected
        removed_ids = set(self.marker_ids) - set(new_ids)
        for marker_id in removed_ids:
            self.remove_collision_cylinder(marker_id)
            if marker_id in self.detected_objects:
                del self.detected_objects[marker_id]
        
        self.marker_ids = new_ids
        
        # Initialize new objects
        for marker_id in self.marker_ids:
            if marker_id not in self.detected_objects:
                self.detected_objects[marker_id] = {
                    'pose': None,
                    'name': OBJECT_MAP.get(marker_id, f'marker_{marker_id}')
                }
    
    def names_callback(self, msg: String):
        """Update object names from JSON - IGNORED during movement"""
        # CRITICAL: Ignore all name updates while moving
        # print("Names callback triggered")
        if self.is_moving:
            return
        
        try:
            names = json.loads(msg.data)
            for i, marker_id in enumerate(self.marker_ids):
                if i < len(names) and marker_id in self.detected_objects:
                    self.detected_objects[marker_id]['name'] = names[i]
        except:
            pass
    
    def update_collision_cylinder(self, marker_id, pose, radius=0.03, height=0.15):
        """Create or update collision cylinder for detected object"""
        object_name = f"detected_object_{marker_id}"
        
        # Create collision object
        collision_object = CollisionObject()
        collision_object.header.frame_id = self.base_frame
        collision_object.header.stamp = self.get_clock().now().to_msg()
        collision_object.id = object_name

        # Calculate height from table (z=0.34 is table height)
        height = max(0.05, (pose.position.z - 0.34) * 2)
        
        # Define cylinder
        cylinder = SolidPrimitive()
        cylinder.type = SolidPrimitive.CYLINDER
        cylinder.dimensions = [height, radius]
        
        # Set pose
        cylinder_pose = Pose()
        cylinder_pose.position.x = pose.position.x
        cylinder_pose.position.y = pose.position.y
        cylinder_pose.position.z = pose.position.z 
        cylinder_pose.orientation.w = 1.0

        collision_object.primitives.append(cylinder)
        collision_object.primitive_poses.append(cylinder_pose)
        collision_object.operation = CollisionObject.ADD
        
        # Publish to planning scene
        planning_scene = PlanningScene()
        planning_scene.world.collision_objects.append(collision_object)
        planning_scene.is_diff = True
        
        self.planning_scene_pub.publish(planning_scene)
        
        if object_name not in self.collision_objects_added:
            self.collision_objects_added.add(object_name)
            self.get_logger().info(
                f"Added collision cylinder for marker {marker_id} at "
                f"({pose.position.x:.3f}, {pose.position.y:.3f}, {pose.position.z:.3f})")
    
    def remove_collision_cylinder(self, marker_id):
        """Remove collision cylinder for object no longer detected"""
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
        
        self.get_logger().info(f"Removed collision cylinder for marker {marker_id}")
    
    def wait_for_valid_pose(self, timeout=10.0):
        """Wait for valid pose to be received"""
        self.get_logger().info("Waiting for valid pose from ArUco detection...")
        
        start_time = time.time()
        
        while not self.valid_pose_available:
            if time.time() - start_time > timeout:
                self.get_logger().error(f"Timeout waiting for valid pose after {timeout}s")
                return False
            
            rclpy.spin_once(self, timeout_sec=0.1)
            time.sleep(0.1)
        
        self.get_logger().info("✓ Valid pose received!")
        return True
    
    def add_table_collision_object(self, table_name="work_table", x=0.8, y=0.0, z=0.4, 
                                  width=1.2, depth=0.8, height=0.05):
        """Add a table as a collision object to the planning scene"""
        self.get_logger().info(f"Adding table collision object '{table_name}'")
        
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
        
        self.get_logger().info(f"Table '{table_name}' added")
        return True
    
    def display_detected_objects(self):
        """Display currently detected objects"""
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
        """Execute complete pick sequence for selected object"""
        # CRITICAL: Set movement flag to block all updates
        self.is_moving = True
        self.get_logger().info("🔒 Movement started - blocking all pose updates")
        
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
        
        # Execute pick
        success = self.pick_object(object_name, grasp_pose, marker_id)
        
        # CRITICAL: Release movement flag to allow updates again
        self.is_moving = False
        self.get_logger().info("🔓 Movement completed - resuming pose updates")
        
        if success:
            self.get_logger().info("✓ Pick sequence completed successfully!")
            print("\n✓ Object picked successfully!")
        else:
            self.get_logger().error("✗ Pick sequence failed!")
            print("\n✗ Pick failed!")
        
        return success
    
    def pick_object(self, object_name, grasp_pose, marker_id, approach_direction="side"):
        """Execute pick operation"""
        # Step 1: Move to approach pose
        approach_pose = self.compute_approach_pose(grasp_pose, distance=0.20, direction=approach_direction)
        
        self.get_logger().info("Step 1: Moving to approach pose...")
        if not self.move_to_pose(approach_pose):
            return False
        
        # Step 2: Open gripper
        self.get_logger().info("Step 2: Opening gripper...")
        if not self.control_gripper("open"):
            return False
        time.sleep(1.0)

        # Step 3: Attach object (for collision-free approach)
        self.get_logger().info("Step 3: Attaching object for collision-free approach...")
        self.attach_object_to_gripper(object_name)
        
        # Step 4: Move closer
        approach_pose.position.x += 0.10
        self.get_logger().info("Step 4: Moving closer to object...")
        if not self.move_to_pose(approach_pose):
            return False
        
        # Step 5: Close gripper
        self.get_logger().info("Step 5: Closing gripper...")
        if not self.control_gripper("close", 0.6):
            return False
        time.sleep(1.0)
        
        # Step 6: Lift
        approach_pose.position.z += 0.10
        self.get_logger().info("Step 6: Lifting object...")
        if not self.move_to_pose(approach_pose):
            return False
        
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
    
    def move_to_pose(self, target_pose, planning_time=5.0):
        """Move arm to target pose"""
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
    
    # Add static environment objects
    chair_seat_depth = 0.5
    chair_seat_x = -0.43
    chair_seat_width = 0.6
    chair_seat_y = -0.1
    chair_seat_z = 0
    chair_seat_height = 1.8

    controller.add_table_collision_object("wheel_chair_seat", x=chair_seat_x, y=chair_seat_y, z=chair_seat_z, 
                                         width=chair_seat_width, depth=chair_seat_depth, height=chair_seat_height)
    
    chair_back_depth = 0.35
    chair_back_x = -0.43
    chair_back_width = 0.34
    chair_back_y = 0.3
    chair_back_height = 1.8
    chair_back_z = -0.54 + chair_back_height / 2
    controller.add_table_collision_object("wheel_chair_back", x=chair_back_x, y=chair_back_y, z=chair_back_z, 
                                         width=chair_back_width, depth=chair_back_depth, height=chair_back_height)
    
    table_x = 0.7
    table_y = -0.3
    table_z = -0.02
    # table_height = 0.72
    table_height = 0.78
    controller.add_table_collision_object("big_table", x=table_x, y=table_y, z=table_z, 
                                         width=0.8, depth=1.2, height=table_height)
    

    table1_depth = 0.55
    table1_x = 0.275
    table1_width = 0.55
    table1_y = 0.605
    table1_height = 0.46
    table1_z = -0.14
    # table_height = 0.72
    table1_height = 0.78
    controller.add_table_collision_object("small_table", x=table1_x, y=table1_y, z=table1_z, 
                                         width=table1_width, depth=table1_depth, height=table1_height)


    # controller.add_table_collision_object("separator", x=0.7, y=0.3, z=0.8, 
    #                                      width=0.5, depth=0.2, height=1.0)
    
    controller.add_table_collision_object("separator1", x=0.7, y=-0.7, z=0.8, 
                                         width=0.5, depth=0.2, height=1.0)
    controller.get_logger().info("Static environment added")
    
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