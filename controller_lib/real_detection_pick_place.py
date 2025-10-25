#!/usr/bin/env python3
"""
Real Detection Pick and Place Controller - Uses ArUco marker detection
Inherits from MoveItController base class for all MoveIt2 functionality
"""

import rclpy
from geometry_msgs.msg import Pose, PoseArray
from std_msgs.msg import Int32MultiArray, String
from moveit_msgs.msg import CollisionObject
import time
import math
import json
from moveit_controller import MoveItController


# Object name mapping
OBJECT_MAP = {
    6: "water_bottle",
    7: "medicine_bottle"
}


class RealDetectionPickPlaceController(MoveItController):
    """
    Real robot controller with ArUco detection
    Inherits all MoveIt2 functionality from MoveItController
    """
    
    def __init__(self):
        super().__init__(node_name='real_detection_pick_place_controller')
        
        # Detected objects storage
        self.detected_objects = {}  # {marker_id: {'pose': Pose, 'name': str}}
        self.marker_ids = []
        
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
        
        self.get_logger().info("=== Real Detection Pick & Place Controller Started ===")
        self.get_logger().info("Waiting for ArUco detections...")
    
    # ==================== ARUCO DETECTION CALLBACKS ====================
    
    def pose_callback(self, msg: PoseArray):
        """Update detected object poses - IGNORED during movement"""
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
    
    def ids_callback(self, msg: Int32MultiArray):
        """Update detected marker IDs - IGNORED during movement"""
        if self.is_moving:
            return
         
        new_ids = list(msg.data)
        
        # Remove objects that are no longer detected
        removed_ids = set(self.marker_ids) - set(new_ids)
        for marker_id in removed_ids:
            self.remove_detected_object_collision(marker_id)
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
        if self.is_moving:
            return
        
        try:
            names = json.loads(msg.data)
            for i, marker_id in enumerate(self.marker_ids):
                if i < len(names) and marker_id in self.detected_objects:
                    self.detected_objects[marker_id]['name'] = names[i]
        except:
            pass
    
    # ==================== COLLISION OBJECT MANAGEMENT ====================
    
    def update_collision_cylinder(self, marker_id, pose, radius=0.03):
        """Create or update collision cylinder for detected object"""
        object_name = f"detected_object_{marker_id}"
        
        # Calculate height from table (z=0.09 is reference)
        height = (pose.position.z - 0.09) * 2
        
        # Add/update cylinder using parent class method
        self.add_cylinder_collision_object(
            object_name,
            x=pose.position.x,
            y=pose.position.y,
            z=pose.position.z,
            radius=radius,
            height=height)
    
    def remove_detected_object_collision(self, marker_id):
        """Remove collision cylinder for object no longer detected"""
        object_name = f"detected_object_{marker_id}"
        self.remove_collision_object(object_name)
    
    # ==================== UTILITY METHODS ====================
    
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
    
    # ==================== PICK EXECUTION ====================
    
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
        
        # Execute pick using parent class method
        success = self.pick_object(object_name, grasp_pose)
        
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
    
    # ==================== ENVIRONMENT SETUP ====================
    def add_real_object(self, name, dx, dy, dz, width, depth, height):
        """Add a real-world object as a collision box"""
        self.add_box_collision_object(
            name,
            x=dx + depth / 2,
            y=dy + width / 2,
            z=dz + height / 2,
            width=math.fabs(width),
            depth=math.fabs(depth),
            height=math.fabs(height))


    def setup_static_environment(self):
        """Setup static collision objects for real robot environment"""
        self.get_logger().info("Setting up static environment...")
        
        # Wheelchair seat
        chair_seat_depth = 0.5
        chair_seat_x = -0.43
        chair_seat_width = 0.6
        chair_seat_y = -0.1
        chair_seat_z = 0
        chair_seat_height = 1.8
        self.add_box_collision_object(
            "wheel_chair_seat", 
            x=chair_seat_x, y=chair_seat_y, z=chair_seat_z, 
            width=chair_seat_width, depth=chair_seat_depth, height=chair_seat_height)
        
        # Wheelchair back
        chair_back_depth = 0.35
        chair_back_x = -0.43
        chair_back_width = 0.34
        chair_back_y = 0.3
        chair_back_height = 1.8
        chair_back_z = -0.54 + chair_back_height / 2
        self.add_box_collision_object(
            "wheel_chair_back", 
            x=chair_back_x, y=chair_back_y, z=chair_back_z, 
            width=chair_back_width, depth=chair_back_depth, height=chair_back_height)
        
        # Big table
        # self.add_real_object("big_table",dx=0.13, dy=-0.53, dz=-0.37, width=-1, depth=0.765, height=0.715)
        table_depth = 0.765
        table_dx = 0.13
        table_width = -1
        table_dy = -0.53
        table_height = 0.715    
        table_dz = -0.37
        table_x = table_depth / 2 + table_dx
        table_y = table_width / 2 + table_dy
        table_z = table_height / 2 + table_dz
        self.add_box_collision_object(
            "big_table", 
            x=table_x, y=table_y, z=table_z, 
            width=math.fabs(table_width), depth=table_depth, height=table_height)
        
        # Small table
        # self.add_real_object("small_table", dx=0.295, dy=-0.315, dz=-0.37, width=0.55, depth=0.55, height=0.46)
        table1_depth = 0.55
        table1_dx = 0.295
        table1_width = 0.55
        table1_dy = -0.315
        table1_height = 0.46
        table1_dz = -0.37
        table1_x = table1_depth / 2 + table1_dx
        table1_y = table1_depth / 2 + table1_dy
        table1_z = table1_height / 2 + table1_dz
        self.add_box_collision_object(
            "small_table", 
            x=table1_x, y=table1_y, z=table1_z, 
            width=table1_width, depth=table1_depth, height=table1_height)
        
        # Wall
        # self.add_real_object("wall", dx=0.85, dy=-7.0, dz=-0.37, width=15.0, depth=0.3, height=4.0)
        # self.add_real_object("wall", dx=-7.0, dy=1.2, dz=-0.37, width=0.3, depth=15.0, height=4.0)

        # wall_depth = 0.3
        # wall_dx = 0.85
        # wall_width = 15.0
        # wall_dy = -7.0
        # wall_height = 4.0
        # wall_dz = -0.37
        # wall_x = wall_depth / 2 + wall_dx
        # wall_y = wall_depth / 2 + wall_dy
        # wall_z = wall_height / 2 + wall_dz
        # self.add_box_collision_object(
        #     "wall", 
        #     x=wall_x, y=wall_y, z=wall_z, 
        #     width=wall_width, depth=wall_depth, height=wall_height)
        
        # Ceiling
        # self.add_real_object("ceiling", dx=0.0, dy=-0.5, dz=0.6, width=1.5, depth=0.01, height=0.3)
        # ceiling_depth = 1.0
        # ceiling_dx = 0.01
        # ceiling_width = 1.5
        # ceiling_dy = -0.5
        # ceiling_height = 0.3
        # ceiling_dz = 0.6
        # ceiling_x = ceiling_depth / 2 + ceiling_dx
        # ceiling_y = ceiling_depth / 2 + ceiling_dy
        # ceiling_z = ceiling_height / 2 + ceiling_dz
        # self.add_box_collision_object(
        #     "ceiling", 
        #     x=ceiling_x, y=ceiling_y, z=ceiling_z, 
        #     width=ceiling_width, depth=ceiling_depth, height=ceiling_height)
        
        self.get_logger().info("✓ Static environment added")


def main(args=None):
    rclpy.init(args=args)
    
    controller = RealDetectionPickPlaceController()
    
    # Setup static environment
    controller.setup_static_environment()
    
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
