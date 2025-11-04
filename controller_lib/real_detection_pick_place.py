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
        self.picked_objects = set()  # Track which objects were picked
        
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
        
        # Update poses ONLY for currently visible objects
        for i, marker_id in enumerate(self.marker_ids):
            if marker_id in self.detected_objects:
                self.detected_objects[marker_id]['pose'] = msg.poses[i]
                # CHANGE THIS LINE - don't update collision if picked:
                if marker_id not in self.picked_objects:
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
        
        # # Remove objects that are no longer detected
        # removed_ids = set(self.marker_ids) - set(new_ids)
        # for marker_id in removed_ids:
        #     self.remove_detected_object_collision(marker_id)
        #     if marker_id in self.detected_objects:
        #         del self.detected_objects[marker_id]
        
        # self.marker_ids = new_ids
        
        # Initialize new objects
        self.marker_ids = new_ids
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
        height = (pose.position.z - 0.29) * 2
        
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
    def execute_pick_and_place_sequence(self, marker_id, place_offset_x=0.0, place_offset_y = 0.2, place_offset_z=0.0):
        """
        Execute complete pick and place sequence for selected object
        
        Args:
            marker_id: ArUco marker ID to pick
            place_offset_y: Y-axis offset for place location (default: 0.2m)
        
        Returns:
            bool: True if both pick and place succeed
        """
        # CRITICAL: Set movement flag to block all updates
        self.is_moving = True
        self.get_logger().info("🔒 Movement started - blocking all pose updates")
        
        self.get_logger().info(f"\n{'='*60}")
        self.get_logger().info(f"Starting pick and place sequence for marker {marker_id}")
        self.get_logger().info(f"{'='*60}")
        
        obj_data = self.detected_objects.get(marker_id)
        if not obj_data or not obj_data.get('pose'):
            self.get_logger().error("No pose data available!")
            self.is_moving = False
            return False
        
        grasp_pose = obj_data['pose']
        object_name = f"detected_object_{marker_id}"
        
        # ==================== PICK PHASE ====================
        self.get_logger().info("\n--- PICK PHASE ---")
        pick_success = self.pick_object(object_name, grasp_pose)
        
        if not pick_success:
            self.get_logger().error("✗ Pick operation failed!")
            print("\n✗ Pick failed!")
            self.is_moving = False
            self.get_logger().info("🔓 Movement completed - resuming pose updates")
            return False
        
        self.picked_objects.add(marker_id)
        self.remove_detected_object_collision(marker_id)
        
        self.get_logger().info("✓ Pick operation completed successfully!")
        print("\n✓ Object picked successfully!")
        
        # ==================== PLACE PHASE ====================
        self.get_logger().info("\n--- PLACE PHASE ---")
        
        # Get current pose after pick
        current_pose = self.get_current_pose()
        if not current_pose:
            self.get_logger().error("Failed to get current pose after pick!")
            self.is_moving = False
            self.get_logger().info("🔓 Movement completed - resuming pose updates")
            return False
        
        self.get_logger().info(
            f"Current pose after pick: x={current_pose.position.x:.3f}, "
            f"y={current_pose.position.y:.3f}, z={current_pose.position.z:.3f}")
        
        # Calculate place pose (offset in Y direction)
        place_pose = Pose()
        place_pose.position.x = current_pose.position.x + place_offset_x
        place_pose.position.y = current_pose.position.y + place_offset_y
        place_pose.position.z = current_pose.position.z + place_offset_z
        place_pose.orientation = current_pose.orientation
        
        self.get_logger().info(
            f"Target place pose: x={place_pose.position.x:.3f}, "
            f"y={place_pose.position.y:.3f}, z={place_pose.position.z:.3f}")
        
        # Execute place
        place_success = self.place_object(object_name, place_pose)
        if place_success:
            # Update the stored pose to the new location
            self.detected_objects[marker_id]['pose'] = place_pose
            self.picked_objects.discard(marker_id)  # No longer in gripper
            self.update_collision_cylinder(marker_id, place_pose)  # Immediately add collision at placed location

        
        # CRITICAL: Release movement flag to allow updates again
        self.is_moving = False
        self.get_logger().info("🔓 Movement completed - resuming pose updates")
        
        if place_success:
            self.get_logger().info("\n🎉 Place successful! Pick and place operation completed.")
            print("\n🎉 Pick and place completed successfully!")
        else:
            self.get_logger().error("✗ Place operation failed!")
            print("\n✗ Place failed!")
        
        return place_success
    
    def execute_pick_sequence(self, marker_id):
        """
        Execute only pick sequence (backward compatibility)
        For pick and place, use execute_pick_and_place_sequence()
        
        Args:
            marker_id: ArUco marker ID to pick
        
        Returns:
            bool: True if pick succeeds
        """
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
        if success:
            self.picked_objects.add(marker_id)
            self.remove_detected_object_collision(marker_id)
                
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
            x=dx + width / 2,
            y=dy + depth / 2,
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
        self.add_real_object("big_table",dx=0.18, dy=-0.49, dz=-0.37, width=0.765, depth=-1, height=0.715)
        
        # Small table
        self.add_real_object("small_table", dx=0.38, dy=-0.28, dz=-0.37, width=0.55, depth=0.55, height=0.66)
    
        
        # Wall
        self.add_real_object("wall", dx=1, dy=-7.0, dz=-0.37, width=0.3, depth=15, height=4.0)
        
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
        controller.execute_pick_and_place_sequence(6)
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
