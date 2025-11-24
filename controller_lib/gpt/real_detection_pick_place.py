#!/usr/bin/env python3
"""
Real Detection Pick and Place Controller - Uses ArUco marker detection
Inherits from MoveItController base class for all MoveIt2 functionality
"""

import rclpy
from geometry_msgs.msg import Pose, PoseArray
from std_msgs.msg import Int32MultiArray, String
from moveit_msgs.msg import CollisionObject
from std_srvs.srv import Trigger
from rclpy.callback_groups import ReentrantCallbackGroup

import time
import math
import json
import os
from moveit_controller import MoveItController


# Object name mapping: marker_id -> (name, height, radius)
# OBJECT_MAP = {
#     6: ("water_bottle", 0.18, 0.03),
#     7: ("medicine_bottle", 0.14, 0.025)
# }

# Extended object map with type information
# Format: marker_id -> {
#   'name': str,
#   'type': 'cylinder' | 'mesh',
#   'aruco_offset': (offset_x, offset_y, offset_z) - offset from ArUco position to object center/origin
#   For 'cylinder' type:
#     'cylinder_params': (radius, height) - cylinder dimensions
#   For 'mesh' type:
#     'mesh_file': str - STL filename
#     'mesh_scale': float - scale factor for STL
#     'handle_params': (radius, height) - cylinder handle dimensions (for moving objects only)
#   For static objects:
#     'is_static': True - object doesn't move, only added once
# }
OBJECT_MAP_EXTENDED = {
    1: {
        'name': 'water_cup',
        'type': 'cylinder',
        'aruco_offset': (0.04, 0.0, 0.0),  # TODO: Measure actual offset
        'cylinder_params': (0.04, 0.097)  # radius, height
    },
    5: {
        'name': 'water_bottle', 
        'type': 'mesh',
        'aruco_offset': (0.025452, 0.0, 0.0),  # TODO: Measure actual offset from ArUco to mesh center
        'mesh_file': 'water_bottle.stl',
        'mesh_scale': 0.001,
        'handle_params': (0.02542, 0.1137),  # handle_radius, handle_height
        'total_height': 0.13929
    },
    
    6: {
        'name': 'water_bottle',
        'type': 'cylinder',
        'aruco_offset': (0.03, 0.0, 0.0),  # TODO: Measure actual offset
        'cylinder_params': (0.03, 0.18)  # radius, height
    },
    7: {
        'name': 'medicine_bottle1', 
        'type': 'cylinder',
        'aruco_offset': (0.025, 0.0, 0.0),  # TODO: Measure actual offset
        'cylinder_params': (0.025, 0.14)  # radius, height
    },
    8: {
        'name': 'medicine_bottle2', 
        'type': 'mesh',
        'aruco_offset': (0.03, 0.0, 0.0),  # TODO: Measure actual offset from ArUco to mesh center
        'mesh_file': 'medicine_bottle.stl',
        'mesh_scale': 0.001,
        'handle_params': (0.03, 0.07849),  # handle_radius, handle_height
        'total_height': 0.09105,
    },
    10: {
        'name': 'plate',
        'type': 'mesh',
        'aruco_offset': (0.0257, 0.0, 0.0),  # TODO: Measure actual offset from ArUco to mesh center
        'mesh_file': 'plate.stl',
        'mesh_scale': 0.001,
        'handle_params': (0.0257, 0.1),  # handle_radius, handle_height
        'total_height': 0.11,
    },
    11: {
        'name': 'bowl',
        'type': 'mesh',
        'aruco_offset': (0.025, 0.0, 0.0),  # TODO: Measure actual offset from ArUco to mesh center
        'mesh_file': 'bowl.stl',
        'mesh_scale': 0.001,
        'handle_params': (0.025, 0.1),  # handle_radius, handle_height
        'total_height': 0.133,
    },
    12: {
        'name': 'fork',
        'type': 'mesh',
        'aruco_offset': (0.083145, 0.0, 0.0),  # TODO: Measure actual offset from ArUco to mesh center
        'mesh_file': 'fork.stl',
        'mesh_scale': 0.001,
        'handle_params': (0.018755, 0.1),  # handle_radius, handle_height
        'total_height': 0.1,
    },
    13: {
        'name': 'spoon',
        'type': 'mesh',
        'aruco_offset': (0.081405, 0.0, 0.0),  # TODO: Measure actual offset from ArUco to mesh center
        'mesh_file': 'spoon.stl',
        'mesh_scale': 0.001,
        'handle_params': (0.018755, 0.1),  # handle_radius, handle_height
        'total_height': 0.1,
    },
    14: {
        'name': 'cereal_box',
        'type': 'mesh',
        'aruco_offset': (0.0024415, 0.0, 0.0),  # TODO: Measure actual offset from ArUco to mesh center
        'mesh_file': 'cereal_box.stl',
        'mesh_scale': 0.001,
        'handle_params': (0.035, 0.128),  # handle_radius, handle_height
        'total_height': 0.14,
    },
    15: {
        'name': 'milk_carton',
        'type': 'mesh',
        'aruco_offset': (0.03, 0.0, 0.0),  # TODO: Measure actual offset from ArUco to mesh center
        'mesh_file': 'milk_carton.stl',
        'mesh_scale': 0.001,
        'handle_params': (0.03, 0.1),  # handle_radius, handle_height
        'total_height': 0.12,
    },
    
    20: {
        'name': 'bookshelf',
        'type': 'mesh',
        'is_static': True,  # Static object - only add once, don't update
        'aruco_offset': (-0.0975, -0.035, 0.0),  # TODO: Measure actual offset
        'mesh_file': 'shelf.stl',
        'mesh_scale': 0.001,
        'bookshelf_params': (0.05, 0.195, 0.14)  # shelf_height, shelf_depth, shelf_width (for compartment calculations)
    },
    21: {
        'name': 'book_1',
        'type': 'mesh',  # Changed to mesh if using STL, or keep as cylinder if using primitives
        'aruco_offset': (0.065, 0.0, 0.0),  # TODO: Measure actual offset
        'mesh_file': 'book.stl',  # If using mesh
        'mesh_scale': 0.001,
        'handle_params': (0.03, 0.15),  # spine_radius, spine_height (cylinder handle for grasping)
        'total_height': 0.15,
    },
    22: {
        'name': 'book_2',
        'type': 'mesh',  # Changed to mesh if using STL, or keep as cylinder if using primitives
        'aruco_offset': (0.065, 0.0, 0.0),  # TODO: Measure actual offset
        'mesh_file': 'book.stl',  # If using mesh
        'mesh_scale': 0.001,
        'handle_params': (0.03, 0.15),  # spine_radius, spine_height (cylinder handle for grasping)
        'total_height': 0.15,
    }  

}


class RealDetectionPickPlaceController(MoveItController):
    """
    Real robot controller with ArUco detection
    Inherits all MoveIt2 functionality from MoveItController
    """
    
    def __init__(self):
        super().__init__(node_name='real_detection_pick_place_controller')

        # Workspace table parameters
        self.workspace_table = {
            'dx': 0.43,
            'dy': -0.31,
            'dz': -0.37,
            'width': 0.55,
            'depth': 0.55,
            'height': 0.63
        }

        self.z_surface = self.workspace_table['dz'] + self.workspace_table['height']

        self.serving_area = [(self.workspace_table['dx']+0.1, self.workspace_table['dy']+0.06875+0.1375*3, self.z_surface),
                             (self.workspace_table['dx']+0.1, self.workspace_table['dy']+0.06875+0.1375*2, self.z_surface),
                             (self.workspace_table['dx']+0.1, self.workspace_table['dy']+0.06875+0.1375, self.z_surface),
                             (self.workspace_table['dx']+0.1, self.workspace_table['dy']+0.06875, self.z_surface),
                            ]  # X,Y ranges for placing objects

        # Detected objects storage
        self.detected_objects = {}  # {marker_id: {'pose': Pose, 'name': str}}
        self.marker_ids = []
        self.picked_objects = set()  # Track which objects were picked
        self.static_objects_added = set()  # Track which static objects have been added (don't update)
        
        # Movement state - CRITICAL: blocks updates during movement
        self.is_moving = False
        self.pose_received = False
        self.valid_pose_available = False

        # Create callback group for service
        self.callback_group = ReentrantCallbackGroup()
        
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
        # Create service for medicine preparation
        self.prepare_medicine_service = self.create_service(
            Trigger,
            'prepare_medicine',
            self.prepare_medicine_service_callback,
            callback_group=self.callback_group
        )
        
        self.get_logger().info("Medicine preparation service available at /prepare_medicine")
        
        self.get_logger().info("=== Real Detection Pick & Place Controller Started ===")
        self.get_logger().info("Waiting for ArUco detections...")
    
    # ==================== SERVICE CALLBACKS ====================

    def prepare_medicine_service_callback(self, request, response):
        """Service callback to trigger medicine preparation"""
        self.get_logger().info("Received medicine preparation request from frontend")
        
        success = self.prepare_medicine()
        
        response.success = success
        if success:
            response.message = "Medicine preparation completed successfully"
        else:
            response.message = "Medicine preparation failed"
        
        return response 
    # ==================== ARUCO DETECTION CALLBACKS ====================
    
    def pose_callback(self, msg: PoseArray):
        """Update detected object poses - IGNORED during movement"""
        if self.is_moving:
            return
        
        if len(msg.poses) != len(self.marker_ids):
            return
        
        # Update poses ONLY for currently visible objects that aren't picked
        for i, marker_id in enumerate(self.marker_ids):
            if marker_id in self.detected_objects:
                # msg.poses[i].position.y -= 0.05 #LAB CHANGE ()
                self.detected_objects[marker_id]['pose'] = msg.poses[i]
                # Only update collision if object is NOT picked and NOT moving
                if marker_id not in self.picked_objects:
                    self.update_collision_object_smart(marker_id, msg.poses[i])
        
        if len(msg.poses) > 0:
            self.pose_received = True
            self.valid_pose_available = True
    
    def ids_callback(self, msg: Int32MultiArray):
        """Update detected marker IDs - IGNORED during movement"""
        if self.is_moving:
            return
         
        new_ids = list(msg.data)
        
        
        # Initialize new objects
        self.marker_ids = new_ids
        for marker_id in self.marker_ids:
            if marker_id not in self.detected_objects:
                if marker_id in OBJECT_MAP_EXTENDED:
                    obj_config = OBJECT_MAP_EXTENDED[marker_id]
                    obj_type = obj_config.get('type', 'cylinder')
                    
                    # Extract dimensions based on object type
                    # NOTE: 'height' is REQUIRED (used for place z-position calculation)
                    # NOTE: 'radius' is REQUIRED for moving objects (cylinder handle for grasping)
                    if obj_type == 'cylinder':
                        height = obj_config['cylinder_params'][1]  # cylinder height
                        radius = obj_config['cylinder_params'][0]  # cylinder radius
                    elif obj_type == 'mesh':
                        # For mesh objects, get handle dimensions (for moving objects)
                        if obj_config.get('is_static', False):
                            # Static objects don't have handles
                            height = obj_config.get('bookshelf_params', (0.05,))[0] if 'bookshelf_params' in obj_config else 0.15
                            radius = None
                        else:
                            # Moving mesh objects have cylinder handles
                            handle_params = obj_config.get('handle_params', (0.03, 0.15))
                            height = handle_params[1]  # handle height
                            radius = handle_params[0]  # handle radius
                    else:
                        # Fallback
                        height = 0.15
                        radius = 0.03
                    
                    self.detected_objects[marker_id] = {
                        'pose': None,
                        'name': obj_config['name'],
                        'height': height,  # REQUIRED: Used in place operations
                        'radius': radius,  # OPTIONAL: For future use (grasp planning, collision estimation)
                        'type': obj_type   # NEW: Store type for easier access
                    }
                else:
                    # Fallback for unknown marker IDs
                    self.detected_objects[marker_id] = {
                        'pose': None,
                        'name': f'marker_{marker_id}',
                        'height': 0.15,
                        'radius': 0.03,
                        'type': 'cylinder'
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
    
    def update_collision_cylinder(self, marker_id, pose: Pose):
        """
        Create or update collision cylinder for detected object.
        Applies aruco_offset to calculate the correct cylinder center position.
        
        Args:
            marker_id: ArUco marker ID
            pose: Detected ArUco marker pose
        """
        object_name = f"detected_object_{marker_id}"
        obj_config = OBJECT_MAP_EXTENDED.get(marker_id, {})
        
        # Get cylinder parameters
        cylinder_params = obj_config.get('cylinder_params', (0.03, 0.15))
        radius = cylinder_params[0]
        base_height = cylinder_params[1]
        
        # Get ArUco offset (offset from ArUco to cylinder center)
        aruco_offset = obj_config.get('aruco_offset', (0.0, 0.0, 0.0))
        offset_x, offset_y, offset_z = aruco_offset
        
        # Calculate actual height from table
        height = (pose.position.z - (self.z_surface)) * 2
        
        # Calculate cylinder center position: ArUco position + offset
        center_x = pose.position.x + offset_x
        center_y = pose.position.y + offset_y
        center_z = pose.position.z + offset_z
        
        self.get_logger().debug(
            f"Cylinder {marker_id}: ArUco at ({pose.position.x:.3f}, {pose.position.y:.3f}, {pose.position.z:.3f}), "
            f"offset ({offset_x:.3f}, {offset_y:.3f}, {offset_z:.3f}), "
            f"center at ({center_x:.3f}, {center_y:.3f}, {center_z:.3f})")
        
        # Add/update cylinder using parent class method
        self.add_cylinder_collision_object(
            object_name,
            x=center_x,
            y=center_y,
            z=center_z,
            radius=radius,
            height=height)
    
    def update_collision_mesh(self, marker_id, pose: Pose):
        """
        Create or update collision object for a mesh object from STL file.
        Applies aruco_offset to calculate the correct mesh position.
        
        Args:
            marker_id: ArUco marker ID
            pose: Detected ArUco marker pose
        """
        obj_config = OBJECT_MAP_EXTENDED[marker_id]
        mesh_file = obj_config.get('mesh_file', 'object.stl')
        mesh_scale = obj_config.get('mesh_scale', 0.001)
        
        object_name = f"detected_object_{marker_id}"
        
        # Get ArUco offset (offset from ArUco to mesh center/origin)
        aruco_offset = obj_config.get('aruco_offset', (0.0, 0.0, 0.0))
        offset_x, offset_y, offset_z = aruco_offset
        
        # Calculate mesh position: ArUco position + offset
        mesh_x = pose.position.x + offset_x
        mesh_y = pose.position.y + offset_y
        mesh_z = pose.position.z + offset_z
        
        self.get_logger().info(
            f"Detected mesh object {marker_id} ArUco at: "
            f"x={pose.position.x:.3f}, y={pose.position.y:.3f}, z={pose.position.z:.3f}")
        self.get_logger().info(
            f"Applying offset ({offset_x:.3f}, {offset_y:.3f}, {offset_z:.3f}) -> "
            f"mesh at ({mesh_x:.3f}, {mesh_y:.3f}, {mesh_z:.3f})")
        self.get_logger().info(f"Loading mesh: {mesh_file}")
        
        # Add mesh using parent class method
        # Mesh file path will be automatically resolved in collision_objects.py
        # Use center_origin=True if STL is already centered at origin, False otherwise
        success = self.add_mesh_collision_object(
            name=object_name,
            mesh_file=mesh_file,  # Just pass filename, path resolution happens in collision_objects.py
            x=mesh_x,
            y=mesh_y,
            z=mesh_z,
            scale_x=mesh_scale,
            scale_y=mesh_scale,
            scale_z=mesh_scale,
        )
        
        if not success:
            self.get_logger().error(f"Failed to load mesh: {mesh_file}")
    
    def update_collision_bookshelf(self, marker_id, pose: Pose):
        """
        Create bookshelf collision object from STL mesh based on detected ArUco marker.
        This is called ONCE when bookshelf is first detected, then marked as static.
        Uses the unified update_collision_mesh function.
        
        Args:
            marker_id: ArUco marker ID
            pose: Detected pose (ArUco marker position)
        """
        # Use the unified mesh update function
        self.update_collision_mesh(marker_id, pose)
        
        # Mark as static - don't update again
        self.static_objects_added.add(marker_id)
        
        obj_config = OBJECT_MAP_EXTENDED[marker_id]
        bookshelf_params = obj_config.get('bookshelf_params', (0.05, 0.195, 0.14))
        shelf_height = bookshelf_params[0]
        shelf_depth = bookshelf_params[1]
        shelf_width = bookshelf_params[2]
        mesh_scale = obj_config.get('mesh_scale', 0.001)
        
        self.get_logger().info(
            f"✓ Bookshelf mesh added as static object (marker {marker_id}): "
            f"h={shelf_height:.3f}m, d={shelf_depth:.3f}m, w={shelf_width:.3f}m, scale={mesh_scale}")
    
    def update_collision_object_smart(self, marker_id, pose: Pose):
        """
        Automatically update collision object based on OBJECT_MAP_EXTENDED.
        Handles 'cylinder' and 'mesh' types.
        Static objects (like bookshelf) are only added once.
        
        Args:
            marker_id: ArUco marker ID
            pose: Detected pose (ArUco marker position)
        """
        if marker_id in OBJECT_MAP_EXTENDED:
            obj_config = OBJECT_MAP_EXTENDED[marker_id]
            obj_type = obj_config.get('type', 'cylinder')
            is_static = obj_config.get('is_static', False)
            
            # Check if static object already added
            if is_static and marker_id in self.static_objects_added:
                # Don't update static objects after first detection
                return
            
            if obj_type == 'mesh':
                # Use mesh collision object
                if is_static:
                    # Static objects use special function (e.g., bookshelf)
                    self.update_collision_bookshelf(marker_id, pose)
                else:
                    # Moving mesh objects
                    self.update_collision_mesh(marker_id, pose)
            elif obj_type == 'cylinder':
                # Use cylinder collision object
                self.update_collision_cylinder(marker_id, pose)
            else:
                # Fallback to cylinder for unknown types
                self.get_logger().warn(
                    f"Unknown object type '{obj_type}' for marker {marker_id}, using cylinder")
                self.update_collision_cylinder(marker_id, pose)
        else:
            # Fallback to simple cylinder if not in extended map
            self.get_logger().warn(
                f"Marker {marker_id} not in OBJECT_MAP_EXTENDED, using default cylinder")
            self.update_collision_cylinder(marker_id, pose)
    
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
        self.get_logger().info("\n" + "="*60)
        self.get_logger().info("  DETECTED OBJECTS")
        self.get_logger().info("="*60)
        
        if len(self.detected_objects) == 0:
            self.get_logger().info("  No objects detected")
        else:
            for marker_id, obj_data in self.detected_objects.items():
                pose = obj_data.get('pose')
                name = obj_data.get('name', f'marker_{marker_id}')
                
                if pose:
                    self.get_logger().info(f"  [{marker_id}] {name}")
                    self.get_logger().info(f"      Position: ({pose.position.x:.3f}, "
                          f"{pose.position.y:.3f}, {pose.position.z:.3f})")
                else:
                    self.get_logger().info(f"  [{marker_id}] {name} (no pose data)")
        
        self.get_logger().info("="*60)
    
    # ==================== HELPER FUNCTIONS ====================
    def get_aruco_z(self, marker_id):
        object_data = OBJECT_MAP_EXTENDED.get(marker_id)
        if not object_data:
            self.get_logger().error(f"No object data found for marker {marker_id}")
            return None
        
        if object_data.get('type') == 'cylinder':
            return self.z_surface + object_data.get('cylinder_params')[1] / 2
        elif object_data.get('type') == 'mesh':
            return self.z_surface + object_data.get('total_height') / 2
        else:
            self.get_logger().error(f"Unknown object type '{object_data.get('type')}' for marker {marker_id}")
            return None
        
    
    # ==================== PICK EXECUTION ====================
    def execute_pick_move_sequence(self, marker_id, place_pose):
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
        
        object_pose = obj_data['pose']
        object_name = f"detected_object_{marker_id}"
        if marker_id == 7: 
            grip_force = 25/44.0

        self.move_to_home_position()
        # ==================== PICK PHASE ====================
        # approach_distance = 0.0  # Meters from the gripper to the object before pick
        grasp_distance = 0.04  # Meters from the gripper to the object after pick
        # self.remove_collision_object(object_name)  # Remove existing collision to avoid interference

        self.get_logger().info(
            f"Target place pose: x={place_pose.position.x:.3f}, "
            f"y={place_pose.position.y:.3f}, z={place_pose.position.z:.3f}")
        
        move_success = self.grasp_move_object(object_name, object_pose, place_pose, grasp_distance=grasp_distance, grip_force=grip_force if marker_id ==7 else 1.0)
        if not move_success:
            self.get_logger().error("✗ Move operation failed!")
            self.is_moving = False
            self.get_logger().info("🔓 Movement completed - resuming pose updates")
            return False
        
        else:
            self.get_logger().info("✓ Place operation completed successfully!")
            if self.get_current_pose():
                object_pose = self.get_current_pose()                    
                object_pose.position.x += self.GRIPPER_INNER_LENGTH + grasp_distance # x of ARUCO 
                object_pose.position.z = self.get_aruco_z(marker_id)
                self.get_logger().info(f"Marker {marker_id} placed at: {object_pose.position.x:.3f}, {object_pose.position.y:.3f}, {object_pose.position.z:.3f}")
                self.detected_objects[marker_id]['pose'] = object_pose
                self.get_logger().info(f"Add collision object {marker_id} at new place location: {object_pose.position.x:.3f}, {object_pose.position.y:.3f}, {object_pose.position.z:.3f}")
                self.update_collision_object_smart(marker_id, object_pose)  # Immediately add collision at placed location
            else:
                self.get_logger().warn(f"Failed to get current pose for object {marker_id}")
                object_pose = place_pose
                self.detected_objects[marker_id]['pose'] = object_pose
                self.update_collision_object_smart(marker_id, object_pose)  # Immediately add collision at placed location
                self.get_logger().info(f"Added collision object {marker_id} at new place location: {object_pose.position.x:.3f}, {object_pose.position.y:.3f}, {object_pose.position.z:.3f}")
      
        # CRITICAL: Release movement flag to allow updates again
        self.picked_objects.add(marker_id)

        self.is_moving = False
        self.get_logger().info("🔓 Movement completed - resuming pose updates")

        return move_success

        

    def execute_pick_and_place_sequence(self, marker_id, place_pose):
        """
        Execute complete pick and place sequence for selected object
        
        Args:
            marker_id: ArUco marker ID to pick
            place_pose: Pose to place the object at
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
        
        object_pose = obj_data['pose']
        object_name = f"detected_object_{marker_id}"

        self.move_to_home_position()
        
        # ==================== PICK PHASE ====================
        # approach_distance = 0.0  # Meters from the gripper to the object before pick
        grasp_distance = 0.04  # Meters from the gripper to the object after pick
        approach_distance = 0.02
        retreat_distance_x = 0.0
        if marker_id in [10, 11]:
            retreat_distance_x = 0.07
            approach_distance += 0.07

        self.get_logger().info("===================== PICK PHASE ====================")
        # self.remove_collision_object(object_name)  # Remove existing collision to avoid interference
        pick_success = self.pick_object(object_name, object_pose, approach_distance=approach_distance, grasp_distance=grasp_distance)
 
        if not pick_success:
            self.get_logger().error("✗ Pick operation failed!")
            print("\n✗ Pick failed!")
            self.is_moving = False
            self.get_logger().info("🔓 Movement completed - resuming pose updates")
            return False
        
        self.picked_objects.add(marker_id)
        # self.remove_detected_object_collision(marker_id)
        
        self.get_logger().info("✓ Pick operation completed successfully!")
        print("\n✓ Object picked successfully!")
        
        # ==================== PLACE PHASE ====================
        self.get_logger().info("==================== PLACE PHASE ====================")

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
       
        
        self.get_logger().info(
            f"Target place pose: x={place_pose.position.x:.3f}, "
            f"y={place_pose.position.y:.3f}, z={place_pose.position.z:.3f}")
        
        # Execute place
        
        place_success = self.place_object_with_start(object_name, object_pose, place_pose, retreat_distance_x=retreat_distance_x)
        if place_success:
            self.get_logger().info("✓ Place operation completed successfully!")
            if self.get_current_pose():
                object_pose = self.get_current_pose()                    
                object_pose.position.x += self.GRIPPER_INNER_LENGTH + grasp_distance + retreat_distance_x
                object_pose.position.z = self.get_aruco_z(marker_id)
                self.get_logger().info(f"Marker {marker_id} placed at: {object_pose.position.x:.3f}, {object_pose.position.y:.3f}, {object_pose.position.z:.3f}")
                self.detected_objects[marker_id]['pose'] = object_pose
                self.get_logger().info(f"Add collision object {marker_id} at new place location: {object_pose.position.x:.3f}, {object_pose.position.y:.3f}, {object_pose.position.z:.3f}")
                self.update_collision_object_smart(marker_id, object_pose)  # Immediately add collision at placed location
            else:
                self.get_logger().warn(f"Failed to get current pose for object {marker_id}")
                object_pose = place_pose
                self.detected_objects[marker_id]['pose'] = object_pose
                self.update_collision_object_smart(marker_id, object_pose)  # Immediately add collision at placed location
                self.get_logger().info(f"Added collision object {marker_id} at new place location: {object_pose.position.x:.3f}, {object_pose.position.y:.3f}, {object_pose.position.z:.3f}")
        else:
            self.get_logger().error("✗ Place operation failed!")
        
        # CRITICAL: Release movement flag to allow updates again
        self.is_moving = False
        self.get_logger().info("🔓 Movement completed - resuming pose updates")
        
        return place_success

    def execute_pick_pour_place_sequence(self, marker_id, place_pose, pour_angle_degree=45):
        """
        Execute complete pick and place sequence for selected object
        
        Args:
            marker_id: ArUco marker ID to pick
            place_pose: Pose to place the object at
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
        
        object_pose = obj_data['pose']
        object_name = f"detected_object_{marker_id}"
       
        grip_force = 20/44.0

        self.move_to_home_position()
        
        # ==================== PICK PHASE ====================
        # approach_distance = 0.0  # Meters from the gripper to the object before pick
        grasp_distance = 0.02  # Meters from the gripper to the object after pick
        self.get_logger().info("===================== PICK PHASE ====================")
        # self.remove_collision_object(object_name)  # Remove existing collision to avoid interference
        pick_success = self.pick_object(object_name, object_pose, lift_distance=0.3, grasp_distance=grasp_distance, grip_force=grip_force)
 
        if not pick_success:
            self.get_logger().error("✗ Pick operation failed!")
            self.is_moving = False
            self.get_logger().info("🔓 Movement completed - resuming pose updates")
            return False
        
        self.picked_objects.add(marker_id)
        # self.remove_detected_object_collision(marker_id)
        
        self.get_logger().info("✓ Pick operation completed successfully!")
        print("\n✓ Object picked successfully!")
        
        # ==================== PLACE PHASE ====================
        self.get_logger().info("==================== PLACE POUR PHASE ====================")

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
       
        
        self.get_logger().info(
            f"Target place pose: x={place_pose.position.x:.3f}, "
            f"y={place_pose.position.y:.3f}, z={place_pose.position.z:.3f}")
        
        # Execute place
        place_success = self.place_pour_object(object_name, object_pose, place_pose, pour_angle_degree=pour_angle_degree)
        if place_success:
            self.get_logger().info("✓ Place operation completed successfully!")
            if self.get_current_pose():
                object_pose = self.get_current_pose()                    
                object_pose.position.x += self.GRIPPER_INNER_LENGTH + grasp_distance
                object_pose.position.z = self.get_aruco_z(marker_id)
                self.get_logger().info(f"Marker {marker_id} placed at: {object_pose.position.x:.3f}, {object_pose.position.y:.3f}, {object_pose.position.z:.3f}")
                self.detected_objects[marker_id]['pose'] = object_pose
                self.get_logger().info(f"Add collision object {marker_id} at new place location: {object_pose.position.x:.3f}, {object_pose.position.y:.3f}, {object_pose.position.z:.3f}")
                self.update_collision_object_smart(marker_id, object_pose)  # Immediately add collision at placed location
            else:
                self.get_logger().warn(f"Failed to get current pose for object {marker_id}")
                object_pose = place_pose
                object_pose.position.z = self.get_aruco_z(marker_id)
                self.detected_objects[marker_id]['pose'] = object_pose
                self.update_collision_object_smart(marker_id, object_pose)  # Immediately add collision at placed location
                self.get_logger().info(f"Added collision object {marker_id} at new place location: {object_pose.position.x:.3f}, {object_pose.position.y:.3f}, {object_pose.position.z:.3f}")
        else:
            self.get_logger().error("✗ Place operation failed!")
        
        # CRITICAL: Release movement flag to allow updates again
        self.is_moving = False
        self.get_logger().info("🔓 Movement completed - resuming pose updates")
        
        return place_success
    
    def execute_pick_pour_return_sequence(self, marker_id, target_pose, pour_angle_degree=45, offset_left=0.1):
        """
        Execute pick, move to left of target, pour, retreat, and return to original position sequence.
        
        Args:
            marker_id: ArUco marker ID to pick
            target_pose: Pose of the target (cup/plate) to pour into
            pour_angle_degree: Angle to rotate for pouring (degrees)
            offset_left: Distance to move left of target (in -X direction, meters)
        Returns:
            bool: True if successful
        """
        # CRITICAL: Set movement flag to block all updates
        self.is_moving = True
        self.get_logger().info("🔒 Movement started - blocking all pose updates")
        
        self.get_logger().info(f"\n{'='*60}")
        self.get_logger().info(f"Starting pick, pour, and return sequence for marker {marker_id}")
        self.get_logger().info(f"{'='*60}")
        
        obj_data = self.detected_objects.get(marker_id)
        if not obj_data or not obj_data.get('pose'):
            self.get_logger().error("No pose data available!")
            self.is_moving = False
            return False
        
        # Save original object position
        original_object_pose = obj_data['pose']
        object_name = f"detected_object_{marker_id}"
        grasp_distance = 0.02
        grip_force = 20/44.0 if marker_id == 5 else 25/44.0

        self.move_to_home_position()
        
        # ==================== PICK PHASE ====================
        self.get_logger().info("===================== PICK PHASE ====================")
        pick_success = self.pick_object(object_name, original_object_pose, lift_distance=0.3, 
                                       grasp_distance=grasp_distance, grip_force=grip_force)
 
        if not pick_success:
            self.get_logger().error("✗ Pick operation failed!")
            self.is_moving = False
            self.get_logger().info("🔓 Movement completed - resuming pose updates")
            return False
        
        self.picked_objects.add(marker_id)
        self.get_logger().info("✓ Pick operation completed successfully!")
        
        # Get current gripper pose after pick
        current_gripper_pose = self.get_current_pose()
        if not current_gripper_pose:
            self.get_logger().error("Failed to get current pose after pick!")
            self.is_moving = False
            return False
        
        # Calculate current object position (relative to gripper)
        current_object_x = current_gripper_pose.position.x + self.GRIPPER_INNER_LENGTH + grasp_distance
        current_object_y = current_gripper_pose.position.y
        
        # ==================== MOVE TO LEFT OF TARGET ====================
        self.get_logger().info("==================== MOVE TO POUR POSITION ====================")
        
        # Calculate pour position: left of target (in -X direction)
        pour_position_x = target_pose.position.x 
        pour_position_y = target_pose.position.y + offset_left

        
        # Calculate movement needed
        dx = pour_position_x - current_object_x
        dy = pour_position_y - current_object_y
        
        self.get_logger().info(
            f"Moving to pour position (left of target): "
            f"target=({target_pose.position.x:.3f}, {target_pose.position.y:.3f}), "
            f"pour_pos=({pour_position_x:.3f}, {pour_position_y:.3f}), "
            f"movement=({dx:.3f}, {dy:.3f})")
        
        # Move in X direction first
        if abs(dx) > 0.001:
            if not self.HIGH_LEVEL_move_lin_relative(dx=dx):
                self.get_logger().error("Failed to move to pour position (X)")
                self.is_moving = False
                return False
            time.sleep(0.5)
        
        # Move in Y direction
        if abs(dy) > 0.001:
            if not self.HIGH_LEVEL_move_lin_relative(dy=dy):
                self.get_logger().error("Failed to move to pour position (Y)")
                self.is_moving = False
                return False
            time.sleep(0.5)
        
        # ==================== DESCEND TO POUR HEIGHT ====================
        self.get_logger().info("==================== DESCEND TO POUR HEIGHT ====================")
        z_pour = target_pose.position.z + 0.2
        current_z = self.get_current_pose().position.z
        dz = z_pour - current_z
        
        self.get_logger().info(
            f"Descending to pour height: z_pour={z_pour:.3f}, current_z={current_z:.3f}, dz={dz:.3f}")
        
        if not self.HIGH_LEVEL_move_lin_relative(dz=dz):
            self.get_logger().error("Failed to descend to pour height")
            self.is_moving = False
            return False
        time.sleep(0.5)
        
        # ==================== POUR ====================
        self.get_logger().info("==================== POURING ====================")
        if not self.HIGH_LEVEL_pour(pour_angle_degree):
            self.get_logger().error("Pouring motion failed")
            self.is_moving = False
            return False
        
        # ==================== RETREAT (LIFT UP) ====================
        self.get_logger().info("==================== RETREAT (LIFT) ====================")
        retreat_distance = 0.15
        if not self.HIGH_LEVEL_move_lin_relative(dz=retreat_distance):
            self.get_logger().error("Failed to retreat upward")
            self.is_moving = False
            return False
        time.sleep(0.5)
        
        # ==================== RETURN TO ORIGINAL POSITION ====================
        self.get_logger().info("==================== RETURN TO ORIGINAL POSITION ====================")
        
        # Get current position after retreat
        current_after_pour = self.get_current_pose()
        if not current_after_pour:
            self.get_logger().error("Failed to get current pose after pour!")
            self.is_moving = False
            return False
        
        # Calculate current object position
        current_obj_x_after = current_after_pour.position.x + self.GRIPPER_INNER_LENGTH + grasp_distance
        current_obj_y_after = current_after_pour.position.y
        
        # Calculate movement back to original position
        dx_return = original_object_pose.position.x - current_obj_x_after
        dy_return = original_object_pose.position.y - current_obj_y_after
        
        self.get_logger().info(
            f"Returning to original position: "
            f"original=({original_object_pose.position.x:.3f}, {original_object_pose.position.y:.3f}), "
            f"current=({current_obj_x_after:.3f}, {current_obj_y_after:.3f}), "
            f"movement=({dx_return:.3f}, {dy_return:.3f})")
        
        # Move back in X
        if abs(dx_return) > 0.001:
            if not self.HIGH_LEVEL_move_lin_relative(dx=dx_return):
                self.get_logger().error("Failed to return to original position (X)")
                self.is_moving = False
                return False
            time.sleep(0.5)
        
        # Move back in Y
        if abs(dy_return) > 0.001:
            if not self.HIGH_LEVEL_move_lin_relative(dy=dy_return):
                self.get_logger().error("Failed to return to original position (Y)")
                self.is_moving = False
                return False
            time.sleep(0.5)
        
        # ==================== DESCEND AND PLACE ====================
        self.get_logger().info("==================== PLACE BACK ====================")
        
        # Descend to original z position
        original_z = self.get_aruco_z(marker_id)
        current_z = self.get_current_pose().position.z
        dz_place = original_z - current_z
        
        self.get_logger().info(
            f"Descending to place: original_z={original_z:.3f}, current_z={current_z:.3f}, dz={dz_place:.3f}")
        
        if abs(dz_place) > 0.001:
            if not self.HIGH_LEVEL_move_lin_relative(dz=dz_place):
                self.get_logger().error("Failed to descend to place position")
                self.is_moving = False
                return False
            time.sleep(0.5)
        
        # Open gripper to release
        self.get_logger().info("Opening gripper to release object...")
        if not self.open_gripper():
            self.get_logger().error("Failed to open gripper")
            self.is_moving = False
            return False
        time.sleep(2.5)
        
        
        # Retreat upward
        if not self.HIGH_LEVEL_move_lin_relative(dz=retreat_distance):
            self.get_logger().error("Failed to retreat after placing")
            self.is_moving = False
            return False
        
        # Detach object
        self.detach_object_from_gripper(object_name)
        
        # Update object pose (should be back at original position)
        if self.get_current_pose():
            final_pose = self.get_current_pose()
            object_pose = Pose()
            object_pose.position.x = final_pose.position.x + self.GRIPPER_INNER_LENGTH + grasp_distance
            object_pose.position.y = final_pose.position.y
            object_pose.position.z = self.get_aruco_z(marker_id)
            object_pose.orientation.w = 1.0
            
            self.detected_objects[marker_id]['pose'] = object_pose
            self.update_collision_object_smart(marker_id, object_pose)
            self.get_logger().info(f"Object returned to original position: ({object_pose.position.x:.3f}, {object_pose.position.y:.3f}, {object_pose.position.z:.3f})")
        
        # CRITICAL: Release movement flag
        self.is_moving = False
        self.get_logger().info("🔓 Movement completed - resuming pose updates")
        self.get_logger().info("✓ Pick, pour, and return sequence completed successfully!")
        
        return True
    
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
        
        object_pose = obj_data['pose']
        object_name = f"detected_object_{marker_id}"
        
        # Execute pick using parent class method
        success = self.pick_object(object_name, object_pose)
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
        chair_seat_height = 2.0
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
        self.add_real_object("big_table",dx=0.26, dy=-0.51, dz=-0.37, width=0.765, depth=-1, height=0.715)

        # Workspace table
        self.add_real_object("workspace_table", dx=self.workspace_table['dx'], dy=self.workspace_table['dy'], dz=self.workspace_table['dz'], width=self.workspace_table['width'], depth=self.workspace_table['depth'], height=self.workspace_table['height'])
    
        
        # Wall
        self.add_real_object("wall", dx=1, dy=-7.0, dz=-0.37, width=0.3, depth=15, height=4.0)
        
        self.get_logger().info("✓ Static environment added")


    def convert_serving_area_to_pose(self, marker_id, area_tuple):
        """Convert serving area tuple to Pose message. With z position of the gripper attached to the object.
        
        Args:
            marker_id: ArUco marker ID
            area_tuple: Tuple containing (x, y, z) position of the serving area
        
        Returns:
            tuple: (x, y, z) position of the serving area
        """
        object_data = OBJECT_MAP_EXTENDED.get(marker_id)
        if object_data.get('type') == 'cylinder':
            gripper_z = object_data.get('cylinder_params')[1] / 2.0
        elif object_data.get('type') == 'mesh':
            gripper_z = object_data.get('handle_params')[1] / 2.0
        else:
            gripper_z = 0.0
            pose_msg.orientation.w = 1.0  # Neutral orientation
            return pose_msg
        
        pose_msg = Pose()
        pose_msg.position.x = area_tuple[0]
        pose_msg.position.y = area_tuple[1]
        pose_msg.position.z = area_tuple[2] + gripper_z
        pose_msg.orientation.w = 1.0  # Neutral orientation
        return pose_msg
    
    def get_bookshelf_compartment_position(self, compartment_index=0):
        """
        Get position for placing books in bookshelf compartments.
        Uses detected bookshelf position if available (marker ID 2).
        
        COORDINATE SYSTEM:
        - Width (X): Left-right along back
        - Depth (Y): Extends in -Y direction from back
        - Compartments: 0=first (near back), 1=second (in -Y direction)
        
        Args:
            compartment_index: 0 for first compartment (near back), 1 for second compartment (in -Y)
        
        Returns:
            tuple: (x, y, z) position in base_link frame
        """
        # Check if bookshelf was detected (marker ID 2)
        bookshelf_marker_id = 2
        if bookshelf_marker_id in self.detected_objects and self.detected_objects[bookshelf_marker_id]['pose'] is not None:
            # Use detected bookshelf position
            bookshelf_pose = self.detected_objects[bookshelf_marker_id]['pose']
            bookshelf_x = bookshelf_pose.position.x   # Back board center X
            bookshelf_y = bookshelf_pose.position.y  # Back board Y position
            bookshelf_z_center = bookshelf_pose.position.z
            
            # Get bookshelf dimensions from config
            shelf_params = OBJECT_MAP_EXTENDED[bookshelf_marker_id]['bookshelf_params']
            shelf_height = shelf_params[0]
            shelf_depth = shelf_params[1]
            shelf_width = shelf_params[2]
            
            # Calculate base z position
            bookshelf_z = bookshelf_z_center - shelf_height / 2.0
            
            self.get_logger().debug(
                f"Using detected bookshelf at: x={bookshelf_x:.3f}, y={bookshelf_y:.3f}, z={bookshelf_z:.3f}")
        else:
            # Fallback to hardcoded position if bookshelf not detected
            self.get_logger().warn("Bookshelf not detected, using default position")
            bookshelf_x = self.workspace_table['dx'] + 0.1
            bookshelf_y = self.workspace_table['dy'] + 0.2
            bookshelf_z = self.workspace_table['dz'] + self.workspace_table['height']
            shelf_depth = 0.15
            shelf_width = 0.25
        
        base_thickness = 0.02
        
        # Calculate compartment positions
        # X: Center of back board (same for all compartments)
        x_pos = bookshelf_x
        
        # Y: Center of each compartment (compartments extend in -Y direction)
        # Compartment 0 (first): between back and middle divider
        # Compartment 1 (second): between middle divider and front
        compartment_depth = shelf_depth / 2.0  # Each compartment is half the depth
        if compartment_index == 0:  # First compartment (near back)
            y_pos = bookshelf_y - shelf_depth / 4.0  # Center of first half
        else:  # Second compartment (in -Y direction)
            y_pos = bookshelf_y - 3.0 * shelf_depth / 4.0  # Center of second half
        
        # Z: On top of base
        z_pos = bookshelf_z + base_thickness / 2.0
        
        return (x_pos, y_pos, z_pos)

    def prepare_medicine(self):
        """Prepare medicine by picking and placing the medicine bottle"""

        self.get_logger().info("Starting medicine preparation...")

        #============PICK AND PLACE WATER CUP (marker 1)=================
        if not self.execute_pick_and_place_sequence(1, self.convert_serving_area_to_pose(1, self.serving_area[1])):
            self.get_logger().error(f"Failed to prepare water cup - pick and place failed")
            return False
        
        self.get_logger().info(f"Water cup placed successfully, preparing to pick plate...")
        # Wait to ensure robot has fully completed all motions
        time.sleep(3.0)
        
        #============PICK AND PLACE PLATE (marker 10)=================
        if not self.execute_pick_and_place_sequence(10, self.convert_serving_area_to_pose(10, self.serving_area[2])):
            self.get_logger().error(f"Failed to prepare plate - pick and place failed")
            return False
        self.get_logger().info(f"Plate placed successfully, preparing to pick water bottle...")
        time.sleep(3.0)

        #============PICK, POUR INTO CUP, AND RETURN WATER BOTTLE=================
        # Get water cup position (target for pouring)
        water_cup_pose = self.detected_objects[1]['pose']
        if not water_cup_pose:
            self.get_logger().error("Water cup pose not available!")
            return False
        
        # Create target pose for pouring (use cup's position)
        water_cup_target_pose = Pose()
        water_cup_target_pose.position.x = water_cup_pose.position.x
        water_cup_target_pose.position.y = water_cup_pose.position.y
        water_cup_target_pose.position.z = water_cup_pose.position.z
        water_cup_target_pose.orientation.w = 1.0
        
        if not self.execute_pick_pour_return_sequence(5, water_cup_target_pose, pour_angle_degree=45, offset_left=0.1):
            self.get_logger().error(f"Failed to pour water bottle into cup")
            return False
        self.get_logger().info(f"Water bottle poured into cup and returned successfully, preparing to pour medicine bottle...")
        time.sleep(3.0)
        
        #============PICK, POUR INTO PLATE, AND RETURN MEDICINE BOTTLE=================
        # Get plate position (target for pouring)
        plate_pose = self.detected_objects[10]['pose']
        if not plate_pose:
            self.get_logger().error("Plate pose not available!")
            return False
        
        # Create target pose for pouring (use plate's position)
        plate_target_pose = Pose()
        plate_target_pose.position.x = plate_pose.position.x
        plate_target_pose.position.y = plate_pose.position.y
        plate_target_pose.position.z = plate_pose.position.z
        plate_target_pose.orientation.w = 1.0
        
        if not self.execute_pick_pour_return_sequence(8, plate_target_pose, pour_angle_degree=-45, offset_left=-0.1):
            self.get_logger().error(f"Failed to pour medicine bottle into plate")
            return False
        self.get_logger().info(f"Medicine bottle poured into plate and returned successfully, medicine preparation completed!")
        return True

    def organize_books(self):
        """Organize books by picking and placing them in bookshelf compartments"""
        if 9 not in self.detected_objects:
            self.get_logger().error("Book (marker 9) not detected!")
            return False
        
        self.get_logger().info("Starting book organization...")
        
        # Get bookshelf compartment position (left compartment)
        compartment_pos = self.get_bookshelf_compartment_position(compartment_index=0)
        
        # Create place pose
        book_place_pose = Pose()
        book_place_pose.position.x = compartment_pos[0]
        book_place_pose.position.y = compartment_pos[1]
        book_place_pose.position.z = compartment_pos[2] + self.detected_objects[9]['height'] / 2.0  # Adjust for book height
        book_place_pose.orientation.w = 1.0
        
        self.get_logger().info(
            f"Placing book in bookshelf compartment at: "
            f"x={book_place_pose.position.x:.3f}, "
            f"y={book_place_pose.position.y:.3f}, "
            f"z={book_place_pose.position.z:.3f}")
        
        if not self.execute_pick_and_place_sequence(9, place_pose=book_place_pose):
            self.get_logger().error("Failed to organize books - book pick and place failed")
            return False
        
        self.get_logger().info("✓ Book placed in bookshelf successfully!")
        self.get_logger().info("✓ Book organization completed!")
        return True



def main(args=None):
    rclpy.init(args=args)
    
    controller = RealDetectionPickPlaceController()
    
    # Setup static environment
    controller.setup_static_environment()
    
    # Wait for valid pose before executing
    if controller.wait_for_valid_pose(timeout=15.0):
        controller.display_detected_objects()
        # controller.organize_books()
        
        controller.prepare_medicine()        
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
