#!/usr/bin/env python3
"""
Collision Object Management Mixin for MoveIt2

This module provides collision object management methods that can be mixed into
MoveItController classes. It handles:
- Box and cylinder primitives
- Composite objects (books)
- Mesh objects from STL files
- STL file loading and dimension checking
"""

import time
import struct
import os
import math
from moveit_msgs.msg import CollisionObject, PlanningScene
from shape_msgs.msg import SolidPrimitive, Mesh, MeshTriangle
from geometry_msgs.msg import Pose, Point


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


class CollisionObjectMixin:
    """
    Mixin class providing collision object management methods.
    
    Classes using this mixin must have the following attributes:
    - self.get_logger(): Logger method
    - self.get_clock(): Clock method (for timestamps)
    - self.base_frame: Base frame name (str)
    - self.planning_scene_pub: Publisher for PlanningScene messages
    - self.collision_objects_added: Set tracking added collision objects
    """
    
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
        self.get_logger().info(f"✓ Cylinder '{name}' added with size {radius:.3f}m, {height:.3f}m")
        return True
    
    def add_book_collision_object(self, name, spine_center_x, spine_center_y, spine_center_z,
                                   spine_radius, spine_height,
                                   book_width, book_depth, book_height):
        """
        Add a composite book collision object consisting of:
        - Cylinder spine/back (for grasping, ArUco marker at center)
        - Box pages (behind the spine in +x direction)
        
        Args:
            name: Unique identifier for the collision object
            spine_center_x, spine_center_y, spine_center_z: Position of spine center (ArUco marker location)
            spine_radius: Radius of the cylindrical spine
            spine_height: Height of the spine (same as book height)
            book_width: Width of the book box (x dimension, depth of pages)
            book_depth: Depth of the book box (y dimension, same as spine diameter)
            book_height: Height of the book box (z dimension, same as spine height)
        
        Returns:
            bool: True if successful
        """
        self.get_logger().info(f"Adding book collision object '{name}'")
        
        collision_object = CollisionObject()
        collision_object.header.frame_id = self.base_frame
        collision_object.header.stamp = self.get_clock().now().to_msg()
        collision_object.id = name
        
        # Create spine cylinder (ArUco marker at center)
        spine_cylinder = SolidPrimitive()
        spine_cylinder.type = SolidPrimitive.CYLINDER
        spine_cylinder.dimensions = [spine_height, spine_radius]
        
        # Spine pose - ArUco marker is at center of spine
        spine_pose = Pose()
        spine_pose.position.x = float(spine_center_x)
        spine_pose.position.y = float(spine_center_y)
        spine_pose.position.z = float(spine_center_z)
        spine_pose.orientation.w = 1.0
        
        # Create book pages box (behind spine in +x direction)
        book_box = SolidPrimitive()
        book_box.type = SolidPrimitive.BOX
        book_box.dimensions = [book_width, book_depth, book_height]
        
        # Book pose - center of box, positioned behind spine
        book_pose = Pose()
        book_pose.position.x = float(spine_center_x) + book_width / 2.0  # Behind spine in +x
        book_pose.position.y = float(spine_center_y)  # Same y as spine
        book_pose.position.z = float(spine_center_z)  # Same height as spine center
        book_pose.orientation.w = 1.0
        
        # Add both primitives to the same collision object
        collision_object.primitives.append(spine_cylinder)
        collision_object.primitive_poses.append(spine_pose)
        collision_object.primitives.append(book_box)
        collision_object.primitive_poses.append(book_pose)
        collision_object.operation = CollisionObject.ADD
        
        # Publish to planning scene
        planning_scene = PlanningScene()
        planning_scene.world.collision_objects.append(collision_object)
        planning_scene.is_diff = True
        
        self.planning_scene_pub.publish(planning_scene)
        time.sleep(0.5)
        
        self.collision_objects_added.add(name)
        self.get_logger().info(
            f"✓ Book '{name}' added "
            f"(spine: r={spine_radius:.3f}m h={spine_height:.3f}m, "
            f"pages: {book_width:.3f}m width, {book_depth:.3f}m depth, {book_height:.3f}m height)")
        return True
    
    def _load_stl_binary(self, filename):
        """
        Load binary STL file and return vertices and triangles.
        
        Args:
            filename: Path to STL file
            
        Returns:
            tuple: (vertices, triangles, bbox_center) where:
                - vertices is list of Point
                - triangles is list of vertex indices
                - bbox_center is (center_x, center_y, center_z) of bounding box
        """
        if not os.path.exists(filename):
            self.get_logger().error(f"STL file not found: {filename}")
            return None, None, None
        
        try:
            with open(filename, 'rb') as f:
                # Skip 80-byte header
                header = f.read(80)
                
                # Read number of triangles
                num_triangles = struct.unpack('<I', f.read(4))[0]
                
                vertices = []
                triangles = []
                vertex_map = {}  # To avoid duplicate vertices
                vertex_index = 0
                
                # Track min/max for bounding box
                min_x = min_y = min_z = float('inf')
                max_x = max_y = max_z = float('-inf')
                
                for i in range(num_triangles):
                    # Read normal (3 floats) - we'll skip this as MoveIt can compute it
                    normal = struct.unpack('<3f', f.read(12))
                    
                    # Read 3 vertices (each has 3 floats: x, y, z)
                    triangle_indices = []
                    for j in range(3):
                        vertex = struct.unpack('<3f', f.read(12))
                        
                        # Update bounding box
                        min_x = min(min_x, vertex[0])
                        max_x = max(max_x, vertex[0])
                        min_y = min(min_y, vertex[1])
                        max_y = max(max_y, vertex[1])
                        min_z = min(min_z, vertex[2])
                        max_z = max(max_z, vertex[2])
                        
                        # Check if vertex already exists (to reduce duplicates)
                        vertex_key = (round(vertex[0], 6), round(vertex[1], 6), round(vertex[2], 6))
                        if vertex_key not in vertex_map:
                            point = Point()
                            point.x = float(vertex[0])
                            point.y = float(vertex[1])
                            point.z = float(vertex[2])
                            vertices.append(point)
                            vertex_map[vertex_key] = vertex_index
                            triangle_indices.append(vertex_index)
                            vertex_index += 1
                        else:
                            triangle_indices.append(vertex_map[vertex_key])
                    
                    # Add triangle (3 vertex indices)
                    triangles.extend(triangle_indices)
                    
                    # Skip attribute byte count (2 bytes)
                    f.read(2)
                
                # Calculate dimensions and center
                size_x = max_x - min_x
                size_y = max_y - min_y
                size_z = max_z - min_z
                center_x = (min_x + max_x) / 2.0
                center_y = (min_y + max_y) / 2.0
                center_z = (min_z + max_z) / 2.0
                
                # self.get_logger().info(
                #     f"Loaded STL: {num_triangles} triangles, {len(vertices)} unique vertices")
                # self.get_logger().info(
                #     f"STL dimensions (raw units): "
                self.get_logger().info(f"X={size_x:.3f}, Y={size_y:.3f}, Z={size_z:.3f}")
                # self.get_logger().info(
                #     f"Bounding box: X[{min_x:.3f}, {max_x:.3f}], "
                #     f"Y[{min_y:.3f}, {max_y:.3f}], Z[{min_z:.3f}, {max_z:.3f}]")
                # self.get_logger().info(
                #     f"Bounding box center: ({center_x:.3f}, {center_y:.3f}, {center_z:.3f})")
                
                return vertices, triangles, (center_x, center_y, center_z)
                
        except Exception as e:
            self.get_logger().error(f"Failed to load STL file: {e}")
            return None, None, None
    
    def _resolve_mesh_path(self, mesh_file):
        """
        Resolve mesh file path. If relative, looks in 'mesh' subdirectory.
        
        Args:
            mesh_file: Filename (e.g., 'plate.stl') or absolute path
            
        Returns:
            str: Resolved absolute path, or None if not found
        """
        # If already an absolute path and exists, use it
        if os.path.isabs(mesh_file) and os.path.exists(mesh_file):
            return mesh_file
        
        # If relative path exists as-is, use it
        if os.path.exists(mesh_file):
            return os.path.abspath(mesh_file)
        
        # Try in 'mesh' subdirectory relative to this file's location
        # This file is in controller_lib/gpt/, so mesh files are in controller_lib/gpt/mesh/
        this_file_dir = os.path.dirname(os.path.abspath(__file__))
        mesh_dir = os.path.join(this_file_dir, 'mesh')
        mesh_path = os.path.join(mesh_dir, mesh_file)
        
        if os.path.exists(mesh_path):
            return mesh_path
        
        # File not found
        self.get_logger().error(
            f"Mesh file not found: {mesh_file}\n"
            f"  Tried: {os.path.abspath(mesh_file)}\n"
            f"  Tried: {mesh_path}")
        return None
    
    def _center_vertices_at_origin(self, vertices, bbox_center):
        """
        Translate all vertices so that the bounding box center is at origin (0,0,0).
        
        Args:
            vertices: List of Point objects to translate
            bbox_center: Tuple (center_x, center_y, center_z) of bounding box center
            
        Returns:
            None (modifies vertices in place)
        """
        if bbox_center is None:
            return
        
        center_x, center_y, center_z = bbox_center
        
        for vertex in vertices:
            vertex.x -= center_x
            vertex.y -= center_y
            vertex.z -= center_z
    
    def rotate_stl_file(self, input_file, output_file, roll=0.0, pitch=0.0, yaw=0.0):
        """
        Create a new STL file with vertices rotated by specified Euler angles.
        Rotation is applied around the origin (0,0,0).
        
        Args:
            input_file: Path to input STL file (can be relative filename, will look in 'mesh' subdirectory)
            output_file: Path to output STL file (will be overwritten if exists)
            roll: Rotation around X axis in radians
            pitch: Rotation around Y axis in radians
            yaw: Rotation around Z axis in radians
            
        Returns:
            bool: True if successful
        """
        # Resolve input file path
        input_path = self._resolve_mesh_path(input_file)
        if input_path is None:
            return False
        
        self.get_logger().info(
            f"Rotating STL file: {input_path} -> {output_file}")
        self.get_logger().info(
            f"Rotation: roll={roll:.3f} ({math.degrees(roll):.1f}°), "
            f"pitch={pitch:.3f} ({math.degrees(pitch):.1f}°), "
            f"yaw={yaw:.3f} ({math.degrees(yaw):.1f}°)")
        
        # Pre-compute rotation matrices for efficiency
        # Rotation around X (roll)
        cr = math.cos(roll)
        sr = math.sin(roll)
        # Rotation around Y (pitch)
        cp = math.cos(pitch)
        sp = math.sin(pitch)
        # Rotation around Z (yaw)
        cy = math.cos(yaw)
        sy = math.sin(yaw)
        
        # Combined rotation matrix: R = Rz(yaw) * Ry(pitch) * Rx(roll)
        # For efficiency, we'll apply rotations in order: Z, Y, X
        def rotate_vertex(x, y, z):
            # Rotate around Z (yaw)
            x1 = x * cy - y * sy
            y1 = x * sy + y * cy
            z1 = z
            
            # Rotate around Y (pitch)
            x2 = x1 * cp + z1 * sp
            y2 = y1
            z2 = -x1 * sp + z1 * cp
            
            # Rotate around X (roll)
            x3 = x2
            y3 = y2 * cr - z2 * sr
            z3 = y2 * sr + z2 * cr
            
            return x3, y3, z3
        
        # Check file size first
        file_size = os.path.getsize(input_path)
        if file_size < 84:  # 80 bytes header + 4 bytes triangle count
            self.get_logger().error(
                f"STL file is too small ({file_size} bytes). Minimum size is 84 bytes. "
                f"File may be empty or corrupted.")
            return False
        
        # Read original STL and write rotated version
        try:
            with open(input_path, 'rb') as f_in, open(output_file, 'wb') as f_out:
                # Copy header
                header = f_in.read(80)
                if len(header) < 80:
                    self.get_logger().error("Failed to read STL header (file too small)")
                    return False
                f_out.write(header)
                
                # Read number of triangles
                triangle_count_bytes = f_in.read(4)
                if len(triangle_count_bytes) < 4:
                    self.get_logger().error("Failed to read triangle count (file too small)")
                    return False
                num_triangles = struct.unpack('<I', triangle_count_bytes)[0]
                f_out.write(struct.pack('<I', num_triangles))
                
                # Process each triangle
                for i in range(num_triangles):
                    # Read normal (will be recalculated by MoveIt, but we'll rotate it too)
                    normal_bytes = f_in.read(12)
                    if len(normal_bytes) < 12:
                        self.get_logger().error(
                            f"Failed to read normal for triangle {i} (file truncated)")
                        return False
                    nx, ny, nz = struct.unpack('<3f', normal_bytes)
                    # Rotate normal vector
                    nx_rot, ny_rot, nz_rot = rotate_vertex(nx, ny, nz)
                    f_out.write(struct.pack('<3f', nx_rot, ny_rot, nz_rot))
                    
                    # Read, rotate, and write 3 vertices
                    for j in range(3):
                        vertex_bytes = f_in.read(12)
                        if len(vertex_bytes) < 12:
                            self.get_logger().error(
                                f"Failed to read vertex {j} for triangle {i} (file truncated)")
                            return False
                        x, y, z = struct.unpack('<3f', vertex_bytes)
                        # Apply rotation
                        x_rot, y_rot, z_rot = rotate_vertex(x, y, z)
                        f_out.write(struct.pack('<3f', x_rot, y_rot, z_rot))
                    
                    # Copy attribute byte count
                    attr = f_in.read(2)
                    if len(attr) < 2:
                        self.get_logger().error(
                            f"Failed to read attribute for triangle {i} (file truncated)")
                        return False
                    f_out.write(attr)
            
            self.get_logger().info(
                f"✓ Successfully created rotated STL: {output_file}")
            self.get_logger().info(
                f"  Applied rotation: roll={math.degrees(roll):.1f}°, "
                f"pitch={math.degrees(pitch):.1f}°, yaw={math.degrees(yaw):.1f}°")
            return True
            
        except Exception as e:
            self.get_logger().error(f"Failed to rotate STL file: {e}")
            return False
    
    def center_stl_file(self, input_file, output_file):
        """
        Create a new STL file with vertices centered at origin (bounding box center at 0,0,0).
        This permanently modifies the STL file so the origin is at the geometric center.
        
        Args:
            input_file: Path to input STL file (can be relative filename, will look in 'mesh' subdirectory)
            output_file: Path to output STL file (will be overwritten if exists)
            
        Returns:
            bool: True if successful
        """
        # Resolve input file path
        input_path = self._resolve_mesh_path(input_file)
        if input_path is None:
            return False
        
        self.get_logger().info(f"Centering STL file: {input_path} -> {output_file}")
        
        # Load STL to get bounding box center
        vertices, triangles, bbox_center = self._load_stl_binary(input_path)
        if vertices is None or triangles is None or bbox_center is None:
            self.get_logger().error("Failed to load STL file for centering")
            return False
        
        center_x, center_y, center_z = bbox_center
        self.get_logger().info(
            f"Bounding box center: ({center_x:.6f}, {center_y:.6f}, {center_z:.6f})")
        
        # Read original STL and write centered version
        try:
            with open(input_path, 'rb') as f_in, open(output_file, 'wb') as f_out:
                # Copy header
                header = f_in.read(80)
                f_out.write(header)
                
                # Read number of triangles
                num_triangles = struct.unpack('<I', f_in.read(4))[0]
                f_out.write(struct.pack('<I', num_triangles))
                
                # Process each triangle
                for i in range(num_triangles):
                    # Read and copy normal
                    normal = f_in.read(12)
                    f_out.write(normal)
                    
                    # Read, translate, and write 3 vertices
                    for j in range(3):
                        x, y, z = struct.unpack('<3f', f_in.read(12))
                        # Apply translation to center at origin
                        x -= center_x
                        y -= center_y
                        z -= center_z
                        f_out.write(struct.pack('<3f', x, y, z))
                    
                    # Copy attribute byte count
                    attr = f_in.read(2)
                    f_out.write(attr)
            
            self.get_logger().info(
                f"✓ Successfully created centered STL: {output_file}")
            self.get_logger().info(
                f"  Original center was at: ({center_x:.6f}, {center_y:.6f}, {center_z:.6f})")
            self.get_logger().info(
                f"  New center is at: (0.0, 0.0, 0.0)")
            return True
            
        except Exception as e:
            self.get_logger().error(f"Failed to center STL file: {e}")
            return False
    
    def check_stl_dimensions(self, mesh_file):
        """
        Check STL file dimensions without adding to scene.
        Useful for determining correct scale factor.
        
        Args:
            mesh_file: Path to STL file (can be relative filename, will look in 'mesh' subdirectory)
            
        Returns:
            dict: {'size_x', 'size_y', 'size_z', 'min_x', 'max_x', ..., 'center_x', 'center_y', 'center_z'} or None if failed
        """
        # Resolve mesh file path
        mesh_path = self._resolve_mesh_path(mesh_file)
        if mesh_path is None:
            return None
        
        vertices, _, bbox_center = self._load_stl_binary(mesh_path)
        if vertices is None:
            return None
        
        # Calculate bounding box
        min_x = min(v.x for v in vertices)
        max_x = max(v.x for v in vertices)
        min_y = min(v.y for v in vertices)
        max_y = max(v.y for v in vertices)
        min_z = min(v.z for v in vertices)
        max_z = max(v.z for v in vertices)
        
        result = {
            'size_x': max_x - min_x,
            'size_y': max_y - min_y,
            'size_z': max_z - min_z,
            'min_x': min_x, 'max_x': max_x,
            'min_y': min_y, 'max_y': max_y,
            'min_z': min_z, 'max_z': max_z
        }
        
        if bbox_center:
            result['center_x'] = bbox_center[0]
            result['center_y'] = bbox_center[1]
            result['center_z'] = bbox_center[2]
        
        return result
    
    def add_mesh_collision_object(self, name, mesh_file, x, y, z, 
                                  roll=0.0, pitch=0.0, yaw=0.0,
                                  scale_x=1.0, scale_y=1.0, scale_z=1.0,
                                  center_origin=False):
        """
        Add a mesh collision object from an STL file.
        
        Args:
            name: Unique identifier for the collision object
            mesh_file: Path to STL file (can be relative filename like 'plate.stl' or absolute path).
                      If relative, will look in 'mesh' subdirectory relative to this file's location.
            x, y, z: Position of mesh center (or origin if center_origin=False)
            roll, pitch, yaw: Orientation in radians
            scale_x, scale_y, scale_z: Scaling factors for each axis
            center_origin: If True, translate mesh so bounding box center is at origin (0,0,0).
                          This ensures (x,y,z) represents the geometric center of the mesh.
            
        Returns:
            bool: True if successful
        """
        # Resolve mesh file path
        mesh_path = self._resolve_mesh_path(mesh_file)
        if mesh_path is None:
            self.get_logger().error(f"Could not resolve mesh file path: {mesh_file}")
            return False
        
        self.get_logger().info(f"Adding mesh collision object '{name}' from {mesh_path}")
        
        # Load STL file
        vertices, triangles, bbox_center = self._load_stl_binary(mesh_path)
        if vertices is None or triangles is None:
            return False
        
        # Center vertices at origin if requested
        if center_origin:
            self.get_logger().info("Centering mesh at origin (bounding box center -> 0,0,0)")
            self._center_vertices_at_origin(vertices, bbox_center)
            self.get_logger().info(
                f"Mesh centered. (x,y,z) now represents geometric center of the mesh.")
        
        # Apply scaling to vertices
        if scale_x != 1.0 or scale_y != 1.0 or scale_z != 1.0:
            self.get_logger().info(f"Applying scale: x={scale_x}, y={scale_y}, z={scale_z}")
            for vertex in vertices:
                vertex.x *= scale_x
                vertex.y *= scale_y
                vertex.z *= scale_z
        
        # Create mesh message
        mesh = Mesh()
        mesh.vertices = vertices
        mesh.triangles = []
        
        # Convert flat triangle list to MeshTriangle messages
        for i in range(0, len(triangles), 3):
            triangle = MeshTriangle()
            triangle.vertex_indices = [triangles[i], triangles[i+1], triangles[i+2]]
            mesh.triangles.append(triangle)
        
        # Create collision object
        collision_object = CollisionObject()
        collision_object.header.frame_id = self.base_frame
        collision_object.header.stamp = self.get_clock().now().to_msg()
        collision_object.id = name
        
        # Set mesh
        collision_object.meshes.append(mesh)
        
        # Set pose with scaling
        mesh_pose = Pose()
        mesh_pose.position.x = float(x)
        mesh_pose.position.y = float(y)
        mesh_pose.position.z = float(z)
        
        # Convert euler angles to quaternion
        qx, qy, qz, qw = euler_to_quaternion(roll, pitch, yaw)
        mesh_pose.orientation.x = qx
        mesh_pose.orientation.y = qy
        mesh_pose.orientation.z = qz
        mesh_pose.orientation.w = qw
        
        collision_object.mesh_poses.append(mesh_pose)
        collision_object.operation = CollisionObject.ADD
        
        # Publish to planning scene
        planning_scene = PlanningScene()
        planning_scene.world.collision_objects.append(collision_object)
        planning_scene.is_diff = True
        
        self.planning_scene_pub.publish(planning_scene)
        time.sleep(0.5)
        
        self.collision_objects_added.add(name)
        self.get_logger().info(
            f"✓ Mesh '{name}' added at ({x:.3f}, {y:.3f}, {z:.3f}) "
            f"with {len(mesh.triangles)} triangles")
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

