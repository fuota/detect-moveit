#!/usr/bin/env python3
"""
Utility script to center STL files at origin (bounding box center -> 0,0,0).

Usage:
    python3 center_stl.py input.stl output.stl
    python3 center_stl.py input.stl  # (creates input_centered.stl)
"""

import sys
import os
import rclpy
from rclpy.node import Node
from collision_objects import CollisionObjectMixin


class STLCenterer(Node, CollisionObjectMixin):
    """Simple node to center STL files"""
    
    def __init__(self):
        super().__init__('stl_centerer')
        # Initialize required attributes for CollisionObjectMixin
        self.base_frame = "base_link"
        self.collision_objects_added = set()
        # Create a dummy publisher (not actually used for file operations)
        from moveit_msgs.msg import PlanningScene
        self.planning_scene_pub = self.create_publisher(PlanningScene, '/planning_scene', 10)
    
    def center_file(self, input_file, output_file):
        """Center an STL file and save to output"""
        return self.center_stl_file(input_file, output_file)


def main():
    if len(sys.argv) < 2:
        print("Usage: python3 center_stl.py <input.stl> [output.stl]")
        print("  If output.stl is not provided, creates <input>_centered.stl")
        sys.exit(1)
    
    input_file = sys.argv[1]
    
    if not os.path.exists(input_file):
        print(f"Error: Input file not found: {input_file}")
        sys.exit(1)
    
    # Determine output file
    if len(sys.argv) >= 3:
        output_file = sys.argv[2]
    else:
        base, ext = os.path.splitext(input_file)
        output_file = f"{base}_centered{ext}"
    
    # Initialize ROS2 (required for logger)
    rclpy.init()
    
    try:
        centerer = STLCenterer()
        success = centerer.center_file(input_file, output_file)
        
        if success:
            print(f"\n✓ Success! Centered STL saved to: {output_file}")
            print(f"  You can now use this file with center_origin=True")
            print(f"  or use it directly - the origin is now at the geometric center.")
        else:
            print(f"\n✗ Failed to center STL file")
            sys.exit(1)
    
    finally:
        rclpy.shutdown()


if __name__ == '__main__':
    main()


