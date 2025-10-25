#!/usr/bin/env python3
"""
Simulation Pick and Place using MoveIt2 Controller
Creates collision objects manually for testing in simulation.
"""

import rclpy
from moveit_controller import MoveItController
from geometry_msgs.msg import Pose
import time
import math


class SimulationPickPlace(MoveItController):
    """Simulation version with manually created objects"""
    
    def __init__(self):
        super().__init__(node_name='simulation_pick_place')
        self.get_logger().info("=== Simulation Pick & Place Started ===")
    
    def setup_simulation_environment(self):
        """Setup collision objects for simulation"""
        self.get_logger().info("Setting up simulation environment...")
        
        # Wheelchair seat
        self.add_box_collision_object(
            "wheel_chair_seat", 
            x=-0.43, y=-0.1, z=0, 
            width=0.6, depth=0.5, height=1.86)
        
        # Wheelchair back
        chair_back_z = -0.54 + 1.86/2
        self.add_box_collision_object(
            "wheel_chair_back", 
            x=-0.43, y=0.3, z=chair_back_z, 
            width=0.34, depth=0.35, height=1.86)
        
        # Work table
        table_x = 0.7 
        table_y = -0.5
        table_z = 0.2 
        table_height = 0.7
        self.add_box_collision_object(
            "work_table", 
            x=table_x, y=table_y, z=table_z, 
            width=0.8, depth=1.2, height=table_height)
        
        # Target cylinder
        cylinder_x = 0.6
        cylinder_y = -0.5
        cylinder_height = 0.15
        cylinder_z = table_z + table_height/2 + cylinder_height/2 
        cylinder_radius = 0.025 
        self.add_cylinder_collision_object(
            "target_cylinder", 
            x=cylinder_x, y=cylinder_y, z=cylinder_z,
            radius=cylinder_radius, height=cylinder_height)

        self.get_logger().info("✓ Simulation environment setup completed")
        
        # Return the target pose for pick
        target_pose = Pose()
        target_pose.position.x = cylinder_x
        target_pose.position.y = cylinder_y
        target_pose.position.z = cylinder_z

        return target_pose


def main(args=None):
    rclpy.init(args=args)
    
    controller = SimulationPickPlace()
    
    # Setup environment and get target pose
    grasp_pose = controller.setup_simulation_environment()
    time.sleep(1.0)
    
    controller.get_logger().info(
        f"\nTarget grasp pose: x={grasp_pose.position.x:.3f}, "
        f"y={grasp_pose.position.y:.3f}, z={grasp_pose.position.z:.3f}")
    
    # Execute pick with Cartesian paths
    if controller.pick_object("target_cylinder", grasp_pose):
        controller.get_logger().info("\n🎉 Pick successful!")

        current_pose = controller.get_current_pose()
        if current_pose:
            controller.get_logger().info(
                f"Current pose after pick: x={current_pose.position.x:.3f}, "
                f"y={current_pose.position.y:.3f}, z={current_pose.position.z:.3f}")
            
            # Place 
            place_pose = current_pose
            place_pose.position.y = current_pose.position.y + 0.2

            if controller.place_object("target_cylinder", place_pose):
                controller.get_logger().info(
                    "\n🎉 Place successful! Pick and place operation completed.")
            else:
                controller.get_logger().error("Place operation failed.")
        else:
            controller.get_logger().error("Failed to get current pose after pick.")
    else:
        controller.get_logger().error("\n❌ Pick operation failed.")
    
    # Keep node alive briefly
    rclpy.spin_once(controller, timeout_sec=1.0)
    controller.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
