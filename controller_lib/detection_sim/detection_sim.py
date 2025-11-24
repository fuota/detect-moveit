#!/usr/bin/env python3
"""
Simulated ArUco Detection Publisher with YAML Configuration
Mimics the real ArUco detection node API for testing without a camera.

Publishes the same topics as the real detection node:
- /detected_objects/poses (PoseArray)
- /detected_objects/names (String - JSON)
- /detected_objects/ids (Int32MultiArray)
- /detected_objects/markers (MarkerArray - for RViz)

Can load configuration from YAML file or use hardcoded values.
"""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseArray, Pose
from std_msgs.msg import String, Int32MultiArray
from visualization_msgs.msg import Marker, MarkerArray
import json
import yaml
import os
from std_srvs.srv import Trigger


# Default object name mapping
DEFAULT_OBJECT_MAP = {
    6: "water_bottle",
    7: "medicine_bottle"
}


class SimulatedDetectionPublisher(Node):
    def __init__(self):
        super().__init__('simulated_detection_publisher')

        # Parameters
        self.declare_parameter('detection_rate', 5.0)  # Hz
        self.declare_parameter('marker_size', 0.025)  # meters
        self.declare_parameter('config_file', '')  # Optional YAML config file
        self.declare_parameter('clear_on_start', False)  # clear objects at startup

        
        detection_rate = self.get_parameter('detection_rate').value
        self.marker_size = self.get_parameter('marker_size').value
        config_file = self.get_parameter('config_file').value

        # Publishers (same topics as real detection)
        self.pose_array_pub = self.create_publisher(PoseArray, '/detected_objects/poses', 10)
        self.names_pub = self.create_publisher(String, '/detected_objects/names', 10)
        self.ids_pub = self.create_publisher(Int32MultiArray, '/detected_objects/ids', 10)
        self.marker_viz_pub = self.create_publisher(MarkerArray, '/detected_objects/markers', 10)

        # Load configuration
        if config_file and os.path.exists(config_file):
            self.get_logger().info(f"Loading configuration from: {config_file}")
            self.load_config_from_yaml(config_file)
        else:
            if config_file:
                self.get_logger().warn(f"Config file not found: {config_file}, using hardcoded values")
            else:
                self.get_logger().info("No config file specified, using hardcoded values")
            self.load_default_config()

        # Timer for periodic publishing
        self.timer = self.create_timer(1.0 / detection_rate, self.publish_detections)
        # Clear service
        self.clear_srv = self.create_service(Trigger, '~/clear_objects', self.handle_clear_objects)

        # Optionally clear at startup
        if self.get_parameter('clear_on_start').value:
            self.clear_objects()

        self.get_logger().info(f"Simulated Detection Publisher started (rate: {detection_rate} Hz)")
        self.get_logger().info(f"Simulating {len(self.simulated_objects)} objects:")
        for obj in self.simulated_objects:
            obj_id = obj['id']
            name = self.object_map.get(obj_id, f"marker_{obj_id}")
            pose = obj['pose']
            self.get_logger().info(
                f"  [{obj_id}] {name} at ({float(pose['x']):.3f}, {float(pose['y']):.3f}, {float(pose['z']):.3f})")

    def load_config_from_yaml(self, config_file):
        """Load configuration from YAML file"""
        self.get_logger().info(f"Loading configuration from: {config_file}")
        try:
            with open(config_file, 'r') as f:
                config = yaml.safe_load(f)
            
            # Load object map
            self.object_map = config.get('object_names', DEFAULT_OBJECT_MAP)
            
            # Override parameters if specified in YAML
            if 'detection_rate' in config:
                detection_rate = config['detection_rate']
                self.set_parameters([rclpy.parameter.Parameter(
                    'detection_rate', 
                    rclpy.Parameter.Type.DOUBLE, 
                    detection_rate)])
            
            if 'marker_size' in config:
                self.marker_size = config['marker_size']
            
            # Load simulated objects
            self.simulated_objects = config.get('simulated_objects', [])
            
            self.get_logger().info(f"✓ Configuration loaded from YAML")
            
        except Exception as e:
            self.get_logger().error(f"Failed to load config: {e}")
            self.get_logger().info("Falling back to default configuration")
            self.load_default_config()

    def load_default_config(self):
        """Load default hardcoded configuration"""
        self.object_map = DEFAULT_OBJECT_MAP
        
        # ============================================================
        # EDIT THIS: Define your simulated detected objects
        # ============================================================
        self.simulated_objects = [
            {
                'id': 6,  # water_bottle
                'pose': {
                    'x': 0.7,
                    'y': -0.02,
                    'z': 0.29,  # table_z + table_height/2 + cylinder_height/2
                    'qx': 0.0,
                    'qy': 0.0,
                    'qz': 0.0,
                    'qw': 1.0
                }
            },
            # Add more objects here:
            # {
            #     'id': 7,  # medicine_bottle
            #     'pose': {
            #         'x': 0.7,
            #         'y': -0.3,
            #         'z': 0.625,
            #         'qx': 0.0,
            #         'qy': 0.0,
            #         'qz': 0.0,
            #         'qw': 1.0
            #     }
            # },
        ]
        # ============================================================

    def publish_detections(self):
        """Publish simulated detections to all topics"""
        
        # Prepare messages
        pose_array = PoseArray()
        pose_array.header.frame_id = "base_link"
        pose_array.header.stamp = self.get_clock().now().to_msg()

        names_list = []
        ids_msg = Int32MultiArray()
        marker_array = MarkerArray()

        # Process each simulated object
        for i, obj in enumerate(self.simulated_objects):
            marker_id = obj['id']
            pose_data = obj['pose']
            
            # Skip if not in object map
            if marker_id not in self.object_map:
                continue

            # Create pose
            pose = Pose()
            pose.position.x = pose_data['x']
            pose.position.y = pose_data['y']
            pose.position.z = pose_data['z']
            pose.orientation.x = pose_data['qx']
            pose.orientation.y = pose_data['qy']
            pose.orientation.z = pose_data['qz']
            pose.orientation.w = pose_data['qw']

            # Add to messages
            pose_array.poses.append(pose)
            names_list.append(self.object_map[marker_id])
            ids_msg.data.append(int(marker_id))

            # Create visualization marker
            marker = self.create_marker(marker_id, self.object_map[marker_id], pose, i)
            marker_array.markers.append(marker)

        # Publish all messages (same as real detection)
        self.pose_array_pub.publish(pose_array)
        self.ids_pub.publish(ids_msg)
        
        # Publish names as JSON string
        names_msg = String()
        names_msg.data = json.dumps(names_list)
        self.names_pub.publish(names_msg)

        # Publish markers for RViz
        if len(marker_array.markers) > 0:
            self.marker_viz_pub.publish(marker_array)
        else:
            # Clear markers if nothing detected
            delete_marker = Marker()
            delete_marker.action = Marker.DELETEALL
            marker_array.markers = [delete_marker]
            self.marker_viz_pub.publish(marker_array)

    def create_marker(self, marker_id, name, pose, index):
        """Create visualization marker (same as real detection)"""
        marker = Marker()
        marker.header.frame_id = "base_link"
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.ns = "detected_objects"
        marker.id = index
        marker.type = Marker.CUBE
        marker.action = Marker.ADD
        
        marker.pose = pose
        marker.scale.x = self.marker_size
        marker.scale.y = self.marker_size
        marker.scale.z = 0.005
        
        # Color by ID (same as real detection)
        if marker_id == 6:
            marker.color.r = 0.0
            marker.color.g = 0.5
            marker.color.b = 1.0
        elif marker_id == 7:
            marker.color.r = 1.0
            marker.color.g = 0.0
            marker.color.b = 0.0
        else:
            marker.color.r = 0.5
            marker.color.g = 0.5
            marker.color.b = 0.5
        
        marker.color.a = 0.7
        marker.lifetime = rclpy.duration.Duration(seconds=0.5).to_msg()
        
        return marker
    
    def clear_objects(self):
        """Clear all detected objects"""
        self.simulated_objects = []
        self.get_logger().info("Cleared all simulated objects.")

    def handle_clear_objects(self, request, response):
        """Service callback to clear simulated objects at runtime"""
        self.clear_objects()
        response.success = True
        response.message = "Simulated objects cleared."
        return response


def main(args=None):
    rclpy.init(args=args)
    node = SimulatedDetectionPublisher()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()