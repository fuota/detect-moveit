#!/usr/bin/env python3
import sys
import os

# Add the install path to Python path for service imports
install_dir = os.environ.get('AMENT_PREFIX_PATH', '').split(':')[0]
if install_dir:
    srv_path = os.path.join(install_dir, 'local', 'lib', 'python3.10', 'site-packages')
    if os.path.exists(srv_path):
        sys.path.insert(0, srv_path)

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, CameraInfo
from geometry_msgs.msg import PoseStamped, PoseArray
from std_msgs.msg import String, Int32MultiArray
from visualization_msgs.msg import Marker, MarkerArray

import cv2
import cv2.aruco as aruco
import numpy as np
from cv_bridge import CvBridge

import tf2_ros
import tf2_geometry_msgs
import json


OBJECT_MAP = {
    6: "water_bottle",
    7: "medicine_bottle"
}


class DetectObjectsPublisher(Node):
    def __init__(self):
        super().__init__('detect_objects_publisher')

        # Parameters
        self.declare_parameter('detection_rate', 5.0)  # Hz
        self.declare_parameter('marker_size', 0.025)  # meters
        
        detection_rate = self.get_parameter('detection_rate').value
        self.marker_size = self.get_parameter('marker_size').value

        # Bridge for ROS <-> OpenCV
        self.bridge = CvBridge()

        # Camera info (intrinsics)
        self.camera_matrix = None
        self.dist_coeffs = None
        self.latest_image = None

        # Subscriptions
        self.create_subscription(CameraInfo, "/camera/camera/color/camera_info", self.info_callback, 10)
        self.create_subscription(Image, "/camera/camera/color/image_raw", self.image_callback, 10)

        # Publishers
        self.pose_array_pub = self.create_publisher(PoseArray, '/detected_objects/poses', 10)
        self.names_pub = self.create_publisher(String, '/detected_objects/names', 10)
        self.ids_pub = self.create_publisher(Int32MultiArray, '/detected_objects/ids', 10)
        self.marker_viz_pub = self.create_publisher(MarkerArray, '/detected_objects/markers', 10)
        self.annotated_image_pub = self.create_publisher(Image, '/detected_objects/image', 10)

        # TF with cache
        self.tf_buffer = tf2_ros.Buffer(cache_time=rclpy.duration.Duration(seconds=10.0))
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        # ArUco setup
        self.aruco_dict = aruco.getPredefinedDictionary(aruco.DICT_4X4_50)
        self.parameters = aruco.DetectorParameters()

        # Timer for periodic detection
        self.timer = self.create_timer(1.0 / detection_rate, self.detect_and_publish)
        
        self.get_logger().info(f"DetectObjects Publisher started (rate: {detection_rate} Hz)")

    def info_callback(self, msg: CameraInfo):
        if self.camera_matrix is None:
            self.camera_matrix = np.array(msg.k).reshape(3, 3)
            self.dist_coeffs = np.array(msg.d)
            self.get_logger().info("Loaded camera intrinsics.")

    def image_callback(self, msg: Image):
        self.latest_image = msg

    def detect_and_publish(self):
        """Detect objects and publish to topics"""
        if self.camera_matrix is None or self.latest_image is None:
            return

        frame = self.bridge.imgmsg_to_cv2(self.latest_image, "bgr8")
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

        # Detect markers
        corners, ids, _ = aruco.detectMarkers(gray, self.aruco_dict, parameters=self.parameters)

        # Prepare messages
        pose_array = PoseArray()
        pose_array.header.frame_id = "base_link"
        pose_array.header.stamp = self.get_clock().now().to_msg()

        names_list = []
        ids_msg = Int32MultiArray()
        marker_array = MarkerArray()

        # Draw on image
        annotated_frame = frame.copy()
        if ids is not None:
            aruco.drawDetectedMarkers(annotated_frame, corners, ids)

        if ids is not None:
            self.get_logger().info(f"Detected marker IDs: {ids.flatten().tolist()}", 
                                  throttle_duration_sec=2.0)
            
            for i, marker_id in enumerate(ids.flatten()):
                if marker_id not in OBJECT_MAP:
                    continue

                # Estimate pose
                rvec, tvec, _ = aruco.estimatePoseSingleMarkers(
                    corners[i], self.marker_size, self.camera_matrix, self.dist_coeffs)

                # Draw axis
                cv2.drawFrameAxes(annotated_frame, self.camera_matrix, self.dist_coeffs,
                                rvec[0], tvec[0], self.marker_size * 0.5)

                # Create pose in camera frame
                pose_cam = PoseStamped()
                pose_cam.header.frame_id = "camera_color_optical_frame"
                pose_cam.header.stamp = self.get_clock().now().to_msg()
                pose_cam.pose.position.x = float(tvec[0][0][0])
                pose_cam.pose.position.y = float(tvec[0][0][1])
                pose_cam.pose.position.z = float(tvec[0][0][2])

                # Convert rvec to quaternion
                R, _ = cv2.Rodrigues(rvec[0][0])
                qw = np.sqrt(max(0, 1.0 + R[0, 0] + R[1, 1] + R[2, 2])) / 2.0
                if qw < 1e-6:
                    qw = 1.0
                    qx = qy = qz = 0.0
                else:
                    qx = (R[2, 1] - R[1, 2]) / (4.0 * qw)
                    qy = (R[0, 2] - R[2, 0]) / (4.0 * qw)
                    qz = (R[1, 0] - R[0, 1]) / (4.0 * qw)
                
                pose_cam.pose.orientation.x = qx
                pose_cam.pose.orientation.y = qy
                pose_cam.pose.orientation.z = qz
                pose_cam.pose.orientation.w = qw

                try:
                    # Transform to base_link
                    trans = self.tf_buffer.lookup_transform(
                        "base_link",
                        pose_cam.header.frame_id,
                        rclpy.time.Time(),
                        timeout=rclpy.duration.Duration(seconds=0.5))

                    pose_base = tf2_geometry_msgs.do_transform_pose_stamped(pose_cam, trans)

                    # Add to messages
                    pose_array.poses.append(pose_base.pose)
                    names_list.append(OBJECT_MAP[marker_id])
                    ids_msg.data.append(int(marker_id))

                    # Create visualization marker
                    marker = self.create_marker(marker_id, OBJECT_MAP[marker_id], 
                                               pose_base.pose, i)
                    marker_array.markers.append(marker)

                    # Log detection
                    self.get_logger().info(
                        f"{OBJECT_MAP[marker_id]} at "
                        f"({pose_base.pose.position.x:.3f}, "
                        f"{pose_base.pose.position.y:.3f}, "
                        f"{pose_base.pose.position.z:.3f})",
                        throttle_duration_sec=2.0
                    )

                except Exception as e:
                    self.get_logger().warn(f"TF transform failed: {e}", 
                                          throttle_duration_sec=1.0)

        # Publish all messages
        self.pose_array_pub.publish(pose_array)
        self.ids_pub.publish(ids_msg)
        
        # Publish names as JSON string
        names_msg = String()
        names_msg.data = json.dumps(names_list)
        self.names_pub.publish(names_msg)

        # Publish markers
        if len(marker_array.markers) > 0:
            self.marker_viz_pub.publish(marker_array)
        else:
            # Clear markers if nothing detected
            delete_marker = Marker()
            delete_marker.action = Marker.DELETEALL
            marker_array.markers = [delete_marker]
            self.marker_viz_pub.publish(marker_array)

        # Publish annotated image
        try:
            annotated_msg = self.bridge.cv2_to_imgmsg(annotated_frame, "bgr8")
            self.annotated_image_pub.publish(annotated_msg)
        except Exception as e:
            self.get_logger().warn(f"Failed to publish image: {e}")

        # Log summary
        if len(names_list) > 0:
            self.get_logger().info(
                f"Published {len(names_list)} detections: {names_list}",
                throttle_duration_sec=2.0)

    def create_marker(self, marker_id, name, pose, index):
        """Create visualization marker"""
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
        
        # Color by ID
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


def main(args=None):
    rclpy.init(args=args)
    node = DetectObjectsPublisher()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()