# ROS2 Backend Setup Guide

## Project Overview

This project provides a ROS2-based robotic manipulation system for autonomous pick-and-place operations. The system uses ArUco marker detection to identify objects in the workspace and executes complex manipulation tasks such as:

- **Medicine Preparation:** Placing water cups, plates, and pouring liquids
- **Table Setup:** Organizing tableware (bowls, forks, spoons) in serving areas
- **Book Organization:** Placing books in bookshelf compartments

The system integrates with a web/mobile UI through ROSBridge, allowing users to monitor task progress in real-time and trigger tasks remotely. The backend handles object detection, motion planning with MoveIt2, and collision avoidance to ensure safe and accurate manipulation.

This guide explains how to set up and run the ROS2 backend for the robot control system.

**Important:** All commands in this guide should be run from the `detect-moveit` directory (the repository root). After cloning the repository, navigate to the directory:

```bash
cd detect-moveit
```

## Hardware Requirements

This project is designed to work with:
- **Robot:** Kinova Gen3 7-DOF robotic arm
- **Gripper:** Robotiq 2F-85 adaptive gripper
- **Camera:** Intel RealSense depth camera (mounted on end effector)

**Network Requirements:**
- The control computer and the robot must be connected to the **same WiFi network**
- The robot must have a **static IP address** (default: `192.168.0.10`)
- Ensure the control computer can ping the robot's IP address before starting

## Prerequisites

- ROS2 (Humble)
- RealSense camera drivers (`realsense2_camera`)
- Kinova Gen3 robot MoveIt configuration
- Python 3.8 or later
- pip (Python package manager)

## Installation

Before running the ROS nodes, install the required Python dependencies:

```bash
pip3 install -r requirements.txt
```

**What it installs:**
- `numpy` - Numerical computing library
- `opencv-python` - Computer vision library for ArUco marker detection
- `pillow` - Image processing library
- `charset-normalizer` - Character encoding detection
- `reportlab` - PDF generation (for ArUco marker generation)

**Note:** If you're using a virtual environment or conda, activate it before installing dependencies.

## Environment Configuration

Before running the system, you need to configure the static environment to match your physical setup.

### Workspace Table Setup

The system uses a workspace table where objects are placed. The default workspace table parameters are defined in `controller_lib/controller/real_detection_pick_place.py`:

```python
self.workspace_table = {
    'dx': 0.43,      # X position offset from base (meters)
    'dy': -0.31,     # Y position offset from base (meters)
    'dz': -0.37,     # Z position offset from base (meters)
    'width': 0.55,   # Table width (meters)
    'depth': 0.55,   # Table depth (meters)
    'height': 0.63  # Table height (meters)
}
```

**Important:** Measure your actual workspace table dimensions and position relative to the robot base, then update these values in the code.

### Static Environment Configuration

The `setup_static_environment()` method in `controller_lib/controller/real_detection_pick_place.py` defines collision objects for:
- Wheelchair (seat and back)
- Big table
- Workspace table
- Wall

**You must modify these values to match your actual physical environment:**

1. Open `controller_lib/controller/real_detection_pick_place.py`
2. Locate the `setup_static_environment()` method (around line 1287)
3. Update the position and dimensions of:
   - `wheel_chair_seat` and `wheel_chair_back` (if applicable)
   - `big_table` (if present in your setup)
   - `workspace_table` (must match your actual table)
   - `wall` (if present)

All measurements are in meters relative to the robot's base frame (`base_link`). Use a measuring tool to get accurate dimensions and positions.

## Setup Instructions

Run the following commands in **separate terminals** (5 terminals total):

### Terminal 1: RealSense Camera
Launches the RealSense depth camera with optimized settings for object detection.

```bash
ros2 launch realsense2_camera rs_launch.py depth_module.profile:=640x480x30 color_module.enable:=false
```

**What it does:** Starts the RealSense camera node publishing depth images at 640x480 resolution at 30fps. Color is disabled to optimize performance.

---

### Terminal 2: Kinova Robot & MoveIt
Launches the Kinova Gen3 robot controller and MoveIt motion planning framework.

```bash
ros2 launch kinova_gen3_7dof_robotiq_2f_85_moveit_config robot.launch.py \
  robot_ip:=192.168.0.10
```

**What it does:** 
- Connects to the Kinova Gen3 robot at IP `192.168.0.10`
- Starts MoveIt for motion planning and execution
- Enables the Robotiq 2F-85 gripper control

**Note:** 
- Update `robot_ip` if your robot has a different IP address
- Ensure your computer and robot are on the same WiFi network
- Verify connectivity by pinging the robot: `ping 192.168.0.10`

---

### Terminal 3: Camera Transform (Color Optical Frame)
Publishes the static transform between the end effector and the camera's color optical frame.

```bash
ros2 run tf2_ros static_transform_publisher \
  0.052364 0.060511 0.023395 \
  0.012047 0.003220 0.999053 0.041678 \
  end_effector_link camera_color_optical_frame
```

**What it does:** Establishes the coordinate frame transformation so the system knows where the camera is relative to the robot's end effector. This transform includes both position (x, y, z) and orientation (quaternion).

---

### Terminal 4: Camera Transform (Camera Link)
Publishes the static transform between the end effector and the camera link frame.

```bash
ros2 run tf2_ros static_transform_publisher \
  0.0 0.0 0.0 0.0 0.0 0.0 1.0 \
  end_effector_link camera_link
```

**What it does:** Provides an identity transform for the camera link frame (no offset, no rotation).

---

### Terminal 5: ROSBridge Server (WebSocket API)
Starts the ROSBridge server to expose ROS2 services and topics via WebSocket for UI integration.

```bash
ros2 run rosbridge_server rosbridge_websocket --ros-args -p port:=9090
```

**What it does:** 
- Starts a WebSocket server on port 9090
- Allows web/mobile UIs to connect and interact with ROS2 services/topics
- Your UI can connect to `ws://localhost:9090` using ROSLIB.js or similar libraries

---

## Running the Application

After all 5 ROS nodes are running, execute the Python scripts:

### Terminal 6: Object Detection
Starts the ArUco marker detection system that identifies objects in the camera view.

```bash
python3 controller_lib/detection_real/detection.py
```

**What it does:** 
- Subscribes to camera depth/image topics
- Detects ArUco markers and publishes object poses
- Publishes to `/detected_objects/poses`, `/detected_objects/ids`, and `/detected_objects/names`

---

### Terminal 7: Pick and Place Controller
Starts the main robot controller that handles task execution.

```bash
python3 controller_lib/controller/real_detection_pick_place.py
```

**What it does:**
- Subscribes to detected object poses
- Provides ROS2 services for tasks: `/prepare_medicine`, `/setup_tableware`, `/organize_books`
- Publishes task progress to `/task_progress` topic
- Executes pick-and-place operations using MoveIt

---

## Verifying the Setup

Once everything is running, you can test the system:

1. **Check available tasks:**
   ```bash
   ros2 service call /get_available_tasks std_srvs/srv/Trigger
   ```

2. **Monitor task progress:**
   ```bash
   ros2 topic echo /task_progress
   ```

3. **Start a task (e.g., prepare medicine):**
   ```bash
   ros2 service call /prepare_medicine std_srvs/srv/Trigger
   ```

## Troubleshooting

- **Robot connection issues:** 
  - Verify the robot IP address and network connectivity
  - Ensure both computer and robot are on the same WiFi network
  - Check robot IP configuration matches the launch file parameter
  - Test connectivity: `ping <robot_ip>`
- **Camera not detected:** Check USB connection and ensure RealSense drivers are installed
- **Transform errors:** Ensure both transform publishers are running before starting detection
- **ROSBridge connection:** Verify port 9090 is not in use by another application
- **Collision detection issues:** Verify static environment parameters in `real_detection_pick_place.py` match your physical setup
- **Workspace table errors:** Ensure workspace table dimensions and position are correctly configured

