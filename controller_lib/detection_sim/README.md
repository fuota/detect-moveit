# Simulated ArUco Detection for Testing

This simulated detection node publishes the **exact same topics** as your real ArUco detection, allowing you to test your `real_world_pick_place.py` without a camera!

## 📦 Files Created

1. **`simulated_aruco_detection.py`** - Simple version with hardcoded objects
2. **`simulated_aruco_detection_yaml.py`** - Enhanced version that loads from YAML
3. **`simulated_detection_config.yaml`** - Configuration file for easy editing

## 🎯 Published Topics (Same as Real Detection)

| Topic | Type | Description |
|-------|------|-------------|
| `/detected_objects/poses` | `PoseArray` | Object poses in base_link frame |
| `/detected_objects/names` | `String` | JSON array of object names |
| `/detected_objects/ids` | `Int32MultiArray` | ArUco marker IDs |
| `/detected_objects/markers` | `MarkerArray` | RViz visualization markers |

## 🚀 Quick Start

### Option 1: Simple Hardcoded Version

1. **Edit the object list** in `simulated_aruco_detection.py`:
```python
self.simulated_objects = [
    {
        'id': 6,  # water_bottle
        'pose': {
            'x': 0.6,   # Change position
            'y': -0.5,
            'z': 0.625,
            'qx': 0.0,
            'qy': 0.0,
            'qz': 0.0,
            'qw': 1.0
        }
    },
    {
        'id': 7,  # medicine_bottle
        'pose': {
            'x': 0.7,
            'y': -0.3,
            'z': 0.625,
            'qx': 0.0,
            'qy': 0.0,
            'qz': 0.0,
            'qw': 1.0
        }
    },
]
```

2. **Run the simulated detection:**
```bash
python3 simulated_aruco_detection.py
```

3. **Run your real-world pick and place** (no code changes needed!):
```bash
python3 real_world_pick_place.py
```

### Option 2: YAML Configuration Version

1. **Edit `simulated_detection_config.yaml`:**
```yaml
detection_rate: 5.0
marker_size: 0.025

simulated_objects:
  - id: 6  # water_bottle
    pose:
      x: 0.6
      y: -0.5
      z: 0.625
      qx: 0.0
      qy: 0.0
      qz: 0.0
      qw: 1.0
  
  - id: 7  # medicine_bottle
    pose:
      x: 0.7
      y: -0.3
      z: 0.625
      qx: 0.0
      qy: 0.0
      qz: 0.0
      qw: 1.0

object_names:
  6: "water_bottle"
  7: "medicine_bottle"
```

2. **Run with YAML config:**
```bash
python3 simulated_aruco_detection_yaml.py --ros-args -p config_file:=simulated_detection_config.yaml
```

3. **Run your real-world pick and place:**
```bash
python3 real_world_pick_place.py
```

## ✨ Key Features

✅ **Drop-in Replacement** - Uses exact same API as real detection
✅ **No Code Changes** - Your `real_world_pick_place.py` works unchanged
✅ **Easy Configuration** - Edit object positions in code or YAML
✅ **RViz Visualization** - See markers in RViz just like real detection
✅ **Same Message Format** - Identical message structure and timing

## 🔄 Workflow Comparison

### Real Robot Workflow:
```
Camera → Real Detection Node → Topics → real_world_pick_place.py → Robot
```

### Simulation Workflow:
```
YAML/Code → Simulated Detection Node → Topics → real_world_pick_place.py → Robot
```

**The pick and place code stays identical!** 🎉

## 📝 How to Add/Edit Objects

### Method 1: Edit Python File Directly

In `simulated_aruco_detection.py`, find the `simulated_objects` list:

```python
self.simulated_objects = [
    # Add new object:
    {
        'id': 6,           # ArUco marker ID
        'pose': {
            'x': 0.6,      # Position in meters (base_link frame)
            'y': -0.5,
            'z': 0.625,
            'qx': 0.0,     # Orientation quaternion
            'qy': 0.0,
            'qz': 0.0,
            'qw': 1.0
        }
    },
]
```

### Method 2: Edit YAML File

In `simulated_detection_config.yaml`:

```yaml
simulated_objects:
  - id: 6
    pose:
      x: 0.6
      y: -0.5
      z: 0.625
      qx: 0.0
      qy: 0.0
      qz: 0.0
      qw: 1.0
```

## 🎨 RViz Visualization

Both nodes publish markers to `/detected_objects/markers`. To visualize:

1. Open RViz
2. Add MarkerArray display
3. Set topic to `/detected_objects/markers`
4. You'll see cubes at object positions with colors:
   - Blue (ID 6): Water bottle
   - Red (ID 7): Medicine bottle

## 🧪 Testing Your Setup

### Terminal 1: Run simulated detection
```bash
python3 simulated_aruco_detection.py
```

### Terminal 2: Check topics are publishing
```bash
# Check pose array
ros2 topic echo /detected_objects/poses

# Check IDs
ros2 topic echo /detected_objects/ids

# Check names
ros2 topic echo /detected_objects/names
```

### Terminal 3: Run your pick and place
```bash
python3 real_world_pick_place.py
```

## 🔧 Advanced: Adding New Object Types

1. **Add to object map** in both files:
```python
OBJECT_MAP = {
    6: "water_bottle",
    7: "medicine_bottle",
    8: "new_object",  # Add here
}
```

2. **Add detection in simulation:**
```python
{
    'id': 8,  # new_object
    'pose': {
        'x': 0.8,
        'y': -0.4,
        'z': 0.625,
        'qx': 0.0,
        'qy': 0.0,
        'qz': 0.0,
        'qw': 1.0
    }
}
```

3. **Update YAML if using YAML version:**
```yaml
object_names:
  6: "water_bottle"
  7: "medicine_bottle"
  8: "new_object"

simulated_objects:
  - id: 8
    pose:
      x: 0.8
      y: -0.4
      z: 0.625
      # ... etc
```

## 🐛 Troubleshooting

### Objects Not Detected
- Check that IDs are in `OBJECT_MAP` 
- Verify poses are in `base_link` frame
- Check topics are publishing: `ros2 topic list | grep detected_objects`

### Position Doesn't Match Expected
- Ensure coordinates are in `base_link` frame (not camera frame!)
- Check your collision object positions match simulated poses
- Verify z-coordinate includes table height

### Pick Fails in Simulation
- Make sure simulated pose matches where you want to grasp
- Check collision objects in environment are correctly placed
- Verify approach distance can reach the object

## 📊 Object Pose Calculation Helper

If you have table setup like in your simulation:
```python
table_x = 0.7 
table_y = -0.5
table_z = 0.2 
table_height = 0.7
cylinder_height = 0.15

# Object position on top of table:
object_x = 0.6  # Adjust as needed
object_y = -0.5
object_z = table_z + table_height/2 + cylinder_height/2  # = 0.625
```

## 🎯 Complete Example

Here's a complete example with 2 objects:

**simulated_detection_config.yaml:**
```yaml
detection_rate: 5.0
marker_size: 0.025

simulated_objects:
  - id: 6  # water_bottle at position 1
    pose:
      x: 0.6
      y: -0.5
      z: 0.625
      qx: 0.0
      qy: 0.0
      qz: 0.0
      qw: 1.0
  
  - id: 7  # medicine_bottle at position 2
    pose:
      x: 0.7
      y: -0.3
      z: 0.625
      qx: 0.0
      qy: 0.0
      qz: 0.0
      qw: 1.0

object_names:
  6: "water_bottle"
  7: "medicine_bottle"
```

**Running the system:**
```bash
# Terminal 1: Simulated detection
python3 detection_sim.py --ros-args -p config_file:=config.yaml

# Terminal 2: Your pick and place (unchanged!)
python3 real_world_pick_place.py
```

## 🔀 Switching Between Real and Simulated

**To use REAL camera detection:**
```bash
python3 detect_objects_publisher.py  # Your original file
python3 real_world_pick_place.py
```

**To use SIMULATED detection:**
```bash
python3 detection_sim.py  # OR the YAML version
python3 real_world_pick_place.py      # Same file, no changes!
```
