# Modular MoveIt2 Pick and Place System

This package provides a modular approach to pick and place operations using MoveIt2 with Cartesian path planning.

## Architecture

### Base Class: `moveit2_controller.py`
**Core MoveIt2 functionality that both versions inherit from:**

- ✅ MoveIt2 action clients and service clients
- ✅ Cartesian path planning
- ✅ Standard motion planning
- ✅ Gripper control (open/close)
- ✅ Collision object management
- ✅ Pick and place operations
- ✅ TF-based pose tracking

### Implementation 1: `simulation_pick_place.py`
**For simulation with manually created objects to test at home:**

```python
class SimulationPickPlace(MoveIt2Controller):
    def setup_simulation_environment(self):
        # Manually create collision objects
        self.add_table_collision_object(...)
        self.add_cylinder_collision_object(...)
```

**Usage:**
```bash
ros2 run your_package simulation_pick_place.py
```

### Implementation 2: `real_world_pick_place.py`
**For real-world with ArUco detection to use in the lab:**

```python
class RealWorldPickPlace(MoveIt2Controller):
    def __init__(self):
        # Subscribe to ArUco topics
        # Dynamically create collision objects
```

**Usage:**
```bash
ros2 run your_package real_world_pick_place.py
```

## Key Features

### 1. **Shared MoveIt2 Functionality**
Both implementations use the same core methods:
- `pick_object(object_name, grasp_pose)` - Complete pick operation
- `place_object(object_name, place_pose)` - Complete place operation
- `move_to_pose(target_pose)` - Standard motion planning
- `move_cartesian(waypoints)` - Cartesian path planning
- `move_relative_cartesian(delta_x, delta_y, delta_z)` - Relative Cartesian motion

### 2. **Automatic Fallback**
If Cartesian path planning fails or is unavailable:
- Automatically falls back to standard motion planning
- Logs warnings for debugging
- Ensures operation continues

### 3. **Collision Object Management**
- `add_table_collision_object()` - Add box-shaped obstacles
- `add_cylinder_collision_object()` - Add cylindrical objects
- `remove_collision_object()` - Remove objects
- `attach_object_to_gripper()` - Attach for collision-aware motion
- `detach_object_from_gripper()` - Release object

## How to Adapt for Your Use Case

### Option 1: Use Simulation Version
If you want to test with manually placed objects:
1. Modify `setup_simulation_environment()` in `simulation_pick_place.py`
2. Add/remove collision objects as needed
3. Set target grasp pose

### Option 2: Use Real-world Version
If you have ArUco detection or similar:
1. Modify topic names in `real_world_pick_place.py` if needed
2. Adjust `update_collision_cylinder()` for your object dimensions
3. Modify `setup_static_environment()` for your workspace

### Option 3: Create Custom Implementation
Inherit from `MoveIt2Controller` for any custom use case:

```python
from moveit2_controller_base import MoveIt2Controller

class MyCustomPickPlace(MoveIt2Controller):
    def __init__(self):
        super().__init__(node_name='my_custom_node')
        # Add your custom initialization
    
    def my_custom_method(self):
        # Use inherited methods:
        # - self.pick_object(...)
        # - self.place_object(...)
        # - self.move_to_pose(...)
        # - self.add_cylinder_collision_object(...)
        pass
```

## Benefits of Modular Design

✅ **No Code Duplication** - MoveIt2 code is written once, used everywhere

✅ **Easy Updates** - Fix bugs or add features in one place

✅ **Flexible** - Create new implementations without touching base code

✅ **Maintainable** - Clear separation of concerns

✅ **Testable** - Test base functionality independently

## Method Reference

### Motion Control
| Method | Description | Returns |
|--------|-------------|---------|
| `move_to_pose(pose)` | Move to target pose (standard) | bool |
| `move_cartesian(waypoints)` | Move through waypoints (Cartesian) | bool |
| `move_relative_cartesian(dx, dy, dz)` | Relative Cartesian motion | bool |
| `get_current_pose()` | Get end-effector pose | Pose |

### Gripper Control
| Method | Description | Returns |
|--------|-------------|---------|
| `open_gripper()` | Open gripper | bool |
| `close_gripper(force=0.8)` | Close gripper | bool |

### Collision Objects
| Method | Description | Returns |
|--------|-------------|---------|
| `add_table_collision_object(name, x, y, z, width, depth, height)` | Add box | bool |
| `add_cylinder_collision_object(name, x, y, z, radius, height)` | Add cylinder | bool |
| `remove_collision_object(name)` | Remove object | bool |
| `attach_object_to_gripper(name)` | Attach to gripper | bool |
| `detach_object_from_gripper(name)` | Detach from gripper | bool |

### High-Level Operations
| Method | Description | Returns |
|--------|-------------|---------|
| `pick_object(name, pose, **params)` | Complete pick sequence | bool |
| `place_object(name, pose, **params)` | Complete place sequence | bool |

## Pick Operation Steps

The `pick_object()` method executes these steps automatically:

1. **Approach** - Move to pre-grasp pose (standard planning)
2. **Open Gripper** - Open gripper to prepare
3. **Attach Object** - Attach for collision checking
4. **Grasp** - Move closer using **Cartesian path** (smooth approach)
5. **Close Gripper** - Grasp the object
6. **Lift** - Lift using **Cartesian path** (straight up motion)

## Place Operation Steps

The `place_object()` method executes these steps automatically:

1. **Move to Place** - Move to place pose
2. **Open Gripper** - Release object
3. **Detach Object** - Detach from gripper
4. **Retreat** - Move away using **Cartesian path**

## Example: Converting Existing Code

### Before (Monolithic):
```python
# 500 lines of code mixing MoveIt2 + application logic
class PickPlace(Node):
    # MoveIt2 setup...
    # ArUco detection...
    # All mixed together
```

### After (Modular):
```python
# Base: moveit2_controller_base.py (400 lines, reusable)
# Implementation: real_world_pick_place.py (150 lines, specific)

from moveit2_controller_base import MoveIt2Controller

class RealWorldPickPlace(MoveIt2Controller):
    # Only ArUco-specific code here
    # MoveIt2 handled by base class
```

## Troubleshooting

### Cartesian Path Not Available
If you see: `Cartesian path service not available`
- Check that MoveIt2 is running with compute_cartesian_path service
- System will automatically use standard planning as fallback

### Import Error
If you see: `ModuleNotFoundError: No module named 'moveit2_controller_base'`
- Ensure `moveit2_controller_base.py` is in your Python path
- Or place all files in the same directory

### Pick Fails at Approach
- Check collision objects are correctly placed
- Verify grasp pose is reachable
- Check logs for which step failed

## Files Overview

```
├── moveit2_controller_base.py      # Base class (400 lines)
├── simulation_pick_place.py        # Simulation version (100 lines)
├── real_world_pick_place.py        # Real-world version (200 lines)
└── README.md                       # This file
```

## Next Steps

1. **Test Base Functionality**: Run simulation version first
2. **Customize for Your Robot**: Adjust collision objects and poses
3. **Add Features**: Extend base class if needed
4. **Create New Versions**: Inherit for other use cases
