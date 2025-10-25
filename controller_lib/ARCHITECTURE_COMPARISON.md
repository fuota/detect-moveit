# Architecture Comparison: Before vs After

## BEFORE (Code Duplication)

```
┌────────────────────────────────────────────┐
│     simulation_pick_place.py               │
│  ┌──────────────────────────────────────┐  │
│  │ • euler_to_quaternion()              │  │
│  │ • get_current_pose()                 │  │
│  │ • move_to_pose()                     │  │
│  │ • move_cartesian()                   │  │
│  │ • execute_trajectory()               │  │
│  │ • move_relative_cartesian()          │  │
│  │ • control_gripper()                  │  │
│  │ • open_gripper()                     │  │
│  │ • close_gripper()                    │  │
│  │ • add_box_collision_object()         │  │
│  │ • add_cylinder_collision_object()    │  │
│  │ • attach_object_to_gripper()         │  │
│  │ • detach_object_from_gripper()       │  │
│  │ • pick_object()                      │  │
│  │ • place_object()                     │  │
│  │                                      │  │
│  │ PLUS: Manual cylinder setup          │  │
│  └──────────────────────────────────────┘  │
└────────────────────────────────────────────┘

┌────────────────────────────────────────────┐
│     real_detection_pick_place.py           │
│  ┌──────────────────────────────────────┐  │
│  │ • euler_to_quaternion()              │  │  ← DUPLICATED!
│  │ • get_current_pose()                 │  │  ← DUPLICATED!
│  │ • move_to_pose()                     │  │  ← DUPLICATED!
│  │ • move_cartesian()                   │  │  ← DUPLICATED!
│  │ • execute_trajectory()               │  │  ← DUPLICATED!
│  │ • move_relative_cartesian()          │  │  ← DUPLICATED!
│  │ • control_gripper()                  │  │  ← DUPLICATED!
│  │ • open_gripper()                     │  │  ← DUPLICATED!
│  │ • close_gripper()                    │  │  ← DUPLICATED!
│  │ • add_box_collision_object()         │  │  ← DUPLICATED!
│  │ • add_cylinder_collision_object()    │  │  ← DUPLICATED!
│  │ • attach_object_to_gripper()         │  │  ← DUPLICATED!
│  │ • detach_object_from_gripper()       │  │  ← DUPLICATED!
│  │ • pick_object()                      │  │  ← DUPLICATED!
│  │ • place_object()                     │  │  ← DUPLICATED!
│  │                                      │  │
│  │ PLUS: ArUco detection callbacks      │  │
│  └──────────────────────────────────────┘  │
└────────────────────────────────────────────┘

❌ Problems:
- 500+ lines duplicated across both files
- Bug fixes need to be applied twice
- Feature additions need to be coded twice
- Parameters must be updated in both places
- Easy to introduce inconsistencies
```

## AFTER (Clean Inheritance)

```
┌─────────────────────────────────────────────────────────┐
│              moveit_controller.py (Base Class)          │
│  ┌───────────────────────────────────────────────────┐  │
│  │ REUSABLE MOVEIT2 FUNCTIONALITY                    │  │
│  │                                                   │  │
│  │ Pose Utilities:                                  │  │
│  │  • get_current_pose()                            │  │
│  │  • compute_approach_pose()                       │  │
│  │  • set_grasp_orientation()                       │  │
│  │                                                   │  │
│  │ Motion Planning:                                  │  │
│  │  • move_to_pose()                                │  │
│  │  • move_cartesian()                              │  │
│  │  • execute_trajectory()                          │  │
│  │  • move_relative_cartesian()                     │  │
│  │                                                   │  │
│  │ Gripper Control:                                  │  │
│  │  • control_gripper()                             │  │
│  │  • open_gripper()                                │  │
│  │  • close_gripper()                               │  │
│  │                                                   │  │
│  │ Collision Management:                             │  │
│  │  • add_box_collision_object()                    │  │
│  │  • add_cylinder_collision_object()               │  │
│  │  • remove_collision_object()                     │  │
│  │                                                   │  │
│  │ Object Attachment:                                │  │
│  │  • attach_object_to_gripper()                    │  │
│  │  • detach_object_from_gripper()                  │  │
│  │                                                   │  │
│  │ High-Level Operations:                            │  │
│  │  • pick_object()                                 │  │
│  │  • place_object()                                │  │
│  └───────────────────────────────────────────────────┘  │
└─────────────────────────┬───────────────────────────────┘
                          │
                          │ INHERITS FROM
                          │
        ┌─────────────────┴─────────────────┐
        │                                   │
        ▼                                   ▼
┌──────────────────────┐         ┌─────────────────────────┐
│ simulation_pick_     │         │ real_detection_pick_    │
│ place.py             │         │ place.py                │
│                      │         │                         │
│ Only ~100 lines!     │         │ Only ~250 lines!        │
│                      │         │                         │
│ ┌──────────────────┐ │         │ ┌─────────────────────┐ │
│ │ Specific to      │ │         │ │ Specific to         │ │
│ │ Simulation:      │ │         │ │ Real Detection:     │ │
│ │                  │ │         │ │                     │ │
│ │ • setup_         │ │         │ │ • ArUco callbacks   │ │
│ │   environment()  │ │         │ │   - pose_callback   │ │
│ │ • run_pick_and_  │ │         │ │   - ids_callback    │ │
│ │   place_demo()   │ │         │ │   - names_callback  │ │
│ └──────────────────┘ │         │ │                     │ │
│                      │         │ │ • update_collision_ │ │
│ Uses inherited       │         │ │   cylinder()        │ │
│ methods:             │         │ │ • display_detected_ │ │
│                      │         │ │   objects()         │ │
│ self.pick_object()   │         │ │ • execute_pick_     │ │
│ self.place_object()  │         │ │   sequence()        │ │
│ self.add_box_...()   │         │ │ • setup_static_     │ │
│ self.add_cylinder..()│         │ │   environment()     │ │
└──────────────────────┘         │ │                     │ │
                                 │ │ Uses inherited      │ │
                                 │ │ methods:            │ │
                                 │ │                     │ │
                                 │ │ self.pick_object()  │ │
                                 │ │ self.add_box_...()  │ │
                                 │ │ self.add_cylinder..()│ │
                                 │ └─────────────────────┘ │
                                 └─────────────────────────┘

✅ Benefits:
- MoveIt2 logic in ONE place (500+ lines)
- Fix bugs ONCE, affects both versions
- Add features ONCE, both get it
- Update parameters ONCE
- Each child class is small and focused
- Easy to add new controllers (just inherit!)
```

## Code Size Comparison

### Before Refactoring:
```
simulation_pick_place.py:        ~650 lines
real_detection_pick_place.py:    ~750 lines
────────────────────────────────────────────
TOTAL:                           ~1400 lines
DUPLICATION:                     ~500 lines (36%)
```

### After Refactoring:
```
moveit_controller.py:            ~580 lines (shared base)
simulation_pick_place.py:        ~100 lines (specific)
real_detection_pick_place.py:    ~280 lines (specific)
────────────────────────────────────────────
TOTAL:                           ~960 lines
DUPLICATION:                     0 lines (0%)
SAVED:                           ~440 lines!
```

## Maintenance Example

### Scenario: Need to change gripper closing force

#### BEFORE:
```python
# ❌ Must change in BOTH files

# File 1: simulation_pick_place.py
def close_gripper(self, grip_force=0.8):  # ← Change here
    return self.control_gripper("close", grip_force)

# File 2: real_detection_pick_place.py  
def close_gripper(self, grip_force=0.8):  # ← AND here!
    return self.control_gripper("close", grip_force)
```

#### AFTER:
```python
# ✅ Change in ONE place

# File: moveit_controller.py
def close_gripper(self, grip_force=0.7):  # ← Change ONCE
    return self.control_gripper("close", grip_force)

# Both simulation and real detection automatically get the update!
```

## Adding New Feature Example

### Scenario: Add place_object_with_retreat() method

#### BEFORE:
```python
# ❌ Must implement in BOTH files
# Copy 50 lines of code twice!
# Test twice!
# Debug twice!
```

#### AFTER:
```python
# ✅ Implement ONCE in base class

# File: moveit_controller.py
def place_object_with_retreat(self, object_name, place_pose, retreat_distance=0.15):
    """New feature"""
    # Implementation here (50 lines)
    ...

# Both child classes inherit it immediately!
controller.place_object_with_retreat("my_object", pose)  # Works in both!
```

## Summary

| Aspect | Before | After |
|--------|--------|-------|
| **Code Duplication** | 36% | 0% |
| **Total Lines** | ~1400 | ~960 |
| **Files to Edit (bug fix)** | 2 | 1 |
| **Files to Edit (new feature)** | 2 | 1 |
| **Consistency** | Manual | Automatic |
| **Maintenance Effort** | High | Low |
| **Extensibility** | Hard | Easy |
| **Testing Effort** | 2x | 1x |
