# Pilz Industrial Motion Planner Integration

This document describes how to use the Pilz Industrial Motion Planner with UR robots through C++ code.

## Overview

The `hello_pilz.cpp` example demonstrates how to control a UR robot using the Pilz Industrial Motion Planner, which provides industrial-grade motion planning capabilities including:

- **PTP (Point-to-Point)**: Fast joint-space motion between points
- **LIN (Linear)**: Straight-line Cartesian motion
- **CIRC (Circular)**: Circular arc motion between waypoints

## Files Created

1. **`src/hello_pilz.cpp`**: C++ code that uses Pilz motion planner
2. **`launch/hello_pilz.launch.py`**: Launch file that integrates Pilz configuration
3. **Updated `CMakeLists.txt`**: Added build configuration for the new executable

## Prerequisites

- ROS2 Humble
- MoveIt2
- Pilz Industrial Motion Planner
- UR robot description packages
- The `ur_pilz_demo` package (for Pilz configuration)

## Building

```bash
cd /home/orin_nano/pilz_ws
colcon build --packages-select ros2_ur_moveit_examples
source install/setup.bash
```

## Usage

### Option 1: Run with the integrated launch file

```bash
# Launch the complete system with Pilz planner and C++ controller
ros2 launch ros2_ur_moveit_examples hello_pilz.launch.py

# With specific parameters
ros2 launch ros2_ur_moveit_examples hello_pilz.launch.py ur_type:=ur5e use_fake_hardware:=true launch_rviz:=true
```

### Option 2: Run components separately

```bash
# Terminal 1: Launch the Pilz MoveIt configuration
ros2 launch ur_pilz_demo ur_pilz.launch.py

# Terminal 2: Run the C++ controller
ros2 run ros2_ur_moveit_examples hello_pilz
```

## Code Features

The `hello_pilz.cpp` demonstrates:

1. **PTP Motion**: Point-to-point motion to a target pose
2. **LIN Motion**: Linear motion to a different target
3. **CIRC Motion**: Circular motion with waypoints

### Key Configuration

```cpp
// Set the planning pipeline to use Pilz
move_group_interface.setPlanningPipelineId("pilz_industrial_motion_planner");
move_group_interface.setPlannerId("PTP"); // or "LIN", "CIRC"
```

## Motion Types

### PTP (Point-to-Point)
- Fastest motion between points
- Joint-space planning
- Use for: Fast positioning, avoiding obstacles

### LIN (Linear)
- Straight-line Cartesian motion
- Maintains tool orientation
- Use for: Precise linear operations, welding, cutting

### CIRC (Circular)
- Circular arc motion
- Requires waypoint and target
- Use for: Smooth curved paths, avoiding obstacles

## Parameters

The launch file supports the following parameters:

- `ur_type`: Robot type (ur3, ur3e, ur5, ur5e, ur10, ur10e, ur16e)
- `use_fake_hardware`: Use simulation (true/false)
- `launch_rviz`: Launch RViz visualization (true/false)
- `use_sim_time`: Use simulation time (true/false)

## Troubleshooting

1. **Planning fails**: Check that the target pose is reachable and within workspace limits
2. **No motion**: Verify that the robot is properly connected and controllers are running
3. **Configuration errors**: Ensure all required packages are installed and sourced

## Integration with Existing Code

The Pilz integration is designed to work alongside existing MoveIt examples:

- Uses the same `MoveGroupInterface` API
- Compatible with existing UR robot configurations
- Can be extended with additional motion types
- Supports the same safety and workspace constraints

## Next Steps

- Modify target poses in the code for your specific application
- Add collision objects for more realistic planning
- Implement custom motion sequences using different Pilz planners
- Integrate with your specific robot hardware setup
