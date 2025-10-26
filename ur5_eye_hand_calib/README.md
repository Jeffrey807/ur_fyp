# UR5 Eye-to-Hand Calibration Package

This package provides tools for camera-to-robot calibration using a fixed checkerboard approach, optimized for 2D picking applications (X, Y, yaw only).

## Features

- **Fixed Board Calibration**: Place checkerboard on ground/table, move robot TCP to touch intersections
- **2D Picking Optimized**: Focus on X, Y, yaw coordinates for pick-and-place applications
- **Manual Control**: SPACE to capture, C to compute, Q to quit
- **Transform Utilities**: Convert pixel coordinates to robot base frame
- **Static Transform**: Generate tf2 static transform publisher commands

## Quick Start

### 1. Camera Calibration (First Time Only)
```bash
# Install camera calibration tools
sudo apt install ros-humble-camera-calibration

# Run camera calibration
ros2 run camera_calibration cameracalibrator \
  --size 9x6 --square 0.026 \
  --ros-args -r image:=/camera/color/image_raw -r camera:=/camera
```

### 2. Build Package
```bash
cd /home/orin_nano/calib_ws
colcon build --packages-select ur5_eye_hand_calib
source install/setup.bash
```

### 3. Run Fixed Board Calibration
```bash
# Method 1: Direct launch
ros2 launch ur5_eye_hand_calib fixed_board_calibration.launch.py

# Method 2: Manual execution
ros2 run ur5_eye_hand_calib fixed_board_calibrator \
  --ros-args --params-file src/ur5_eye_hand_calib/ur5_eye_hand_calib/config/fixed_board_params.yaml
```

### 4. Calibration Procedure

1. **Place checkerboard**: Flat on ground/table, fully visible to camera
2. **Move robot TCP**: Touch known board intersections (corners of squares)
3. **Capture samples**: Press SPACE when TCP is at intersection
4. **Compute transform**: Press C when you have ≥6 samples
5. **Save result**: Transform saved to `camera_to_robot_transform.yaml`

### 5. Test Transform
```bash
# Test the computed transform
python3 src/ur5_eye_hand_calib/scripts/test_transform.py
```

### 6. Use Static Transform
```bash
# Get the static transform command from test output
ros2 run tf2_ros static_transform_publisher <x> <y> <z> <qx> <qy> <qz> <qw> base_link camera_link
```

## Configuration

Edit `config/fixed_board_params.yaml`:

```yaml
ur5_eye_hand_calib:
  ros__parameters:
    image_topic: "/camera/color/image_raw"
    pose_topic: "/tcp_pose_broadcaster/pose"
    grid_size: 0.026  # 26mm squares
    board_dims: [9, 6]  # inner corners
    min_samples: 6
    max_samples: 20
```

## Usage in Your Application

```python
from ur5_eye_hand_calib.transform_utils import TransformUtils

# Load transform
utils = TransformUtils("camera_to_robot_transform.yaml")

# Convert pixel to robot coordinates
x, y, z, yaw = utils.pixel_to_robot(pixel_x, pixel_y)
```

## Tips for Good Results

- **Robot TCP**: Ensure TCP physically touches the exact board intersections
- **Robot Orientation**: Keep end-effector vertical (roll=pitch=0) for consistent Z
- **Frame Consistency**: Use same base frame throughout (base vs base_link)
- **Sampling**: Use 6-12 well-distributed points across the board
- **Lighting**: Ensure good lighting and focus for accurate corner detection

## Troubleshooting

- **No board detected**: Check lighting, focus, and board dimensions
- **Poor calibration**: Ensure varied robot orientations, not just translations
- **Wrong results**: Verify frame consistency and TCP positioning accuracy
- **Import errors**: Install missing dependencies: `sudo apt install python3-scipy`

## Files Structure

```
src/ur5_eye_hand_calib/
├── ur5_eye_hand_calib/
│   ├── fixed_board_calibrator.py    # Main calibration node
│   ├── transform_utils.py           # Transform utilities
│   ├── calibration_node.py          # Original hand-eye calibrator
│   └── config/
│       └── fixed_board_params.yaml  # Parameters
├── launch/
│   └── fixed_board_calibration.launch.py
├── scripts/
│   └── test_transform.py            # Test utilities
└── README.md
```
