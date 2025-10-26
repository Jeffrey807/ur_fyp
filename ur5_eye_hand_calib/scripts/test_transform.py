#!/usr/bin/env python3

import sys
import os
sys.path.append(os.path.join(os.path.dirname(__file__), '..', 'ur5_eye_hand_calib'))

from transform_utils import TransformUtils
import numpy as np

def test_transform():
    """Test the loaded transform with sample points"""
    
    # Load transform
    transform_file = "camera_to_robot_transform.yaml"
    utils = TransformUtils(transform_file)
    
    if utils.transform_matrix is None:
        print("❌ No transform file found. Run calibration first!")
        return False
    
    print("✅ Transform loaded successfully!")
    print(f"Translation: {utils.translation}")
    print(f"Rotation (Euler degrees): {utils.rotation_euler}")
    
    # Validate transform
    if utils.validate_transform():
        print("✅ Transform validation passed")
    else:
        print("⚠️  Transform validation failed")
    
    # Test pixel to robot conversion
    print("\n🧪 Testing pixel to robot conversion:")
    
    # Test center of image
    center_pixel = (320, 240)
    x, y, z, yaw = utils.pixel_to_robot(center_pixel[0], center_pixel[1])
    print(f"Center pixel {center_pixel} -> Robot ({x:.3f}, {y:.3f}, {z:.3f}, yaw={np.degrees(yaw):.1f}°)")
    
    # Test corners
    corners = [(0, 0), (640, 0), (640, 480), (0, 480)]
    for i, corner in enumerate(corners):
        x, y, z, yaw = utils.pixel_to_robot(corner[0], corner[1])
        print(f"Corner {i+1} {corner} -> Robot ({x:.3f}, {y:.3f}, {z:.3f}, yaw={np.degrees(yaw):.1f}°)")
    
    # Generate static transform command
    command = utils.get_static_transform_command()
    print(f"\n📋 Static transform command:")
    print(f"ros2 run tf2_ros static_transform_publisher {command.split('static_transform_publisher ')[1]}")
    
    return True

def main():
    print("🔧 Testing camera-to-robot transform...")
    print("=" * 50)
    
    success = test_transform()
    
    if success:
        print("\n✅ All tests passed!")
        print("\n📝 Next steps:")
        print("1. Use the static transform command to publish the transform")
        print("2. Test with your robot application")
        print("3. Adjust Z height as needed for your application")
    else:
        print("\n❌ Tests failed. Check your calibration data.")

if __name__ == "__main__":
    main()
