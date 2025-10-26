#!/usr/bin/env python3

import numpy as np
import yaml
import os
from ament_index_python.packages import get_package_share_directory

class TransformUtils:
    """Utilities for camera-to-robot transform operations"""
    
    def __init__(self, transform_file=None):
        self.transform_matrix = None
        self.translation = None
        self.rotation_quaternion = None
        self.rotation_euler = None
        
        if transform_file:
            self.load_transform(transform_file)
    
    def load_transform(self, transform_file):
        """Load transform from YAML file"""
        try:
            with open(transform_file, 'r') as f:
                data = yaml.safe_load(f)
            
            self.transform_matrix = np.array(data['transform_matrix'])
            self.translation = np.array(data['translation'])
            self.rotation_quaternion = np.array(data['rotation_quaternion'])
            self.rotation_euler = np.array(data['rotation_euler'])
            
            return True
        except Exception as e:
            print(f"Failed to load transform: {e}")
            return False
    
    def pixel_to_robot(self, pixel_x, pixel_y, depth=None):
        """
        Convert pixel coordinates to robot base frame coordinates.
        
        Args:
            pixel_x, pixel_y: Pixel coordinates
            depth: Optional depth value (if None, uses fixed Z)
            
        Returns:
            (x, y, z, yaw) in robot base frame
        """
        if self.transform_matrix is None:
            raise ValueError("No transform loaded")
        
        # For 2D picking, we typically use a fixed Z height
        if depth is None:
            depth = 0.0  # Adjust based on your application
        
        # Convert pixel to camera frame (assuming pinhole camera)
        # This is a simplified version - you might need proper camera intrinsics
        # for accurate 3D reconstruction
        
        # For now, return the transform applied to a point at the given depth
        point_cam = np.array([pixel_x, pixel_y, depth, 1.0])
        point_robot = self.transform_matrix @ point_cam
        
        # Extract position and yaw
        x, y, z = point_robot[:3]
        yaw = np.arctan2(self.transform_matrix[1, 0], self.transform_matrix[0, 0])
        
        return x, y, z, yaw
    
    def get_static_transform_command(self):
        """Generate static_transform_publisher command"""
        if self.transform_matrix is None:
            return None
        
        # Extract translation
        x, y, z = self.translation
        
        # Convert quaternion to ROS format (w, x, y, z)
        qx, qy, qz, qw = self.rotation_quaternion
        
        command = (
            f"ros2 run tf2_ros static_transform_publisher "
            f"{x:.6f} {y:.6f} {z:.6f} "
            f"{qx:.6f} {qy:.6f} {qz:.6f} {qw:.6f} "
            f"base_link camera_link"
        )
        
        return command
    
    def validate_transform(self, test_points=None):
        """Validate the transform with test points"""
        if self.transform_matrix is None:
            return False
        
        # Basic validation checks
        det = np.linalg.det(self.transform_matrix[:3, :3])
        if abs(det - 1.0) > 0.01:
            print(f"Warning: Determinant is {det:.3f}, should be close to 1.0")
            return False
        
        # Check if rotation matrix is orthogonal
        R = self.transform_matrix[:3, :3]
        should_be_identity = R @ R.T
        if not np.allclose(should_be_identity, np.eye(3), atol=0.01):
            print("Warning: Rotation matrix is not orthogonal")
            return False
        
        return True

def create_launch_file():
    """Create a launch file for the static transform publisher"""
    launch_content = '''<?xml version="1.0"?>
<launch>
    <!-- Static transform publisher for camera to robot base -->
    <node pkg="tf2_ros" exec="static_transform_publisher" name="camera_to_robot_tf" 
          args="0.0 0.0 0.0 0.0 0.0 0.0 1.0 base_link camera_link" />
</launch>'''
    
    return launch_content

def main():
    """Example usage"""
    # Load transform
    transform_file = "camera_to_robot_transform.yaml"
    utils = TransformUtils(transform_file)
    
    if utils.transform_matrix is not None:
        print("Transform loaded successfully!")
        print(f"Translation: {utils.translation}")
        print(f"Rotation (Euler): {utils.rotation_euler}")
        
        # Test pixel to robot conversion
        test_pixel = (320, 240)  # Center of image
        x, y, z, yaw = utils.pixel_to_robot(test_pixel[0], test_pixel[1])
        print(f"Pixel {test_pixel} -> Robot ({x:.3f}, {y:.3f}, {z:.3f}, yaw={yaw:.3f})")
        
        # Generate static transform command
        command = utils.get_static_transform_command()
        print(f"Static transform command: {command}")
        
    else:
        print("Failed to load transform")

if __name__ == "__main__":
    main()
