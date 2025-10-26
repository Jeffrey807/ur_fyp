#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from geometry_msgs.msg import PoseStamped
from cv_bridge import CvBridge
import cv2
import numpy as np
import yaml
import json
from tf_transformations import quaternion_matrix, euler_from_matrix
from ament_index_python.packages import get_package_share_directory
import os

class FixedBoardCalibrator(Node):
    """
    Camera-to-Robot calibration using fixed checkerboard.
    Robot TCP touches known board intersections to establish transform.
    """
    
    def __init__(self):
        super().__init__('fixed_board_calibrator')
        
        # Declare parameters
        self.declare_parameters(
            namespace='',
            parameters=[
                ('image_topic', '/camera/color/image_raw'),
                ('pose_topic', '/tcp_pose_broadcaster/pose'),
                ('grid_size', 0.024),  # 24mm squares
                ('board_dims', [8, 6]),  # inner corners
                ('output_file', 'camera_to_robot_transform.yaml'),
                ('camera_info_file', 'camera_calibration.yaml'),
                ('min_samples', 6),
                ('max_samples', 20)
            ]
        )
        
        # Get parameters
        self.image_topic = self.get_parameter('image_topic').get_parameter_value().string_value
        self.pose_topic = self.get_parameter('pose_topic').get_parameter_value().string_value
        self.grid_size = self.get_parameter('grid_size').get_parameter_value().double_value
        self.board_dims = list(self.get_parameter('board_dims').get_parameter_value().integer_array_value)
        self.output_file = self.get_parameter('output_file').get_parameter_value().string_value
        self.camera_info_file = self.get_parameter('camera_info_file').get_parameter_value().string_value
        self.min_samples = self.get_parameter('min_samples').get_parameter_value().integer_value
        self.max_samples = self.get_parameter('max_samples').get_parameter_value().integer_value
        
        # Initialize
        self.bridge = CvBridge()
        self.latest_pose = None
        self.calibration_data = []
        self.camera_matrix = None
        self.dist_coeffs = None
        self.window_name = "Fixed Board Calibration"
        
        # Create subscriptions
        self.subscription_image = self.create_subscription(
            Image, self.image_topic, self.image_callback, 10)
        self.subscription_pose = self.create_subscription(
            PoseStamped, self.pose_topic, self.pose_callback, 10)
        
        # Load camera intrinsics
        self.load_camera_intrinsics()
        
        # Prepare 3D board coordinates
        self.objp = np.zeros((self.board_dims[0] * self.board_dims[1], 3), np.float32)
        self.objp[:, :2] = np.mgrid[0:self.board_dims[0], 0:self.board_dims[1]].T.reshape(-1, 2)
        self.objp *= self.grid_size
        
        self.get_logger().info("Fixed Board Calibrator ready!")
        self.get_logger().info("Instructions:")
        self.get_logger().info("1. Place checkerboard flat and visible to camera")
        self.get_logger().info("2. Move robot TCP to touch board intersections")
        self.get_logger().info("3. Press SPACE when TCP is at intersection")
        self.get_logger().info("4. Press C to compute transform (need ≥6 samples)")
        self.get_logger().info("5. Press Q to quit")
        
    def load_camera_intrinsics(self):
        """Load camera calibration parameters"""
        try:
            pkg_share = get_package_share_directory('ur5_eye_hand_calib')
            calib_path = os.path.join(pkg_share, 'calibration', self.camera_info_file)
            
            with open(calib_path, 'r') as f:
                calib_data = yaml.safe_load(f)
            
            self.camera_matrix = np.array(calib_data['camera_matrix']['data']).reshape(3, 3)
            self.dist_coeffs = np.array(calib_data['distortion_coefficients']['data'])
            
            self.get_logger().info(f"Loaded camera intrinsics from {calib_path}")
            
        except Exception as e:
            self.get_logger().error(f"Failed to load camera intrinsics: {e}")
            self.get_logger().error("Please run camera calibration first!")
            rclpy.shutdown()
    
    def pose_callback(self, msg):
        """Store latest robot pose"""
        self.latest_pose = msg
    
    def image_callback(self, msg):
        """Process camera image and handle user input"""
        if self.latest_pose is None:
            return
            
        # Convert image
        cv_img = self.bridge.imgmsg_to_cv2(msg, desired_encoding='rgb8')
        gray = cv2.cvtColor(cv_img, cv2.COLOR_RGB2GRAY)
        
        # Detect checkerboard
        ret, corners = cv2.findChessboardCorners(
            gray, (self.board_dims[0], self.board_dims[1]), None)
        
        if ret:
            # Refine corner detection
            criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 0.001)
            corners = cv2.cornerSubPix(gray, corners, (11, 11), (-1, -1), criteria)
            cv2.drawChessboardCorners(cv_img, (self.board_dims[0], self.board_dims[1]), corners, ret)
        
        # Display status
        status_text = f"Samples: {len(self.calibration_data)}/{self.max_samples}"
        cv2.putText(cv_img, status_text, (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2)
        
        instructions = "SPACE=capture  C=compute  Q=quit"
        cv2.putText(cv_img, instructions, (10, 70), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 0), 2)
        
        if ret:
            cv2.putText(cv_img, "BOARD DETECTED", (10, 110), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)
        else:
            cv2.putText(cv_img, "NO BOARD", (10, 110), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 0, 255), 2)
        
        cv2.imshow(self.window_name, cv_img)
        key = cv2.waitKey(1) & 0xFF
        
        # Handle key presses
        if key == ord('q') or key == ord('Q'):
            self.cleanup()
            rclpy.shutdown()
            return
            
        elif key == ord(' ') and ret and self.latest_pose is not None:
            self.capture_sample(corners)
            
        elif key == ord('c') or key == ord('C'):
            if len(self.calibration_data) >= self.min_samples:
                self.compute_transform()
            else:
                self.get_logger().warn(f"Need at least {self.min_samples} samples, have {len(self.calibration_data)}")
    
    def capture_sample(self, corners):
        """Capture a calibration sample"""
        if len(self.calibration_data) >= self.max_samples:
            self.get_logger().warn(f"Maximum samples ({self.max_samples}) reached")
            return
            
        # Get robot pose
        pose = self.latest_pose.pose
        position = [pose.position.x, pose.position.y, pose.position.z]
        orientation = [pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w]
        
        # Store sample
        sample = {
            'robot_position': position,
            'robot_orientation': orientation,
            'image_corners': corners.reshape(-1, 2).tolist(),
            'board_points': self.objp.tolist()
        }
        
        self.calibration_data.append(sample)
        self.get_logger().info(f"Captured sample {len(self.calibration_data)}: TCP at [{position[0]:.3f}, {position[1]:.3f}, {position[2]:.3f}]")
    
    def compute_transform(self):
        """Compute camera-to-robot transform"""
        self.get_logger().info("Computing camera-to-robot transform...")
        
        try:
            # Extract data
            robot_positions = []
            board_points_3d = []
            image_points = []
            
            for sample in self.calibration_data:
                robot_positions.append(sample['robot_position'])
                board_points_3d.append(np.array(sample['board_points']))
                image_points.append(np.array(sample['image_corners']))
            
            # Compute camera-to-board transform for each sample
            camera_to_board_transforms = []
            
            for i in range(len(board_points_3d)):
                success, rvec, tvec = cv2.solvePnP(
                    board_points_3d[i], image_points[i], 
                    self.camera_matrix, self.dist_coeffs)
                
                if success:
                    R_cam_board = cv2.Rodrigues(rvec)[0]
                    t_cam_board = tvec.flatten()
                    
                    # Create 4x4 transform matrix
                    T_cam_board = np.eye(4)
                    T_cam_board[:3, :3] = R_cam_board
                    T_cam_board[:3, 3] = t_cam_board
                    
                    camera_to_board_transforms.append(T_cam_board)
            
            # Compute robot-to-board transform (assuming robot base frame)
            robot_to_board_transforms = []
            
            for pos in robot_positions:
                # Create 4x4 transform matrix for robot pose
                # Assuming robot is pointing straight down (Z-axis down)
                T_robot_board = np.eye(4)
                T_robot_board[:3, 3] = pos  # Translation
                # Keep identity rotation (robot pointing down)
                robot_to_board_transforms.append(T_robot_board)
            
            # Solve for camera-to-robot transform using hand-eye calibration
            if len(camera_to_board_transforms) >= 2:
                # Use OpenCV's hand-eye calibration
                R_cam_robot, t_cam_robot = cv2.calibrateHandEye(
                    [t[:3, :3] for t in robot_to_board_transforms],
                    [t[:3, 3] for t in robot_to_board_transforms],
                    [t[:3, :3] for t in camera_to_board_transforms],
                    [t[:3, 3] for t in camera_to_board_transforms]
                )
                
                # Create final transform matrix
                T_cam_robot = np.eye(4)
                T_cam_robot[:3, :3] = R_cam_robot
                T_cam_robot[:3, 3] = t_cam_robot.flatten()
                
                # Save transform
                self.save_transform(T_cam_robot)
                
                self.get_logger().info("Calibration completed successfully!")
                self.get_logger().info(f"Camera-to-Robot transform:\n{T_cam_robot}")
                
            else:
                self.get_logger().error("Need at least 2 samples for hand-eye calibration")
                
        except Exception as e:
            self.get_logger().error(f"Calibration failed: {e}")
    
    def save_transform(self, transform_matrix):
        """Save the computed transform to file"""
        try:
            # Convert to quaternion for easier use
            R = transform_matrix[:3, :3]
            t = transform_matrix[:3, 3]
            
            # Convert rotation matrix to quaternion
            from scipy.spatial.transform import Rotation
            r = Rotation.from_matrix(R)
            quat = r.as_quat()  # [x, y, z, w]
            
            # Save transform data
            transform_data = {
                'transform_matrix': transform_matrix.tolist(),
                'translation': t.tolist(),
                'rotation_quaternion': quat.tolist(),
                'rotation_euler': r.as_euler('xyz', degrees=True).tolist(),
                'calibration_samples': len(self.calibration_data),
                'board_dimensions': self.board_dims,
                'grid_size': self.grid_size
            }
            
            with open(self.output_file, 'w') as f:
                yaml.dump(transform_data, f, default_flow_style=False)
            
            self.get_logger().info(f"Transform saved to {self.output_file}")
            
        except Exception as e:
            self.get_logger().error(f"Failed to save transform: {e}")
    
    def cleanup(self):
        """Clean up resources"""
        cv2.destroyAllWindows()

def main(args=None):
    rclpy.init(args=args)
    node = FixedBoardCalibrator()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.cleanup()
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
