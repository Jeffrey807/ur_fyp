#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import numpy as np
import yaml
import math

class VisualTesterNode(Node):
    def __init__(self):
        super().__init__('visual_tester_node')
        
        # --- Parameters ---
        self.declare_parameter('image_topic', '/camera/color/image_raw')
        self.declare_parameter('intrinsics_file', 'camera_calibration.yaml')
        self.declare_parameter('extrinsics_file', 'final_base_to_camera_transform.yaml')
        self.declare_parameter('target_z_height_mm', 150.0) # The Z height you specified, in millimeters

        self.image_topic = self.get_parameter('image_topic').get_parameter_value().string_value
        intrinsics_file = self.get_parameter('intrinsics_file').get_parameter_value().string_value
        extrinsics_file = self.get_parameter('extrinsics_file').get_parameter_value().string_value
        # Convert the Z height from millimeters to meters for calculation
        self.target_z_meters = self.get_parameter('target_z_height_mm').get_parameter_value().double_value / 1000.0
        
        # --- Load Calibration Data ---
        try:
            with open(intrinsics_file, 'r') as f:
                intr = yaml.safe_load(f)
                self.K = np.array(intr['camera_matrix']['data']).reshape(3, 3)
                self.D = np.array(intr['distortion_coefficients']['data'])
            self.get_logger().info(f"Successfully loaded intrinsics from '{intrinsics_file}'")
        except FileNotFoundError:
            self.get_logger().error(f"FATAL: Intrinsics file not found at: '{intrinsics_file}'")
            rclpy.shutdown()
            return

        try:
            with open(extrinsics_file, 'r') as f:
                self.T_base_cam = np.array(yaml.safe_load(f)['camera_pose_in_base'])
            self.get_logger().info(f"Successfully loaded extrinsics from '{extrinsics_file}'")
        except FileNotFoundError:
            self.get_logger().error(f"FATAL: Extrinsics file not found at: '{extrinsics_file}'")
            rclpy.shutdown()
            return
            
        # --- ROS & OpenCV Setup ---
        self.bridge = CvBridge()
        self.subscription = self.create_subscription(Image, self.image_topic, self.image_callback, 10)
        self.window_name = "Click to Verify Pose"
        self.latest_image = None
        cv2.namedWindow(self.window_name)
        cv2.setMouseCallback(self.window_name, self.mouse_callback)

        self.get_logger().info("Node started. Click on the camera view to calculate a pose.")
        self.get_logger().info("The pose will be printed to the console.")

    def image_callback(self, msg):
        cv_img_raw = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        # Undistort the image so clicks are on a geometrically correct view
        self.latest_image = cv2.undistort(cv_img_raw, self.K, self.D)
        
        cv2.putText(self.latest_image, "Click on a point to calculate its pose.", (10, 30), 
                    cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)
        cv2.imshow(self.window_name, self.latest_image)
        key = cv2.waitKey(1) & 0xFF
        if key == ord('q'):
            cv2.destroyAllWindows()
            rclpy.shutdown()

    def mouse_callback(self, event, x, y, flags, param):
        if event == cv2.EVENT_LBUTTONDOWN and self.latest_image is not None:
            self.calculate_and_display_pose((x, y))
            
    def calculate_and_display_pose(self, pixel):
        u, v = pixel
        self.get_logger().info(f"\n--- Click Detected at Pixel: ({u}, {v}) ---")

        # --- Unproject pixel to 3D point ---
        fx, fy = self.K[0, 0], self.K[1, 1]
        cx, cy = self.K[0, 2], self.K[1, 2]

        x_cam = (u - cx) / fx
        y_cam = (v - cy) / fy
        ray_dir_cam = np.array([x_cam, y_cam, 1.0])

        cam_pos_base = self.T_base_cam[:3, 3]
        R_base_cam = self.T_base_cam[:3, :3]
        ray_dir_base = R_base_cam @ ray_dir_cam

        # Calculate intersection with the Z-plane
        t = (self.target_z_meters - cam_pos_base[2]) / ray_dir_base[2]
        target_point_base = cam_pos_base + t * ray_dir_base

        # --- Define Orientation ---
        # To point down, we need a 180-degree roll
        roll_deg, pitch_deg, yaw_deg = 180.0, 0.0, 0.0

        # --- Display for Teach Pendant ---
        self.get_logger().info("Enter the following pose into the teach pendant:")
        self.get_logger().info(f"  Position (mm):")
        self.get_logger().info(f"    X: {target_point_base[0] * 1000.0:.2f}")
        self.get_logger().info(f"    Y: {target_point_base[1] * 1000.0:.2f}")
        self.get_logger().info(f"    Z: {target_point_base[2] * 1000.0:.2f}")
        self.get_logger().info(f"  Orientation (degrees):")
        self.get_logger().info(f"    Roll (Rx): {roll_deg:.2f}")
        self.get_logger().info(f"    Pitch (Ry): {pitch_deg:.2f}")
        self.get_logger().info(f"    Yaw (Rz): {yaw_deg:.2f}")
        self.get_logger().info("-------------------------------------------------")

        # --- Visual Feedback on Image ---
        feedback_image = self.latest_image.copy()
        cv2.circle(feedback_image, (u, v), 7, (0, 0, 255), -1)
        cv2.line(feedback_image, (u-15, v), (u+15, v), (0, 0, 255), 2)
        cv2.line(feedback_image, (u, v-15), (u, v+15), (0, 0, 255), 2)
        
        pos_text = f"X:{target_point_base[0]*1000:.1f} Y:{target_point_base[1]*1000:.1f}"
        cv2.putText(feedback_image, pos_text, (u + 10, v - 10), 
                    cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 0, 255), 2)
        cv2.imshow(self.window_name, feedback_image)

def main(args=None):
    rclpy.init(args=args)
    node = VisualTesterNode()
    rclpy.spin(node)
    node.destroy_node()

if __name__ == '__main__':
    main()