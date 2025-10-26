#!/usr/bin/env python3
import yaml
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image
from std_msgs.msg import Float32MultiArray
from cv_bridge import CvBridge
import cv2


class OverlayVisualizer(Node):
    def __init__(self) -> None:
        super().__init__('overlay_visualizer')

        # Params
        self.declare_parameter('intrinsics_file', '')
        self.declare_parameter('extrinsics_file', '')
        self.declare_parameter('target_z_height_mm', 150.0)
        self.declare_parameter('overlay_topic', '/pin_housing/seg_overlay')
        self.declare_parameter('pixel_pose_topic', '/pin_housing/pixel_pose')

        intrinsics_file = self.get_parameter('intrinsics_file').get_parameter_value().string_value
        extrinsics_file = self.get_parameter('extrinsics_file').get_parameter_value().string_value
        self.target_z_m = float(self.get_parameter('target_z_height_mm').get_parameter_value().double_value) / 1000.0
        overlay_topic = self.get_parameter('overlay_topic').get_parameter_value().string_value
        pixel_topic = self.get_parameter('pixel_pose_topic').get_parameter_value().string_value

        # Load intrinsics/extrinsics
        with open(intrinsics_file, 'r') as f:
            intr = yaml.safe_load(f)
        self.K = np.array(intr['camera_matrix']['data']).reshape(3, 3)
        self.D = np.array(intr['distortion_coefficients']['data'])
        with open(extrinsics_file, 'r') as f:
            self.T_base_cam = np.array(yaml.safe_load(f)['camera_pose_in_base'])

        # State
        self._last_pixel_pose = None  # (u, v, yaw_deg)
        self._bridge = CvBridge()
        self._win = 'Segmentation Overlay + Poses'
        cv2.namedWindow(self._win)

        # Subs
        self.create_subscription(Image, overlay_topic, self._on_overlay, qos_profile_sensor_data)
        self.create_subscription(Float32MultiArray, pixel_topic, self._on_pixel_pose, 10)

    def _on_pixel_pose(self, msg: Float32MultiArray) -> None:
        if len(msg.data) >= 3:
            self._last_pixel_pose = (float(msg.data[0]), float(msg.data[1]), float(msg.data[2]))

    def _on_overlay(self, msg: Image) -> None:
        try:
            img = self._bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        except Exception:
            return

        # Show pixel/camera and TCP/base info if available
        if self._last_pixel_pose is not None:
            u, v, yaw_deg = self._last_pixel_pose

            # Draw pixel point
            cv2.circle(img, (int(u), int(v)), 5, (0, 0, 255), -1)
            cv2.putText(img, f"pix:({int(u)},{int(v)}) yaw={yaw_deg:.1f}deg", (10, 30),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 0, 255), 2)

            # Compute base-frame target using same math as pixel_to_tcp
            fx, fy = self.K[0, 0], self.K[1, 1]
            cx, cy = self.K[0, 2], self.K[1, 2]
            x_cam = (u - cx) / fx
            y_cam = (v - cy) / fy
            ray_dir_cam = np.array([x_cam, y_cam, 1.0])
            ray_dir_cam = ray_dir_cam / np.linalg.norm(ray_dir_cam)
            cam_pos_base = self.T_base_cam[:3, 3]
            R_base_cam = self.T_base_cam[:3, :3]
            ray_dir_base = R_base_cam @ ray_dir_cam
            denom = ray_dir_base[2]
            if abs(denom) > 1e-6:
                t = (self.target_z_m - cam_pos_base[2]) / denom
                target_point_base = cam_pos_base + t * ray_dir_base
                tx, ty, tz = target_point_base.tolist()
                cv2.putText(img, f"tcp_base: X:{tx:.3f} Y:{ty:.3f} Z:{tz:.3f}", (10, 60),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 255, 0), 2)

        cv2.imshow(self._win, img)
        key = cv2.waitKey(1) & 0xFF
        if key == ord('q'):
            cv2.destroyAllWindows()
            rclpy.shutdown()


def main(args=None) -> None:
    rclpy.init(args=args)
    node = OverlayVisualizer()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        rclpy.shutdown()


if __name__ == '__main__':
    main()


