#!/usr/bin/env python3
import math
import yaml
import numpy as np

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray
from geometry_msgs.msg import PoseStamped, Quaternion
from tf_transformations import quaternion_from_euler


class PixelToTCPNode(Node):
    def __init__(self) -> None:
        super().__init__('pixel_to_tcp_node')

        # Parameters
        self.declare_parameter('pixel_pose_topic', '/pin_housing/pixel_pose')
        self.declare_parameter('target_pose_topic', '/pin_housing/target_pose')
        self.declare_parameter('intrinsics_file', '')
        self.declare_parameter('extrinsics_file', '')
        self.declare_parameter('target_z_height_mm', 150.0)
        self.declare_parameter('base_frame', 'base_link')

        pixel_topic = self.get_parameter('pixel_pose_topic').get_parameter_value().string_value
        target_topic = self.get_parameter('target_pose_topic').get_parameter_value().string_value
        intrinsics_file = self.get_parameter('intrinsics_file').get_parameter_value().string_value
        extrinsics_file = self.get_parameter('extrinsics_file').get_parameter_value().string_value
        self.target_z_m = float(self.get_parameter('target_z_height_mm').get_parameter_value().double_value) / 1000.0
        self.base_frame = self.get_parameter('base_frame').get_parameter_value().string_value

        # Load intrinsics
        if not intrinsics_file:
            self.get_logger().fatal('intrinsics_file parameter is empty')
            raise RuntimeError('Missing intrinsics_file')
        try:
            with open(intrinsics_file, 'r') as f:
                intr = yaml.safe_load(f)
            self.K = np.array(intr['camera_matrix']['data']).reshape(3, 3)
            self.D = np.array(intr['distortion_coefficients']['data'])
        except Exception as e:
            self.get_logger().fatal(f'Failed to load intrinsics: {e}')
            raise

        # Load extrinsics (T_base_cam)
        if not extrinsics_file:
            self.get_logger().fatal('extrinsics_file parameter is empty')
            raise RuntimeError('Missing extrinsics_file')
        try:
            with open(extrinsics_file, 'r') as f:
                self.T_base_cam = np.array(yaml.safe_load(f)['camera_pose_in_base'])
        except Exception as e:
            self.get_logger().fatal(f'Failed to load extrinsics: {e}')
            raise

        # Sub/Pub
        self.sub = self.create_subscription(Float32MultiArray, pixel_topic, self._on_pixel_pose, 10)
        self.pub = self.create_publisher(PoseStamped, target_topic, 10)

        self.get_logger().info(f'Listening: {pixel_topic}  -> Publishing: {target_topic}')

    def _on_pixel_pose(self, msg: Float32MultiArray) -> None:
        try:
            if len(msg.data) < 3:
                return
            u = float(msg.data[0])
            v = float(msg.data[1])
            # Ignore incoming yaw; we fix tool orientation to face down

            # Unproject pixel to camera ray
            fx, fy = self.K[0, 0], self.K[1, 1]
            cx, cy = self.K[0, 2], self.K[1, 2]
            x_cam = (u - cx) / fx
            y_cam = (v - cy) / fy
            ray_dir_cam = np.array([x_cam, y_cam, 1.0])
            ray_dir_cam = ray_dir_cam / np.linalg.norm(ray_dir_cam)

            cam_pos_base = self.T_base_cam[:3, 3]
            R_base_cam = self.T_base_cam[:3, :3]
            ray_dir_base = R_base_cam @ ray_dir_cam

            # Intersect with plane Z = target_z
            denom = ray_dir_base[2]
            if abs(denom) < 1e-6:
                self.get_logger().warn('Ray nearly parallel to target Z plane; skipping')
                return
            t = (self.target_z_m - cam_pos_base[2]) / denom
            target_point_base = cam_pos_base + t * ray_dir_base

            # Fixed tool orientation: rx=2.288, ry=2.151, rz=0
            # quaternion_from_euler expects (roll, pitch, yaw) in radians
            roll = 2.151   # ry (rotation around X-axis)
            pitch = 2.288  # rx (rotation around Y-axis) 
            yaw = 0.0      # rz (rotation around Z-axis)
            qx, qy, qz, qw = quaternion_from_euler(roll, pitch, yaw)

            pose = PoseStamped()
            pose.header.stamp = self.get_clock().now().to_msg()
            pose.header.frame_id = self.base_frame
            pose.pose.position.x = float(target_point_base[0])
            pose.pose.position.y = float(target_point_base[1])
            pose.pose.position.z = float(target_point_base[2])
            pose.pose.orientation = Quaternion(x=float(qx), y=float(qy), z=float(qz), w=float(qw))

            self.pub.publish(pose)
        except Exception as e:
            self.get_logger().error(f'pixel->tcp conversion failed: {e}')


def main(args=None) -> None:
    rclpy.init(args=args)
    node = PixelToTCPNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()


