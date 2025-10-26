import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from geometry_msgs.msg import PoseStamped
from cv_bridge import CvBridge
import cv2
import numpy as np
import yaml
from tf_transformations import quaternion_matrix
from ament_index_python.packages import get_package_share_directory

class EyeHandCalibrator(Node):
    def __init__(self):
        super().__init__('eye_hand_calibrator')
        self.declare_parameters(
            namespace='',
            parameters=[
                ('image_topic', '/camera/color/image_raw'),
                ('pose_topic', '/tcp_pose_broadcaster/pose'),
                ('grid_size', 0.026),
                ('board_dims', [9, 6]),
                ('output_transform', 'camera_in_base.yaml')
            ]
        )
        self.image_topic = self.get_parameter('image_topic').get_parameter_value().string_value
        self.pose_topic = self.get_parameter('pose_topic').get_parameter_value().string_value
        self.grid_size = self.get_parameter('grid_size').get_parameter_value().double_value
        self.board_dims = list(self.get_parameter('board_dims').get_parameter_value().integer_array_value)
        self.output_file = self.get_parameter('output_transform').get_parameter_value().string_value

        self.bridge = CvBridge()
        self.latest_pose = None
        self.obj_points = []  # real 3D checker points
        self.img_points = []  # detected corners
        self.robot_R, self.robot_t = [], []
        self.window_name = "Checkerboard"

        self.subscription_image = self.create_subscription(Image, self.image_topic, self.image_callback, 10)
        self.subscription_pose = self.create_subscription(PoseStamped, self.pose_topic, self.pose_callback, 10)
        self.get_logger().info(f"Listening to {self.image_topic} and {self.pose_topic}")

        # Prepare 3D checkerboard coordinates
        self.objp = np.zeros((self.board_dims[0]*self.board_dims[1], 3), np.float32)
        self.objp[:, :2] = np.mgrid[0:self.board_dims[0], 0:self.board_dims[1]].T.reshape(-1, 2)
        self.objp *= self.grid_size

        # Load intrinsics from installed package share directory
        pkg_share = get_package_share_directory('ur5_eye_hand_calib')
        calib_yaml_path = f"{pkg_share}/calibration/camera_calibration.yaml"
        with open(calib_yaml_path, 'r') as f:
            intr = yaml.safe_load(f)
        self.K = np.array(intr['camera_matrix']['data']).reshape(3, 3)
        self.D = np.array(intr['distortion_coefficients']['data'])

        self.get_logger().info("Calibrator ready. Move robot and show checkerboard to camera.")

    def pose_callback(self, msg):
        self.latest_pose = msg

    def image_callback(self, msg):
        cv_img = self.bridge.imgmsg_to_cv2(msg, desired_encoding='rgb8')
        gray = cv2.cvtColor(cv_img, cv2.COLOR_RGB2GRAY)
        ret, corners = cv2.findChessboardCorners(gray, (self.board_dims[0], self.board_dims[1]), None)

        if ret:
            cv2.drawChessboardCorners(cv_img, (self.board_dims[0], self.board_dims[1]), corners, ret)

        # HUD instructions
        hint = "SPACE=capture  C=compute  Q=quit"
        cv2.putText(cv_img, hint, (12, 28), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2, cv2.LINE_AA)
        cv2.putText(cv_img, f"samples: {len(self.img_points)}", (12, 56), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 255), 2, cv2.LINE_AA)

        cv2.imshow(self.window_name, cv_img)
        key = cv2.waitKey(1) & 0xFF

        if key == ord('q') or key == ord('Q'):
            cv2.destroyAllWindows()
            rclpy.shutdown()
            return

        if key == ord(' ') and ret and self.latest_pose is not None:
            self.img_points.append(corners)
            self.obj_points.append(self.objp)
            q = self.latest_pose.pose.orientation
            p = self.latest_pose.pose.position
            Rg = quaternion_matrix([q.x, q.y, q.z, q.w])[:3, :3]
            tg = np.array([[p.x], [p.y], [p.z]])
            self.robot_R.append(Rg)
            self.robot_t.append(tg)
            self.get_logger().info(f"Captured sample {len(self.img_points)}")

        if key == ord('c') or key == ord('C'):
            if len(self.img_points) >= 10:
                self.compute_extrinsic()
            else:
                self.get_logger().warn("Not enough samples to compute extrinsics. Capture at least 10.")

    def compute_extrinsic(self):
        self.get_logger().info("Computing extrinsics for eye-to-hand setup...")
        
        # Get checkerboard poses in the camera frame
        R_target2cam, t_target2cam = [], []
        for i in range(len(self.img_points)):
            ret, rvec, tvec = cv2.solvePnP(self.obj_points[i], self.img_points[i], self.K, self.D)
            R_target2cam.append(cv2.Rodrigues(rvec)[0])
            t_target2cam.append(tvec)

        # Step 1: Solve for the gripper-to-target transform (X in AX=ZB)
        # The output of this function is the pose of the target (checkerboard) in the gripper (TCP) frame.
        R_gripper2target, t_gripper2target = cv2.calibrateHandEye(
            R_gripper2base=self.robot_R,
            t_gripper2base=self.robot_t,
            R_target2cam=R_target2cam,
            t_target2cam=t_target2cam,
            method=cv2.CALIB_HAND_EYE_PARK) # PARK is a common and robust method

        # Step 2: Calculate the camera-in-base transform (Z in AX=ZB)
        # We use the first recorded pose as a reference to solve for Z.
        # Z = A * X * B^-1
        
        # A: First base-to-gripper transform
        T_base2gripper = np.eye(4)
        T_base2gripper[:3, :3] = self.robot_R[0]
        T_base2gripper[:3, 3] = self.robot_t[0].flatten()
        
        # X: gripper-to-target transform (from step 1)
        T_gripper2target = np.eye(4)
        T_gripper2target[:3, :3] = R_gripper2target
        T_gripper2target[:3, 3] = t_gripper2target.flatten()
        
        # B: First camera-to-target transform
        T_cam2target = np.eye(4)
        T_cam2target[:3, :3] = R_target2cam[0]
        T_cam2target[:3, 3] = t_target2cam[0].flatten()

        # B^-1: Inverse of camera-to-target is target-to-camera
        T_target2cam = np.linalg.inv(T_cam2target)

        # Final calculation: T_cam_in_base = T_base2gripper * T_gripper2target * T_target2cam
        T_cam_in_base = T_base2gripper @ T_gripper2target @ T_target2cam

        self.get_logger().info(f"\nCamera in base frame (T_cam_in_base):\n{T_cam_in_base}")
        with open(self.output_file, 'w') as f:
            yaml.dump({'camera_pose_in_base': T_cam_in_base.tolist()}, f)
        self.get_logger().info(f"Saved result to {self.output_file}")
        cv2.destroyAllWindows()
        rclpy.shutdown()

def main(args=None):
    rclpy.init(args=args)
    node = EyeHandCalibrator()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()