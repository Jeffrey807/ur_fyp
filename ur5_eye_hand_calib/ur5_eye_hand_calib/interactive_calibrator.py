import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from geometry_msgs.msg import PoseStamped
from cv_bridge import CvBridge
import cv2
import numpy as np
import yaml
from ament_index_python.packages import get_package_share_directory

class InteractiveCalibrator(Node):
    def __init__(self):
        super().__init__('interactive_calibrator')
        
        # --- Parameters ---
        self.declare_parameter('image_topic', '/camera/color/image_raw')
        self.declare_parameter('pose_topic', '/tcp_pose_broadcaster/pose')
        self.declare_parameter('board_dims', [8, 6])
        self.declare_parameter('grid_size', 0.024)
        
        self.image_topic = self.get_parameter('image_topic').get_parameter_value().string_value
        self.pose_topic = self.get_parameter('pose_topic').get_parameter_value().string_value
        self.board_dims = self.get_parameter('board_dims').get_parameter_value().integer_array_value
        self.grid_size = self.get_parameter('grid_size').get_parameter_value().double_value

        # --- State Machine ---
        self.state = 0 # 0: board, 1: origin, 2: x-axis, 3: y-axis, 4: done
        self.latest_pose = None
        
        # --- Data Storage ---
        self.T_cam_board = None
        self.origin_coords = None
        self.x_axis_coords = None
        self.y_axis_coords = None

        # --- Load Intrinsics ---
        pkg_share = get_package_share_directory('ur5_eye_hand_calib')
        calib_yaml_path = f"{pkg_share}/calibration/camera_calibration.yaml"
        with open(calib_yaml_path, 'r') as f:
            intr = yaml.safe_load(f)
        self.K = np.array(intr['camera_matrix']['data']).reshape(3, 3)
        self.D = np.array(intr['distortion_coefficients']['data'])
        self.get_logger().info("Successfully loaded camera intrinsics.")

        # --- Setup ---
        self.bridge = CvBridge()
        self.objp = np.zeros((self.board_dims[0] * self.board_dims[1], 3), np.float32)
        self.objp[:, :2] = np.mgrid[0:self.board_dims[0], 0:self.board_dims[1]].T.reshape(-1, 2)
        self.objp *= self.grid_size
        
        self.subscription_img = self.create_subscription(Image, self.image_topic, self.image_callback, 10)
        self.subscription_pose = self.create_subscription(PoseStamped, self.pose_topic, self.pose_callback, 10)
        self.get_logger().info("Node started. Follow the instructions on the screen.")

    def pose_callback(self, msg):
        self.latest_pose = msg

    def image_callback(self, msg):
        cv_img = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        ret, corners = cv2.findChessboardCorners(cv_img, tuple(self.board_dims), None)
        if ret:
            cv2.drawChessboardCorners(cv_img, tuple(self.board_dims), corners, ret)

        # --- Display Instructions based on State ---
        self.display_instructions(cv_img)
        cv2.imshow("Interactive Calibration", cv_img)
        key = cv2.waitKey(1) & 0xFF

        if key == ord('q'):
            cv2.destroyAllWindows()
            rclpy.shutdown()

        if key == ord(' '):
            self.process_state(ret, corners)
            
    def process_state(self, board_visible, corners):
        if self.state == 0: # Capture board pose
            if not board_visible:
                self.get_logger().warn("Board not visible! Cannot capture.")
                return
            ret, rvec, tvec = cv2.solvePnP(self.objp, corners, self.K, self.D)
            R_cam_board, _ = cv2.Rodrigues(rvec)
            self.T_cam_board = np.eye(4)
            self.T_cam_board[:3, :3] = R_cam_board
            self.T_cam_board[:3, 3] = tvec.flatten()
            self.get_logger().info("Step 1/4: Camera-to-Board transform captured.")
            self.state += 1
        
        elif self.state in [1, 2, 3]: # Capture robot points
            if self.latest_pose is None:
                self.get_logger().warn("Robot pose not received yet!")
                return
            p = self.latest_pose.pose.position
            coords = [p.x, p.y, p.z]
            
            if self.state == 1:
                self.origin_coords = coords
                self.get_logger().info(f"Step 2/4: Origin point captured: {np.round(coords, 4)}")
            elif self.state == 2:
                self.x_axis_coords = coords
                self.get_logger().info(f"Step 3/4: X-axis point captured: {np.round(coords, 4)}")
            elif self.state == 3:
                self.y_axis_coords = coords
                self.get_logger().info(f"Step 4/4: Y-axis point captured: {np.round(coords, 4)}")
                self.perform_final_calculation()
            self.state += 1

    def perform_final_calculation(self):
        # --- Calculate T_base_board from the 3 points ---
        origin = np.array(self.origin_coords, dtype=float)
        point_x = np.array(self.x_axis_coords, dtype=float)
        point_y = np.array(self.y_axis_coords, dtype=float)

        x_axis = point_x - origin
        x_axis /= np.linalg.norm(x_axis)
        y_vec_temp = point_y - origin
        z_axis = np.cross(x_axis, y_vec_temp)
        z_axis /= np.linalg.norm(z_axis)
        y_axis = np.cross(z_axis, x_axis)
        
        T_base_board = np.eye(4)
        T_base_board[:3, 0] = x_axis
        T_base_board[:3, 1] = y_axis
        T_base_board[:3, 2] = z_axis
        T_base_board[:3, 3] = origin

        # --- Calculate the final T_base_cam ---
        T_board_cam = np.linalg.inv(self.T_cam_board)
        T_base_cam = T_base_board @ T_board_cam

        self.get_logger().info("\n--- Final Base-to-Camera Transformation ---\n" + str(np.round(T_base_cam, 4)))
        with open('final_base_to_camera_transform.yaml', 'w') as f:
            yaml.dump({'camera_pose_in_base': T_base_cam.tolist()}, f, indent=4)
        self.get_logger().info("Result saved to 'final_base_to_camera_transform.yaml'")

    def display_instructions(self, img):
        if self.state == 0:
            text = "1. Show board to camera. Press SPACE."
        elif self.state == 1:
            text = "2. Jog robot pointer to ORIGIN. Press SPACE."
        elif self.state == 2:
            text = "3. Jog robot pointer to X-AXIS point. Press SPACE."
        elif self.state == 3:
            text = "4. Jog robot pointer to Y-AXIS point. Press SPACE."
        elif self.state == 4:
            text = "Calculation complete! Check console. (Q to quit)"
        
        cv2.putText(img, text, (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)

def main(args=None):
    rclpy.init(args=args)
    node = InteractiveCalibrator()
    rclpy.spin(node)
    node.destroy_node()

if __name__ == '__main__':
    main()


