#include <rclcpp/rclcpp.hpp>
#include <rclcpp/executors/single_threaded_executor.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <moveit/move_group_interface/move_group_interface.h>
#include <sensor_msgs/msg/joint_state.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

#include <termios.h>
#include <unistd.h>
#include <sys/select.h>
#include <cstdlib>
#include <thread>
#include <chrono>
#include <algorithm>

using moveit::planning_interface::MoveGroupInterface;

namespace
{
char read_key_nonblock()
{
  termios oldt, newt;
  tcgetattr(STDIN_FILENO, &oldt);
  newt = oldt;
  newt.c_lflag &= ~(ICANON | ECHO);
  tcsetattr(STDIN_FILENO, TCSANOW, &newt);
  char ch = 0;
  timeval tv {0, 0};
  fd_set rfds; FD_ZERO(&rfds); FD_SET(STDIN_FILENO, &rfds);
  if (select(STDIN_FILENO + 1, &rfds, nullptr, nullptr, &tv) > 0)
  {
    if (read(STDIN_FILENO, &ch, 1) <= 0) ch = 0;
  }
  tcsetattr(STDIN_FILENO, TCSANOW, &oldt);
  return ch;
}
}

class UR2DPickNode : public rclcpp::Node
{
public:
  UR2DPickNode()
  : Node("ur_2d_pick")
  {
    // Parameters
    declare_parameter<std::string>("planning_group", "ur_manipulator");
    declare_parameter<std::string>("ee_link", "tool0");
    declare_parameter<std::string>("base_frame", "base_link");
    declare_parameter<double>("vel_scale", 0.1);
    declare_parameter<double>("acc_scale", 0.1);
    declare_parameter<bool>("use_cartesian_tcp", true);
    declare_parameter<double>("approach_offset_z", 0.05); // 5 cm above target
    // Diagnostic: raise from current pose by Z
    declare_parameter<bool>("diag_raise_from_current", false);
    declare_parameter<double>("diag_raise_m", 0.05);
    
    // Test TCP position parameters (180° Z-rotated from tcp_pose_broadcaster)
    declare_parameter<double>("test_tcp_x", 0.38916245534468696);
    declare_parameter<double>("test_tcp_y", -0.23241412597629707);
    declare_parameter<double>("test_tcp_z", 0.49028813782131414);
    // Test TCP orientation parameters (quaternion) - 180° Z-rotated from tcp_pose_broadcaster
    declare_parameter<double>("test_qx", -0.7179487478951413);
    declare_parameter<double>("test_qy", 0.6958850418815234);
    declare_parameter<double>("test_qz", 0.01597792534649804);
    declare_parameter<double>("test_qw", -0.006189489718153721);
    
    // Alternative: XYZ + RPY input parameters (in base frame)
    declare_parameter<double>("input_x", 0.0);
    declare_parameter<double>("input_y", 0.0);
    declare_parameter<double>("input_z", 0.0);
    declare_parameter<double>("input_roll", 0.0);
    declare_parameter<double>("input_pitch", 0.0);
    declare_parameter<double>("input_yaw", 0.0);
    
    // Joint positions (in radians) - order: [shoulder_pan, shoulder_lift, elbow, wrist_1, wrist_2, wrist_3]
    declare_parameter<std::vector<double>>("home_joints", std::vector<double>{
      -0.8024595419513147,     // shoulder_pan
      -1.5374625364886683,     // shoulder_lift
      1.8766813278198242,      // elbow
      -1.8868935743915003,     // wrist_1
      -1.576165024434225,      // wrist_2
      -0.8382990995990198      // wrist_3
    });
    
    declare_parameter<std::vector<double>>("ready_joints", std::vector<double>{
      -0.4299863020526331,     // shoulder_pan
      -1.829691235219137,      // shoulder_lift
      1.8737435340881348,      // elbow
      -1.5929144064532679,     // wrist_1
      -1.568348232899801,      // wrist_2
      -0.4645779768573206      // wrist_3
    });

    planning_group_ = get_parameter("planning_group").as_string();
    ee_link_ = get_parameter("ee_link").as_string();
    base_frame_ = get_parameter("base_frame").as_string();
    vel_scale_ = get_parameter("vel_scale").as_double();
    acc_scale_ = get_parameter("acc_scale").as_double();
    use_cartesian_tcp_ = get_parameter("use_cartesian_tcp").as_bool();
    test_tcp_x_ = get_parameter("test_tcp_x").as_double();
    test_tcp_y_ = get_parameter("test_tcp_y").as_double();
    test_tcp_z_ = get_parameter("test_tcp_z").as_double();
    test_qx_ = get_parameter("test_qx").as_double();
    test_qy_ = get_parameter("test_qy").as_double();
    test_qz_ = get_parameter("test_qz").as_double();
    test_qw_ = get_parameter("test_qw").as_double();
    
    // Alternative input parameters
    input_x_ = get_parameter("input_x").as_double();
    input_y_ = get_parameter("input_y").as_double();
    input_z_ = get_parameter("input_z").as_double();
    input_roll_ = get_parameter("input_roll").as_double();
    input_pitch_ = get_parameter("input_pitch").as_double();
    input_yaw_ = get_parameter("input_yaw").as_double();
    approach_offset_z_ = get_parameter("approach_offset_z").as_double();
    diag_raise_from_current_ = get_parameter("diag_raise_from_current").as_bool();
    diag_raise_m_ = get_parameter("diag_raise_m").as_double();
    home_joints_ = get_parameter("home_joints").as_double_array();
    ready_joints_ = get_parameter("ready_joints").as_double_array();

    // MoveIt setup (defer until after construction to avoid bad_weak_ptr)
    move_group_ready_ = false;
    tf_buffer_ = std::make_shared<tf2_ros::Buffer>(get_clock());
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

    // Subscribe to TCP pose
    tcp_sub_ = create_subscription<geometry_msgs::msg::PoseStamped>(
      "/tcp_pose_broadcaster/pose", 10,
      [this](const geometry_msgs::msg::PoseStamped::SharedPtr msg)
      {
        latest_tcp_pose_ = *msg;
        have_tcp_pose_ = true;
      });

    // Subscribe to joint states (fallback/cache for constraints/start state)
    joint_sub_ = create_subscription<sensor_msgs::msg::JointState>(
      "/joint_states", rclcpp::SensorDataQoS(),
      [this](const sensor_msgs::msg::JointState::SharedPtr msg)
      {
        last_joint_names_ = msg->name;
        last_joint_positions_ = msg->position;
        have_joint_state_ = !last_joint_names_.empty() && (last_joint_names_.size() == last_joint_positions_.size());
      });

    RCLCPP_INFO(get_logger(), "UR 2D Pick Node Ready.");
    RCLCPP_INFO(get_logger(), "Commands:");
    RCLCPP_INFO(get_logger(), "  H - Move to HOME position");
    RCLCPP_INFO(get_logger(), "  R - Move to READY position");
    RCLCPP_INFO(get_logger(), "  T - Move to TEST TCP position (linear move)");
    RCLCPP_INFO(get_logger(), "  ↑↓←→ - Move relative to current TCP (Y/Z plane, ±0.10m)");
    RCLCPP_INFO(get_logger(), "  Q - Quit");
    
    timer_ = create_wall_timer(std::chrono::milliseconds(50), std::bind(&UR2DPickNode::spin_keypoll, this));
  }

private:
  void spin_keypoll()
  {
    // Lazy-initialize MoveGroupInterface once a shared_ptr to this exists
    if (!move_group_ready_)
    {
      try
      {
        auto node_ptr = std::static_pointer_cast<rclcpp::Node>(shared_from_this());
        move_group_ = std::make_shared<MoveGroupInterface>(node_ptr, planning_group_);
        move_group_->setMaxVelocityScalingFactor(vel_scale_);
        move_group_->setMaxAccelerationScalingFactor(acc_scale_);
        move_group_->setPoseReferenceFrame(base_frame_);
        move_group_->setEndEffectorLink(ee_link_);
        // Planner id will be set per-motion (PTP/LIN) when Pilz is active
        
        // Set workspace bounds to prevent planning below robot base
        move_group_->setWorkspace(-2.0, -2.0, 0.0, 2.0, 2.0, 2.0); // min_x, min_y, min_z, max_x, max_y, max_z
        
        // move_group_->setPlanningFrame("base");
        move_group_ready_ = true;
        // Force base_link everywhere for simplicity
        planning_frame_ = base_frame_;
        move_group_->setPoseReferenceFrame(base_frame_);
        RCLCPP_INFO(get_logger(), "MoveGroupInterface initialized for group: %s", planning_group_.c_str());
        RCLCPP_INFO(get_logger(), "Planning frame: %s", move_group_->getPlanningFrame().c_str());
        RCLCPP_INFO(get_logger(), "Pose reference frame: %s", move_group_->getPoseReferenceFrame().c_str());
        RCLCPP_INFO(get_logger(), "Using base frame: %s", base_frame_.c_str());
        RCLCPP_INFO(get_logger(), "End effector link: %s", move_group_->getEndEffectorLink().c_str());
        RCLCPP_INFO(get_logger(), "Workspace bounds: X=[-2.0, 2.0], Y=[-2.0, 2.0], Z=[0.0, 2.0] (prevents planning below base)");
        // Small warm-up to ensure state monitor has data before first plan
        std::this_thread::sleep_for(std::chrono::milliseconds(300));
      }
      catch (const std::exception & e)
      {
        RCLCPP_WARN(get_logger(), "MoveGroup init not ready: %s", e.what());
      }
    }

    char k = read_key_nonblock();
    if (k == 'h' || k == 'H')
    {
      if (!move_group_ready_)
      {
        RCLCPP_WARN(get_logger(), "MoveGroup not initialized yet, try again.");
        return;
      }
      move_to_home();
    }
    else if (k == 'r' || k == 'R')
    {
      if (!move_group_ready_)
      {
        RCLCPP_WARN(get_logger(), "MoveGroup not initialized yet, try again.");
        return;
      }
      move_to_ready();
    }
    else if (k == 't' || k == 'T')
    {
      if (!move_group_ready_)
      {
        RCLCPP_WARN(get_logger(), "MoveGroup not initialized yet, try again.");
        return;
      }
      move_to_test_tcp();
    }
    else if (k == 0x1B) // ESC sequence start for arrow keys
    {
      // Attempt to read the next two chars without blocking
      char k1 = read_key_nonblock();
      char k2 = read_key_nonblock();
      if (k1 == '[')
      {
        if (!move_group_ready_)
        {
          RCLCPP_WARN(get_logger(), "MoveGroup not initialized yet, try again.");
          return;
        }
        // Up arrow: Z +0.10 m
        if (k2 == 'A')
        {
          move_relative_from_tcp(0.0, +0.10);
        }
        // Down arrow: Z -0.10 m
        else if (k2 == 'B')
        {
          move_relative_from_tcp(0.0, -0.10);
        }
        // Right arrow: Y -0.10 m
        else if (k2 == 'C')
        {
          move_relative_from_tcp(-0.10, 0.0);
        }
        // Left arrow: Y +0.10 m
        else if (k2 == 'D')
        {
          move_relative_from_tcp(+0.10, 0.0);
        }
      }
    }
    else if (k == 'q' || k == 'Q')
    {
      RCLCPP_INFO(get_logger(), "Shutting down...");
      rclcpp::shutdown();
    }
  }

  bool plan_and_execute()
  {
    RCLCPP_INFO(get_logger(), "Planning... (vel_scale=%.2f, acc_scale=%.2f)", vel_scale_, acc_scale_);
    // Ensure start state matches current robot state
    move_group_->startStateMonitor();
    // wait briefly for fresh joint_states
    std::this_thread::sleep_for(std::chrono::milliseconds(800));
    move_group_->setStartStateToCurrentState();
    // Start state is current; keep default planner behavior (OMPL)
    moveit::planning_interface::MoveGroupInterface::Plan plan;
    if (!static_cast<bool>(move_group_->plan(plan)))
    {
      RCLCPP_ERROR(get_logger(), "Planning failed");
      return false;
    }
    
    // Apply velocity and acceleration scaling
    for (auto& point : plan.trajectory_.joint_trajectory.points)
    {
      for (auto& velocity : point.velocities)
      {
        velocity *= vel_scale_;
      }
      for (auto& acceleration : point.accelerations)
      {
        acceleration *= acc_scale_;
      }
    }
    
    RCLCPP_INFO(get_logger(), "Plan computed. Executing...");
    return static_cast<bool>(move_group_->execute(plan));
  }

  // Convert RPY to quaternion
  geometry_msgs::msg::Quaternion rpy_to_quaternion(double roll, double pitch, double yaw)
  {
    geometry_msgs::msg::Quaternion q;
    
    double cy = cos(yaw * 0.5);
    double sy = sin(yaw * 0.5);
    double cp = cos(pitch * 0.5);
    double sp = sin(pitch * 0.5);
    double cr = cos(roll * 0.5);
    double sr = sin(roll * 0.5);

    q.w = cr * cp * cy + sr * sp * sy;
    q.x = sr * cp * cy - cr * sp * sy;
    q.y = cr * sp * cy + sr * cp * sy;
    q.z = cr * cp * sy - sr * sp * cy;
    
    return q;
  }

  // Apply 180° Z-axis rotation transformation (position only, keep orientation)
  void apply_180_z_rotation(double& x, double& y, double& z, 
                           double& qx, double& qy, double& qz, double& qw)
  {
    // Position transformation: (x, y, z) -> (-x, -y, z)
    x = -x;
    y = -y;
    // z remains unchanged
    
    // Keep orientation unchanged for simple linear motion
    // (qx, qy, qz, qw) remain the same
    (void)z; (void)qx; (void)qy; (void)qz; (void)qw; // Suppress unused parameter warnings
  }

  // Apply 180° Z-axis rotation transformation (both position and orientation)
  void apply_180_z_rotation_full(double& x, double& y, double& z, 
                                double& qx, double& qy, double& qz, double& qw)
  {
    // Position transformation: (x, y, z) -> (-x, -y, z)
    x = -x;
    y = -y;
    // z remains unchanged
    (void)z; // Suppress unused parameter warning
    
    // Quaternion transformation for 180° Z rotation
    // q_rot = (0, 0, 1, 0) for 180° around Z
    double q_rot_x = 0.0, q_rot_y = 0.0, q_rot_z = 1.0, q_rot_w = 0.0;
    
    // Quaternion multiplication: q_new = q_rot × q_orig
    double new_w = q_rot_w * qw - q_rot_x * qx - q_rot_y * qy - q_rot_z * qz;
    double new_x = q_rot_w * qx + q_rot_x * qw + q_rot_y * qz - q_rot_z * qy;
    double new_y = q_rot_w * qy - q_rot_x * qz + q_rot_y * qw + q_rot_z * qx;
    double new_z = q_rot_w * qz + q_rot_x * qy - q_rot_y * qx + q_rot_z * qw;
    
    qx = new_x;
    qy = new_y;
    qz = new_z;
    qw = new_w;
  }

  void move_to_home()
  {
    RCLCPP_INFO(get_logger(), "Moving to HOME position...");
    RCLCPP_INFO(get_logger(), "HOME joints (radians): [%.4f, %.4f, %.4f, %.4f, %.4f, %.4f]",
                home_joints_[0], home_joints_[1], home_joints_[2], home_joints_[3], home_joints_[4], home_joints_[5]);
    
    // Use default planner (OMPL)
    move_group_->setPlannerId("");
    move_group_->setJointValueTarget(home_joints_);
    if (plan_and_execute())
    {
      RCLCPP_INFO(get_logger(), "Successfully moved to HOME position");
    }
    else
    {
      RCLCPP_ERROR(get_logger(), "Failed to move to HOME position");
    }
  }

  void move_to_ready()
  {
    RCLCPP_INFO(get_logger(), "Moving to READY position...");
    RCLCPP_INFO(get_logger(), "READY joints (radians): [%.4f, %.4f, %.4f, %.4f, %.4f, %.4f]",
                ready_joints_[0], ready_joints_[1], ready_joints_[2], ready_joints_[3], ready_joints_[4], ready_joints_[5]);
    
    // Use default planner (OMPL)
    move_group_->setPlannerId("");
    move_group_->setJointValueTarget(ready_joints_);
    if (plan_and_execute())
    {
      RCLCPP_INFO(get_logger(), "Successfully moved to READY position");
    }
    else
    {
      RCLCPP_ERROR(get_logger(), "Failed to move to READY position");
    }
  }

  void move_to_test_tcp()
  {
    RCLCPP_INFO(get_logger(), "Moving to TEST TCP position (linear move)...");
    
    // Debug: Check what frame MoveIt is using
    RCLCPP_INFO(get_logger(), "MoveIt planning frame: %s", move_group_->getPlanningFrame().c_str());
    RCLCPP_INFO(get_logger(), "MoveIt pose reference frame: %s", move_group_->getPoseReferenceFrame().c_str());
    
    // Use the exact target pose you specified (already in base_link frame)
    geometry_msgs::msg::Pose test_pose;
    test_pose.position.x = 0.41077980111952567;
    test_pose.position.y = -0.2674434723652385;
    test_pose.position.z = 0.33779377789121495;
    test_pose.orientation.x = 0.6946431472456444; //0.6945853943904723; //-0.7193238054573408;
    test_pose.orientation.y = 0.7192677440288445; //0.7193238054573408; //0.6945853943904723;
    test_pose.orientation.z = 0.00012794497354252145; //6.099433104659104e-05; //0.01115298531755227;
    test_pose.orientation.w = 0.011171124133520415; //0.01115298531755227; //-6.099433104659104e-05;
    
    RCLCPP_INFO(get_logger(), "Target TCP pose (base_link frame): XYZ = [%.3f, %.3f, %.3f]", 
                test_pose.position.x, test_pose.position.y, test_pose.position.z);
    RCLCPP_INFO(get_logger(), "Target TCP orientation: [%.6f, %.6f, %.6f, %.6f]", 
                test_pose.orientation.x, test_pose.orientation.y, test_pose.orientation.z, test_pose.orientation.w);
    
    // Use PoseStamped with base_link frame
    geometry_msgs::msg::PoseStamped target_pose_stamped;
    target_pose_stamped.header.stamp = now();
    target_pose_stamped.header.frame_id = base_frame_;  // base_link
    target_pose_stamped.pose = test_pose;

    // Planning in base_link frame (no cross-frame transforms needed)
    geometry_msgs::msg::PoseStamped target_in_planning = target_pose_stamped;
    bool have_planning_frame_target = true;

    const bool cartesian_mode = false; // Disable Cartesian planning for more optimal paths
    if (cartesian_mode)
    {
      // Get current pose from MoveIt
      move_group_->startStateMonitor();
      std::this_thread::sleep_for(std::chrono::milliseconds(500));
      auto current_pose_stamped = move_group_->getCurrentPose(ee_link_);
      geometry_msgs::msg::Pose current_pose = current_pose_stamped.pose;
      
      // Fallback to TF if needed
      if (current_pose_stamped.header.stamp.sec == 0 && current_pose_stamped.header.stamp.nanosec == 0)
      {
        try
        {
          auto tf = tf_buffer_->lookupTransform(base_frame_, ee_link_, tf2::TimePointZero, tf2::durationFromSec(1.0));
          current_pose.position.x = tf.transform.translation.x;
          current_pose.position.y = tf.transform.translation.y;
          current_pose.position.z = tf.transform.translation.z;
          current_pose.orientation = tf.transform.rotation;
          RCLCPP_WARN(get_logger(), "Current pose via TF fallback");
        }
        catch (const std::exception& ex)
        {
          RCLCPP_ERROR(get_logger(), "Failed to get current pose: %s", ex.what());
          return;
        }
      }
      
      std::vector<geometry_msgs::msg::Pose> waypoints;
      waypoints.push_back(current_pose);
      waypoints.push_back(target_in_planning.pose);

      moveit_msgs::msg::RobotTrajectory trajectory;
      const double eef_step = 0.02;  // 20 mm resolution for robustness
      const double jump_threshold = 0.0; // disable jump threshold
      double fraction = move_group_->computeCartesianPath(waypoints, eef_step, jump_threshold, trajectory, true);
      RCLCPP_INFO(get_logger(), "Cartesian path fraction: %.3f", fraction);
      if (fraction < 0.80)
      {
        RCLCPP_ERROR(get_logger(), "Cartesian planning failed (fraction < 0.99), falling back to standard planning");
        move_group_->setPoseTarget(target_in_planning, ee_link_);
      if (plan_and_execute())
      {
        RCLCPP_INFO(get_logger(), "Successfully moved to TEST TCP position (fallback)");
      }
      else
      {
        RCLCPP_ERROR(get_logger(), "Failed to move to TEST TCP position (fallback)");
        RCLCPP_ERROR(get_logger(), "Target pose: XYZ=[%.3f, %.3f, %.3f], Quat=[%.6f, %.6f, %.6f, %.6f]",
                    target_in_planning.pose.position.x, target_in_planning.pose.position.y, target_in_planning.pose.position.z,
                    target_in_planning.pose.orientation.x, target_in_planning.pose.orientation.y, 
                    target_in_planning.pose.orientation.z, target_in_planning.pose.orientation.w);
      }
        return;
      }
      
      // Apply time, velocity, and acceleration scaling to Cartesian trajectory
      RCLCPP_INFO(get_logger(), "Applying time, velocity, and acceleration scaling to Cartesian path (vel_scale=%.2f, acc_scale=%.2f)", vel_scale_, acc_scale_);
      for (auto& point : trajectory.joint_trajectory.points)
      {
        // Scale the time it takes to reach this point (more robust calculation)
        double total_time_sec = point.time_from_start.sec + (point.time_from_start.nanosec / 1e9);
        double scaled_time_sec = total_time_sec / vel_scale_;
        
        point.time_from_start.sec = static_cast<int32_t>(scaled_time_sec);
        point.time_from_start.nanosec = static_cast<uint32_t>((scaled_time_sec - point.time_from_start.sec) * 1e9);

        // Scale velocities
        for (auto& velocity : point.velocities)
        {
          velocity *= vel_scale_;
        }

        // Scale accelerations (scales with velocity squared)
        for (auto& acceleration : point.accelerations)
        {
          acceleration *= (vel_scale_ * vel_scale_);
        }
      }
      
      moveit::planning_interface::MoveGroupInterface::Plan plan;
      plan.trajectory_ = trajectory;
      RCLCPP_INFO(get_logger(), "Executing scaled Cartesian trajectory...");
      RCLCPP_INFO(get_logger(), "Trajectory has %zu points, %zu joints", 
                  trajectory.joint_trajectory.points.size(), 
                  trajectory.joint_trajectory.joint_names.size());
      
      // Check if we have valid trajectory data
      if (trajectory.joint_trajectory.points.empty())
      {
        RCLCPP_ERROR(get_logger(), "Trajectory is empty - cannot execute");
        return;
      }
      
      // Try execution with timeout
      auto start_time = std::chrono::steady_clock::now();
      bool execution_result = static_cast<bool>(move_group_->execute(plan));
      auto end_time = std::chrono::steady_clock::now();
      auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(end_time - start_time);
      
      if (execution_result)
      {
        RCLCPP_INFO(get_logger(), "Successfully moved to TEST TCP position (Cartesian) in %ld ms", duration.count());
      }
      else
      {
        RCLCPP_ERROR(get_logger(), "Execution failed for Cartesian TCP move after %ld ms", duration.count());
        RCLCPP_ERROR(get_logger(), "Trajectory has %zu points", trajectory.joint_trajectory.points.size());
        if (!trajectory.joint_trajectory.points.empty())
        {
          RCLCPP_ERROR(get_logger(), "First point: %zu joints, time=%.3f", 
                      trajectory.joint_trajectory.points[0].positions.size(),
                      trajectory.joint_trajectory.points[0].time_from_start.sec + 
                      trajectory.joint_trajectory.points[0].time_from_start.nanosec / 1e9);
        }
      }
    }
    else
    {
      // Standard planning with fallback options
      move_group_->clearPathConstraints();
      
      // Use default planner with better parameters for shorter paths
      move_group_->setPlanningPipelineId(""); // Use default pipeline
      move_group_->setPlannerId(""); // Use default planner
      
      // Set planning parameters for better paths
      move_group_->setPlanningTime(3.0); // More time for better planning
      move_group_->setNumPlanningAttempts(5); // Try multiple times for better paths
      move_group_->setMaxVelocityScalingFactor(vel_scale_);
      move_group_->setMaxAccelerationScalingFactor(acc_scale_);
      
      // Standard planning: prefer planning-frame target if available, otherwise use stamped input frame
      if (have_planning_frame_target)
      {
        move_group_->setPoseTarget(target_in_planning, ee_link_);
      }
      else
      {
        move_group_->setPoseTarget(target_pose_stamped, ee_link_);
      }
      
      // Try planning with constraints first
      moveit_msgs::msg::OrientationConstraint ocm;
      ocm.link_name = ee_link_;
      ocm.header.frame_id = base_frame_;
      ocm.orientation = target_in_planning.pose.orientation;
      ocm.absolute_x_axis_tolerance = 1.0; // More flexible
      ocm.absolute_y_axis_tolerance = 1.0;
      ocm.absolute_z_axis_tolerance = 0.5; // Allow more yaw flexibility
      ocm.weight = 0.3; // Lower weight
      
      moveit_msgs::msg::Constraints constraints;
      constraints.orientation_constraints.push_back(ocm);
      move_group_->setPathConstraints(constraints);
      
      RCLCPP_INFO(get_logger(), "Trying planning with orientation constraints...");
      if (plan_and_execute())
      {
        RCLCPP_INFO(get_logger(), "Successfully moved to TEST TCP position (with constraints)");
      }
      else
      {
        RCLCPP_WARN(get_logger(), "Planning with constraints failed, trying without constraints...");
        move_group_->clearPathConstraints();
        
        // Fallback without constraints
        move_group_->setPlanningTime(2.0);
        move_group_->setNumPlanningAttempts(3);
        
        if (plan_and_execute())
        {
          RCLCPP_INFO(get_logger(), "Successfully moved to TEST TCP position (OMPL fallback)");
        }
        else
        {
          RCLCPP_ERROR(get_logger(), "Failed to move to TEST TCP position (all methods failed)");
          RCLCPP_ERROR(get_logger(), "Target pose: XYZ=[%.3f, %.3f, %.3f], Quat=[%.6f, %.6f, %.6f, %.6f]",
                      target_in_planning.pose.position.x, target_in_planning.pose.position.y, target_in_planning.pose.position.z,
                      target_in_planning.pose.orientation.x, target_in_planning.pose.orientation.y, 
                      target_in_planning.pose.orientation.z, target_in_planning.pose.orientation.w);
        }
      }
      
      // Clear constraints after planning
      move_group_->clearPathConstraints();
    }
  }

  void move_to_input_pose()
  {
    RCLCPP_INFO(get_logger(), "Moving to INPUT pose (XYZ+RPY in base frame)...");
    
    // Convert RPY to quaternion
    geometry_msgs::msg::Quaternion input_quat = rpy_to_quaternion(input_roll_, input_pitch_, input_yaw_);
    
    // Apply 180° Z-axis rotation transformation
    double x = input_x_;
    double y = input_y_;
    double z = input_z_;
    double qx = input_quat.x;
    double qy = input_quat.y;
    double qz = input_quat.z;
    double qw = input_quat.w;
    
    apply_180_z_rotation(x, y, z, qx, qy, qz, qw);
    
    RCLCPP_INFO(get_logger(), "Input pose (base frame): XYZ = [%.3f, %.3f, %.3f], RPY = [%.3f, %.3f, %.3f]", 
                input_x_, input_y_, input_z_, input_roll_, input_pitch_, input_yaw_);
    RCLCPP_INFO(get_logger(), "Transformed pose (base_link frame): XYZ = [%.3f, %.3f, %.3f], Quat = [%.6f, %.6f, %.6f, %.6f]", 
                x, y, z, qx, qy, qz, qw);
    
    // Create target pose
    geometry_msgs::msg::Pose target_pose;
    target_pose.position.x = x;
    target_pose.position.y = y;
    target_pose.position.z = z;
    target_pose.orientation.x = qx;
    target_pose.orientation.y = qy;
    target_pose.orientation.z = qz;
    target_pose.orientation.w = qw;
    
    // Use PoseStamped with explicit frame
    geometry_msgs::msg::PoseStamped target_pose_stamped;
    target_pose_stamped.header.stamp = now();
    target_pose_stamped.header.frame_id = base_frame_;
    target_pose_stamped.pose = target_pose;
    
    // Set pose target and execute
    move_group_->setPoseTarget(target_pose_stamped, ee_link_);
    if (plan_and_execute())
    {
      RCLCPP_INFO(get_logger(), "Successfully moved to INPUT pose");
    }
    else
    {
      RCLCPP_ERROR(get_logger(), "Failed to move to INPUT pose");
    }
  }

  // Move relative to latest TCP pose from /tcp_pose_broadcaster/pose
  void move_relative_from_tcp(double delta_y, double delta_z)
  {
    if (!have_tcp_pose_)
    {
      RCLCPP_WARN(get_logger(), "No TCP pose received yet from /tcp_pose_broadcaster/pose");
      return;
    }

    // Get current TCP pose from tcp_pose_broadcaster (in base frame)
    double x = latest_tcp_pose_.pose.position.x;
    double y = latest_tcp_pose_.pose.position.y;
    double z = latest_tcp_pose_.pose.position.z;
    double qx = latest_tcp_pose_.pose.orientation.x;
    double qy = latest_tcp_pose_.pose.orientation.y;
    double qz = latest_tcp_pose_.pose.orientation.z;
    double qw = latest_tcp_pose_.pose.orientation.w;

    RCLCPP_INFO(get_logger(), "Current TCP pose (base frame): XYZ=[%.3f, %.3f, %.3f], Quat=[%.6f, %.6f, %.6f, %.6f]", 
                x, y, z, qx, qy, qz, qw);

    // Apply keyboard offsets in base-frame axes
    y += delta_y;
    z += delta_z;

    RCLCPP_INFO(get_logger(), "After offset (base frame): XYZ=[%.3f, %.3f, %.3f]", x, y, z);

    // Transform base -> base_link (180° Z rotation for both position and orientation)
    apply_180_z_rotation_full(x, y, z, qx, qy, qz, qw);

    RCLCPP_INFO(get_logger(), "After 180° Z transform (base_link frame): XYZ=[%.3f, %.3f, %.3f], Quat=[%.6f, %.6f, %.6f, %.6f]", 
                x, y, z, qx, qy, qz, qw);

    // Create target pose for MoveIt
    geometry_msgs::msg::PoseStamped target_pose_stamped;
    target_pose_stamped.header.stamp = now();
    target_pose_stamped.header.frame_id = base_frame_;
    target_pose_stamped.pose.position.x = x;
    target_pose_stamped.pose.position.y = y;
    target_pose_stamped.pose.position.z = z;
    target_pose_stamped.pose.orientation.x = qx;
    target_pose_stamped.pose.orientation.y = qy;
    target_pose_stamped.pose.orientation.z = qz;
    target_pose_stamped.pose.orientation.w = qw;

    RCLCPP_INFO(get_logger(), "Moving relative from TCP: dY=%.3f m, dZ=%.3f m", delta_y, delta_z);

    // Ensure we have a fresh start state
    move_group_->startStateMonitor();
    {
      const int max_retries = 10;
      for (int i = 0; i < max_retries; ++i)
      {
        std::this_thread::sleep_for(std::chrono::milliseconds(200));
        move_group_->setStartStateToCurrentState();
        auto pose_stamped = move_group_->getCurrentPose(ee_link_);
        if (pose_stamped.header.stamp.sec != 0 || pose_stamped.header.stamp.nanosec != 0)
          break;
      }
    }

    // Try Cartesian jog: current -> target (no intermediate waypoint)
    const auto current_pose = move_group_->getCurrentPose(ee_link_).pose;

    std::vector<geometry_msgs::msg::Pose> waypoints;
    waypoints.reserve(2);
    waypoints.push_back(current_pose);
    waypoints.push_back(target_pose_stamped.pose);

    moveit_msgs::msg::RobotTrajectory trajectory;
    const double eef_step = 0.01;  // 10 mm finer for reliability
    const double jump_threshold = 0.0;
    double fraction = move_group_->computeCartesianPath(waypoints, eef_step, jump_threshold, trajectory, true);
    RCLCPP_INFO(get_logger(), "Cartesian jog fraction: %.3f", fraction);
    if (fraction < 0.90)
    {
      // Retry without collision checking to improve fraction for small jogs
      moveit_msgs::msg::RobotTrajectory traj_nc;
      double fraction_nc = move_group_->computeCartesianPath(waypoints, eef_step, jump_threshold, traj_nc, false);
      RCLCPP_INFO(get_logger(), "Cartesian jog (no-collision) fraction: %.3f", fraction_nc);
      if (fraction_nc > fraction)
      {
        fraction = fraction_nc;
        trajectory = std::move(traj_nc);
      }
    }

    if (fraction >= 0.90)
    {
      // Scale time, velocity, and acceleration (same scheme as before)
      RCLCPP_INFO(get_logger(), "Executing Cartesian jog with scaling (vel=%.2f, acc=%.2f)", vel_scale_, acc_scale_);
      for (auto& point : trajectory.joint_trajectory.points)
      {
        double total_time_sec = point.time_from_start.sec + (point.time_from_start.nanosec / 1e9);
        double scaled_time_sec = total_time_sec / vel_scale_;
        point.time_from_start.sec = static_cast<int32_t>(scaled_time_sec);
        point.time_from_start.nanosec = static_cast<uint32_t>((scaled_time_sec - point.time_from_start.sec) * 1e9);
        for (auto& v : point.velocities) v *= vel_scale_;
        for (auto& a : point.accelerations) a *= (vel_scale_ * vel_scale_);
      }
      moveit::planning_interface::MoveGroupInterface::Plan plan;
      plan.trajectory_ = trajectory;
      if (static_cast<bool>(move_group_->execute(plan)))
      {
        RCLCPP_INFO(get_logger(), "Successfully jogged to target (Cartesian)");
        return;
      }
      RCLCPP_WARN(get_logger(), "Cartesian execution failed; falling back to planning");
    }

    // Fallback: standard planning to pose target with mild orientation constraint to avoid spins
    {
      moveit_msgs::msg::OrientationConstraint ocm;
      ocm.link_name = ee_link_;
      ocm.header.frame_id = base_frame_;
      ocm.orientation = target_pose_stamped.pose.orientation;
      // Constrain yaw to ~10 deg, allow large roll/pitch to avoid overconstraint
      ocm.absolute_x_axis_tolerance = 1.5707963; // 90 deg
      ocm.absolute_y_axis_tolerance = 1.5707963; // 90 deg
      ocm.absolute_z_axis_tolerance = 0.1745329;  // 10 deg
      ocm.weight = 1.0;
      moveit_msgs::msg::Constraints c;
      c.orientation_constraints.push_back(ocm);
      move_group_->setPathConstraints(c);
    }
    // Also relax goal tolerances to bias nearest solution without spins
    // Use default planner (OMPL)
    move_group_->setPlannerId("");
    move_group_->setGoalOrientationTolerance(0.1745329); // ~10 deg
    move_group_->setGoalPositionTolerance(0.001); // 1 mm
    move_group_->setPoseTarget(target_pose_stamped, ee_link_);
    if (plan_and_execute())
    {
      RCLCPP_INFO(get_logger(), "Successfully moved to relative TCP target (planned)");
    }
    else
    {
      RCLCPP_ERROR(get_logger(), "Failed to move to relative TCP target (planned)");
    }
    // Clear constraints set for fallback planning
    move_group_->clearPathConstraints();
  }

  std::string planning_group_;
  std::string ee_link_ {"tool0"};
  std::string base_frame_ {"base_link"};
  std::string planning_frame_ {"world"};
  double vel_scale_ {0.1};
  double acc_scale_ {0.1};
  bool use_cartesian_tcp_ {false};
  double approach_offset_z_ {0.05};
  bool diag_raise_from_current_ {false};
  double diag_raise_m_ {0.05};
  double test_tcp_x_ {0.38916245534468696};
  double test_tcp_y_ {-0.23241412597629707};
  double test_tcp_z_ {0.49028813782131414};
  double test_qx_ {-0.7179487478951413};
  double test_qy_ {0.6958850418815234};
  double test_qz_ {0.01597792534649804};
  double test_qw_ {-0.006189489718153721};
  
  // Alternative input parameters (in base frame)
  double input_x_ {0.0};
  double input_y_ {0.0};
  double input_z_ {0.0};
  double input_roll_ {0.0};
  double input_pitch_ {0.0};
  double input_yaw_ {0.0};
  std::vector<double> home_joints_;
  std::vector<double> ready_joints_;

  rclcpp::TimerBase::SharedPtr timer_;
  std::shared_ptr<MoveGroupInterface> move_group_;
  bool move_group_ready_ {false};
  std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
  // TCP pose subscriber state
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr tcp_sub_;
  geometry_msgs::msg::PoseStamped latest_tcp_pose_;
  bool have_tcp_pose_ {false};
  // Joint states cache
  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joint_sub_;
  std::vector<std::string> last_joint_names_;
  std::vector<double> last_joint_positions_;
  bool have_joint_state_ {false};
};

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<UR2DPickNode>();
  rclcpp::executors::SingleThreadedExecutor exec;
  exec.add_node(node);
  exec.spin();
  rclcpp::shutdown();
  return 0;
}
