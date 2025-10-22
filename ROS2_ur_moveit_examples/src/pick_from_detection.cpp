#include <rclcpp/rclcpp.hpp>
#include <rclcpp/executors/single_threaded_executor.hpp>
#include <rclcpp/qos.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <moveit/move_group_interface/move_group_interface.h>
#include <ur_msgs/srv/set_io.hpp>

#include <termios.h>
#include <unistd.h>
#include <sys/select.h>
#include <cstdlib>
#include <thread>
#include <chrono>

using moveit::planning_interface::MoveGroupInterface;
using rclcpp::QoS;

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

class PickFromDetectionNode : public rclcpp::Node
{
public:
  PickFromDetectionNode()
  : Node("pick_from_detection")
  {
    // Parameters
    declare_parameter<std::string>("planning_group", "ur_manipulator");
    declare_parameter<std::string>("target_pose_topic", "/pin_housing/target_pose");
    declare_parameter<double>("z_offset_mm", 15.0);
    declare_parameter<double>("vel_scale", 0.1);
    declare_parameter<double>("acc_scale", 0.1);
    declare_parameter<int>("io_pin", 0);
    declare_parameter<std::vector<double>>("ready_joints", std::vector<double>{});
    declare_parameter<std::vector<double>>("home_joints", std::vector<double>{});

    planning_group_ = get_parameter("planning_group").as_string();
    target_topic_ = get_parameter("target_pose_topic").as_string();
    z_offset_ = get_parameter("z_offset_mm").as_double() / 1000.0;
    vel_scale_ = get_parameter("vel_scale").as_double();
    acc_scale_ = get_parameter("acc_scale").as_double();
    io_pin_ = get_parameter("io_pin").as_int();
    ready_joints_ = get_parameter("ready_joints").as_double_array();
    home_joints_ = get_parameter("home_joints").as_double_array();

    // MoveIt setup (defer until after construction to avoid bad_weak_ptr)
    move_group_ready_ = false;

    // IO client
    io_client_ = create_client<ur_msgs::srv::SetIO>("/io_and_status_controller/set_io");

    // Subscriber
    auto qos = QoS(rclcpp::QoSInitialization::from_rmw(rmw_qos_profile_sensor_data));
    target_sub_ = create_subscription<geometry_msgs::msg::PoseStamped>(
      target_topic_, qos,
      [this](geometry_msgs::msg::PoseStamped::SharedPtr msg){ latest_target_ = *msg; has_target_ = true; });

    RCLCPP_INFO(get_logger(), "Ready. Press SPACE to execute pick with latest /pin_housing/target_pose.");
    timer_ = create_wall_timer(std::chrono::milliseconds(50), std::bind(&PickFromDetectionNode::spin_keypoll, this));
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
        // Ensure planning is done in the expected frame
        move_group_->setPoseReferenceFrame("base");
        move_group_ready_ = true;
        RCLCPP_INFO(get_logger(), "MoveGroupInterface initialized for group: %s", planning_group_.c_str());
      }
      catch (const std::exception & e)
      {
        RCLCPP_WARN(get_logger(), "MoveGroup init not ready: %s", e.what());
      }
    }

    char k = read_key_nonblock();
    if (k == ' ')
    {
      if (!has_target_)
      {
        RCLCPP_WARN(get_logger(), "No target pose received yet.");
        return;
      }
      if (!move_group_ready_)
      {
        RCLCPP_WARN(get_logger(), "MoveGroup not initialized yet, try again.");
        return;
      }
      execute_cycle();
    }
  }

  bool plan_and_execute()
  {
    RCLCPP_INFO(get_logger(), "Planning... (vel_scale=%.2f, acc_scale=%.2f)", vel_scale_, acc_scale_);
    moveit::planning_interface::MoveGroupInterface::Plan plan;
    if (!static_cast<bool>(move_group_->plan(plan)))
    {
      RCLCPP_ERROR(get_logger(), "Planning failed");
      return false;
    }
    RCLCPP_INFO(get_logger(), "Plan computed. Executing...");
    return static_cast<bool>(move_group_->execute(plan));
  }

  bool set_tool_output(bool high)
  {
    if (!io_client_->wait_for_service(std::chrono::seconds(2)))
    {
      RCLCPP_WARN(get_logger(), "IO service not available");
      return false;
    }
    auto req = std::make_shared<ur_msgs::srv::SetIO::Request>();
    req->fun = ur_msgs::srv::SetIO::Request::FUN_SET_DIGITAL_OUT;
    req->pin = static_cast<uint8_t>(io_pin_);
    req->state = high ? 1.0 : 0.0;
    auto fut = io_client_->async_send_request(req);
    auto ret = fut.wait_for(std::chrono::seconds(2));
    return ret == std::future_status::ready;
  }

  void execute_cycle()
  {
    try
    {
      // Check if we have a target pose BEFORE moving anywhere
      if (!has_target_)
      {
        RCLCPP_ERROR(get_logger(), "No target pose available. Make sure pixel_to_tcp_node is running and detecting targets.");
        return;
      }

      // Capture the target pose while camera can see it
      geometry_msgs::msg::Pose target = latest_target_.pose;
      RCLCPP_INFO(get_logger(), "Step: Captured target pose -> XYZ = [%.3f, %.3f, %.3f] (frame=base)",
                  target.position.x, target.position.y, target.position.z);

      // Move to Home joints first if provided
      if (!home_joints_.empty())
      {
        RCLCPP_INFO(get_logger(), "Step: Move to HOME (radians) -> [%.4f, %.4f, %.4f, %.4f, %.4f, %.4f]",
                    home_joints_[0], home_joints_[1], home_joints_[2], home_joints_[3], home_joints_[4], home_joints_[5]);
        move_group_->setJointValueTarget(home_joints_);
        if (!plan_and_execute()) { RCLCPP_ERROR(get_logger(), "Home move failed"); return; }
      }

      // Move to Ready joints if provided
      if (!ready_joints_.empty())
      {
        RCLCPP_INFO(get_logger(), "Step: Move to READY (radians) -> [%.4f, %.4f, %.4f, %.4f, %.4f, %.4f]",
                    ready_joints_[0], ready_joints_[1], ready_joints_[2], ready_joints_[3], ready_joints_[4], ready_joints_[5]);
        move_group_->setJointValueTarget(ready_joints_);
        if (!plan_and_execute()) { RCLCPP_ERROR(get_logger(), "Ready move failed"); return; }
      }

      // Compose approach (z + offset) using the captured target pose
      geometry_msgs::msg::Pose approach = target;
      approach.position.z += z_offset_;

      RCLCPP_INFO(get_logger(), "Step: Approach above target -> XYZ = [%.3f, %.3f, %.3f] (frame=base)",
                  approach.position.x, approach.position.y, approach.position.z);

      move_group_->setPoseTarget(approach);
      if (!plan_and_execute()) { RCLCPP_ERROR(get_logger(), "Approach move failed"); return; }

      // Descend to target Z
      RCLCPP_INFO(get_logger(), "Step: Descend to target Z -> XYZ = [%.3f, %.3f, %.3f] (frame=base)",
                  target.position.x, target.position.y, target.position.z);
      move_group_->setPoseTarget(target);
      move_group_->setMaxVelocityScalingFactor(std::min(vel_scale_, 0.05));
      move_group_->setMaxAccelerationScalingFactor(std::min(acc_scale_, 0.05));
      if (!plan_and_execute()) { RCLCPP_ERROR(get_logger(), "Descend move failed"); return; }

      // Tool ON
      RCLCPP_INFO(get_logger(), "Step: Set IO pin %d = HIGH", io_pin_);
      (void)set_tool_output(true);

      // Retreat back to approach
      move_group_->setMaxVelocityScalingFactor(vel_scale_);
      move_group_->setMaxAccelerationScalingFactor(acc_scale_);
      RCLCPP_INFO(get_logger(), "Step: Retreat to approach height -> XYZ = [%.3f, %.3f, %.3f]",
                  approach.position.x, approach.position.y, approach.position.z);
      move_group_->setPoseTarget(approach);
      if (!plan_and_execute()) { RCLCPP_ERROR(get_logger(), "Retreat move failed"); }

      // Go Home if provided
      if (!home_joints_.empty())
      {
        RCLCPP_INFO(get_logger(), "Step: Move to HOME (radians) -> [%.4f, %.4f, %.4f, %.4f, %.4f, %.4f]",
                    home_joints_[0], home_joints_[1], home_joints_[2], home_joints_[3], home_joints_[4], home_joints_[5]);
        move_group_->setJointValueTarget(home_joints_);
        (void)plan_and_execute();
      }

      // Tool OFF after returning Home
      RCLCPP_INFO(get_logger(), "Step: Set IO pin %d = LOW", io_pin_);
      (void)set_tool_output(false);
    }
    catch (const std::exception & e)
    {
      RCLCPP_ERROR(get_logger(), "Exception in execute_cycle: %s", e.what());
    }
  }

  std::string planning_group_;
  std::string target_topic_;
  double z_offset_ {0.015};
  double vel_scale_ {0.1};
  double acc_scale_ {0.1};
  int io_pin_ {0};
  std::vector<double> ready_joints_;
  std::vector<double> home_joints_;

  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr target_sub_;
  rclcpp::Client<ur_msgs::srv::SetIO>::SharedPtr io_client_;
  rclcpp::TimerBase::SharedPtr timer_;

  geometry_msgs::msg::PoseStamped latest_target_;
  bool has_target_ {false};
  std::shared_ptr<MoveGroupInterface> move_group_;
  bool move_group_ready_ {false};
};

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<PickFromDetectionNode>();
  rclcpp::executors::SingleThreadedExecutor exec;
  exec.add_node(node);
  exec.spin();
  rclcpp::shutdown();
  return 0;
}


