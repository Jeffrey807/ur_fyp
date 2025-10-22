#include <rclcpp/rclcpp.hpp>
#include <moveit/moveit_cpp/moveit_cpp.h>
#include <moveit/moveit_cpp/planning_component.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

static const rclcpp::Logger LOGGER = rclcpp::get_logger("ur_moveit_cpp_tcp");

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::NodeOptions node_options;
  node_options.automatically_declare_parameters_from_overrides(true);
  auto node = rclcpp::Node::make_shared("ur_moveit_cpp_tcp", "", node_options);

  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(node);
  std::thread([&executor]() { executor.spin(); }).detach();

  std::string planning_group = "ur_manipulator";
  (void)node->get_parameter("planning_group", planning_group);
  std::string base_frame = "base";
  (void)node->get_parameter("base_frame", base_frame);
  std::string ee_link = "tool0";
  (void)node->get_parameter("ee_link", ee_link);

  double target_x = 0.35;
  (void)node->get_parameter("target_x", target_x);
  double target_y = 0.0;
  (void)node->get_parameter("target_y", target_y);
  double target_z = 0.20;
  (void)node->get_parameter("target_z", target_z);
  double target_yaw = 0.0;
  (void)node->get_parameter("target_yaw", target_yaw);

  // Expect robot_description parameters to be provided via --params-file

  // Fixed down orientation (rx, ry fixed; yaw param)
  const double rx = 2.288;
  const double ry = 2.151;
  const double rz = target_yaw;

  auto moveit_cpp = std::make_shared<moveit_cpp::MoveItCpp>(node);
  moveit_cpp->getPlanningSceneMonitor()->providePlanningSceneService();
  auto planning_components = std::make_shared<moveit_cpp::PlanningComponent>(planning_group, moveit_cpp);

  geometry_msgs::msg::PoseStamped target_pose;
  target_pose.header.frame_id = base_frame;
  target_pose.pose.position.x = target_x;
  target_pose.pose.position.y = target_y;
  target_pose.pose.position.z = target_z;

  tf2::Quaternion q;
  q.setRPY(rx, ry, rz);
  target_pose.pose.orientation = tf2::toMsg(q);

  // Give the CurrentStateMonitor time to receive fresh joint states
  rclcpp::sleep_for(std::chrono::milliseconds(500));
  planning_components->setStartStateToCurrentState();
  planning_components->setGoal(target_pose, ee_link);

  RCLCPP_INFO(LOGGER, "Planning to TCP (%.3f, %.3f, %.3f) yaw=%.3f frame='%s' ee='%s'",
              target_x, target_y, target_z, target_yaw, base_frame.c_str(), ee_link.c_str());

  moveit_cpp::PlanningComponent::PlanRequestParameters plan_params;
  plan_params.planning_pipeline = "ompl";  // match the params file
  plan_params.planning_time = 3.0;
  auto plan_solution = planning_components->plan(plan_params);
  if (!plan_solution)
  {
    RCLCPP_ERROR(LOGGER, "Planning failed");
    rclcpp::shutdown();
    return 1;
  }

  if (!planning_components->execute())
  {
    RCLCPP_ERROR(LOGGER, "Execution failed");
    rclcpp::shutdown();
    return 2;
  }

  RCLCPP_INFO(LOGGER, "Motion complete");
  rclcpp::shutdown();
  return 0;
}


