#include <rclcpp/rclcpp.hpp>
#include <moveit/move_group_interface/move_group_interface.h>
#include <chrono>
#include <thread>
#include <vector>

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<rclcpp::Node>(
      "test_joint_move",
      rclcpp::NodeOptions().automatically_declare_parameters_from_overrides(true));

  auto LOGGER = node->get_logger();
  RCLCPP_INFO(LOGGER, "Starting minimal joint-move test node...");

  using moveit::planning_interface::MoveGroupInterface;
  MoveGroupInterface move_group(node, "ur_manipulator");

  // Conservative, reliable settings for a verification move
  move_group.setPoseReferenceFrame("base_link");
  move_group.setEndEffectorLink("tool0");
  move_group.setMaxVelocityScalingFactor(0.1);
  move_group.setMaxAccelerationScalingFactor(0.1);
  move_group.setPlanningTime(5.0);
  move_group.setNumPlanningAttempts(5);

  // Waypoint 0 joints from hello_pilz2_with_suction.cpp
  // Order: [shoulder_pan, shoulder_lift, elbow, wrist_1, wrist_2, wrist_3]
  std::vector<double> waypoint0_joints{
      -0.0036109129535120132,
      -2.1629932562457483,
      2.0472545623779297,
      -1.4392817656146448,
      -1.5554221312152308,
      -0.037923638020650685};

  // Waypoint 2 joints
  // Order: [shoulder_pan, shoulder_lift, elbow, wrist_1, wrist_2, wrist_3]
  std::vector<double> waypoint2_joints{
      -0.5450833479510706,    // shoulder_pan
      -2.1810339132892054,    // shoulder_lift
      1.8573007583618164,     // elbow
      -1.222872559224264,     // wrist_1
      -1.5657218138324183,    // wrist_2
      -0.5784691015826624};   // wrist_3

  // Allow override via parameters (optional)
  std::vector<double> p;
  if (node->get_parameter("waypoint0_joints", p) && p.size() == 6)
  {
    waypoint0_joints = p;
    RCLCPP_INFO(LOGGER, "Using parameter override for waypoint0_joints.");
  }
  if (node->get_parameter("waypoint2_joints", p) && p.size() == 6)
  {
    waypoint2_joints = p;
    RCLCPP_INFO(LOGGER, "Using parameter override for waypoint2_joints.");
  }

  // Make sure start state is current for reliable planning
  move_group.startStateMonitor();
  std::this_thread::sleep_for(std::chrono::milliseconds(500));
  move_group.setStartStateToCurrentState();

  // Wait for controller to become available (important for execution)
  RCLCPP_INFO(LOGGER, "Waiting for joint_trajectory_controller to become available...");
  std::this_thread::sleep_for(std::chrono::seconds(2));

  RCLCPP_INFO(LOGGER,
              "Target joints (rad): [%.4f, %.4f, %.4f, %.4f, %.4f, %.4f]",
              waypoint0_joints[0], waypoint0_joints[1], waypoint0_joints[2],
              waypoint0_joints[3], waypoint0_joints[4], waypoint0_joints[5]);

  move_group.setJointValueTarget(waypoint0_joints);

  moveit::planning_interface::MoveGroupInterface::Plan plan;
  if (!static_cast<bool>(move_group.plan(plan)))
  {
    RCLCPP_ERROR(LOGGER, "Joint plan failed (direct). Trying intermediate midpoint...");
    // Try midpoint if direct plan fails
    auto current = move_group.getCurrentJointValues();
    if (current.size() == 6)
    {
      std::vector<double> mid(6);
      for (size_t i = 0; i < 6; ++i) mid[i] = current[i] + 0.5 * (waypoint0_joints[i] - current[i]);
      move_group.setJointValueTarget(mid);
      moveit::planning_interface::MoveGroupInterface::Plan mid_plan;
      if (static_cast<bool>(move_group.plan(mid_plan)) && static_cast<bool>(move_group.execute(mid_plan)))
      {
        // Try final again
        move_group.setStartStateToCurrentState();
        move_group.setJointValueTarget(waypoint0_joints);
        if (!static_cast<bool>(move_group.plan(plan)))
        {
          RCLCPP_ERROR(LOGGER, "Final plan still failed after midpoint");
          rclcpp::shutdown();
          return 1;
        }
      }
      else
      {
        RCLCPP_ERROR(LOGGER, "Midpoint plan/exec failed");
        rclcpp::shutdown();
        return 1;
      }
    }
    else
    {
      RCLCPP_ERROR(LOGGER, "No current joint state available");
      rclcpp::shutdown();
      return 1;
    }
  }

  RCLCPP_INFO(LOGGER, "Executing joint move to waypoint 0...");
  RCLCPP_INFO(LOGGER, "Note: Ensure joint_trajectory_controller action server is running (robot driver must be started)");
  
  auto execute_result = move_group.execute(plan);
  if (execute_result != moveit::core::MoveItErrorCode::SUCCESS)
  {
    RCLCPP_ERROR(LOGGER, "Waypoint 0 execution failed!");
    RCLCPP_ERROR(LOGGER, "Error code: %d", execute_result.val);
    RCLCPP_ERROR(LOGGER, "Make sure the robot driver is running and joint_trajectory_controller action server is available.");
    RCLCPP_ERROR(LOGGER, "Check with: ros2 action list | grep joint_trajectory_controller");
    rclcpp::shutdown();
    return 1;
  }

  RCLCPP_INFO(LOGGER, "Waypoint 0 reached successfully. Moving to waypoint 2...");
  
  // Wait a bit before next move
  std::this_thread::sleep_for(std::chrono::milliseconds(500));
  move_group.setStartStateToCurrentState();

  // Plan and execute waypoint 2
  RCLCPP_INFO(LOGGER,
              "Waypoint 2 target joints (rad): [%.4f, %.4f, %.4f, %.4f, %.4f, %.4f]",
              waypoint2_joints[0], waypoint2_joints[1], waypoint2_joints[2],
              waypoint2_joints[3], waypoint2_joints[4], waypoint2_joints[5]);

  move_group.setJointValueTarget(waypoint2_joints);
  moveit::planning_interface::MoveGroupInterface::Plan plan2;
  
  if (!static_cast<bool>(move_group.plan(plan2)))
  {
    RCLCPP_ERROR(LOGGER, "Waypoint 2 planning failed");
    rclcpp::shutdown();
    return 1;
  }

  execute_result = move_group.execute(plan2);
  if (execute_result != moveit::core::MoveItErrorCode::SUCCESS)
  {
    RCLCPP_ERROR(LOGGER, "Waypoint 2 execution failed!");
    RCLCPP_ERROR(LOGGER, "Error code: %d", execute_result.val);
    rclcpp::shutdown();
    return 1;
  }

  RCLCPP_INFO(LOGGER, "All joint moves completed successfully (Waypoint 0 -> Waypoint 2)!");
  rclcpp::shutdown();
  return 0;
}


