// Pilz Industrial Motion Planner example for UR robot
#include <memory>

#include <rclcpp/rclcpp.hpp>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_interface/planning_interface.h>
#include <moveit_msgs/msg/display_robot_state.hpp>
#include <moveit_msgs/msg/display_trajectory.hpp>
#include <moveit_msgs/msg/attached_collision_object.hpp>
#include <moveit_msgs/msg/collision_object.hpp>

int main(int argc, char *argv[])
{
    // Initialize ROS and create the Node
    rclcpp::init(argc, argv);
    auto const node = std::make_shared<rclcpp::Node>(
        "hello_pilz",
        rclcpp::NodeOptions().automatically_declare_parameters_from_overrides(true));

    // Create a ROS logger
    auto const logger = rclcpp::get_logger("hello_pilz");

    // Create the MoveIt MoveGroup Interface
    using moveit::planning_interface::MoveGroupInterface;
    auto move_group_interface = MoveGroupInterface(node, "ur_manipulator");
    
    // Configure MoveGroup for Pilz planner
    move_group_interface.setPoseReferenceFrame("base_link");
    move_group_interface.setEndEffectorLink("tool0");
    move_group_interface.setMaxVelocityScalingFactor(0.1);
    move_group_interface.setMaxAccelerationScalingFactor(0.1);
    move_group_interface.setWorkspace(-2.0, -2.0, 0.0, 2.0, 2.0, 2.0); // Prevent planning below base

    // Set the planning pipeline to use Pilz Industrial Motion Planner
    move_group_interface.setPlanningPipelineId("pilz_industrial_motion_planner");
    move_group_interface.setPlannerId("PTP"); // PTP (Point-to-Point) planner from Pilz

    RCLCPP_INFO(logger, "Using Pilz Industrial Motion Planner with PTP planner");

    // Set a target Pose - using the same coordinates from hello_moveit.cpp
    auto const target_pose = []
    {
        geometry_msgs::msg::Pose msg;
        // Position from hello_moveit.cpp
        msg.position.x = 0.41077980111952567;
        msg.position.y = -0.2674434723652385;
        msg.position.z = 0.33779377789121495;
        // Orientation from hello_moveit.cpp
        msg.orientation.x = -0.7192677440288445;
        msg.orientation.y = 0.6946431472456444;
        msg.orientation.z = 0.011171124133520415;
        msg.orientation.w = -0.00012794497354252145;
        return msg;
    }();
    
    move_group_interface.setPoseTarget(target_pose, "tool0");

    RCLCPP_INFO(logger, "Planning to target pose using Pilz PTP planner...");

    // Create a plan to that target pose using Pilz planner
    auto const [success, plan] = [&move_group_interface]
    {
        moveit::planning_interface::MoveGroupInterface::Plan msg;
        auto const ok = static_cast<bool>(move_group_interface.plan(msg));
        return std::make_pair(ok, msg);
    }();

    // Execute the plan
    if (success)
    {
        RCLCPP_INFO(logger, "Pilz planning successful! Executing trajectory...");
        move_group_interface.execute(plan);
        RCLCPP_INFO(logger, "Trajectory execution completed successfully!");
    }
    else
    {
        RCLCPP_ERROR(logger, "Pilz planning failed!");
    }

    // Demonstrate different Pilz motion types
    RCLCPP_INFO(logger, "Demonstrating different Pilz motion types...");

    // 1. PTP (Point-to-Point) motion - already used above
    RCLCPP_INFO(logger, "PTP motion completed");

    // 2. LIN (Linear) motion example
    RCLCPP_INFO(logger, "Planning LIN (Linear) motion...");
    move_group_interface.setPlannerId("LIN"); // Switch to Linear planner
    
    // Set a different target for linear motion
    auto const lin_target_pose = []
    {
        geometry_msgs::msg::Pose msg;
        msg.position.x = 0.3;
        msg.position.y = 0.0;
        msg.position.z = 0.4;
        msg.orientation.x = 0.0;
        msg.orientation.y = 0.0;
        msg.orientation.z = 0.0;
        msg.orientation.w = 1.0;
        return msg;
    }();
    
    move_group_interface.setPoseTarget(lin_target_pose, "tool0");
    
    auto const [lin_success, lin_plan] = [&move_group_interface]
    {
        moveit::planning_interface::MoveGroupInterface::Plan msg;
        auto const ok = static_cast<bool>(move_group_interface.plan(msg));
        return std::make_pair(ok, msg);
    }();

    if (lin_success)
    {
        RCLCPP_INFO(logger, "LIN planning successful! Executing linear trajectory...");
        move_group_interface.execute(lin_plan);
        RCLCPP_INFO(logger, "Linear trajectory execution completed!");
    }
    else
    {
        RCLCPP_ERROR(logger, "LIN planning failed!");
    }

    // 3. CIRC (Circular) motion example
    RCLCPP_INFO(logger, "Planning CIRC (Circular) motion...");
    move_group_interface.setPlannerId("CIRC"); // Switch to Circular planner
    
    // For circular motion, we need to set a waypoint
    auto const circ_waypoint = []
    {
        geometry_msgs::msg::Pose msg;
        msg.position.x = 0.35;
        msg.position.y = 0.1;
        msg.position.z = 0.4;
        msg.orientation.x = 0.0;
        msg.orientation.y = 0.0;
        msg.orientation.z = 0.0;
        msg.orientation.w = 1.0;
        return msg;
    }();
    
    // Set waypoint for circular motion
    move_group_interface.setPoseTarget(circ_waypoint, "tool0");
    
    auto const [circ_success, circ_plan] = [&move_group_interface]
    {
        moveit::planning_interface::MoveGroupInterface::Plan msg;
        auto const ok = static_cast<bool>(move_group_interface.plan(msg));
        return std::make_pair(ok, msg);
    }();

    if (circ_success)
    {
        RCLCPP_INFO(logger, "CIRC planning successful! Executing circular trajectory...");
        move_group_interface.execute(circ_plan);
        RCLCPP_INFO(logger, "Circular trajectory execution completed!");
    }
    else
    {
        RCLCPP_ERROR(logger, "CIRC planning failed!");
    }

    RCLCPP_INFO(logger, "Pilz Industrial Motion Planner demonstration completed!");

    // Shutdown ROS
    rclcpp::shutdown();
    return 0;
}
