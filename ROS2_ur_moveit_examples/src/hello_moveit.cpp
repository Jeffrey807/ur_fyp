
// example no 1
#include <memory>

#include <rclcpp/rclcpp.hpp>
#include <moveit/move_group_interface/move_group_interface.h>

int main(int argc, char *argv[])
{
    // Initialize ROS and create the Node
    rclcpp::init(argc, argv);
    auto const node = std::make_shared<rclcpp::Node>(
        "hello_moveit",
        rclcpp::NodeOptions().automatically_declare_parameters_from_overrides(true));

    // Create a ROS logger
    auto const logger = rclcpp::get_logger("hello_moveit");

    // Next step goes here
    // Create the MoveIt MoveGroup Interface
    using moveit::planning_interface::MoveGroupInterface;
    auto move_group_interface = MoveGroupInterface(node, "ur_manipulator");
    
    // Configure MoveGroup like in ur_2d_pick.cpp
    move_group_interface.setPoseReferenceFrame("base_link");
    move_group_interface.setEndEffectorLink("tool0");
    move_group_interface.setMaxVelocityScalingFactor(0.1);
    move_group_interface.setMaxAccelerationScalingFactor(0.1);
    move_group_interface.setWorkspace(-2.0, -2.0, 0.0, 2.0, 2.0, 2.0); // Prevent planning below base

    // Set a target Pose - using the same coordinates from ur_2d_pick.cpp TEST TCP
    auto const target_pose = []
    {
        geometry_msgs::msg::Pose msg;
        // Position from ur_2d_pick.cpp TEST TCP
        msg.position.x = 0.41077980111952567;
        msg.position.y = -0.2674434723652385;
        msg.position.z = 0.33779377789121495;
        // Orientation from ur_2d_pick.cpp TEST TCP
        msg.orientation.x = -0.7192677440288445; //0.6946431472456444;
        msg.orientation.y = 0.6946431472456444; //0.7192677440288445;
        msg.orientation.z = 0.011171124133520415;//0.00012794497354252145;
        msg.orientation.w = -0.00012794497354252145;//0.011171124133520415;
        return msg;
    }();
    move_group_interface.setPoseTarget(target_pose, "tool0");

    // Create a plan to that target pose
    auto const [success, plan] = [&move_group_interface]
    {
        moveit::planning_interface::MoveGroupInterface::Plan msg;
        auto const ok = static_cast<bool>(move_group_interface.plan(msg));
        return std::make_pair(ok, msg);
    }();

    // Execute the plan
    if (success)
    {
        move_group_interface.execute(plan);
    }
    else
    {
        RCLCPP_ERROR(logger, "Planing failed!");
    }

    // Shutdown ROS
    rclcpp::shutdown();
    return 0;
}
