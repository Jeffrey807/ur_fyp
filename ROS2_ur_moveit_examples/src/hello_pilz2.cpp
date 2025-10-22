// Pilz Industrial Motion Planner - Custom Motion Sequence
// Waypoint 1 -> Waypoint 2 (+10mm Z) -> 10mm down -> 10mm up -> back to Waypoint 1
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
        "hello_pilz2",
        rclcpp::NodeOptions().automatically_declare_parameters_from_overrides(true));

    // Create a ROS logger
    auto const logger = rclcpp::get_logger("hello_pilz2");

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

    RCLCPP_INFO(logger, "Starting custom motion sequence with Pilz PTP planner");

    // ===== VARIABLES FOR COORDINATES AND ORIENTATION =====
    // Waypoint 1 coordinates (modify these as needed)
    double waypoint1_x = 0.41077980111952567;
    double waypoint1_y = -0.2674434723652385;
    double waypoint1_z = 0.33779377789121495;
    
    // Waypoint 2 coordinates (modify these as needed)
    double waypoint2_x = 0.4197158221097292;  // Change this to your desired X coordinate
    double waypoint2_y = -0.008449074293799572;  // Change this to your desired Y coordinate
    double waypoint2_z = 0.28201676550458815;  // Change this to your desired Z coordinate
    
    // Orientation (same for all waypoints - modify as needed)
    double orientation_x = -0.7192677440288445;
    double orientation_y = 0.6946431472456444;
    double orientation_z = 0.011171124133520415;
    double orientation_w = -0.00012794497354252145;
    
    // Z offset for up/down movements (10mm = 0.01m)
    double z_offset = 0.01;

    // ===== MOTION SEQUENCE =====
    
    // 1. Go to Waypoint 1
    RCLCPP_INFO(logger, "Step 1: Moving to Waypoint 1...");
    auto const waypoint1_pose = [waypoint1_x, waypoint1_y, waypoint1_z, orientation_x, orientation_y, orientation_z, orientation_w]()
    {
        geometry_msgs::msg::Pose msg;
        msg.position.x = waypoint1_x;
        msg.position.y = waypoint1_y;
        msg.position.z = waypoint1_z;
        msg.orientation.x = orientation_x;
        msg.orientation.y = orientation_y;
        msg.orientation.z = orientation_z;
        msg.orientation.w = orientation_w;
        return msg;
    }();
    
    move_group_interface.setPoseTarget(waypoint1_pose, "tool0");
    
    auto const [success1, plan1] = [&move_group_interface]
    {
        moveit::planning_interface::MoveGroupInterface::Plan msg;
        auto const ok = static_cast<bool>(move_group_interface.plan(msg));
        return std::make_pair(ok, msg);
    }();

    if (success1)
    {
        RCLCPP_INFO(logger, "Waypoint 1 planning successful! Executing...");
        move_group_interface.execute(plan1);
        RCLCPP_INFO(logger, "Waypoint 1 reached!");
    }
    else
    {
        RCLCPP_ERROR(logger, "Waypoint 1 planning failed!");
        return -1;
    }

    // 2. Go to Waypoint 2 + 10mm Z
    RCLCPP_INFO(logger, "Step 2: Moving to Waypoint 2 + 10mm Z...");
    auto const waypoint2_pose = [waypoint2_x, waypoint2_y, waypoint2_z, z_offset, orientation_x, orientation_y, orientation_z, orientation_w]()
    {
        geometry_msgs::msg::Pose msg;
        msg.position.x = waypoint2_x;
        msg.position.y = waypoint2_y;
        msg.position.z = waypoint2_z + z_offset;  // +10mm Z
        msg.orientation.x = orientation_x;
        msg.orientation.y = orientation_y;
        msg.orientation.z = orientation_z;
        msg.orientation.w = orientation_w;
        return msg;
    }();
    
    move_group_interface.setPoseTarget(waypoint2_pose, "tool0");
    
    auto const [success2, plan2] = [&move_group_interface]
    {
        moveit::planning_interface::MoveGroupInterface::Plan msg;
        auto const ok = static_cast<bool>(move_group_interface.plan(msg));
        return std::make_pair(ok, msg);
    }();

    if (success2)
    {
        RCLCPP_INFO(logger, "Waypoint 2 + 10mm Z planning successful! Executing...");
        move_group_interface.execute(plan2);
        RCLCPP_INFO(logger, "Waypoint 2 + 10mm Z reached!");
    }
    else
    {
        RCLCPP_ERROR(logger, "Waypoint 2 + 10mm Z planning failed!");
        return -1;
    }

    // 3. Go down 10mm (from current position)
    RCLCPP_INFO(logger, "Step 3: Moving down 10mm...");
    auto const down_pose = [waypoint2_x, waypoint2_y, waypoint2_z, orientation_x, orientation_y, orientation_z, orientation_w]()
    {
        geometry_msgs::msg::Pose msg;
        msg.position.x = waypoint2_x;
        msg.position.y = waypoint2_y;
        msg.position.z = waypoint2_z;  // Original Z position (down 10mm)
        msg.orientation.x = orientation_x;
        msg.orientation.y = orientation_y;
        msg.orientation.z = orientation_z;
        msg.orientation.w = orientation_w;
        return msg;
    }();
    
    move_group_interface.setPoseTarget(down_pose, "tool0");
    
    auto const [success3, plan3] = [&move_group_interface]
    {
        moveit::planning_interface::MoveGroupInterface::Plan msg;
        auto const ok = static_cast<bool>(move_group_interface.plan(msg));
        return std::make_pair(ok, msg);
    }();

    if (success3)
    {
        RCLCPP_INFO(logger, "Down 10mm planning successful! Executing...");
        move_group_interface.execute(plan3);
        RCLCPP_INFO(logger, "Down 10mm completed!");
    }
    else
    {
        RCLCPP_ERROR(logger, "Down 10mm planning failed!");
        return -1;
    }

    // 4. Go up 10mm (back to waypoint 2 + 10mm Z)
    RCLCPP_INFO(logger, "Step 4: Moving up 10mm...");
    auto const up_pose = [waypoint2_x, waypoint2_y, waypoint2_z, z_offset, orientation_x, orientation_y, orientation_z, orientation_w]()
    {
        geometry_msgs::msg::Pose msg;
        msg.position.x = waypoint2_x;
        msg.position.y = waypoint2_y;
        msg.position.z = waypoint2_z + z_offset;  // Back up 10mm
        msg.orientation.x = orientation_x;
        msg.orientation.y = orientation_y;
        msg.orientation.z = orientation_z;
        msg.orientation.w = orientation_w;
        return msg;
    }();
    
    move_group_interface.setPoseTarget(up_pose, "tool0");
    
    auto const [success4, plan4] = [&move_group_interface]
    {
        moveit::planning_interface::MoveGroupInterface::Plan msg;
        auto const ok = static_cast<bool>(move_group_interface.plan(msg));
        return std::make_pair(ok, msg);
    }();

    if (success4)
    {
        RCLCPP_INFO(logger, "Up 10mm planning successful! Executing...");
        move_group_interface.execute(plan4);
        RCLCPP_INFO(logger, "Up 10mm completed!");
    }
    else
    {
        RCLCPP_ERROR(logger, "Up 10mm planning failed!");
        return -1;
    }

    // 5. Return to Waypoint 1
    RCLCPP_INFO(logger, "Step 5: Returning to Waypoint 1...");
    move_group_interface.setPoseTarget(waypoint1_pose, "tool0");
    
    auto const [success5, plan5] = [&move_group_interface]
    {
        moveit::planning_interface::MoveGroupInterface::Plan msg;
        auto const ok = static_cast<bool>(move_group_interface.plan(msg));
        return std::make_pair(ok, msg);
    }();

    if (success5)
    {
        RCLCPP_INFO(logger, "Return to Waypoint 1 planning successful! Executing...");
        move_group_interface.execute(plan5);
        RCLCPP_INFO(logger, "Returned to Waypoint 1!");
    }
    else
    {
        RCLCPP_ERROR(logger, "Return to Waypoint 1 planning failed!");
        return -1;
    }

    RCLCPP_INFO(logger, "Custom motion sequence completed successfully!");

    // Shutdown ROS
    rclcpp::shutdown();
    return 0;
}
