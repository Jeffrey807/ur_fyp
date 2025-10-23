// Pilz Industrial Motion Planner - Custom Motion Sequence with Suction Gripper Control
// Waypoint 1 -> Waypoint 2 (+10mm Z) -> 10mm down -> SUCTION ON -> 10mm up -> SUCTION OFF -> back to Waypoint 1
#include <memory>

#include <rclcpp/rclcpp.hpp>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_interface/planning_interface.h>
#include <moveit_msgs/msg/display_robot_state.hpp>
#include <moveit_msgs/msg/display_trajectory.hpp>
#include <moveit_msgs/msg/attached_collision_object.hpp>
#include <moveit_msgs/msg/collision_object.hpp>
#include <ur_msgs/srv/set_io.hpp>
#include <chrono>
#include <thread>
#include <geometry_msgs/msg/pose.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <std_msgs/msg/header.hpp>

int main(int argc, char *argv[])
{
    // Initialize ROS and create the Node
    rclcpp::init(argc, argv);
    auto const node = std::make_shared<rclcpp::Node>(
        "hello_pilz2_with_suction",
        rclcpp::NodeOptions().automatically_declare_parameters_from_overrides(true));

    // Create a ROS logger
    auto const logger = rclcpp::get_logger("hello_pilz2_with_suction");

    // Create the MoveIt MoveGroup Interface
    using moveit::planning_interface::MoveGroupInterface;
    auto move_group_interface = MoveGroupInterface(node, "ur_manipulator");
    
    // Configure MoveGroup for Pilz planner
    move_group_interface.setPoseReferenceFrame("base_link");
    move_group_interface.setEndEffectorLink("tool0");
    move_group_interface.setMaxVelocityScalingFactor(0.25);
    move_group_interface.setMaxAccelerationScalingFactor(0.25);
    move_group_interface.setWorkspace(-2.0, -2.0, 0.0, 2.0, 2.0, 2.0); // Prevent planning below base

    // Set the planning pipeline to use Pilz Industrial Motion Planner
    move_group_interface.setPlanningPipelineId("pilz_industrial_motion_planner");
    move_group_interface.setPlannerId("PTP"); // PTP (Point-to-Point) planner from Pilz

    // ===== SUCTION GRIPPER IO CONTROL =====
    // Create IO client for controlling suction gripper
    auto io_client = node->create_client<ur_msgs::srv::SetIO>("/io_and_status_controller/set_io");
    
    // Wait for IO service to be available (simplified)
    io_client->wait_for_service(std::chrono::seconds(5));
    RCLCPP_INFO(logger, "IO service ready for commands");

    // Function to control suction gripper (Channel 0) - Simplified
    auto set_suction = [&io_client, &logger](bool suction_on) -> void
    {
        auto req = std::make_shared<ur_msgs::srv::SetIO::Request>();
        req->fun = ur_msgs::srv::SetIO::Request::FUN_SET_DIGITAL_OUT;
        req->pin = 0;  // Channel 0: Suction gripper
        req->state = suction_on ? 1.0 : 0.0;  // 1.0 = suction ON, 0.0 = suction OFF

        RCLCPP_INFO(logger, "Setting Channel 0 (suction): %s", suction_on ? "ON" : "OFF");
        io_client->async_send_request(req);  // Fire and forget
    };

    // Function to control blow gripper (Channel 1) - Simplified
    auto set_blow = [&io_client, &logger](bool blow_on) -> void
    {
        auto req = std::make_shared<ur_msgs::srv::SetIO::Request>();
        req->fun = ur_msgs::srv::SetIO::Request::FUN_SET_DIGITAL_OUT;
        req->pin = 1;  // Channel 1: Blow gripper
        req->state = blow_on ? 1.0 : 0.0;  // 1.0 = blow ON, 0.0 = blow OFF

        RCLCPP_INFO(logger, "Setting Channel 1 (blow): %s", blow_on ? "ON" : "OFF");
        io_client->async_send_request(req);  // Fire and forget
    };

    RCLCPP_INFO(logger, "Starting custom motion sequence with suction gripper control");

    // ===== CONTINUOUS LOOP =====
    int cycle_count = 0;
    while (rclcpp::ok()) {
        cycle_count++;
        RCLCPP_INFO(logger, "=== STARTING CYCLE %d ===", cycle_count);

        // ===== GET WAYPOINT 2 FROM TOPIC =====
        RCLCPP_INFO(logger, "Getting Waypoint 2 coordinates from /pin_housing/target_pose topic...");
        
        // Subscribe to the topic and get one message
        geometry_msgs::msg::PoseStamped::SharedPtr waypoint2_pose_msg = nullptr;
        auto subscription = node->create_subscription<geometry_msgs::msg::PoseStamped>(
            "/pin_housing/target_pose", 10,
            [&waypoint2_pose_msg](const geometry_msgs::msg::PoseStamped::SharedPtr msg) {
                waypoint2_pose_msg = msg;
            });
        
        // Wait for one message (with timeout)
        auto start_time = std::chrono::steady_clock::now();
        while (!waypoint2_pose_msg && rclcpp::ok()) {
            rclcpp::spin_some(node);
            auto current_time = std::chrono::steady_clock::now();
            auto elapsed = std::chrono::duration_cast<std::chrono::seconds>(current_time - start_time);
            if (elapsed.count() > 5) {  // 5 second timeout
                RCLCPP_ERROR(logger, "Timeout waiting for /pin_housing/pixel_pose message!");
                return -1;
            }
            std::this_thread::sleep_for(std::chrono::milliseconds(10));
        }
        
        if (!waypoint2_pose_msg) {
            RCLCPP_ERROR(logger, "Failed to get Waypoint 2 coordinates!");
            return -1;
        }
        
        // Apply coordinate transformation: flip x and y signs, set fixed z
        double waypoint3_x = -waypoint2_pose_msg->pose.position.x;
        double waypoint3_y = -waypoint2_pose_msg->pose.position.y;
        double waypoint3_z = 0.265;  // Fixed z coordinate
        
        RCLCPP_INFO(logger, "Received coordinates: x=%.3f, y=%.3f, z=%.3f", 
                   waypoint2_pose_msg->pose.position.x, waypoint2_pose_msg->pose.position.y, waypoint2_pose_msg->pose.position.z);
        RCLCPP_INFO(logger, "Waypoint 3 coordinates: x=%.3f, y=%.3f, z=%.3f", 
                   waypoint3_x, waypoint3_y, waypoint3_z);

        // ===== INITIALIZATION: Go to Waypoint 1 from current position =====
    RCLCPP_INFO(logger, "Initialization: Moving to Waypoint 1 from current position...");

    // ===== VARIABLES FOR COORDINATES AND ORIENTATION =====
    // Waypoint 1 coordinates (modify these as needed)
    double waypoint1_x = 0.2459503198788797; //0.41077980111952567;
    double waypoint1_y = 0.10942917898100181; //-0.2674434723652385;
    double waypoint1_z = 0.31294021384470305; //0.33779377789121495;
    
    // Waypoint 2 coordinates (fixed intermediate approach position)
    double waypoint2_x = 0.24261504113229335;
    double waypoint2_y = 0.4412653576982412;
    double waypoint2_z = 0.31294021384470305;
    
    // Orientation (same for all waypoints - modify as needed)
    double orientation_x = -0.7192677440288445;
    double orientation_y = 0.6946431472456444;
    double orientation_z = 0.011171124133520415;
    double orientation_w = -0.00012794497354252145;
    
    // Z offset for up/down movements (10mm = 0.01m)
    double z_offset = 0.03;  // 30mm offset

    // ===== INITIALIZATION: Go to Waypoint 1 from current position =====
    RCLCPP_INFO(logger, "Initialization: Moving to Waypoint 1 from current position...");
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
    
    auto const [init_success, init_plan] = [&move_group_interface]
    {
        moveit::planning_interface::MoveGroupInterface::Plan msg;
        auto const ok = static_cast<bool>(move_group_interface.plan(msg));
        return std::make_pair(ok, msg);
    }();

    if (init_success)
    {
        RCLCPP_INFO(logger, "Waypoint 1 planning successful! Executing...");
        auto execute_result = move_group_interface.execute(init_plan);
        if (execute_result == moveit::planning_interface::MoveItErrorCode::SUCCESS)
        {
            RCLCPP_INFO(logger, "Waypoint 1 reached! Starting motion sequence...");
        }
        else
        {
            RCLCPP_ERROR(logger, "Waypoint 1 execution failed! Error code: %d", execute_result.val);
            return -1;
        }
    }
    else
    {
        RCLCPP_ERROR(logger, "Waypoint 1 planning failed!");
        return -1;
    }

    // ===== MOTION SEQUENCE WITH SUCTION CONTROL =====
    
    // 1. Go to Waypoint 1 (already there, but for clarity)
    RCLCPP_INFO(logger, "Step 1: Already at Waypoint 1, proceeding to Step 2...");

    // 2. Go to Waypoint 2 (Intermediate approach position)
    RCLCPP_INFO(logger, "Step 2: Moving to Waypoint 2 (intermediate approach position)...");
    auto const waypoint2_pose = [waypoint2_x, waypoint2_y, waypoint2_z, orientation_x, orientation_y, orientation_z, orientation_w]()
    {
        geometry_msgs::msg::Pose msg;
        msg.position.x = waypoint2_x;
        msg.position.y = waypoint2_y;
        msg.position.z = waypoint2_z;
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
        RCLCPP_INFO(logger, "Waypoint 2 planning successful! Executing...");
        auto execute_result = move_group_interface.execute(plan2);
        if (execute_result == moveit::planning_interface::MoveItErrorCode::SUCCESS)
        {
            RCLCPP_INFO(logger, "Waypoint 2 reached!");
        }
        else
        {
            RCLCPP_ERROR(logger, "Waypoint 2 execution failed! Error code: %d", execute_result.val);
            return -1;
        }
    }
    else
    {
        RCLCPP_ERROR(logger, "Waypoint 2 planning failed!");
        return -1;
    }

    // 3. Go to Waypoint 3 + 30mm (Target position from topic + 30mm Z)
    RCLCPP_INFO(logger, "Step 3: Moving to Waypoint 3 + 30mm Z...");
    auto const waypoint3_high_pose = [waypoint3_x, waypoint3_y, waypoint3_z, z_offset, orientation_x, orientation_y, orientation_z, orientation_w]()
    {
        geometry_msgs::msg::Pose msg;
        msg.position.x = waypoint3_x;
        msg.position.y = waypoint3_y;
        msg.position.z = waypoint3_z + z_offset;  // +30mm Z
        msg.orientation.x = orientation_x;
        msg.orientation.y = orientation_y;
        msg.orientation.z = orientation_z;
        msg.orientation.w = orientation_w;
        return msg;
    }();
    
    move_group_interface.setPoseTarget(waypoint3_high_pose, "tool0");
    
    auto const [success3_high, plan3_high] = [&move_group_interface]
    {
        moveit::planning_interface::MoveGroupInterface::Plan msg;
        auto const ok = static_cast<bool>(move_group_interface.plan(msg));
        return std::make_pair(ok, msg);
    }();

    if (success3_high)
    {
        RCLCPP_INFO(logger, "Waypoint 3 + 30mm Z planning successful! Executing...");
        auto execute_result = move_group_interface.execute(plan3_high);
        if (execute_result == moveit::planning_interface::MoveItErrorCode::SUCCESS)
        {
            RCLCPP_INFO(logger, "Waypoint 3 + 30mm Z reached!");
        }
        else
        {
            RCLCPP_ERROR(logger, "Waypoint 3 + 30mm Z execution failed! Error code: %d", execute_result.val);
            return -1;
        }
    }
    else
    {
        RCLCPP_ERROR(logger, "Waypoint 3 + 30mm Z planning failed!");
        return -1;
    }

    // 4. Go to Waypoint 3 (Target position from topic)
    RCLCPP_INFO(logger, "Step 4: Moving to Waypoint 3 (target position)...");
    auto const waypoint3_pose = [waypoint3_x, waypoint3_y, waypoint3_z, orientation_x, orientation_y, orientation_z, orientation_w]()
    {
        geometry_msgs::msg::Pose msg;
        msg.position.x = waypoint3_x;
        msg.position.y = waypoint3_y;
        msg.position.z = waypoint3_z;
        msg.orientation.x = orientation_x;
        msg.orientation.y = orientation_y;
        msg.orientation.z = orientation_z;
        msg.orientation.w = orientation_w;
        return msg;
    }();
    
    move_group_interface.setPoseTarget(waypoint3_pose, "tool0");
    
    auto const [success3, plan3] = [&move_group_interface]
    {
        moveit::planning_interface::MoveGroupInterface::Plan msg;
        auto const ok = static_cast<bool>(move_group_interface.plan(msg));
        return std::make_pair(ok, msg);
    }();

    if (success3)
    {
        RCLCPP_INFO(logger, "Waypoint 3 planning successful! Executing...");
        auto execute_result = move_group_interface.execute(plan3);
        if (execute_result == moveit::planning_interface::MoveItErrorCode::SUCCESS)
        {
            RCLCPP_INFO(logger, "Waypoint 3 reached!");
        }
        else
        {
            RCLCPP_ERROR(logger, "Waypoint 3 execution failed! Error code: %d", execute_result.val);
            return -1;
        }
    }
    else
    {
        RCLCPP_ERROR(logger, "Waypoint 3 planning failed!");
        return -1;
    }

    // 5. TURN ON SUCTION GRIPPER (grip object at Waypoint 3)
    RCLCPP_INFO(logger, "Step 5: Turning ON suction gripper to grip object...");
    set_suction(true);  // Don't fail if suction doesn't work
    
    // Wait a moment for suction to engage
    std::this_thread::sleep_for(std::chrono::milliseconds(500));
    RCLCPP_INFO(logger, "Suction gripper is now ON and gripping object!");

    // 6. Go up 30mm (back to Waypoint 3 + 30mm) - WITH OBJECT
    RCLCPP_INFO(logger, "Step 6: Moving up 30mm with gripped object...");
    auto const up_pose = [waypoint3_x, waypoint3_y, waypoint3_z, z_offset, orientation_x, orientation_y, orientation_z, orientation_w]()
    {
        geometry_msgs::msg::Pose msg;
        msg.position.x = waypoint3_x;
        msg.position.y = waypoint3_y;
        msg.position.z = waypoint3_z + z_offset;  // Back to Waypoint 3 + 30mm
        msg.orientation.x = orientation_x;
        msg.orientation.y = orientation_y;
        msg.orientation.z = orientation_z;
        msg.orientation.w = orientation_w;
        return msg;
    }();
    
    move_group_interface.setPoseTarget(up_pose, "tool0");
    
    auto const [success5, plan5] = [&move_group_interface]
    {
        moveit::planning_interface::MoveGroupInterface::Plan msg;
        auto const ok = static_cast<bool>(move_group_interface.plan(msg));
        return std::make_pair(ok, msg);
    }();

    if (success5)
    {
        RCLCPP_INFO(logger, "Up 10mm planning successful! Executing...");
        move_group_interface.execute(plan5);
        RCLCPP_INFO(logger, "Up 10mm completed with object!");
    }
    else
    {
        RCLCPP_ERROR(logger, "Up 10mm planning failed!");
        return -1;
    }

    // 7. Return to Waypoint 2 WITH OBJECT
    RCLCPP_INFO(logger, "Step 7: Moving to Waypoint 2 with object...");
    move_group_interface.setPoseTarget(waypoint2_pose, "tool0");
    
    auto const [success7, plan7] = [&move_group_interface]
    {
        moveit::planning_interface::MoveGroupInterface::Plan msg;
        auto const ok = static_cast<bool>(move_group_interface.plan(msg));
        return std::make_pair(ok, msg);
    }();

    if (success7)
    {
        RCLCPP_INFO(logger, "Move to Waypoint 2 planning successful! Executing...");
        move_group_interface.execute(plan7);
        RCLCPP_INFO(logger, "Moved to Waypoint 2 with object!");
    }
    else
    {
        RCLCPP_ERROR(logger, "Move to Waypoint 2 planning failed!");
        return -1;
    }

    // 8. Return to Waypoint 1 WITH OBJECT
    RCLCPP_INFO(logger, "Step 8: Moving to Waypoint 1 with object...");
    move_group_interface.setPoseTarget(waypoint1_pose, "tool0");
    
    auto const [success8, plan8] = [&move_group_interface]
    {
        moveit::planning_interface::MoveGroupInterface::Plan msg;
        auto const ok = static_cast<bool>(move_group_interface.plan(msg));
        return std::make_pair(ok, msg);
    }();

    if (success8)
    {
        RCLCPP_INFO(logger, "Move to Waypoint 1 planning successful! Executing...");
        move_group_interface.execute(plan8);
        RCLCPP_INFO(logger, "Moved to Waypoint 1 with object!");
    }
    else
    {
        RCLCPP_ERROR(logger, "Move to Waypoint 1 planning failed!");
        return -1;
    }

    // 9. TURN OFF SUCTION + TURN ON BLOW (release object with blow)
    RCLCPP_INFO(logger, "Step 9: Turning OFF Channel 0 (suction) and ON Channel 1 (blow) to release object...");
    set_suction(false);  // Channel 0 LOW (suction off)
    std::this_thread::sleep_for(std::chrono::milliseconds(200));
    set_blow(true);      // Channel 1 ON (blow on)
    
    // Wait 1 second for object to be released
    std::this_thread::sleep_for(std::chrono::milliseconds(1000));
    RCLCPP_INFO(logger, "Object released with blow!");

    // 10. TURN OFF BLOW
    RCLCPP_INFO(logger, "Step 10: Turning OFF Channel 1 (blow)...");
    set_blow(false);     // Channel 1 LOW (blow off)
        std::this_thread::sleep_for(std::chrono::milliseconds(200));
        RCLCPP_INFO(logger, "Channel 1 (blow) turned OFF - sequence completed!");

        RCLCPP_INFO(logger, "=== CYCLE %d COMPLETED SUCCESSFULLY! ===", cycle_count);
        
        // Wait 2 seconds before starting next cycle
        RCLCPP_INFO(logger, "Waiting 2 seconds before starting next cycle... (Press Ctrl+C to exit)");
        std::this_thread::sleep_for(std::chrono::milliseconds(2000));
    }

    RCLCPP_INFO(logger, "Continuous motion sequence terminated. Total cycles completed: %d", cycle_count);

    // Shutdown ROS
    rclcpp::shutdown();
    return 0;
}
