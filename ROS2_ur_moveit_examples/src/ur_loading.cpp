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
#include <cmath>
#include <geometry_msgs/msg/pose.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <std_msgs/msg/header.hpp>
#include <std_msgs/msg/bool.hpp>
#include <std_msgs/msg/float32_multi_array.hpp>
#include <std_msgs/msg/string.hpp>

int main(int argc, char *argv[])
{
    // Initialize ROS and create the Node
    rclcpp::init(argc, argv);
    auto const node = std::make_shared<rclcpp::Node>(
        "ur_loading",
        rclcpp::NodeOptions().automatically_declare_parameters_from_overrides(true));

    // Create a ROS logger
    auto const logger = rclcpp::get_logger("ur_loading");

    // Create the MoveIt MoveGroup Interface
    using moveit::planning_interface::MoveGroupInterface;
    auto move_group_interface = MoveGroupInterface(node, "ur_manipulator");
    
    // Configure MoveGroup for Pilz planner
    move_group_interface.setPoseReferenceFrame("base_link");
    move_group_interface.setEndEffectorLink("tool0");
    move_group_interface.setMaxVelocityScalingFactor(1);  // Reduced for reliability
    move_group_interface.setMaxAccelerationScalingFactor(1);  // Reduced for reliability
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
    
    // ===== JOINT MOVEMENT FUNCTION (using OMPL for joint space) =====
    auto plan_and_execute_joint = [&move_group_interface, &logger](const std::vector<double>& joint_positions, const std::string& description) -> bool
    {
        RCLCPP_INFO(logger, "Joint move: %s", description.c_str());
        RCLCPP_INFO(logger, "Target joints (rad): [%.4f, %.4f, %.4f, %.4f, %.4f, %.4f]",
                    joint_positions[0], joint_positions[1], joint_positions[2], 
                    joint_positions[3], joint_positions[4], joint_positions[5]);
        
        // Switch to OMPL for joint moves
        move_group_interface.setPlanningPipelineId("ompl");
        move_group_interface.setPlannerId("");  // Use default OMPL planner
        move_group_interface.setPlanningTime(5.0);
        move_group_interface.setNumPlanningAttempts(5);
        
        // Ensure start state is current
        move_group_interface.startStateMonitor();
        std::this_thread::sleep_for(std::chrono::milliseconds(300));
        move_group_interface.setStartStateToCurrentState();
        
        move_group_interface.setJointValueTarget(joint_positions);
        
        moveit::planning_interface::MoveGroupInterface::Plan plan;
        if (!static_cast<bool>(move_group_interface.plan(plan)))
        {
            RCLCPP_WARN(logger, "Direct joint plan failed. Trying intermediate waypoint...");
            // Try intermediate waypoint if direct plan fails
            auto current = move_group_interface.getCurrentJointValues();
            if (current.size() == 6)
            {
                std::vector<double> mid(6);
                for (size_t i = 0; i < 6; ++i) 
                    mid[i] = current[i] + 0.5 * (joint_positions[i] - current[i]);
                
                move_group_interface.setJointValueTarget(mid);
                moveit::planning_interface::MoveGroupInterface::Plan mid_plan;
                if (static_cast<bool>(move_group_interface.plan(mid_plan)) && 
                    move_group_interface.execute(mid_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS)
                {
                    // Now try final
                    move_group_interface.setStartStateToCurrentState();
                    move_group_interface.setJointValueTarget(joint_positions);
                    if (!static_cast<bool>(move_group_interface.plan(plan)))
                    {
                        RCLCPP_ERROR(logger, "Joint move failed after intermediate waypoint");
                        return false;
                    }
                }
                else
                {
                    RCLCPP_ERROR(logger, "Intermediate waypoint move failed");
                    return false;
                }
            }
            else
            {
                RCLCPP_ERROR(logger, "Joint move planning failed - no current joint state");
                return false;
            }
        }
        
        RCLCPP_INFO(logger, "Joint move plan computed. Executing...");
        auto execute_result = move_group_interface.execute(plan);
        if (execute_result != moveit::planning_interface::MoveItErrorCode::SUCCESS)
        {
            RCLCPP_ERROR(logger, "Joint move execution failed! Error code: %d", execute_result.val);
            return false;
        }
        RCLCPP_INFO(logger, "Joint move completed successfully");
        
        // Switch back to Pilz for TCP moves
        move_group_interface.setPlanningPipelineId("pilz_industrial_motion_planner");
        move_group_interface.setPlannerId("PTP");
        
        return true;
    };

    // ===== WAYPOINT 0 JOINT POSITIONS =====
    // Format: [shoulder_pan, shoulder_lift, elbow, wrist_1, wrist_2, wrist_3]
    std::vector<double> waypoint0_joints = {
        -0.0036109129535120132,    // shoulder_pan_joint
        -2.1629932562457483,       // shoulder_lift_joint  
        2.0472545623779297,        // elbow_joint
        -1.4392817656146448,       // wrist_1_joint
        -1.5554221312152308,       // wrist_2_joint
        -0.037923638020650685      // wrist_3_joint
    };
    // ===== QUATERNION ROTATION FUNCTION =====
    auto apply_yaw_rotation = [](double yaw_degrees, double orig_x, double orig_y, double orig_z, double orig_w) -> std::tuple<double, double, double, double> {
        // Convert yaw degrees to radians
        double yaw_rad = yaw_degrees * M_PI / 180.0;
        
        // Create rotation quaternion around Z-axis
        double cos_half_yaw = cos(yaw_rad / 2.0);
        double sin_half_yaw = sin(yaw_rad / 2.0);
        
        double rot_x = 0.0;
        double rot_y = 0.0;
        double rot_z = sin_half_yaw;
        double rot_w = cos_half_yaw;
        
        // Apply rotation to original quaternion
        double new_x = orig_w * rot_x + orig_x * rot_w + orig_y * rot_z - orig_z * rot_y;
        double new_y = orig_w * rot_y - orig_x * rot_z + orig_y * rot_w + orig_z * rot_x;
        double new_z = orig_w * rot_z + orig_x * rot_y - orig_y * rot_x + orig_z * rot_w;
        double new_w = orig_w * rot_w - orig_x * rot_x - orig_y * rot_y - orig_z * rot_z;
        
        return std::make_tuple(new_x, new_y, new_z, new_w);
    };

    // ===== RED DETECTION WAIT FUNCTION =====
    auto wait_for_red_detected = [&node, &logger]() -> bool {
        RCLCPP_INFO(logger, "Waiting for red DETECTED for 1 second...");
        
        // Subscribe to red detection topic
        std_msgs::msg::Bool::SharedPtr latest_red_msg = nullptr;
        auto red_subscription = node->create_subscription<std_msgs::msg::Bool>(
            "/red_detected", 10,
            [&latest_red_msg](const std_msgs::msg::Bool::SharedPtr msg) {
                latest_red_msg = msg;
            });
        
        auto start_time = std::chrono::steady_clock::now();
        auto red_detected_start = std::chrono::steady_clock::now();
        bool red_detected_timer_started = false;
        
        while (rclcpp::ok()) {
            rclcpp::spin_some(node);
            
            if (latest_red_msg) {
                bool red_detected = latest_red_msg->data;
                
                if (red_detected) {
                    // Red detected - start/continue timer
                    if (!red_detected_timer_started) {
                        red_detected_start = std::chrono::steady_clock::now();
                        red_detected_timer_started = true;
                        RCLCPP_INFO(logger, "Red DETECTED - starting 1 second timer...");
                    } else {
                        // Check if 1 second has passed
                        auto current_time = std::chrono::steady_clock::now();
                        auto elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(current_time - red_detected_start);
                        
                        if (elapsed.count() >= 1000) {  // 1 second
                            RCLCPP_INFO(logger, "Red DETECTED for 1 second - proceeding with motion!");
                            return true;
                        }
                    }
                } else {
                    // Red not detected - reset timer
                    if (red_detected_timer_started) {
                        RCLCPP_INFO(logger, "Red NOT detected - resetting timer...");
                        red_detected_timer_started = false;
                    }
                }
            }
            
            // Check for overall timeout (30 seconds)
            auto current_time = std::chrono::steady_clock::now();
            auto total_elapsed = std::chrono::duration_cast<std::chrono::seconds>(current_time - start_time);
            if (total_elapsed.count() > 30) {
                RCLCPP_ERROR(logger, "Timeout waiting for red DETECTED!");
                return false;
            }
            
            std::this_thread::sleep_for(std::chrono::milliseconds(50));
        }
        
        return false;
    };

    // ===== CONTINUOUS LOOP =====
    int cycle_count = 0;
    while (rclcpp::ok()) {
        cycle_count++;
        RCLCPP_INFO(logger, "=== STARTING CYCLE %d ===", cycle_count);

        // ===== WAIT FOR RED DETECTED =====
        if (!wait_for_red_detected()) {
            RCLCPP_ERROR(logger, "Failed to get red DETECTED status - skipping cycle");
            continue;
        }
        
        // ===== WAYPOINT 0: Move to joint positions first (using OMPL) =====
        RCLCPP_INFO(logger, "=== WAYPOINT 0: Moving to specified joint positions (OMPL) ===");
        if (!plan_and_execute_joint(waypoint0_joints, "Waypoint 0 joint positions")) {
            RCLCPP_ERROR(logger, "Waypoint 0 joint move failed - skipping cycle");
            continue;
        }
        RCLCPP_INFO(logger, "Waypoint 0 reached successfully! Switching to Pilz for TCP moves...");
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

        // ===== READ YAW ORIENTATION FROM TOPIC =====
        RCLCPP_INFO(logger, "Reading yaw orientation from /pin_housing/pixel_pose topic...");
        
        // Subscribe to pixel pose topic to get yaw orientation
        std_msgs::msg::Float32MultiArray::SharedPtr pixel_pose_msg = nullptr;
        auto pixel_subscription = node->create_subscription<std_msgs::msg::Float32MultiArray>(
            "/pin_housing/pixel_pose", 10,
            [&pixel_pose_msg](const std_msgs::msg::Float32MultiArray::SharedPtr msg) {
                pixel_pose_msg = msg;
            });
        
        // Wait for pixel pose message (with timeout)
        auto pixel_start_time = std::chrono::steady_clock::now();
        while (!pixel_pose_msg && rclcpp::ok()) {
            rclcpp::spin_some(node);
            auto pixel_current_time = std::chrono::steady_clock::now();
            auto pixel_elapsed = std::chrono::duration_cast<std::chrono::seconds>(pixel_current_time - pixel_start_time);
            if (pixel_elapsed.count() > 3) {  // 3 second timeout
                RCLCPP_ERROR(logger, "Timeout waiting for /pin_housing/pixel_pose message!");
                return -1;
            }
            std::this_thread::sleep_for(std::chrono::milliseconds(10));
        }
        
        if (!pixel_pose_msg || pixel_pose_msg->data.size() < 3) {
            RCLCPP_ERROR(logger, "Failed to get yaw orientation from pixel pose!");
            return -1;
        }
        
        // Extract yaw orientation (3rd element: [u, v, yaw_deg])
        double current_yaw = pixel_pose_msg->data[2];
        double yaw_correction = 90.0 - current_yaw;  // Calculate correction to make yaw = 90°
        
        RCLCPP_INFO(logger, "Current yaw: %.2f°, Yaw correction needed: %.2f°", current_yaw, yaw_correction);

        // ===== INITIALIZATION: Go to Waypoint 1 from current position =====
    RCLCPP_INFO(logger, "Initialization: Moving to Waypoint 1 from current position...");

    // ===== VARIABLES FOR COORDINATES AND ORIENTATION =====
    // Waypoint 1 coordinates (modify these as needed)
    double waypoint1_x = 0.2089260118102154;
    double waypoint1_y = 0.13332203698052786;
    double waypoint1_z = 0.3190106214221309;
    
    // Waypoint 2 coordinates (original intermediate approach position)
    double waypoint2_x = 0.20891027198833542;
    double waypoint2_y = 0.4492916032052308;
    double waypoint2_z = 0.31895691845638846;
    
    // Waypoint 4 coordinates (orientation fixing position)
    double waypoint4_x = 0.2540252815331876;
    double waypoint4_y = 0.4325847235030795;
    double waypoint4_z = 0.29099281780614344;
    
    // Waypoint 5 coordinates (first intermediate position)
    double waypoint5_x = 0.2522885702097998;
    double waypoint5_y = 0.4396583465695335;
    double waypoint5_z = 0.2627460983462466;
    
    // Waypoint 6 coordinates (final release position)
    double waypoint6_x = 0.27415139420948165;
    double waypoint6_y = 0.42230192199148725;
    double waypoint6_z = 0.26283116855426103;
    
    // Additional waypoints (7-10) to visit after suction OFF and first 30mm up
    double waypoint7_x = 0.2621176624669639;
    double waypoint7_y = 0.4557977169287189;
    double waypoint7_z = 0.25800820385423406;
    
    double waypoint8_x = 0.26214343803528317;
    double waypoint8_y = 0.438323873193058;
    double waypoint8_z = 0.2580415716957996;
    
    double waypoint9_x = 0.22916806412457758;
    double waypoint9_y = 0.42393476759245424;
    double waypoint9_z = 0.2580644333402114;
    
    double waypoint10_x = 0.2390634480629392;
    double waypoint10_y = 0.4239284140560104;
    double waypoint10_z = 0.25803515829781354;
    
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

    // 2. Go to Waypoint 2 (Original intermediate approach position)
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
        RCLCPP_INFO(logger, "Up 30mm planning successful! Executing...");
        auto execute_result = move_group_interface.execute(plan5);
        if (execute_result == moveit::planning_interface::MoveItErrorCode::SUCCESS)
        {
            RCLCPP_INFO(logger, "Up 30mm completed with object!");
        }
        else
        {
            RCLCPP_ERROR(logger, "Up 30mm execution failed! Error code: %d", execute_result.val);
            return -1;
        }
    }
    else
    {
        RCLCPP_ERROR(logger, "Up 30mm planning failed!");
        return -1;
    }

    // 7. Return to Waypoint 2 WITH OBJECT
    RCLCPP_INFO(logger, "Step 7: Returning to Waypoint 2 with gripped object...");
    move_group_interface.setPoseTarget(waypoint2_pose, "tool0");
    
    auto const [success7, plan7] = [&move_group_interface]
    {
        moveit::planning_interface::MoveGroupInterface::Plan msg;
        auto const ok = static_cast<bool>(move_group_interface.plan(msg));
        return std::make_pair(ok, msg);
    }();

    if (success7)
    {
        RCLCPP_INFO(logger, "Waypoint 2 planning successful! Executing...");
        auto execute_result = move_group_interface.execute(plan7);
        if (execute_result == moveit::planning_interface::MoveItErrorCode::SUCCESS)
        {
            RCLCPP_INFO(logger, "Waypoint 2 reached with object!");
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

    // 8. Return to Waypoint 1 WITH OBJECT
    RCLCPP_INFO(logger, "Step 8: Returning to Waypoint 1 with gripped object...");
    move_group_interface.setPoseTarget(waypoint1_pose, "tool0");
    
    auto const [success8, plan8] = [&move_group_interface]
    {
        moveit::planning_interface::MoveGroupInterface::Plan msg;
        auto const ok = static_cast<bool>(move_group_interface.plan(msg));
        return std::make_pair(ok, msg);
    }();

    if (success8)
    {
        RCLCPP_INFO(logger, "Waypoint 1 planning successful! Executing...");
        auto execute_result = move_group_interface.execute(plan8);
        if (execute_result == moveit::planning_interface::MoveItErrorCode::SUCCESS)
        {
            RCLCPP_INFO(logger, "Waypoint 1 reached with object!");
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

    // 9. TURN OFF SUCTION (wait 1 second)
    RCLCPP_INFO(logger, "Step 9: Turning OFF Channel 0 (suction) and waiting 1 second...");
    set_suction(false);  // Channel 0 LOW (suction off)
    
    // Wait 1 second for object to be released
    std::this_thread::sleep_for(std::chrono::milliseconds(1000));
    RCLCPP_INFO(logger, "Object released! Suction OFF.");

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
