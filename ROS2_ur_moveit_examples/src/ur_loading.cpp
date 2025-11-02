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
#include <algorithm>
#include <cctype>
#include <sstream>

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
    
    // ===== PLC INTERLOCK PUBLISHERS =====
    // Publisher for signaling Python interlock node to initialize (set coil 8194 to LOW at startup)
    auto interlock_init_publisher = node->create_publisher<std_msgs::msg::Bool>("/init_interlock_startup", 10);
    // Publisher for signaling Python interlock node to set coil LOW when red is detected (Step 7.5)
    auto interlock_set_low_publisher = node->create_publisher<std_msgs::msg::Bool>("/set_interlock_low", 10);
    // Publisher for signaling Python interlock node to set coil HIGH when red is NOT detected (Step 7.5)
    auto interlock_set_high_publisher = node->create_publisher<std_msgs::msg::Bool>("/set_interlock_high", 10);
    // Publisher for signaling Python interlock node to check red detection after Waypoint 2
    auto interlock_reset_publisher = node->create_publisher<std_msgs::msg::Bool>("/reset_interlock_after_waypoint2", 10);
    RCLCPP_INFO(logger, "PLC interlock publishers created");
    
    // ===== INITIALIZE PLC INTERLOCK AT STARTUP =====
    // Set PLC coil 8194 to LOW (0) when program starts - unlock other robot
    RCLCPP_INFO(logger, "Initializing PLC interlock: Setting coil 8194 to LOW (0) at startup...");
    std_msgs::msg::Bool init_msg;
    init_msg.data = true;
    interlock_init_publisher->publish(init_msg);
    std::this_thread::sleep_for(std::chrono::milliseconds(100));  // Give time for message to be sent
    RCLCPP_INFO(logger, "PLC interlock initialization signal sent - coil 8194 should be set to LOW");

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
        
        // Ensure start state is current (no delay needed - state is fresh from previous execution)
        move_group_interface.startStateMonitor();
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

    
    // ===== YAW CORRECTION CALCULATION FUNCTION =====
    // Calculates yaw correction based on orientation class and current yaw angle
    // Returns correction angle in degrees: positive = CW, negative = CCW
    auto calculate_yaw_correction = [](const std::string& orientation_class, double current_yaw) -> double {
        // Normalize angle to -180 to 180 range
        double theta = current_yaw;
        while (theta > 180.0) theta -= 360.0;
        while (theta < -180.0) theta += 360.0;
        
        // Convert orientation class to lowercase for case-insensitive matching
        std::string orientation = orientation_class;
        std::transform(orientation.begin(), orientation.end(), orientation.begin(), ::tolower);
        
        double correction = 0.0;
        
        // Top Right
        if (orientation.find("topright") != std::string::npos) {
            if (theta < 90.0) {
                correction = 90.0 - theta;  // CW (+)
            } else if (std::abs(theta - 90.0) < 0.1) {
                correction = 0.0;  // No rotation
            } else {  // theta > 90.0
                correction = -(theta - 90.0);  // CCW (-)
            }
        }
        // Top Bottom
        else if (orientation.find("topbottom") != std::string::npos) {
            if (theta > 90.0) {
                correction = -(theta - 90.0);  // CCW (-)
            } else if (std::abs(theta) < 0.1 || std::abs(std::abs(theta) - 180.0) < 0.1) {
                correction = -90.0;  // CCW (-) 90 degrees
            } else {  // theta < 90.0
                correction = -(theta + 90.0);  // CCW (-)
            }
        }
        // Top Left
        else if (orientation.find("topleft") != std::string::npos) {
            if (theta < 90.0) {
                correction = -(theta + 90.0);  // CCW (-)
            } else if (std::abs(theta - 90.0) < 0.1) {
                correction = -180.0;  // CCW (-) 180 degrees
            } else {  // theta > 90.0
                correction = (270.0 - theta);  // CW (+)
            }
        }
        // Top Top
        else if (orientation.find("toptop") != std::string::npos) {
            if (theta > 90.0) {
                correction = (270.0 - theta);  // CW (+)
            } else if (std::abs(theta) < 0.1 || std::abs(std::abs(theta) - 180.0) < 0.1) {
                correction = 90.0;  // CW (+) 90 degrees
            } else {  // theta < 90.0
                correction = (90.0 - theta);  // CW (+)
            }
        }
        // Bottom Right
        else if (orientation.find("bottomright") != std::string::npos) {
            if (theta < 90.0) {
                correction = -(theta + 90.0);  // CCW (-)
            } else if (std::abs(theta - 90.0) < 0.1) {
                correction = 180.0;  // CW (+) 180 degrees
            } else {  // theta > 90.0
                correction = (270.0 - theta);  // CW (+)
            }
        }
        // Bottom Bottom
        else if (orientation.find("bottombottom") != std::string::npos) {
            if (theta > 90.0) {
                correction = (270.0 - theta);  // CW (+)
            } else if (std::abs(theta) < 0.1 || std::abs(std::abs(theta) - 180.0) < 0.1) {
                correction = 90.0;  // CW (+) 90 degrees
            } else {  // theta < 90.0
                correction = (90.0 - theta);  // CW (+)
            }
        }
        // Bottom Left
        else if (orientation.find("bottomleft") != std::string::npos) {
            if (theta < 90.0) {
                correction = (90.0 - theta);  // CW (+)
            } else if (std::abs(theta - 90.0) < 0.1) {
                correction = 0.0;  // No rotation
            } else {  // theta > 90.0
                correction = -(theta - 90.0);  // CCW (-)
            }
        }
        // Bottom Top
        else if (orientation.find("bottomtop") != std::string::npos) {
            if (theta > 90.0) {
                correction = -(theta - 90.0);  // CCW (-)
            } else if (std::abs(theta) < 0.1 || std::abs(std::abs(theta) - 180.0) < 0.1) {
                correction = -90.0;  // CCW (-) 90 degrees
            } else {  // theta < 90.0
                correction = -(theta + 90.0);  // CCW (-)
            }
        }
        else {
            // Unknown orientation - return 0 (no correction)
            return 0.0;
        }
        
        return correction;
    };

    // ===== QUATERNION ROTATION FUNCTION =====
    // Note: yaw_degrees should be positive for CW, negative for CCW
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
    auto wait_for_red_detected = [&node, &logger, &interlock_set_high_publisher]() -> bool {
        RCLCPP_INFO(logger, "Waiting for red DETECTED for 2 seconds...");
        
        // Subscribe to red detection topic
        std_msgs::msg::Bool::SharedPtr latest_red_msg = nullptr;
        auto red_subscription = node->create_subscription<std_msgs::msg::Bool>(
            "/red_detected", 10,
            [&latest_red_msg](const std_msgs::msg::Bool::SharedPtr msg) {
                latest_red_msg = msg;
            });
        
        auto red_detected_start = std::chrono::steady_clock::now();
        auto red_not_detected_start = std::chrono::steady_clock::now();
        bool red_detected_timer_started = false;
        bool red_not_detected_timer_started = false;
        bool high_signal_sent = false;  // Track if we've already sent HIGH signal
        
        while (rclcpp::ok()) {
            rclcpp::spin_some(node);
            
            if (latest_red_msg) {
                bool red_detected = latest_red_msg->data;
                
                if (red_detected) {
                    // Red detected - start/continue timer for red detection
                    if (!red_detected_timer_started) {
                        red_detected_start = std::chrono::steady_clock::now();
                        red_detected_timer_started = true;
                        red_not_detected_timer_started = false;  // Reset NOT detected timer
                        high_signal_sent = false;  // Reset HIGH signal flag when red is detected
                        RCLCPP_INFO(logger, "Red DETECTED - starting 2 second timer...");
                    } else {
                        // Check if 2 seconds have passed
                        auto current_time = std::chrono::steady_clock::now();
                        auto elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(current_time - red_detected_start);
                        
                        if (elapsed.count() >= 2000) {  // 2 seconds
                            RCLCPP_INFO(logger, "Red DETECTED for 2 seconds - proceeding with motion!");
                            return true;
                        }
                    }
                } else {
                    // Red not detected - start timer for NOT detected
                    if (red_detected_timer_started) {
                        RCLCPP_INFO(logger, "Red NOT detected - resetting red detection timer...");
                        red_detected_timer_started = false;
                    }
                    
                    // Start timer for red NOT detected
                    if (!red_not_detected_timer_started) {
                        red_not_detected_start = std::chrono::steady_clock::now();
                        red_not_detected_timer_started = true;
                        RCLCPP_INFO(logger, "Red NOT detected - starting 1 second timer...");
                    } else {
                        // Check if 1 second has passed with red NOT detected
                        auto current_time = std::chrono::steady_clock::now();
                        auto elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(current_time - red_not_detected_start);
                        
                        if (elapsed.count() >= 1000 && !high_signal_sent) {  // 1 second and haven't sent signal yet
                            RCLCPP_INFO(logger, "Red NOT detected for 1 second - Setting PLC interlock coil 8194 to HIGH (1)...");
                            std_msgs::msg::Bool set_high_msg;
                            set_high_msg.data = true;
                            interlock_set_high_publisher->publish(set_high_msg);
                            high_signal_sent = true;
                            RCLCPP_INFO(logger, "Interlock HIGH signal sent - Other robot blocked");
                        }
                    }
                }
            } else {
                // No red detection message received yet - start timer for NOT detected
                if (!red_not_detected_timer_started) {
                    red_not_detected_start = std::chrono::steady_clock::now();
                    red_not_detected_timer_started = true;
                    RCLCPP_INFO(logger, "No red detection data - starting 1 second timer...");
                } else {
                    // Check if 1 second has passed without red detection message
                    auto current_time = std::chrono::steady_clock::now();
                    auto elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(current_time - red_not_detected_start);
                    
                    if (elapsed.count() >= 1000 && !high_signal_sent) {  // 1 second and haven't sent signal yet
                        RCLCPP_INFO(logger, "No red detection data for 1 second - Setting PLC interlock coil 8194 to HIGH (1)...");
                        std_msgs::msg::Bool set_high_msg;
                        set_high_msg.data = true;
                        interlock_set_high_publisher->publish(set_high_msg);
                        high_signal_sent = true;
                        RCLCPP_INFO(logger, "Interlock HIGH signal sent - Other robot blocked");
                    }
                }
            }
            
            std::this_thread::sleep_for(std::chrono::milliseconds(50));
        }
        
        return false;
    };

    // ===== CHECK RED NOT DETECTED FOR 1 SECOND (for Step 12.5) =====
    auto check_red_not_detected_for_1_second = [&node, &logger, &interlock_set_high_publisher]() -> void {
        RCLCPP_INFO(logger, "Checking if red is NOT detected for 1 second...");
        
        // Subscribe to red detection topic
        std_msgs::msg::Bool::SharedPtr latest_red_msg = nullptr;
        auto red_subscription = node->create_subscription<std_msgs::msg::Bool>(
            "/red_detected", 10,
            [&latest_red_msg](const std_msgs::msg::Bool::SharedPtr msg) {
                latest_red_msg = msg;
            });
        
        auto red_not_detected_start = std::chrono::steady_clock::now();
        bool red_not_detected_timer_started = false;
        bool high_signal_sent = false;
        
        // Wait for 1 second checking red detection status
        auto check_start = std::chrono::steady_clock::now();
        while (rclcpp::ok()) {
            rclcpp::spin_some(node);
            
            // Check if we've been checking for more than 2 seconds (safety timeout)
            auto current_time = std::chrono::steady_clock::now();
            auto total_elapsed = std::chrono::duration_cast<std::chrono::seconds>(current_time - check_start);
            if (total_elapsed.count() > 2) {
                break;  // Safety timeout
            }
            
            if (latest_red_msg) {
                bool red_detected = latest_red_msg->data;
                
                if (!red_detected) {
                    // Red not detected - start timer
                    if (!red_not_detected_timer_started) {
                        red_not_detected_start = std::chrono::steady_clock::now();
                        red_not_detected_timer_started = true;
                        RCLCPP_INFO(logger, "Red NOT detected - starting 1 second timer...");
                    } else {
                        // Check if 1 second has passed with red NOT detected
                        auto elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(current_time - red_not_detected_start);
                        
                        if (elapsed.count() >= 1000 && !high_signal_sent) {  // 1 second
                            RCLCPP_INFO(logger, "Red NOT detected for 1 second - Setting PLC interlock coil 8194 to HIGH (1)...");
                            std_msgs::msg::Bool set_high_msg;
                            set_high_msg.data = true;
                            interlock_set_high_publisher->publish(set_high_msg);
                            high_signal_sent = true;
                            RCLCPP_INFO(logger, "Interlock HIGH signal sent - Other robot blocked");
                            break;  // Exit after sending signal
                        }
                    }
                } else {
                    // Red detected - reset timer
                    if (red_not_detected_timer_started) {
                        RCLCPP_INFO(logger, "Red detected - resetting NOT detected timer...");
                        red_not_detected_timer_started = false;
                        high_signal_sent = false;
                    }
                }
            } else {
                // No red detection message - treat as NOT detected
                if (!red_not_detected_timer_started) {
                    red_not_detected_start = std::chrono::steady_clock::now();
                    red_not_detected_timer_started = true;
                    RCLCPP_INFO(logger, "No red detection data - starting 1 second timer...");
                } else {
                    auto elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(current_time - red_not_detected_start);
                    
                    if (elapsed.count() >= 1000 && !high_signal_sent) {  // 1 second
                        RCLCPP_INFO(logger, "No red detection data for 1 second - Setting PLC interlock coil 8194 to HIGH (1)...");
                        std_msgs::msg::Bool set_high_msg;
                        set_high_msg.data = true;
                        interlock_set_high_publisher->publish(set_high_msg);
                        high_signal_sent = true;
                        RCLCPP_INFO(logger, "Interlock HIGH signal sent - Other robot blocked");
                        break;  // Exit after sending signal
                    }
                }
            }
            
            std::this_thread::sleep_for(std::chrono::milliseconds(50));
        }
        
        if (!high_signal_sent) {
            RCLCPP_INFO(logger, "Red detection check completed - Red was detected, keeping interlock LOW");
        }
    };

    // ===== WAYPOINT 0 JOINT POSITIONS =====
    // User provided order: [shoulder_lift, elbow, wrist_1, wrist_2, wrist_3, shoulder_pan]
    // MoveIt order: [shoulder_pan, shoulder_lift, elbow, wrist_1, wrist_2, wrist_3]
    std::vector<double> waypoint0_joints = {
        -0.3609736601458948,       // shoulder_pan_joint (last in user order)
        -2.106746021901266,         // shoulder_lift_joint (first in user order)
        2.189321517944336,          // elbow_joint
        -1.2694667021380823,        // wrist_1_joint
        -0.634470287953512,         // wrist_2_joint
        0.08673021197319031         // wrist_3_joint
    };

    // ===== WAYPOINT 0.1 JOINT POSITIONS =====
    // User provided order: [shoulder_lift, elbow, wrist_1, wrist_2, wrist_3, shoulder_pan]
    // MoveIt order: [shoulder_pan, shoulder_lift, elbow, wrist_1, wrist_2, wrist_3]
    std::vector<double> waypoint0_1_joints = {
        0.46626538038253784,        // shoulder_pan_joint (last in user order)
        -2.2785919348346155,        // shoulder_lift_joint (first in user order)
        2.4049112796783447,         // elbow_joint
        -1.941827122365133,         // wrist_1_joint
        -1.8886244932757776,        // wrist_2_joint
        1.0943129062652588          // wrist_3_joint
    };

    // ===== WAYPOINT 2 APPROXIMATE JOINT POSITIONS (for return sequence) =====
    // User provided order: [shoulder_lift, elbow, wrist_1, wrist_2, wrist_3, shoulder_pan]
    // MoveIt order: [shoulder_pan, shoulder_lift, elbow, wrist_1, wrist_2, wrist_3]
    std::vector<double> waypoint2_joints = {
        0.905205249786377,          // shoulder_pan_joint (last in user order)
        -1.5454686323748987,        // shoulder_lift_joint (first in user order)
        1.8361940383911133,         // elbow_joint
        -1.863520924245016,         // wrist_1_joint
        -1.548624340687887,         // wrist_2_joint
        0.8704494833946228          // wrist_3_joint
    };

    // ===== CONTINUOUS LOOP =====
    int cycle_count = 0;
    while (rclcpp::ok()) {
        cycle_count++;
        RCLCPP_INFO(logger, "=== STARTING CYCLE %d ===", cycle_count);

        // Variables to store orientation data (captured early, used after lifting)
        std::string stored_orientation_class = "";
        double stored_current_yaw = 0.0;
        bool has_orientation_data = false;
        
        // ===== READ ORIENTATION DATA (early, for later use before vacuum release) =====
        RCLCPP_INFO(logger, "Reading orientation class and yaw angle for correction...");
        
        std_msgs::msg::String::SharedPtr class_label_msg = nullptr;
        auto class_subscription_pre = node->create_subscription<std_msgs::msg::String>(
            "/pin_housing/class_label", 10,
            [&class_label_msg](const std_msgs::msg::String::SharedPtr msg) {
                class_label_msg = msg;
            });
        
        std_msgs::msg::Float32MultiArray::SharedPtr pixel_pose_msg_pre = nullptr;
        auto pixel_pose_subscription_pre = node->create_subscription<std_msgs::msg::Float32MultiArray>(
            "/pin_housing/pixel_pose", 10,
            [&pixel_pose_msg_pre](const std_msgs::msg::Float32MultiArray::SharedPtr msg) {
                pixel_pose_msg_pre = msg;
            });
        
        // Wait indefinitely until we get both messages
        while ((!class_label_msg || !pixel_pose_msg_pre) && rclcpp::ok()) {
            rclcpp::spin_some(node);
            std::this_thread::sleep_for(std::chrono::milliseconds(10));
        }
        
        if (class_label_msg && pixel_pose_msg_pre && pixel_pose_msg_pre->data.size() >= 3) {
            stored_orientation_class = class_label_msg->data;
            stored_current_yaw = pixel_pose_msg_pre->data[2];
            has_orientation_data = true;
            RCLCPP_INFO(logger, "Orientation captured: Class='%s', Yaw=%.2f°", 
                       stored_orientation_class.c_str(), stored_current_yaw);
        }
        
        RCLCPP_INFO(logger, "Switching to Pilz for TCP moves...");
        
        // ===== GET WAYPOINT 2 FROM TOPIC =====
        RCLCPP_INFO(logger, "Getting Waypoint 2 coordinates from /pin_housing/target_pose topic...");
        
        // Subscribe to the topic and get one message
        geometry_msgs::msg::PoseStamped::SharedPtr waypoint2_pose_msg = nullptr;
        auto subscription = node->create_subscription<geometry_msgs::msg::PoseStamped>(
            "/pin_housing/target_pose", 10,
            [&waypoint2_pose_msg](const geometry_msgs::msg::PoseStamped::SharedPtr msg) {
                waypoint2_pose_msg = msg;
            });
        
        // Wait indefinitely until we get the message
        while (!waypoint2_pose_msg && rclcpp::ok()) {
            rclcpp::spin_some(node);
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

        // Note: Orientation data already captured earlier after Waypoint 0
        // It is stored in stored_orientation_class and stored_current_yaw for later use (e.g., conditional blow)

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
            RCLCPP_INFO(logger, "Waypoint 1 reached!");
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

    // ===== NEW SEQUENCE: Move to Waypoint 2, then intermediate position and execute joint sequence =====
    
    // 7. Return to Waypoint 2
    RCLCPP_INFO(logger, "Step 7: Returning to Waypoint 2 with object (original orientation)...");
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

    // 7.5. Wait for Red Detection (moved from pre-sequence)
    RCLCPP_INFO(logger, "Step 7.5: Waiting for red DETECTED...");
    if (!wait_for_red_detected()) {
        RCLCPP_ERROR(logger, "Failed to get red DETECTED status - skipping cycle");
        continue;
    }
    
    // Red detected for 2 seconds - Set PLC interlock coil 8194 to LOW (0) to unlock other robot
    RCLCPP_INFO(logger, "Red detected for 2 seconds - Setting PLC interlock coil 8194 to LOW (0)...");
    std_msgs::msg::Bool set_low_msg;
    set_low_msg.data = true;
    interlock_set_low_publisher->publish(set_low_msg);
    RCLCPP_INFO(logger, "Interlock LOW signal sent - Other robot can proceed");

    // 8. Move to intermediate position - COMMENTED OUT
    /*
    double intermediate_x = 0.19042828485402838;
    double intermediate_y = 0.5995427932501025;
    double intermediate_z = 0.3117300106544869;
    
    RCLCPP_INFO(logger, "Step 8: Moving to intermediate position (x=%.3f, y=%.3f, z=%.3f)...", 
               intermediate_x, intermediate_y, intermediate_z);
    auto const intermediate_pose = [intermediate_x, intermediate_y, intermediate_z, orientation_x, orientation_y, orientation_z, orientation_w]()
    {
        geometry_msgs::msg::Pose msg;
        msg.position.x = intermediate_x;
        msg.position.y = intermediate_y;
        msg.position.z = intermediate_z;
        msg.orientation.x = orientation_x;
        msg.orientation.y = orientation_y;
        msg.orientation.z = orientation_z;
        msg.orientation.w = orientation_w;
        return msg;
    }();
    move_group_interface.setPoseTarget(intermediate_pose, "tool0");
    
    auto const [success8, plan8] = [&move_group_interface]
    {
        moveit::planning_interface::MoveGroupInterface::Plan msg;
        auto const ok = static_cast<bool>(move_group_interface.plan(msg));
        return std::make_pair(ok, msg);
    }();

    if (success8)
    {
        RCLCPP_INFO(logger, "Intermediate position planning successful! Executing...");
        auto execute_result = move_group_interface.execute(plan8);
        if (execute_result == moveit::planning_interface::MoveItErrorCode::SUCCESS)
        {
            RCLCPP_INFO(logger, "Intermediate position reached!");
        }
        else
        {
            RCLCPP_ERROR(logger, "Intermediate position execution failed! Error code: %d", execute_result.val);
            return -1;
        }
    }
    else
    {
        RCLCPP_ERROR(logger, "Intermediate position planning failed!");
        return -1;
    }
    */
    
    // Define intermediate_pose for later use in return sequence
    double intermediate_x = 0.19042828485402838;
    double intermediate_y = 0.5995427932501025;
    double intermediate_z = 0.3117300106544869;
    auto const intermediate_pose = [intermediate_x, intermediate_y, intermediate_z, orientation_x, orientation_y, orientation_z, orientation_w]()
    {
        geometry_msgs::msg::Pose msg;
        msg.position.x = intermediate_x;
        msg.position.y = intermediate_y;
        msg.position.z = intermediate_z;
        msg.orientation.x = orientation_x;
        msg.orientation.y = orientation_y;
        msg.orientation.z = orientation_z;
        msg.orientation.w = orientation_w;
        return msg;
    }();

    // 9. Execute joint sequence (2 joint moves: Sets 4 and 5, Set 3 COMMENTED OUT)
    // OPTIMIZATION: Apply orientation compensation directly to Set 5 target, not as separate step
    RCLCPP_INFO(logger, "Step 9: Executing forward joint sequence (Set 4 → Set 5 with orientation compensation, Set 3 skipped)...");
    
    // Joint position sets 
    // User provided order: [shoulder_lift, elbow, wrist_1, wrist_2, wrist_3, shoulder_pan]
    // Standard UR order: [shoulder_pan, shoulder_lift, elbow, wrist_1, wrist_2, wrist_3]
    // NOTE: Set 3 is commented out in forward sequence but kept for reverse sequence
    std::vector<std::vector<double>> joint_sequence = {
        // Set 3: [shoulder_pan, shoulder_lift, elbow, wrist_1, wrist_2, wrist_3] - COMMENTED OUT in forward, kept for reverse
        {1.0157181024551392, -1.3367255369769495, 1.745710849761963, -2.48492938676943, -1.4261038939105433, 1.5937026739120483},
        // Set 4: [shoulder_pan, shoulder_lift, elbow, wrist_1, wrist_2, wrist_3]
        {0.8890299201011658, -1.3253091017352503, 1.8641610145568848, -2.855262104664938, -1.1843927542315882, 1.6333327293395996},
        // Set 5: [shoulder_pan, shoulder_lift, elbow, wrist_1, wrist_2, wrist_3] - UPDATED
        {0.7784405946731567, -1.171955410634176, 1.8949699401855469, -3.1571925322162073, -1.0007088820086878, 1.6858445405960083}
    };
    
    // ===== APPLY ORIENTATION COMPENSATION DIRECTLY TO SET 5 TARGET =====
    // Get joint names and find wrist_3_joint index once, before the loop
    std::vector<std::string> joint_names = move_group_interface.getJointNames();
    size_t wrist_3_idx = SIZE_MAX;
    for (size_t i = 0; i < joint_names.size(); ++i) {
        if (joint_names[i] == "wrist_3_joint") {
            wrist_3_idx = i;
            break;
        }
    }
    
    // Create a copy of joint_sequence for execution (with orientation compensation applied to Set 5)
    std::vector<std::vector<double>> joint_sequence_to_execute = joint_sequence;
    
    if (wrist_3_idx != SIZE_MAX && wrist_3_idx < joint_sequence[2].size() && has_orientation_data) {
        double yaw_correction_deg = calculate_yaw_correction(stored_orientation_class, stored_current_yaw);
        
        if (std::abs(yaw_correction_deg) > 0.1) {
            RCLCPP_INFO(logger, "Applying yaw correction to Set 5: %.2f° (%s)", 
                       std::abs(yaw_correction_deg), yaw_correction_deg >= 0 ? "CW" : "CCW");
            
            // Apply correction directly to Set 5 (index 2) wrist_3_joint
            double yaw_correction_rad = yaw_correction_deg * M_PI / 180.0;
            joint_sequence_to_execute[2][wrist_3_idx] += yaw_correction_rad;
            
            // Normalize to [-pi, pi] range
            while (joint_sequence_to_execute[2][wrist_3_idx] > M_PI) 
                joint_sequence_to_execute[2][wrist_3_idx] -= 2.0 * M_PI;
            while (joint_sequence_to_execute[2][wrist_3_idx] < -M_PI) 
                joint_sequence_to_execute[2][wrist_3_idx] += 2.0 * M_PI;
            
            RCLCPP_INFO(logger, "Set 5 original wrist_3: %.6f rad (%.2f°)", 
                       joint_sequence[2][wrist_3_idx], joint_sequence[2][wrist_3_idx] * 180.0 / M_PI);
            RCLCPP_INFO(logger, "Set 5 corrected wrist_3: %.6f rad (%.2f°)", 
                       joint_sequence_to_execute[2][wrist_3_idx], joint_sequence_to_execute[2][wrist_3_idx] * 180.0 / M_PI);
        } else {
            RCLCPP_INFO(logger, "No orientation correction needed (correction < 0.1°), using original Set 5.");
        }
    } else {
        if (wrist_3_idx == SIZE_MAX) {
            RCLCPP_WARN(logger, "Could not find wrist_3_joint - using original Set 5 without correction");
        } else if (!has_orientation_data) {
            RCLCPP_WARN(logger, "No orientation data available - using original Set 5 without correction");
        }
    }
    
    // Forward joint sequence: Start from index 1 (Set 4), skip Set 3 (index 0)
    std::vector<size_t> forward_sequence_indices = {1, 2};  // Set 4, Set 5
    
    for (size_t seq_idx = 0; seq_idx < forward_sequence_indices.size(); ++seq_idx) {
        size_t i = forward_sequence_indices[seq_idx];
        RCLCPP_INFO(logger, "Executing joint move %zu/%zu (Set %zu)...", seq_idx+1, forward_sequence_indices.size(), i+1);
        if (i == 2) {  // Set 5 - use corrected version
            if (!plan_and_execute_joint(joint_sequence_to_execute[i], "Joint sequence forward " + std::to_string(seq_idx+1) + " (Set " + std::to_string(i+1) + " with orientation compensation)")) {
                RCLCPP_ERROR(logger, "Joint sequence forward %zu (Set %zu) failed!", seq_idx+1, i+1);
                return -1;
            }
        } else {  // Set 4 - use original
            if (!plan_and_execute_joint(joint_sequence[i], "Joint sequence forward " + std::to_string(seq_idx+1) + " (Set " + std::to_string(i+1) + ")")) {
                RCLCPP_ERROR(logger, "Joint sequence forward %zu (Set %zu) failed!", seq_idx+1, i+1);
                return -1;
            }
        }
    }
    RCLCPP_INFO(logger, "Forward joint sequence completed! (Set 4 → Set 5 with orientation compensation)");

    // 10. Release vacuum (and blow for bottom orientation)
    RCLCPP_INFO(logger, "Step 10: Turning OFF Channel 0 (suction)...");
    set_suction(false);  // Channel 0 LOW (suction off)
    
    // Wait 1 second for object to be released
    std::this_thread::sleep_for(std::chrono::milliseconds(1000));
    RCLCPP_INFO(logger, "Object released! Suction OFF.");

    // 11. Reverse joint sequence (move directly to Set 4, skip Set 5 since we're already there)
    RCLCPP_INFO(logger, "Step 11: Executing reverse joint sequence (moving directly to Set 4, skipping Set 5)...");
    // After orientation fix and release, we're already at Set 5, so go directly to Set 4
    // Set 4 is at index 1
    RCLCPP_INFO(logger, "Executing reverse joint move (Set 4)...");
    if (!plan_and_execute_joint(joint_sequence[1], "Joint sequence reverse Set 4")) {
        RCLCPP_ERROR(logger, "Joint sequence reverse Set 4 failed!");
        return -1;
    }
    RCLCPP_INFO(logger, "Reverse joint sequence completed! (Directly to Set 4)");

    // Conditional joint sequence for bottom orientation (instead of blow)
    // Check if orientation class string starts with "bottom" or "top"
    // Valid classes: 'bottombottom', 'bottomleft', 'bottomright', 'bottomtop', 'topbottom', 'topleft', 'topright', 'toptop'
    bool should_execute_bottom = false;
    
    if (has_orientation_data && !stored_orientation_class.empty()) {
        // Convert to lowercase for case-insensitive comparison
        std::string orientation_lower = stored_orientation_class;
        std::transform(orientation_lower.begin(), orientation_lower.end(), orientation_lower.begin(), ::tolower);
        
        // Check if string starts with "bottom"
        should_execute_bottom = (orientation_lower.find("bottom") == 0);
        
        RCLCPP_INFO(logger, "Orientation classification: Class='%s' (lowercase='%s'), Starts with 'bottom'? %s", 
                   stored_orientation_class.c_str(), orientation_lower.c_str(), 
                   should_execute_bottom ? "YES - Executing bottom sequence" : "NO - Skipping bottom sequence");
    } else {
        RCLCPP_WARN(logger, "No orientation data available: has_orientation_data=%d, class='%s'", 
                   has_orientation_data, stored_orientation_class.c_str());
    }
    
    if (should_execute_bottom) {
        RCLCPP_INFO(logger, "*** CONDITION MET: Class starts with 'bottom' - Executing bottom sequence! ***");
        // Joint move between Set 4 and bottom sequence (only for bottom orientation)
        // User provided order: [shoulder_lift, elbow, wrist_1, wrist_2, wrist_3, shoulder_pan]
        // MoveIt order: [shoulder_pan, shoulder_lift, elbow, wrist_1, wrist_2, wrist_3]
        std::vector<double> pre_bottom_intermediate_joints = {
            1.2461328506469727,        // shoulder_pan_joint (last in user order)
            -0.9604228178607386,       // shoulder_lift_joint (first in user order)
            1.143284797668457,         // elbow_joint
            -1.6187623182879847,       // wrist_1_joint
            -2.3182864824878138,       // wrist_2_joint
            1.6663473844528198          // wrist_3_joint
        };
        
        RCLCPP_INFO(logger, "Moving to intermediate position before conditional bottom sequence...");
        if (!plan_and_execute_joint(pre_bottom_intermediate_joints, "Pre-bottom orientation intermediate position")) {
            RCLCPP_ERROR(logger, "Pre-bottom orientation intermediate position failed!");
            return -1;
        }
        RCLCPP_INFO(logger, "Pre-bottom intermediate position reached!");
        
        RCLCPP_INFO(logger, "Detected 'bottom' orientation - Executing additional joint sequence...");
        
        // Intermediate joint position between Set 4 and bottom sequence
        // User provided order: [shoulder_lift, elbow, wrist_1, wrist_2, wrist_3, shoulder_pan]
        // MoveIt order: [shoulder_pan, shoulder_lift, elbow, wrist_1, wrist_2, wrist_3]
        std::vector<double> bottom_intermediate_joints = {
            1.204429268836975,         // shoulder_pan_joint (last in user order)
            -0.9643128553973597,       // shoulder_lift_joint (first in user order)
            1.2435078620910645,        // elbow_joint
            -1.754063908253805,        // wrist_1_joint
            -2.32298452058901,          // wrist_2_joint
            1.6091736555099487          // wrist_3_joint
        };
        
        RCLCPP_INFO(logger, "Moving to intermediate position before bottom joint sequence...");
        if (!plan_and_execute_joint(bottom_intermediate_joints, "Bottom orientation intermediate position")) {
            RCLCPP_ERROR(logger, "Bottom orientation intermediate position failed!");
            return -1;
        }
        RCLCPP_INFO(logger, "Intermediate position reached!");
        
        // Additional joint moves for bottom orientation
        // User provided order: [shoulder_lift, elbow, wrist_1, wrist_2, wrist_3, shoulder_pan]
        // MoveIt order: [shoulder_pan, shoulder_lift, elbow, wrist_1, wrist_2, wrist_3]
        std::vector<std::vector<double>> bottom_joint_sequence = {
            // Move 1: [shoulder_pan, shoulder_lift, elbow, wrist_1, wrist_2, wrist_3]
            {1.2037932872772217, -0.9647210280047815, 1.2454023361206055, -1.7563183943377894, -2.322517220173971, 1.608406662940979},
            // Move 2: [shoulder_pan, shoulder_lift, elbow, wrist_1, wrist_2, wrist_3]
            {1.1907035112380981, -1.0042412916766565, 1.2717208862304688, -1.7551067511187952, -2.3237159887896937, 1.590611219406128},
            // Move 3: [shoulder_pan, shoulder_lift, elbow, wrist_1, wrist_2, wrist_3]
            {1.1907395124435425, -1.004265610371725, 1.2717208862304688, -1.7550233046161097, -2.323751989995138, -1.4020584265338343}
        };
        
        // Execute bottom sequence with vacuum control
        for (size_t i = 0; i < bottom_joint_sequence.size(); ++i) {
            // Turn vacuum ON before move 2 (12b)
            if (i == 1) {  // Before position 2 (index 1)
                RCLCPP_INFO(logger, "Turning ON Channel 0 (suction) before bottom position 2...");
                set_suction(true);  // Channel 0 HIGH (suction ON)
                std::this_thread::sleep_for(std::chrono::milliseconds(500));
                RCLCPP_INFO(logger, "Suction ON! Waiting for vacuum to engage...");
                std::this_thread::sleep_for(std::chrono::milliseconds(1500));  // Additional delay for vacuum to fully engage (total 2 seconds)
                RCLCPP_INFO(logger, "Vacuum engaged. Proceeding to position 2...");
            }
            
            RCLCPP_INFO(logger, "Executing bottom orientation joint move %zu/%zu...", i+1, bottom_joint_sequence.size());
            if (!plan_and_execute_joint(bottom_joint_sequence[i], "Bottom orientation joint move " + std::to_string(i+1))) {
                RCLCPP_ERROR(logger, "Bottom orientation joint move %zu failed!", i+1);
                return -1;
            }
            
            // Turn vacuum OFF after move 3 (12d)
            if (i == 2) {  // After position 3 (index 2)
                RCLCPP_INFO(logger, "Turning OFF Channel 0 (suction) after bottom position 3...");
                set_suction(false);  // Channel 0 LOW (suction OFF)
                std::this_thread::sleep_for(std::chrono::milliseconds(1000));  // Wait 1 second for vacuum to disengage
                RCLCPP_INFO(logger, "Suction OFF!");
            }
        }
        RCLCPP_INFO(logger, "Bottom orientation joint sequence completed!");
    } else {
        if (has_orientation_data && !stored_orientation_class.empty()) {
            std::string orientation_lower = stored_orientation_class;
            std::transform(orientation_lower.begin(), orientation_lower.end(), orientation_lower.begin(), ::tolower);
            RCLCPP_INFO(logger, "Detected '%s' orientation (starts with 'top') - Skipping bottom orientation joint sequence.", 
                       stored_orientation_class.c_str());
        } else {
            RCLCPP_INFO(logger, "No orientation data available - Skipping bottom orientation joint sequence.");
        }
    }

    // 12. Move directly to Waypoint 2 joint positions (using OMPL)
    RCLCPP_INFO(logger, "Step 12: Moving directly to Waypoint 2 joint positions (OMPL)...");
    if (!plan_and_execute_joint(waypoint2_joints, "Waypoint 2 joint positions (return)")) {
        RCLCPP_ERROR(logger, "Waypoint 2 joint move failed - skipping cycle");
        return -1;
    }
    RCLCPP_INFO(logger, "Waypoint 2 joint positions reached!");
    
    // 12.5. Check if red is NOT detected for 1 second and set PLC interlock HIGH if needed
    RCLCPP_INFO(logger, "Step 12.5: Checking red detection status...");
    check_red_not_detected_for_1_second();

    // 13. Return to Waypoint 1 (with default orientation, TCP move)
    RCLCPP_INFO(logger, "Step 13: Returning to Waypoint 1 with default orientation...");
    move_group_interface.setPoseTarget(waypoint1_pose, "tool0");
    
    auto const [success14, plan14] = [&move_group_interface]
    {
        moveit::planning_interface::MoveGroupInterface::Plan msg;
        auto const ok = static_cast<bool>(move_group_interface.plan(msg));
        return std::make_pair(ok, msg);
    }();

    if (success14)
    {
        RCLCPP_INFO(logger, "Waypoint 1 planning successful! Executing...");
        auto execute_result = move_group_interface.execute(plan14);
        if (execute_result == moveit::planning_interface::MoveItErrorCode::SUCCESS)
        {
            RCLCPP_INFO(logger, "Waypoint 1 reached!");
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
