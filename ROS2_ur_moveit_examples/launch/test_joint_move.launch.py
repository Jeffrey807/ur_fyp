#!/usr/bin/env python3

import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import Command, FindExecutable, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from ur_moveit_config.launch_common import load_yaml, load_yaml_abs


def launch_setup(context, *args, **kwargs):
    ur_type = LaunchConfiguration("ur_type")
    use_fake_hardware = LaunchConfiguration("use_fake_hardware")
    safety_limits = LaunchConfiguration("safety_limits")
    safety_pos_margin = LaunchConfiguration("safety_pos_margin")
    safety_k_position = LaunchConfiguration("safety_k_position")
    description_package = LaunchConfiguration("description_package")
    description_file = LaunchConfiguration("description_file")
    moveit_config_package = LaunchConfiguration("moveit_config_package")
    moveit_config_file = LaunchConfiguration("moveit_config_file")
    prefix = LaunchConfiguration("prefix")
    use_sim_time = LaunchConfiguration("use_sim_time")

    joint_limit_params = PathJoinSubstitution(
        [FindPackageShare("ur_pilz_demo"), "config", "joint_limits.yaml"]
    )
    kinematics_params = PathJoinSubstitution(
        [FindPackageShare(description_package), "config", ur_type, "default_kinematics.yaml"]
    )
    physical_params = PathJoinSubstitution(
        [FindPackageShare(description_package), "config", ur_type, "physical_parameters.yaml"]
    )
    visual_params = PathJoinSubstitution(
        [FindPackageShare(description_package), "config", ur_type, "visual_parameters.yaml"]
    )

    robot_description_content = Command([
        PathJoinSubstitution([FindExecutable(name="xacro")]),
        " ",
        PathJoinSubstitution([FindPackageShare(description_package), "urdf", description_file]),
        " ",
        "name:=ur",
        " ",
        "prefix:=", prefix,
        " ",
        "use_fake_hardware:=", use_fake_hardware,
        " ",
        "safety_limits:=", safety_limits,
        " ",
        "safety_pos_margin:=", safety_pos_margin,
        " ",
        "safety_k_position:=", safety_k_position,
        " ",
        "joint_limit_params:=", joint_limit_params,
        " ",
        "kinematics_params:=", kinematics_params,
        " ",
        "physical_params:=", physical_params,
        " ",
        "visual_params:=", visual_params,
    ])
    robot_description = {"robot_description": robot_description_content}

    robot_description_semantic_content = Command([
        PathJoinSubstitution([FindExecutable(name="xacro")]),
        " ",
        PathJoinSubstitution([FindPackageShare(moveit_config_package), "srdf", moveit_config_file]),
        " ",
        "name:=ur",
        " ",
        "prefix:=", prefix,
    ])
    robot_description_semantic = {"robot_description_semantic": robot_description_semantic_content}

    robot_description_kinematics = PathJoinSubstitution(
        [FindPackageShare(moveit_config_package), "config", "kinematics.yaml"]
    )
    robot_description_planning_content = load_yaml_abs(str(joint_limit_params.perform(context)))
    if robot_description_planning_content is None:
        robot_description_planning_content = {}
    robot_description_planning = {
        "robot_description_planning": robot_description_planning_content
    }

    # Planning Configuration (use default OMPL for joint moves, not Pilz)
    ompl_planning_yaml = load_yaml("ros2_ur_moveit_examples", "config/ompl_planning.yaml")
    if ompl_planning_yaml is None:
        ompl_planning_yaml = {}
    planning_pipelines_config = {
        "planning_pipelines": ["ompl"],
        "default_planning_pipeline": "ompl",
        "ompl": ompl_planning_yaml,
    }

    # Trajectory Execution Configuration
    controllers_yaml = load_yaml("ur_pilz_demo", "config/controllers.yaml")
    if controllers_yaml is None:
        controllers_yaml = {}
    
    # Reorder controller_names to prioritize joint_trajectory_controller (works with Pilz)
    if controllers_yaml and "controller_names" in controllers_yaml:
        # Put joint_trajectory_controller first in the list
        controller_names_list = controllers_yaml["controller_names"]
        if "joint_trajectory_controller" in controller_names_list:
            controller_names_list.remove("joint_trajectory_controller")
            controller_names_list.insert(0, "joint_trajectory_controller")
        controllers_yaml["controller_names"] = controller_names_list
    
    # Set joint_trajectory_controller as default (works with Pilz)
    if controllers_yaml and "scaled_joint_trajectory_controller" in controllers_yaml:
        controllers_yaml["scaled_joint_trajectory_controller"]["default"] = False
    if controllers_yaml and "joint_trajectory_controller" in controllers_yaml:
        controllers_yaml["joint_trajectory_controller"]["default"] = True
    
    change_controllers = context.perform_substitution(use_fake_hardware)
    if change_controllers == "true" and controllers_yaml:
        if "scaled_joint_trajectory_controller" in controllers_yaml:
            controllers_yaml["scaled_joint_trajectory_controller"]["scaled_joint_trajectory_controller"] = {
                "type": "joint_trajectory_controller/JointTrajectoryController"
            }

    moveit_controllers = {
        "moveit_controller_manager": "moveit_simple_controller_manager/MoveItSimpleControllerManager",
        "moveit_simple_controller_manager": controllers_yaml
    }

    trajectory_execution = {
        "moveit_manage_controllers": True,
        "trajectory_execution.allowed_execution_duration_scaling": 1.2,
        "trajectory_execution.allowed_goal_duration_margin": 0.5,
        "trajectory_execution.allowed_start_tolerance": 0.01,
        "trajectory_execution.wait_for_trajectory_completion": True,
        "trajectory_execution.controller_connection_timeout": 10.0,
    }

    planning_scene_monitor_parameters = {
        "publish_planning_scene": True,
        "publish_geometry_updates": True,
        "publish_state_updates": True,
        "publish_transforms_updates": True,
    }

    # Start move_group node
    move_group_node = Node(
        package="moveit_ros_move_group",
        executable="move_group",
        output="screen",
        parameters=[
            robot_description,
            robot_description_semantic,
            planning_pipelines_config,
            robot_description_kinematics,
            robot_description_planning,
            moveit_controllers,
            trajectory_execution,
            planning_scene_monitor_parameters,
            {"use_sim_time": use_sim_time},
        ],
    )

    # Test joint move node
    test_joint_move_node = Node(
        package="ros2_ur_moveit_examples",
        executable="test_joint_move",
        name="test_joint_move",
        output="screen",
        parameters=[
            robot_description,
            robot_description_semantic,
            planning_pipelines_config,
            robot_description_kinematics,
            {"use_sim_time": use_sim_time},
        ],
    )

    return [move_group_node, test_joint_move_node]


def generate_launch_description():
    declared_arguments = [
        DeclareLaunchArgument("ur_type", default_value="ur5", choices=["ur3", "ur3e", "ur5", "ur5e", "ur10", "ur10e", "ur16e"]),
        DeclareLaunchArgument("use_fake_hardware", default_value="false"),
        DeclareLaunchArgument("safety_limits", default_value="true"),
        DeclareLaunchArgument("safety_pos_margin", default_value="0.15"),
        DeclareLaunchArgument("safety_k_position", default_value="20"),
        DeclareLaunchArgument("description_package", default_value="ur_description"),
        DeclareLaunchArgument("description_file", default_value="ur.urdf.xacro"),
        DeclareLaunchArgument("moveit_config_package", default_value="ur_moveit_config"),
        DeclareLaunchArgument("moveit_config_file", default_value="ur.srdf.xacro"),
        DeclareLaunchArgument("prefix", default_value='""'),
        DeclareLaunchArgument("use_sim_time", default_value="true"),
    ]
    return LaunchDescription(declared_arguments + [OpaqueFunction(function=launch_setup)])

