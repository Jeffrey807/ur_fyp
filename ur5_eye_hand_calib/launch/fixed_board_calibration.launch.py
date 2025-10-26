#!/usr/bin/env python3

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    # Get package share directory
    pkg_share = get_package_share_directory('ur5_eye_hand_calib')
    
    # Declare launch arguments
    image_topic_arg = DeclareLaunchArgument(
        'image_topic',
        default_value='/camera/color/image_raw',
        description='Camera image topic'
    )
    
    pose_topic_arg = DeclareLaunchArgument(
        'pose_topic',
        default_value='/tcp_pose_broadcaster/pose',
        description='Robot pose topic'
    )
    
    params_file_arg = DeclareLaunchArgument(
        'params_file',
        default_value=os.path.join(pkg_share, 'config', 'fixed_board_params.yaml'),
        description='Parameters file path'
    )
    
    # Fixed board calibrator node
    fixed_board_calibrator = Node(
        package='ur5_eye_hand_calib',
        executable='fixed_board_calibrator',
        name='fixed_board_calibrator',
        parameters=[LaunchConfiguration('params_file')],
        remappings=[
            ('image', LaunchConfiguration('image_topic')),
            ('pose', LaunchConfiguration('pose_topic'))
        ],
        output='screen'
    )
    
    return LaunchDescription([
        image_topic_arg,
        pose_topic_arg,
        params_file_arg,
        fixed_board_calibrator
    ])
