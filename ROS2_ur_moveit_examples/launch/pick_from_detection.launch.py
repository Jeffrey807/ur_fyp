#!/usr/bin/env python3
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(
            package='ros2_ur_moveit_examples',
            executable='pick_from_detection',
            name='pick_from_detection',
            output='screen',
            parameters=[{
                'planning_group': 'ur_manipulator',
                'target_pose_topic': '/pin_housing/target_pose',
                'z_offset_mm': 15.0,
                'vel_scale': 0.1,
                'acc_scale': 0.1,
                'io_pin': 0,
                # Joint order: [shoulder_pan, shoulder_lift, elbow, wrist_1, wrist_2, wrist_3]
                # Values are in radians (as provided from joint_state):
                'home_joints': [
                    0.005662546493113041,    # shoulder_pan
                    -2.203487221394674,      # shoulder_lift
                    2.2026333808898926,      # elbow
                    -1.5362151304828089,     # wrist_1
                    -1.5611775557147425,     # wrist_2
                    0.10002259165048599      # wrist_3
                ],
                'ready_joints': [
                    0.8744020462036133,      # shoulder_pan
                    -1.6435344854937952,     # shoulder_lift
                    1.9878783226013184,      # elbow
                    -1.9130352179156702,     # wrist_1
                    -1.574055020009176,      # wrist_2
                    0.8803132772445679       # wrist_3
                ]
            }]
        )
    ])


