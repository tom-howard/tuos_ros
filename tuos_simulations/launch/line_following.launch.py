#!/usr/bin/env python3

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch.conditions import IfCondition, UnlessCondition

def generate_launch_description():

    gz_ros = os.path.join(
        get_package_share_directory('ros_gz_sim'), 'launch'
    )
    tuning_world = os.path.join(
        get_package_share_directory('tuos_simulations'), 
        'worlds', 
        'line_following_setup.world'
    )
    line_following_world = os.path.join(
        get_package_share_directory('tuos_simulations'), 
        'worlds', 
        'line_following.world'
    )

    use_sim_time = LaunchConfiguration('use_sim_time', default='true')

    return LaunchDescription([
        DeclareLaunchArgument(
            'with_gui', 
            description="Select whether to launch Gazebo with or without Gazebo Client (i.e. the GUI).",
            default_value='true'
        ),
        DeclareLaunchArgument(
            'tuning', 
            description="Select whether to launch a world designed for PID tuning (following a straight line) or the standard line following world (tuning:=false).",
            default_value='false'
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(
                    gz_ros,
                    'gz_sim.launch.py'
                )
            ),
            launch_arguments={'gz_args': ['-r -s -v1 ', tuning_world], 'on_exit_shutdown': 'true'}.items(),
            condition = IfCondition(
                LaunchConfiguration('tuning')
            )
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(
                    gz_ros,
                    'gz_sim.launch.py'
                )
            ),
            launch_arguments={'gz_args': ['-r -s -v1 ', line_following_world], 'on_exit_shutdown': 'true'}.items(),
            condition = UnlessCondition(
                LaunchConfiguration('tuning')
            )
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(
                    gz_ros,
                    'gz_sim.launch.py'
                )
            ),
            launch_arguments={'gz_args': '-g -v1 '}.items(),
            condition=IfCondition(
                LaunchConfiguration('with_gui')
            )
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(
                    get_package_share_directory('turtlebot3_gazebo'), 
                    'launch',
                    'robot_state_publisher.launch.py'
                )
            ),
            launch_arguments={'use_sim_time': use_sim_time}.items()
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(
                    get_package_share_directory('tuos_simulations'), 
                    'launch', 
                    'spawn_tb3.launch.py'
                )
            ),
            launch_arguments={
                'x_pose': LaunchConfiguration('x_pose', default='0.5'),
                'y_pose': LaunchConfiguration('y_pose', default='-5.48'),
                'yaw': LaunchConfiguration('yaw', default='1.57'),
            }.items(),
            condition=IfCondition(
                LaunchConfiguration('tuning')
            )
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(
                    get_package_share_directory('tuos_simulations'), 
                    'launch', 
                    'spawn_tb3.launch.py'
                )
            ),
            launch_arguments={
                'x_pose': LaunchConfiguration('x_pose', default='-2.4'),
                'y_pose': LaunchConfiguration('y_pose', default='-2.5'),
                'yaw': LaunchConfiguration('yaw', default='1.57'),
            }.items(),
            condition=UnlessCondition(
                LaunchConfiguration('tuning')
            )
        )
    ])
