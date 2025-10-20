#!/usr/bin/env python3

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch.conditions import IfCondition

def generate_launch_description():

    gz_ros = os.path.join(
        get_package_share_directory('ros_gz_sim'), 'launch'
    )

    world = os.path.join(
        get_package_share_directory('tuos_simulations'),
        'worlds',
        'coloured_pillars.world'
    )

    use_sim_time = LaunchConfiguration('use_sim_time', default='true')
    x_pose = LaunchConfiguration('x_pose', default='0.0')
    y_pose = LaunchConfiguration('y_pose', default='0.0')
    yaw = LaunchConfiguration('yaw', default='0.0')

    return LaunchDescription([
        DeclareLaunchArgument(
            'with_gui', 
            description="Select whether to launch Gazebo with or without Gazebo Client (i.e. the GUI).",
            default_value='true'
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(
                    gz_ros,
                    'gz_sim.launch.py'
                )
            ),
            launch_arguments={'gz_args': ['-r -s -v1 ', world], 'on_exit_shutdown': 'true'}.items(),
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
                'x_pose': x_pose,
                'y_pose': y_pose,
                'yaw': yaw,
            }.items()
        )
    ])
    