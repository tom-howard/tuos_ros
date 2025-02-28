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
        get_package_share_directory('gazebo_ros'), 'launch'
    )
    world = os.path.join(
        get_package_share_directory('tuos_simulations'), 'worlds','beacon_colours.world'
    )

    use_sim_time = LaunchConfiguration('use_sim_time', default='true')
        
    return LaunchDescription([
        DeclareLaunchArgument(
            'with_robot', 
            description="Select whether to spawn a robot into the world or not.",
            default_value='true'
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(
                    gz_ros,
                    'gzserver.launch.py'
                )
            ),
            launch_arguments={'world': world}.items(),
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(
                    gz_ros,
                    'gzclient.launch.py'
                )
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
                'x_pose': LaunchConfiguration('x_pose', default='0.0'),
                'y_pose': LaunchConfiguration('y_pose', default='0.0'),
                'yaw': LaunchConfiguration('yaw', default='0.0')
            }.items(),
            condition=IfCondition(
                LaunchConfiguration('with_robot')
            )
        )
    ])
    