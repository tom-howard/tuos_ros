#!/usr/bin/env python3

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch.conditions import IfCondition

def generate_launch_description():

    tb3_gz_dir = os.path.join(
        get_package_share_directory('turtlebot3_gazebo'), 'launch')
    gz_ros = get_package_share_directory('gazebo_ros')
    this_launch_dir = os.path.join(
        get_package_share_directory('tuos_simulations'), 'launch')

    use_sim_time = LaunchConfiguration('use_sim_time', default='true')
    x_pose = LaunchConfiguration('x_pose', default='2.4')
    y_pose = LaunchConfiguration('y_pose', default='-2.5')
    yaw = LaunchConfiguration('yaw', default='3.14')
    with_gui = LaunchConfiguration('with_gui', default='true')

    world = os.path.join(
        get_package_share_directory('tuos_simulations'),
        'worlds',
        'line_following.world'
    )

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
                    'launch',
                    'gzserver.launch.py'
                )
            ),
            launch_arguments={'world': world}.items(),
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(
                    gz_ros,
                    'launch',
                    'gzclient.launch.py'
                )
            ),
            condition=IfCondition(with_gui)
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(
                    tb3_gz_dir,
                    'robot_state_publisher.launch.py'
                )
            ),
            launch_arguments={'use_sim_time': use_sim_time}.items()
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(
                    this_launch_dir,
                    'spawn_tb3.launch.py'
                )
            ),
            launch_arguments={
                'x_pose': x_pose,
                'y_pose': y_pose,
                'yaw': yaw
            }.items()
        )
    ])
    