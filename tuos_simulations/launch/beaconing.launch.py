#!/usr/bin/env python3

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch.conditions import IfCondition, LaunchConfigurationEquals

def generate_launch_description():

    gz_ros = os.path.join(
        get_package_share_directory('gazebo_ros'), 'launch'
    )
    world = os.path.join(
        get_package_share_directory('tuos_simulations'), 'worlds','beaconing.world'
    )

    use_sim_time = LaunchConfiguration('use_sim_time', default='true')
    with_gui = LaunchConfiguration('with_gui')
    
    return LaunchDescription([
        DeclareLaunchArgument(
            'with_gui', 
            description="Select whether to launch Gazebo with or without Gazebo Client (i.e. the GUI).",
            default_value='true'
        ),
        DeclareLaunchArgument(
            'start_zone', 
            description="Select which start zone to spawn the robot into (a|b|c).",
            default_value='a'
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
            ),
            condition=IfCondition(with_gui)
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
                'x_pose': LaunchConfiguration('x_pose', default='-2.06729'),
                'y_pose': LaunchConfiguration('y_pose', default='-1.97396'),
                'yaw': LaunchConfiguration('yaw', default='1.571')
            }.items(),
            condition=LaunchConfigurationEquals('start_zone','a')
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
                'x_pose': LaunchConfiguration('x_pose', default='-1.24044'),
                'y_pose': LaunchConfiguration('y_pose', default='2.06729'),
                'yaw': LaunchConfiguration('yaw', default='3.142')
            }.items(),
            condition=LaunchConfigurationEquals('start_zone','b')
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
                'x_pose': LaunchConfiguration('x_pose', default='2.06729'),
                'y_pose': LaunchConfiguration('y_pose', default='1.97396'),
                'yaw': LaunchConfiguration('yaw', default='-1.571')
            }.items(),
            condition=LaunchConfigurationEquals('start_zone','c')
        )
    ])
    