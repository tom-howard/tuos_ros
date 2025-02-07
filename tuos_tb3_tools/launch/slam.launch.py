# Adapted from turtlebot3_cartographer (https://github.com/ROBOTIS-GIT/turtlebot3/tree/humble/turtlebot3_cartographer)
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.
#
# Copyright 2019 Open Source Robotics Foundation, Inc.
# Copyright 2019 Darby Lim
# Copyright 2025 Tom Howard (University of Sheffield UK)

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource

def generate_launch_description():
    use_sim_time = LaunchConfiguration('use_sim_time', default='false')
    tb3_cartographer_dir = os.path.join(
        get_package_share_directory('turtlebot3_cartographer')
    )
    
    config_dir = LaunchConfiguration(
        'config_dir', 
        default=os.path.join(
            tb3_cartographer_dir, 'config'
        )
    )
    configuration_basename = LaunchConfiguration(
        'configuration_basename', default='turtlebot3_lds_2d.lua'
    )

    resolution = LaunchConfiguration('resolution', default='0.05')
    publish_period_sec = LaunchConfiguration('publish_period_sec', default='1.0')

    rviz_config = os.path.join(
        get_package_share_directory('tuos_tb3_tools'),
        'rviz', 'waffle_slam.rviz'
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            'config_dir',
            default_value=config_dir,
            description='Full path to config file to load'),
        DeclareLaunchArgument(
            'configuration_basename',
            default_value=configuration_basename,
            description='Name of lua file for cartographer'),
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='false',
            description='Use simulation (Gazebo) clock if true'),

        Node(
            package='cartographer_ros',
            executable='cartographer_node',
            name='cartographer_node',
            output='screen',
            parameters=[{'use_sim_time': use_sim_time}],
            arguments=['-configuration_directory', config_dir,
                       '-configuration_basename', configuration_basename]
        ),

        DeclareLaunchArgument(
            'resolution',
            default_value=resolution,
            description='Resolution of a grid cell in the published occupancy grid'
        ),

        DeclareLaunchArgument(
            'publish_period_sec',
            default_value=publish_period_sec,
            description='OccupancyGrid publishing period'
        ),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([
                os.path.join(
                    tb3_cartographer_dir, 'launch'
                ), 
                '/occupancy_grid.launch.py'
            ]),
            launch_arguments={
                'use_sim_time': use_sim_time, 
                'resolution': resolution,
                'publish_period_sec': publish_period_sec
            }.items(),
        ),

        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            arguments=['-d', rviz_config],
            parameters=[{'use_sim_time': use_sim_time}],
            output='screen'
        ),
    ])
