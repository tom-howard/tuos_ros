# Copied and adapted from turtlebot3_simulations (https://github.com/ROBOTIS-GIT/turtlebot3_simulations/tree/humble)

# Copyright 2019 Open Source Robotics Foundation, Inc.
# Copyright 2025 Tom Howard
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

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    urdf_path = os.path.join(
        get_package_share_directory('turtlebot3_gazebo'),
        'models',
        'turtlebot3_waffle',
        'model.sdf'
    )

    # Launch configuration variables specific to simulation
    x_pose = LaunchConfiguration('x_pose', default='0.0')
    y_pose = LaunchConfiguration('y_pose', default='0.0')
    yaw = LaunchConfiguration('yaw', default='0.0')

    return LaunchDescription([
        DeclareLaunchArgument(
            'x_pose',
            default_value='0.0',
            description='Starting X-position of the robot'
        ),
        DeclareLaunchArgument(
            'y_pose',
            default_value='0.0',
            description='Starting Y-position of the robot'
        ),
        DeclareLaunchArgument(
            'yaw',
            default_value='0.0',
            description='Starting orientation of the robot (radians)'
        ),
        Node(
            package='gazebo_ros',
            executable='spawn_entity.py',
            arguments=[
                '-entity', 'waffle',
                '-file', urdf_path,
                '-x', x_pose,
                '-y', y_pose,
                '-z', '0.01',
                '-Y', yaw,
            ],
            output='screen',
        )
    ])
