import os
from ament_index_python.packages import get_package_share_directory
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration, EqualsSubstitution, TextSubstitution
from launch.conditions import IfCondition

def generate_launch_description():
    
    configs = os.path.join(
        get_package_share_directory('tuos_tb3_tools'),
        'rviz'
    )

    launch_dir = os.path.join(
        get_package_share_directory("tuos_tb3_tools"),
        "include"
    )

    rviz_real = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', os.path.join(configs, 'rviz_real.rviz')],
        output='screen'
    )

    rviz_sim = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', os.path.join(configs, 'rviz_sim.rviz')],
        output='screen'
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            'environment', 
            description="Define the operating environment: 'real' or 'sim'",
            default_value='real'
        ),
        IncludeLaunchDescription( 
            PythonLaunchDescriptionSource( 
                os.path.join( 
                    launch_dir, 
                    "_rviz_real" 
                )
            ),
            condition=IfCondition(
                EqualsSubstitution(
                    LaunchConfiguration('environment'), 
                    TextSubstitution(text='real')
                )
            )
        ),
        IncludeLaunchDescription( 
            PythonLaunchDescriptionSource( 
                os.path.join( 
                    launch_dir, 
                    "_rviz_sim" 
                )
            ),
            condition=IfCondition(
                EqualsSubstitution(
                    LaunchConfiguration('environment'), 
                    TextSubstitution(text='sim')
                )
            )
        )
    ])
