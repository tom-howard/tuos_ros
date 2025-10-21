import os

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
from launch.substitutions import LaunchConfiguration, EqualsSubstitution, TextSubstitution
from launch.conditions import IfCondition

def generate_launch_description():

    this_launch_dir = os.path.join(
        get_package_share_directory("tuos_tb3_tools"),
        "launch"
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
                    this_launch_dir, 
                    "_slam_real" 
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
                    this_launch_dir, 
                    "_slam_sim" 
                )
            ),
            condition=IfCondition(
                EqualsSubstitution(
                    LaunchConfiguration('environment'),
                    TextSubstitution(text='sim')
                )
            )
        ),

    ])