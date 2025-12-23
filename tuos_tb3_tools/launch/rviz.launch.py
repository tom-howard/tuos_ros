import os
from ament_index_python.packages import get_package_share_directory
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch import LaunchDescription
from launch.substitutions import LaunchConfiguration, EqualsSubstitution, TextSubstitution
from launch.conditions import IfCondition

def generate_launch_description():
    
    launch_dir = os.path.join(
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
                    launch_dir, 
                    "_rviz_real.launch.py" 
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
                    "_rviz_sim.launch.py" 
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
