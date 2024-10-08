from launch import LaunchDescription
from ament_index_python.packages import get_package_share_directory
from launch_ros.actions import Node  # Import for ROS 2 nodes
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    # Get package paths
    turtlebot3_bringup_pkg = get_package_share_directory('turtlebot3_bringup')
    realsense2_camera_pkg = get_package_share_directory('realsense2_camera')

    # tb3_status_node = Node(
    #     package=tuos_tb3_tools_pkg,
    #     executable='tb3_status.py',  # Assuming tb3_status.py is executable
    #     name='tb3_status_node',
    #     output='screen',
    # )
    
    # Launch arguments for realsense2_camera
    output = LaunchConfiguration(
        'output', default='log',
    )
    color_width = LaunchConfiguration(
        'color_width', default='848',
    )
    color_height = LaunchConfiguration(
        'color_height', default='480',
    )
    color_fps = LaunchConfiguration(
        'color_fps', default='15'
    )
    align_depth = LaunchConfiguration(
        'align_depth', default='true'
    )

    return LaunchDescription([
        
        DeclareLaunchArgument(
            'output',
            default_value=output,
        ),
        DeclareLaunchArgument(
            'color_width',
            default_value=color_width,
        ),
        DeclareLaunchArgument(
            'color_height',
            default_value=color_height,
        ),
        DeclareLaunchArgument(
            'color_fps',
            default_value=color_fps,
        ),
        DeclareLaunchArgument(
            'align_depth',
            default_value=align_depth,
        ),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                [turtlebot3_bringup_pkg, '/launch/robot.launch.py']
            )
        ),
        
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                [realsense2_camera_pkg, '/launch/rs_launch.py']
            ),
            launch_arguments={
                'output': output,
                'color_width': color_width,
                'color_height': color_height,
                'color_fps': color_fps,
                'align_depth': align_depth,
            }.items(),
        )
    ])