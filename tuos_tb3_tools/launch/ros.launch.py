import launch
from launch_ros.actions import Node  # Import for ROS 2 nodes
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, FindPackage


def generate_launch_description():
    # Get package paths
    turtlebot3_bringup_pkg = FindPackage('turtlebot3_bringup')
    realsense2_camera_pkg = FindPackage('realsense2_camera')
    tuos_tb3_tools_pkg = FindPackage('tuos_tb3_tools')

    # Launch arguments for realsense2_camera
    output = LaunchConfiguration('output', default='log')
    color_width = LaunchConfiguration('color_width', default='848')
    color_height = LaunchConfiguration('color_height', default='480')
    color_fps = LaunchConfiguration('color_fps', default='15')
    align_depth = LaunchConfiguration('align_depth', default='true')

    # Launch descriptions for each package
    turtlebot3_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [turtlebot3_bringup_pkg, '/launch/turtlebot3_robot.launch.py']
        )
    )

    realsense2_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [realsense2_camera_pkg, '/launch/rs_camera.launch.py']
        ),
        launch_arguments=[
            output,
            color_width,
            color_height,
            color_fps,
            align_depth,
        ]
    )

    tb3_status_node = Node(
        package=tuos_tb3_tools_pkg,
        executable='tb3_status.py',  # Assuming tb3_status.py is executable
        name='tb3_status_node',
        output='screen',
    )

    # Combine launch descriptions
    return launch.LaunchDescription([
        turtlebot3_launch,
        realsense2_launch,
        tb3_status_node,
    ])