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
    color_profile = LaunchConfiguration(
        'rgb_camera.color_profile', default='848x480x15'
    )
    depth_profile = LaunchConfiguration(
        'depth_module.depth_profile', default='848x480x15'
    )
    enable_depth = LaunchConfiguration(
        'enable_depth', default='false'
    )
    camera_namespace = LaunchConfiguration(
        'camera_namespace', default=""
    )
    color_qos = LaunchConfiguration(
        'color_qos', default="SENSOR_DATA"
    )

    return LaunchDescription([
        
        DeclareLaunchArgument(
            'output',
            default_value=output,
        ),
        DeclareLaunchArgument(
            'camera_namespace',
            default_value=camera_namespace,
        ),
        DeclareLaunchArgument(
            'rgb_camera.color_profile',
            default_value=color_profile,
        ),
        DeclareLaunchArgument(
            'depth_module.depth_profile',
            default_value=depth_profile,
        ),
        DeclareLaunchArgument(
            'enable_depth',
            default_value=enable_depth,
        ),
        DeclareLaunchArgument(
            'color_qos',
            default_value=color_qos,
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
                'camera_namespace': camera_namespace,
                'rgb_camera.color_profile': color_profile,
                'depth_module.depth_profile': depth_profile,
                'enable_depth': enable_depth,
                'color_qos': color_qos,
            }.items(),
        )
    ])