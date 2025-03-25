from launch import LaunchDescription 
from launch_ros.actions import Node 
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration

def generate_launch_description(): 
    return LaunchDescription([ 
        DeclareLaunchArgument(
            name='some_word', 
            description="A word, any word.",
            default_value='Hi'
        ),
        Node( 
            package='tuos_examples', 
            executable='param_publisher.py', 
            name='param_publisher_node',
            parameters=[{'word': LaunchConfiguration('some_word')}] 
        )
    ])