#!/usr/bin/env python3
"""
A simple ROS publisher node with a Command-line Interface (CLI)

See here for further info:
https://tom-howard.github.io/com2009/course/extras/launch-files/
"""

import rclpy
from rclpy.node import Node
from example_interfaces.msg import String

class Publisher(Node):
    
    def __init__(self):
        super().__init__("param_publisher")

        self.declare_parameter('word', 'hello')
        
        self.publisher = self.create_publisher(
            msg_type=String, topic="chatter", qos_profile=10
        )
        self.timer = self.create_timer(
            timer_period_sec=1, callback=self.loop
        )

        self.get_logger().info(
            f"The '{self.get_name()}' node is running..." 
        )

    def loop(self):
        param = self.get_parameter('word').get_parameter_value().string_value
        msg = String()
        msg.data = f"{param}"
        self.publisher.publish(msg)
        self.get_logger().info(
            f"Publishing the word '{msg.data}'."
        )

def main():
    rclpy.init()
    node = Publisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()