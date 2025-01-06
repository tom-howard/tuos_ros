#!/usr/bin/env python3

"""
Resources:
 - https://docs.ros.org/en/humble/Tutorials/Intermediate/Writing-an-Action-Server-Client/Py.html
 - https://github.com/ros2/examples/blob/humble/rclpy/actions/minimal_action_server/examples_rclpy_minimal_action_server/server.py
"""

import time

import rclpy
from rclpy.node import Node
from rclpy.action import ActionServer, GoalResponse, CancelResponse
from rclpy.executors import MultiThreadedExecutor
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.signals import SignalHandlerOptions

# Import all the necessary ROS message types:
from tuos_interfaces.action import Yaw
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from tuos_examples.tb3_tools import quaternion_to_euler

from math import degrees

class YawActionServer(Node):
        
    def __init__(self):
        super().__init__("waffle_yaw_action_server")

        self.vel_pub = self.create_publisher(
            msg_type=Twist,
            topic="cmd_vel",
            qos_profile=10,
        )

        self.odom_sub = self.create_subscription(
            msg_type=Odometry,
            topic="odom",
            callback=self.odom_callback,
            qos_profile=10)
        
        self.actionserver = ActionServer(
            node=self, 
            action_type=Yaw,
            action_name="waffle_yaw_controller",
            execute_callback=self.server_execution_callback,
            callback_group=ReentrantCallbackGroup(),
            goal_callback=self.goal_callback,
            cancel_callback=self.cancel_callback
        )

        self.shutdown = False
    
    def goal_callback(self, request):
        goal_ok = True
        if request.yaw_angle <= 0 or request.yaw_angle > 180:
            self.get_logger().warn(
                "Invalid yaw_angle! Select a value between 1 and 180 degrees."
            )
            goal_ok = False
        
        if request.msg_count <= 0:
            self.get_logger().warn(
                "Too few messages (msg_count must be greater than 0)."
            )
            goal_ok = False
        elif request.msg_count > 30:
            self.get_logger().warn(
                "Too many messages (I'll do a maximum of 30)."
            )
            goal_ok = False
        return GoalResponse.ACCEPT if goal_ok else GoalResponse.REJECT
    
    def cancel_callback(self, goal):
        self.get_logger().info('Received a cancel request...')
        return CancelResponse.ACCEPT

    def on_shutdown(self):
        for i in range(5):
            self.vel_pub.publish(Twist())
        self.shutdown = True

    def odom_callback(self, odom_msg):
        _, _, yaw = quaternion_to_euler(
            odom_msg.pose.pose.orientation
        )
        self.yaw = abs(degrees(yaw))
        
    def server_execution_callback(self, goal):
        result = Yaw.Result()
        feedback = Yaw.Feedback()
                
        # calculate the angular increments for messages:
        ang_incs = goal.request.yaw_angle/float(goal.request.msg_count)
        
        turn_vel = 0.3 # rad/s
        
        self.get_logger().info(
            "\n#####\n"
            f"The '{self.get_name()}' has been called.\n"
            f"Goal:\n"
            f"  - Turn by {goal.request.yaw_angle} degrees.\n"
            f"Feedback:\n"
            f"  - {goal.request.msg_count} messages (in total)\n"
            f"  - 1 message every {ang_incs:.3f} degrees\n"
            f"Angular Velocity:\n"
            f"  - {turn_vel} rad/s\n\n"
            f"Here we go...\n"
            f"#####\n"
        )
        time.sleep(2)

        # set the robot velocity:
        vel_cmd = Twist()
        vel_cmd.angular.z = turn_vel
        
        # Get the robot's current yaw angle:
        ref_yaw = self.yaw        
        yaw_inc = 0.0
        yaw_total = 0.0
        elapsed_time_seconds = 0.0

        i = 0
        start_time_ns = self.get_clock().now().nanoseconds
        while i < goal.request.msg_count:
            self.vel_pub.publish(vel_cmd)
            # check if there has been a request to cancel the action mid-way through:
            if goal.is_cancel_requested:
                self.get_logger().info(
                    "Cancelling the angular sweep."
                )

                result.summary = f"Cancelled at message {i} at a yaw angle of {yaw_total:.2f} degrees."                
                goal.canceled()
                # stop the robot:
                self.vel_pub.publish(Twist())
                return result

            yaw_inc = yaw_inc + abs(self.yaw - ref_yaw)
            yaw_total = yaw_total + abs(self.yaw - ref_yaw)
            ref_yaw = self.yaw

            if yaw_inc >= ang_incs:
                i += 1 # increment the message counter
                
                feedback.current_msg = i
                feedback.current_angle = yaw_total
                goal.publish_feedback(feedback)

                # update the odometry reference:
                yaw_inc = 0.0
                time_ns = self.get_clock().now().nanoseconds
                        
        for i in range(5):
            self.vel_pub.publish(Twist())

        self.get_logger().info(
            f"{self.get_name()} completed successfully."
        )
        goal.succeed()
        
        elapsed_time_seconds = (time_ns - start_time_ns)* 1e-9
        result.summary = f"Completed an angular sweep of {yaw_total:.2f} degrees in {elapsed_time_seconds:.2f} seconds"        
        return result

def main(args=None):
    rclpy.init(args=args,
        signal_handler_options=SignalHandlerOptions.NO)
    node = YawActionServer()
    executor = MultiThreadedExecutor()
    executor.add_node(node)
    try:
        node.get_logger().info(
            f"Starting {node.get_name()}. Awaiting a goal..."
        )
        executor.spin()
    except KeyboardInterrupt:
        node.get_logger().info(
            f"{node.get_name()} received a shutdown request (Ctrl+C)."
        )
    finally:
        node.on_shutdown()
        while not node.shutdown:
            continue
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()