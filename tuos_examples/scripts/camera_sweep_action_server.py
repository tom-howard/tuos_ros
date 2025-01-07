#!/usr/bin/env python3

"""
Resources:
 - https://docs.ros.org/en/humble/Tutorials/Intermediate/Writing-an-Action-Server-Client/Py.html
 - https://github.com/ros2/examples/blob/humble/rclpy/actions/minimal_action_server/examples_rclpy_minimal_action_server/server.py
"""

import rclpy
from rclpy.node import Node
from rclpy.action import ActionServer, GoalResponse, CancelResponse
from rclpy.executors import MultiThreadedExecutor
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.signals import SignalHandlerOptions

# Import some image processing modules:
import cv2
from cv_bridge import CvBridge

# Import all the necessary ROS message types:
from tuos_interfaces.action import CameraSweep
from sensor_msgs.msg import CompressedImage

from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from tuos_examples.tb3_tools import quaternion_to_euler

# Import some other useful Python Modules
from math import radians, degrees
import datetime as dt
from pathlib import Path

class CameraSweepActionServer(Node):
        
    def __init__(self):
        super().__init__("camera_sweep_action_server")

        self.vel_pub = self.create_publisher(
            msg_type=Twist,
            topic="cmd_vel",
            qos_profile=10,
        )
        self.vel_pub.publish(Twist())

        self.odom_sub = self.create_subscription(
            Odometry,
            'odom',
            self.odom_callback,
            10)
        
        self.camera_sub = self.create_subscription(
            msg_type=CompressedImage,
            topic="/camera/image_raw/compressed",
            callback=self.camera_callback,
            qos_profile=10)

        self.actionserver = ActionServer(
            node=self, 
            action_type=CameraSweep,
            action_name="camera_sweep",
            execute_callback=self.server_execution_callback,
            callback_group=ReentrantCallbackGroup(),
            goal_callback=self.goal_callback,
            cancel_callback=self.cancel_callback
        )

        self.shutdown = False

    def odom_callback(self, odom_msg):
        _, _, yaw = quaternion_to_euler(
            odom_msg.pose.pose.orientation
        )
        self.yaw = abs(degrees(yaw))
        
    def camera_callback(self, img_msg):
        cv_image = CvBridge()
        img = cv_image.compressed_imgmsg_to_cv2(
            img_msg, desired_encoding="passthrough")
        self.current_camera_image = img
    
    def goal_callback(self, request):
        goal_ok = True
        if request.sweep_angle <= 0 or request.sweep_angle > 180:
            self.get_logger().warn(
                "Invalid sweep_angle! Select a value between 1 and 180 degrees."
            )
            goal_ok = False
        
        if request.image_count <= 0:
            self.get_logger().warn(
                "Not enough images (image_count must be greater than 0)."
            )
            goal_ok = False
        elif request.image_count > 30:
            self.get_logger().warn(
                "Too many images (I'll do a maximum of 30)."
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

    def server_execution_callback(self, goal):
        result = CameraSweep.Result()
        feedback = CameraSweep.Feedback()

        # calculate the angular increments over which to capture images:
        ang_incs = goal.request.sweep_angle/float(goal.request.image_count)
        turn_vel = 0.2 # rad/s
        
        self.get_logger().info(
            f"\n#####\n"
            f"The '{self.get_name()}' has been called.\n"
            f"Goal:\n"
            f"  - Perform a {goal.request.sweep_angle} degree sweep\n"
            f"  - Capture {goal.request.image_count} images\n" 
            f"Info:\n"
            f"  - An image will be captured every {ang_incs:.3f} degrees\n"
            f"  - Angular velocity: {turn_vel} rad/s.\n\n"
            f"Here we go..."
            f"\n#####\n")
        
        # set the robot's velocity:
        vel_cmd = Twist()
        vel_cmd.angular.z = turn_vel
        
        # Get the robot's current yaw angle:
        ref_yaw = self.yaw
        yaw_inc = 0.0
        yaw_total = 0.0
        elapsed_time_seconds = 0.0

        # Get the current date and time and create a timestamp string
        # (to use when we construct the image filename):
        start_time = dt.datetime.strftime(
            dt.datetime.now(),'%Y%m%d_%H%M%S')
        self.base_image_path = Path.home().joinpath(
            f"myrosdata/action_examples/{start_time}/")
        result.image_path = str(self.base_image_path).replace(str(Path.home()), "~")
                
        i = 0
        start_time_ns = self.get_clock().now().nanoseconds
        while i < goal.request.image_count:
            self.vel_pub.publish(vel_cmd)
            # check if there has been a request to cancel the action:
            if goal.is_cancel_requested:
                self.get_logger().info(
                    "Cancelling the camera sweep."
                )

                result.image_path = f"{result.image_path} [CANCELLED at image {i} (of {goal.request.image_count})]"                
                goal.canceled()
                # stop the robot:
                self.vel_pub.publish(Twist())
                return result
            
            yaw_inc = yaw_inc + abs(self.yaw - ref_yaw)
            yaw_total = yaw_total + abs(self.yaw - ref_yaw)
            ref_yaw = self.yaw
            if yaw_inc >= ang_incs:
                i += 1 # increment the image counter
                timestamp_ns = self.get_clock().now().nanoseconds

                # populate a feedback message and publish it:
                feedback.current_image = i
                feedback.current_angle = yaw_total
                goal.publish_feedback(feedback)

                # reset the odometry reference:
                yaw_inc = 0.0

                # save the most recently captured image:
                self.base_image_path.mkdir(parents=True, exist_ok=True)
                cv2.imwrite(
                    str(self.base_image_path.joinpath(f"img{i:03.0f}.jpg")),
                    self.current_camera_image
                )
                
        for i in range(5):
            self.vel_pub.publish(Twist())

        elapsed_time_seconds = (timestamp_ns - start_time_ns) * 1e-9
        self.get_logger().info(
            f"{self.get_name()} completed successfully:\n"
            f"  - Angular sweep = {yaw_total:.2f} degrees\n"
            f"  - Images captured = {i}\n"
            f"  - Time taken = {elapsed_time_seconds:.2f} seconds"
        )
        goal.succeed()
        return result

def main(args=None):
    rclpy.init(args=args,
        signal_handler_options=SignalHandlerOptions.NO)
    node = CameraSweepActionServer()
    executor = MultiThreadedExecutor()
    executor.add_node(node)
    try:
        node.get_logger().info(
            "Starting the Camera Sweep Action Server (shut down with Ctrl+C)"
        )
        executor.spin()
    except KeyboardInterrupt:
        node.get_logger().info(
            "Camera Sweep Action Server shut down with Ctrl+C"
        )
    finally:
        node.on_shutdown()
        while not node.shutdown:
            continue
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()