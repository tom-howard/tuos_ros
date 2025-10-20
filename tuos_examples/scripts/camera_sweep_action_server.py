#!/usr/bin/env python3

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
from tuos_interfaces.action import CameraSweepJazzy as CameraSweep
from sensor_msgs.msg import Image

from geometry_msgs.msg import TwistStamped
from nav_msgs.msg import Odometry
from tuos_examples.tb3_tools import quaternion_to_euler

# Import some other useful Python Modules
from math import degrees
from pathlib import Path
import shutil

class CameraSweepActionServer(Node):
        
    def __init__(self):
        super().__init__("camera_sweep_action_server_node")

        self.vel_pub = self.create_publisher(
            msg_type=TwistStamped,
            topic="cmd_vel",
            qos_profile=10,
        )
        self.vel_pub.publish(TwistStamped())

        self.odom_sub = self.create_subscription(
            msg_type=Odometry,
            topic='odom',
            callback=self.odom_callback,
            qos_profile=10
        )
        
        self.camera_sub = self.create_subscription(
            msg_type=Image,
            topic="/camera/image_raw",
            callback=self.camera_callback,
            qos_profile=10
        )

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
        img = cv_image.imgmsg_to_cv2(
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
            self.vel_pub.publish(TwistStamped())
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
        vel_cmd = TwistStamped()
        vel_cmd.twist.angular.z = turn_vel
        
        # Get the robot's current yaw angle:
        ref_yaw = self.yaw
        yaw_inc = 0.0
        yaw_total = 0.0
        elapsed_time_seconds = 0.0

        self.base_image_path = Path.home().joinpath(
            f"ros_action_examples/")
        if self.base_image_path.exists():
            shutil.rmtree(self.base_image_path)
        self.base_image_path.mkdir()
        
        result.image_paths = []

        img_num = 0
        start_time_ns = self.get_clock().now().nanoseconds
        while img_num < goal.request.image_count:
            self.vel_pub.publish(vel_cmd)
            # check if there has been a request to cancel the action:
            if goal.is_cancel_requested:
                self.get_logger().info(
                    f"Cancelling the camera sweep at image {img_num} (of {goal.request.image_count})."
                )

                result.image_paths.append(f"CANCELLED at image {img_num} (of {goal.request.image_count})")                
                goal.canceled()
                # stop the robot:
                for i in range(5):
                    self.vel_pub.publish(TwistStamped())
                return result
            
            yaw_inc = yaw_inc + abs(self.yaw - ref_yaw)
            yaw_total = yaw_total + abs(self.yaw - ref_yaw)
            ref_yaw = self.yaw
            if yaw_inc >= ang_incs:
                img_num += 1 # increment the image counter
                timestamp_ns = self.get_clock().now().nanoseconds

                # populate a feedback message and publish it:
                feedback.current_image = img_num
                feedback.current_angle = yaw_total
                goal.publish_feedback(feedback)

                # reset the odometry reference:
                yaw_inc = 0.0

                # save the most recently captured image:
                img_path = str(self.base_image_path.joinpath(f"img{img_num:02.0f}.jpg"))
                cv2.imwrite(
                    img_path,
                    self.current_camera_image
                )
                result.image_paths.append(
                    img_path.replace(str(Path.home()), "~")
                )
                
        for i in range(5):
            self.vel_pub.publish(TwistStamped())

        elapsed_time_seconds = (timestamp_ns - start_time_ns) * 1e-9
        self.get_logger().info(
            f"{self.get_name()} completed successfully:\n"
            f"  - Angular sweep = {yaw_total:.2f} degrees\n"
            f"  - Images captured = {img_num}\n"
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