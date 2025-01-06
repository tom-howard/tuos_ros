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

# # Import some image processing modules:
# import cv2
# from cv_bridge import CvBridge

# Import all the necessary ROS message types:
from tuos_interfaces.action import CameraSweep
# from sensor_msgs.msg import CompressedImage

from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from tuos_examples.tb3_tools import quaternion_to_euler

# # Import some helper functions from the tb3.py module within this package
# from tb3 import Tb3Move, Tb3Odometry

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

        self.subscription = self.create_subscription(
            Odometry,
            'odom',
            self.odom_callback,
            10)
        
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

        # self.camera_subscriber = rospy.Subscriber("/camera/rgb/image_raw/compressed",
        #     CompressedImage, self.camera_callback)
        # self.cv_image = CvBridge()

        # self.robot_controller = Tb3Move()
        # self.robot_odom = Tb3Odometry()
    
    # def camera_callback(self, img):
    #     image_to_capture = self.cv_image.compressed_imgmsg_to_cv2(img, desired_encoding="passthrough")
    #     self.current_camera_image = image_to_capture
    
    def goal_callback(self, request):
        goal_ok = True
        if request.sweep_angle <= 0 or request.sweep_angle > 180:
            self.get_logger().warn(
                "Invalid sweep_angle! Select a value between 1 and 180 degrees."
            )
            goal_ok = False
        
        if request.image_count <= 0:
            self.get_logger().warn(
                "Too few images (image_count must be greater than 0)."
            )
            goal_ok = False
        elif request.image_count > 50:
            self.get_logger().warn(
                "Too many images (I'll accept a maximum of 50)."
            )
            goal_ok = False

        return GoalResponse.ACCEPT if goal_ok else GoalResponse.REJECT
    
    def cancel_callback(self, goal):
        self.get_logger().info('Received cancel request')
        return CancelResponse.ACCEPT

    def on_shutdown(self):
        for i in range(5):
            self.vel_pub.publish(Twist())
        self.shutdown = True

    def odom_callback(self, odom_msg):
        # pos_x = odom_msg.pose.pose.position.x
        # pos_y = odom_msg.pose.pose.position.y

        _, _, yaw = quaternion_to_euler(
            odom_msg.pose.pose.orientation
        )
        self.yaw = degrees(yaw)
        self.get_logger().info(f"odom callback says: {self.yaw}",throttle_duration_sec=1)

    def server_execution_callback(self, goal):
        result = CameraSweep.Result()
        feedback = CameraSweep.Feedback()

        # success = True
        self.vel_pub.publish(Twist())
        
        # calculate the angular increments over which to capture images:
        ang_incs = goal.request.sweep_angle/float(goal.request.image_count)
        # and the time it will take to perform the action:
        turn_vel = 0.2 # rad/s
        full_sweep_time = radians(goal.request.sweep_angle)/abs(turn_vel)

        print(f"\n#####\n"
            f"The '{self.get_name()}' has been called.\n"
            f"Goal: capture {goal.request.image_count} images over a {goal.request.sweep_angle} degree sweep...\n\n"
            f"An image will therefore be captured every {ang_incs:.3f} degrees,\n"
            f"and the full sweep will take {full_sweep_time:.5f} seconds.\n\n"
            f"Commencing the action...\n"
            f"#####\n")
        
        # set the robot velocity:
        vel_cmd = Twist()
        vel_cmd.angular.z = turn_vel
        
        # Get the robot's current yaw angle:
        ref_yaw = self.yaw

        # Get the current date and time and create a timestamp string of it
        # (to use when we construct the image filename):
        start_time = dt.datetime.strftime(dt.datetime.now(),'%Y%m%d_%H%M%S')
        self.base_image_path = Path.home().joinpath(f"myrosdata/action_examples/{start_time}/")
                
        i = 0
        while i < goal.request.image_count:
            self.vel_pub.publish(vel_cmd)
            # check if there has been a request to cancel the action mid-way through:
            if goal.is_cancel_requested:
                self.get_logger().info(
                    "Cancelling the camera sweep."
                )

                result.image_path = f"{result.image_path} [PRE-EMPTED]"                
                goal.canceled()
                # stop the robot:
                self.vel_pub.publish(Twist())
                # exit the loop:
                return result
            
            self.get_logger().info(
                f"{abs(self.yaw - ref_yaw)} deg", 
                throttle_duration_sec = 1)

            if abs(self.yaw - ref_yaw) >= ang_incs:
                # increment the image counter
                i += 1
                
                # populate the feedback message and publish it:
                self.get_logger().info(
                    f"Captured image {i}"
                )
                feedback.current_image = i
                feedback.current_angle = abs(self.yaw)
                goal.publish_feedback(feedback)

                # update the reference odometry:
                ref_yaw = self.yaw

                # save the most recently captured image:
                self.get_logger().info(
                    str(self.base_image_path.joinpath(f"img{i:03.0f}.jpg"))
                )
                # self.base_image_path.mkdir(parents=True, exist_ok=True)
                # cv2.imwrite(str(self.base_image_path.joinpath(f"img{i:03.0f}.jpg")), 
                #     self.current_camera_image)
                
        for i in range(5):
            self.vel_pub.publish(Twist())

        self.get_logger().info(
            "Camera sweep completed successfully."
        )
        goal.succeed()
        result.image_path = str(self.base_image_path).replace(str(Path.home()), "~")
        
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