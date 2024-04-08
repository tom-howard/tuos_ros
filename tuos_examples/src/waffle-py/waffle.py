#!/usr/bin/env python3
"""
A module for handling key Waffle Operations
Like the tb3.py module, but more advanced!

Tom Howard, March 2024
"""

import rospy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan
from tf.transformations import euler_from_quaternion
from math import degrees
import numpy as np

class Motion():

    """
    Creates a `/cmd_vel` publisher and has callable methods to control velocity

    Optional:
        Set `debug=True/False` to enable/disable ROS log/warn messages (default = `True`) 
    """

    def __init__(self, debug = True):
        self.dbg = debug
        self.publisher = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
        self.publisher_rate = rospy.Rate(10) # Hz
        self.vel_cmd = Twist()
        self.timestamp = rospy.get_time()
        self.high_rate_warn = 1.0 / 1000.0 # seconds
        self.low_rate_warn = 1.0 / 2.0 # seconds

    def set_velocity(self, linear = 0.0, angular = 0.0):
        """
        Creates a `Twist()` message with the specified velocities:
        `linear = Twist().linear.x`, default = 0.0 [Units: m/s]
        `angular = Twist().angular.z`, default = 0.0 [Units: rad/s] 
        """
        if abs(linear) > 0.26:
            lin_org = linear
            linear = np.sign(linear) * 0.26
            if self.dbg: rospy.logwarn(f"LINEAR velocity limited to {linear} m/s ({lin_org} m/s requested).")

        if abs(angular) > 1.82:
            ang_org = angular
            angular = np.sign(angular) * 1.82
            if self.dbg: rospy.logwarn(f"ANGULAR velocity limited to {angular} rad/s ({ang_org} rad/s requested).")
        
        self.vel_cmd.linear.x = linear
        self.vel_cmd.angular.z = angular
        
    def stop(self):
        """
        Creates a Twist() message with all velocities set to zero
        and publishes this to /cmd_vel.
        """
        self.dbg = False
        self.vel_cmd = Twist()
        self.publish_velocity()

    def publish_velocity(self):
        """
        Publishes a pre-configured Twist message to /cmd_vel.

        The Twist message is configured by calling:

            Motion().set_velocity(linear= , angular= )
        """
        last_published = rospy.get_time() - self.timestamp 
        self.timestamp = rospy.get_time()
        self.publisher.publish(self.vel_cmd)
        if last_published < self.high_rate_warn:
            if self.dbg: rospy.logwarn("You're sending velocity commands too quickly!\n" \
                                       "Check your loop rates!")
        if last_published > self.low_rate_warn:
            if self.dbg: rospy.logwarn("You aren't publishing velocity commands quickly enough to ensure smooth motion!\n" \
                                       "Check your loop rates!")

class Pose():

    """
    Creates an `/odom` subscriber and handles the robot's `Odometry` data

    Optional:
        Set `debug=True/False` to enable/disable ROS log/warn messages (default = `True`)
    """

    def odom_cb(self, odom_data: Odometry):
        orientation = odom_data.pose.pose.orientation
        position = odom_data.pose.pose.position
        (_, _, yaw) = euler_from_quaternion(
            [orientation.x, orientation.y, orientation.z, orientation.w],
            'sxyz'
        )
        
        yaw = self._round(degrees(yaw), 4)
        self.yaw_direction = np.sign(yaw)
        self.yaw = abs(yaw)
        self.posx = self._round(position.x, 4)
        self.posy = self._round(position.y, 4)

        self.wait_for_odom = False
    
    def __init__(self, debug = True):
        self.dbg = debug
        self.posx = 0.0; self.posy = 0.0; self.yaw = 0.0
        self.yaw_direction = 0.0
        self.subscriber = rospy.Subscriber('/odom', Odometry, self.odom_cb)
        self.timestamp = rospy.get_time()
        self.wait_for_odom = True
        if self.dbg: rospy.loginfo('Waiting for Odometry Data...')
        while self.wait_for_odom:
            continue
        if self.dbg: rospy.loginfo('Odometry Data is available...')
    
    def show(self):
        """
        A method to return the robot's current odometry.

        Messages will only be displayed once per second, regardless of 
        the rate at which the show() method is called.
        """
        if (rospy.get_time() - self.timestamp) > 1:
            self.timestamp = rospy.get_time()
            print(
                "Odometry Data:\n" \
                f"posx = {self.posx:.3f} (m) posy = {self.posy:.3f} (m), yaw = {self.yaw:.1f} (degrees)\n" \
                "---"
            )
        
    def _round(self, value, precision):
        value = int(value * (10**precision))
        return float(value) / (10**precision)

class Lidar():

    """
    Creates a `/scan` subscriber and handles `LaserScan` data from the Waffle's LiDAR sensor

    Optional:
        Set `debug=True/False` to enable/disable ROS log/warn messages (default = `True`)
    """

    def __init__(self, debug = True):
        self.dbg = debug
        self.subsets = self.scanSubsets()
        self.subscriber = rospy.Subscriber('/scan', LaserScan, self.laserscan_cb)
        self.wait_for_lidar = True
        if self.dbg: rospy.loginfo('Waiting for LiDAR Data...')
        while self.wait_for_lidar:
            continue
        if self.dbg: rospy.loginfo('LiDAR Data is available...')
    
    class scanSubsets():
        def __init__(self):
            self.timestamp = rospy.get_time()
            self.front = 0.0; self.frontArray = []
            self.r1 = 0.0; self.r2 = 0.0; self.r3 = 0.0; self.r4 = 0.0
            self.l1 = 0.0; self.l2 = 0.0; self.l3 = 0.0; self.l4 = 0.0
            self.r1Array = []; self.r2Array = []; self.r3Array = []; self.r4Array = []
            self.l1Array = []; self.l2Array = []; self.l3Array = []; self.l4Array = []
        
        def show(self):
            """
            A function to return distance readings from each LiDAR subset.

            Messages will only be displayed once per second, regardless of 
            the rate at which the show() method is called.
            """

            if (rospy.get_time() - self.timestamp) > 1:
                msg = "LiDAR Subset Readings (meters):\n" \
                    f"              l1     front     r1          \n" \
                    f"       l2     {self.l1:<5.3f}  {self.front:^5.3f}  {self.r1:>5.3f}     r2       \n" \
                    f"l3     {self.l2:<5.3f}                       {self.r2:>5.3f}   r3\n" \
                    f"{self.l3:<5.3f}                                   {self.r3:>5.3f}\n" \
                    f"{self.l4:<5.3f} <-- l4                     r4 --> {self.r4:>5.3f}\n" \
                    "---"
                self.timestamp = rospy.get_time()
                print(msg)
                        
    def filter(self, lidar_subset):
        valid_data = lidar_subset[(lidar_subset > 0.1) & (lidar_subset != float("inf"))]
        subset_array = valid_data.tolist()
        subset_value = valid_data.mean() if np.shape(valid_data)[0] > 0 else float("nan")
        return subset_value, subset_array

    def get_subset(self, start_index, stop_index):
        subset = np.array(self.rangesArray[start_index: stop_index+1])
        return self.filter(subset)
    
    def laserscan_cb(self, scan_data: LaserScan):
        
        self.rangesArray = scan_data.ranges

        ## Get LiDAR Subsets:
        # Front:
        left = scan_data.ranges[0:20+1]
        right = scan_data.ranges[-20:]
        left_right = np.array(left[::-1] + right[::-1])
        self.subsets.front, self.subsets.frontArray = self.filter(left_right)
        
        # Right (x4):
        self.subsets.r1, self.subsets.r1Array = self.get_subset(320, 340)
        self.subsets.r2, self.subsets.r2Array = self.get_subset(300, 320)
        self.subsets.r3, self.subsets.r3Array = self.get_subset(275, 290)
        self.subsets.r4, self.subsets.r4Array = self.get_subset(250, 265)
        
        # Left (x4):
        self.subsets.l1, self.subsets.l1Array = self.get_subset(20, 40)
        self.subsets.l2, self.subsets.l2Array = self.get_subset(40, 60)
        self.subsets.l3, self.subsets.l3Array = self.get_subset(70, 85)
        self.subsets.l4, self.subsets.l4Array = self.get_subset(95, 110)
        
        self.wait_for_lidar = False
