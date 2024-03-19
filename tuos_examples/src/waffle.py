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
    def __init__(self, debug = True):
        self.dbg = debug
        self.publisher = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
        self.publisher_rate = rospy.Rate(10) # Hz
        self.vel_cmd = Twist()
        self.timestamp = rospy.get_time()
        self.high_rate_warn = 1.0 / 1000.0 # seconds
        self.low_rate_warn = 1.0 / 2.0 # seconds
        self.hush = True

    def set_velocity(self, linear = 0.0, angular = 0.0):
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
        self.set_velocity()
        self.hush = True
        self.publish_velocity()

    def publish_velocity(self):
        last_published = rospy.get_time() - self.timestamp 
        self.timestamp = rospy.get_time()
        self.publisher.publish(self.vel_cmd)
        if self.hush:
            self.hush = False
        else:
            if last_published < self.high_rate_warn:
                if self.dbg: rospy.logwarn("You're sending velocity commands too quickly!\n" \
                                "Check your loop rates!")
            if last_published > self.low_rate_warn:
                if self.dbg: rospy.logwarn("You aren't publishing velocity commands quickly enough to ensure smooth motion!\n" \
                                "Check your loop rates!")

class Pose():
    def odom_cb(self, odom_data: Odometry):
        orientation = odom_data.pose.pose.orientation
        position = odom_data.pose.pose.position
        (_, _, yaw) = euler_from_quaternion([orientation.x,
            orientation.y, orientation.z, orientation.w], 'sxyz')
        
        yaw = self.round(degrees(yaw), 4)
        self.yaw_direction = np.sign(yaw)
        self.yaw = abs(yaw)
        self.posx = self.round(position.x, 4)
        self.posy = self.round(position.y, 4)

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
        A method to return the robot's current odometry 
        Messages will only be displayed once per second, regardless of 
        the rate at which the show() method is called.
        """
        if (rospy.get_time() - self.timestamp) > 1:
            self.timestamp = rospy.get_time()
            print(f"posx = {self.posx:.3f} (m) posy = {self.posy:.3f} (m), yaw = {self.yaw:.1f} (degrees)")
        
    def round(self, value, precision):
        value = int(value * (10**precision))
        return float(value) / (10**precision)

class Lidar():

    def __init__(self, debug = True):
        self.dbg = debug
        self.distance = self.scanSubsets()
        self.subscriber = rospy.Subscriber('/scan', LaserScan, self.laserscan_cb)
        self.wait_for_lidar = True
        if self.dbg: rospy.loginfo('Waiting for LiDAR Data...')
        while self.wait_for_lidar:
            continue
        if self.dbg: rospy.loginfo('LiDAR Data is available...')
    
    class scanSubsets():
        def __init__(self):
            self.timestamp = rospy.get_time()
            self.front = 0.0
            self.r1 = 0.0; self.r2 = 0.0; self.r3 = 0.0; self.r4 = 0.0
            self.l1 = 0.0; self.l2 = 0.0; self.l3 = 0.0; self.l4 = 0.0
        
        def show(self):
            """
            A function to return distance readings from each LiDAR subset
            Messages will only be displayed once per second, regardless of 
            the rate at which the show() method is called.
            """

            if (rospy.get_time() - self.timestamp) > 1:
                msg = f"              l1     front     r1          \n" \
                    f"       l2     {self.l1:<5.3f}  {self.front:^5.3f}  {self.r1:>5.3f}     r2       \n" \
                    f"l3     {self.l2:<5.3f}                       {self.r2:>5.3f}   r3\n" \
                    f"{self.l3:<5.3f}                                   {self.r3:>5.3f}\n" \
                    f"{self.l4:<5.3f} <-- l4                     r4 --> {self.r4:>5.3f}"
                self.timestamp = rospy.get_time()
                print(msg)
                        
    def laserscan_cb(self, scan_data: LaserScan):
        
        def strip_oor(lidar_subset):
            valid_data = lidar_subset[(lidar_subset > 0.1) & (lidar_subset != float("inf"))]
            return valid_data.mean() if np.shape(valid_data)[0] > 0 else float("nan")

        def lidar_subset(start_index, stop_index):
            range = np.array(scan_data.ranges[start_index: stop_index+1])
            return strip_oor(range)

        # front:
        left = scan_data.ranges[0:20+1]
        right = scan_data.ranges[-20:]
        left_right = np.array(left[::-1] + right[::-1])
        self.distance.front = strip_oor(left_right)
        
        # right subsets:
        self.distance.r1 = lidar_subset(320, 340)
        self.distance.r2 = lidar_subset(300, 320)
        self.distance.r3 = lidar_subset(275, 290)
        self.distance.r4 = lidar_subset(250, 265)
        
        # left subsets:
        self.distance.l1 = lidar_subset(20, 40)
        self.distance.l2 = lidar_subset(40, 60)
        self.distance.l3 = lidar_subset(70, 85)
        self.distance.l4 = lidar_subset(95, 110)
        
        self.wait_for_lidar = False
