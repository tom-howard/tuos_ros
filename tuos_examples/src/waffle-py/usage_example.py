#!/usr/bin/env python3

import rospy
import waffle
from math import sqrt

node_name = "waffle_py_example"

rospy.init_node(node_name, anonymous=True)
rate = rospy.Rate(10) # hz
rospy.loginfo(f"{node_name}: Initialised.")

motion = waffle.Motion(debug = True)
lidar = waffle.Lidar(debug = True)
odom = waffle.Pose(debug = True)

def stop():
    motion.stop()

rospy.on_shutdown(stop)

movement = "turn" # "move_fwd"
transition = True

while not rospy.is_shutdown():

    if transition:
        motion.stop()
        yaw_ref = odom.yaw
        xpos_ref = odom.posx
        ypos_ref = odom.posy
        current_yaw = 0.0
        current_distance = 0.0
        transition = False
    elif movement == "turn":
        # turn
        current_yaw += abs(odom.yaw - yaw_ref); yaw_ref = odom.yaw
        if current_yaw > 45:
            movement = "move_fwd"
            transition = True
        else:
            motion.set_velocity(angular=0.2)
            odom.show()
    elif movement == "move_fwd":
        current_distance += sqrt((odom.posx - xpos_ref)**2 + (odom.posy - ypos_ref)**2)
        xpos_ref = odom.posx; ypos_ref = odom.posy
        if current_distance > 0.5:
            movement = "turn"
            transition = True
        else:
            motion.set_velocity(linear=0.1)
            lidar.subsets.show()   
    
    if lidar.subsets.front < 0.5:
        rospy.loginfo(f"Detected an object ahead at {lidar.subsets.front:.2f}m. Stopping!")
        break
    else:
        motion.publish_velocity()
        rate.sleep()