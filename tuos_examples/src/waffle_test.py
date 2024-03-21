#!/usr/bin/env python3

import rospy
import waffle

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

counter = 0
inc = 1
while not rospy.is_shutdown():

    rate.sleep()
    counter += inc
    if counter > 50:
        inc = -1
    elif counter < 0:
        inc = 1
    
    if inc > 0:
        motion.set_velocity(angular = 0.3)
        odom.show()
    elif inc < 0:
        motion.set_velocity()
        lidar.subsets.show()
    
    motion.publish_velocity()