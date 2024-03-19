#!/usr/bin/env python3

import rospy
import waffle

node_name = "waffle_py_example"

rospy.init_node(node_name, anonymous=True)
rate = rospy.Rate(1) # hz
rospy.loginfo(f"{node_name}: Initialised.")

motion = waffle.Motion(debug = True)
lidar = waffle.Lidar(debug = True)
odom = waffle.Pose(debug = True)

def stop():
    motion.stop()

rospy.on_shutdown(stop)

motion.set_velocity(linear=0.05, angular=0.0)
while not rospy.is_shutdown():

    lidar.distance.show()
    odom.show()
    try:
        rate.sleep()
    except rospy.ROSInterruptException:
        pass
    motion.publish_velocity()