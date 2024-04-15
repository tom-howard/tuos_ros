#!/usr/bin/env python3

import os
import sys
import rospy
import rosnode
from socket import gethostname
import datetime as dt
from turtlebot3_msgs.msg import Sound
from sensor_msgs.msg import BatteryState
from rosgraph_msgs.msg import Log

hostname = gethostname()

core_nodes = [
    '/turtlebot3_core', 
    '/turtlebot3_lds', 
    '/rosout', 
    '/camera/realsense2_camera', 
    '/camera/realsense2_camera_manager', 
    '/turtlebot3_diagnostics',
]

class tb3Status():
    
    def __init__(self):
        self.node_name = "tb3_status"
        self.robot_status_string = "OK"
        self.startup_complete = False
        self.waffle_core_errors = False
        self.active_nodes = []
        
        rospy.init_node(self.node_name, anonymous=True)
        self.rate = rospy.Rate(1) # hz

        self.beeper = rospy.Publisher("/sound", Sound, queue_size = 10)
        self.battery = rospy.Subscriber("/battery_state", BatteryState, self.battery_callback)
        self.ros_errors = rospy.Subscriber("/rosout", Log, self.rosout_cb)

        self.ctrl_c = False
        rospy.on_shutdown(self.shutdownhook) 

        self.starttime = rospy.get_time()
        self.timestamp = rospy.get_time()

    def shutdownhook(self):
        self.ctrl_c = True
    
    def battery_callback(self, topic_data: BatteryState):
        self.battery_voltage = topic_data.voltage
        self.capacity = int(min((60 * self.battery_voltage) - 650, 100)) # Approx. percentage (capped at 100%)
    
    def rosout_cb(self, rosout: Log):
        node = rosout.name
        msg = rosout.msg
        level = rosout.level

        if msg == "Calibration End" and node == "/turtlebot3_core":
            self.startup_complete = True
        
        if level > 2 and node in core_nodes and msg != "":
            self.waffle_core_errors = True

    def check_active_nodes(self):
        self.active_nodes = rosnode.get_node_names()
        return True if all([i in self.active_nodes for i in core_nodes]) else False

    def print_status_msg(self):
        now = rospy.get_time()
        if (now - self.timestamp) > 5:
            msg_timestamp = dt.datetime.now().strftime("%d/%m/%Y %H:%M:%S")
            runtime = now - self.starttime
            if runtime > 60:
                runtimestring = f"{runtime / 60:.1f}"
                units="minutes"
            else:
                runtimestring = f"{runtime:.0f}"
                units="seconds"
            self.waffle_beeper(hush_if_ok=True)
            os.system('clear')
            print(f"{f'{msg_timestamp: ^21}':#^32}")
            print(
                f"{'Device: ':>16}{hostname}\n"
                f"{'Status: ':>16}{self.robot_status_string}\n"
                f"{'Active Nodes: ':>16}{len(self.active_nodes)}\n"
                f"{'Up Time: ':>16}{runtimestring} {units}\n"
                f"{'Battery: ':>16}{self.battery_voltage:.2f}V [{self.capacity}%]"
            )
            self.timestamp = rospy.get_time()
        else:
            print(".", end="")
            sys.stdout.flush()
    
    def waffle_beeper(self, hush_if_ok = True):
        robot_status_ok = self.check_active_nodes() and not self.waffle_core_errors
        self.robot_status_string = "OK" if robot_status_ok else "ERRORS"
        if robot_status_ok and hush_if_ok:
            pass
        else:
            msg = Sound()
            msg.value = 1 if robot_status_ok else 3
            self.beeper.publish(msg)
        return robot_status_ok

    def main(self):
        while not self.startup_complete:
            continue
        
        bringup_errors = False
        if self.waffle_beeper(hush_if_ok=False):
            rospy.loginfo("--------------------------")
            rospy.loginfo(f"{hostname} is up and running!")
            rospy.loginfo("--------------------------")
        else:
            rospy.logerr("--------------------------")
            rospy.logerr(f"{hostname} failed to launch correctly, please try again.")
            rospy.logerr("--------------------------")
            bringup_errors = True
        
        if not bringup_errors:
            self.timestamp = rospy.get_time()
            while not self.ctrl_c:
                self.print_status_msg()
                self.rate.sleep()
        else:
            rospy.spin()

if __name__ == '__main__':
    node = tb3Status()
    try:
        node.main()
    except rospy.ROSInterruptException:
        pass