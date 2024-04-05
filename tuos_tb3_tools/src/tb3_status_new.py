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
        self.startup = True
        self.robot_error = False
        self.active_nodes = []
        print("The new tb3_status node!!")
        
        rospy.init_node(self.node_name, anonymous=True)
        self.rate = rospy.Rate(1) # hz

        self.beeper = rospy.Publisher("/sound", Sound, queue_size = 10)
        self.battery = rospy.Subscriber("/battery_state", BatteryState, self.battery_callback)
        self.ros_errs = rospy.Subscriber("/rosout", Log, self.roserr_callback)
                
        self.ctrl_c = False
        rospy.on_shutdown(self.shutdownhook) 

        self.starttime = rospy.get_time()

    def shutdownhook(self):
        self.ctrl_c = True
    
    def battery_callback(self, topic_data: BatteryState):
        self.battery_voltage = topic_data.voltage
        self.capacity = int(min((60 * self.battery_voltage) - 650, 100)) # Approx. percentage (capped at 100%)
    
    def roserr_callback(self, rosout: Log):
        node = rosout.name
        msg = rosout.msg
        level = rosout.level
        if level > 2 and node in core_nodes and msg != "":
            self.robot_error = True
            print(f"Error from: {node}:\nmsg: {msg}")
        if msg == "Calibration End" and node == "/turtlebot3_core":
            self.startup = False

    def check_active_nodes(self):
        self.active_nodes = rosnode.get_node_names()
        return True if all([i in self.active_nodes for i in core_nodes]) else False

    def print_status_msg(self):
        ts = dt.datetime.now().strftime("%d/%m/%Y %H:%M:%S")
        runtime = rospy.get_time() - self.starttime
        if runtime > 60:
            runtimestring = f"{runtime / 60:.1f}"
            minsecs="minutes"
        else:
            runtimestring = f"{runtime:.0f}"
            minsecs="seconds"
        os.system('clear')
        print(f"{f'{ts: ^21}':#^32}")
        print(
            f"{'Device: ':>16}{hostname}\n"
            f"{'Status: ':>16}OK\n"
            f"{'Active Nodes: ':>16}{len(self.active_nodes)}\n"
            f"{'Up Time: ':>16}{runtimestring} {minsecs}\n"
            f"{'Voltage: ':>16}{self.battery_voltage:.2f}V [{self.capacity}%]"
        )
    
    def print_error_msg(self):
        ts = dt.datetime.now().strftime("%d/%m/%Y %H:%M:%S")
        print(f"\n{f'{ts: ^21}':#^32}")
        print(
            f"{'Device: ':>16}{hostname}\n"
            f"{'Status: ':>16}ERROR."
        )
        msg = Sound()
        msg.value = 3
        self.beeper.publish(msg)
    
    def main(self):
        while self.startup:
            continue
        
        msg = Sound()
        if self.check_active_nodes() and not self.robot_error:
            msg.value = 1
            rospy.loginfo("--------------------------")
            rospy.loginfo(f"{hostname} is up and running!")
            rospy.loginfo("--------------------------")
        else:
            msg.value = 3
            rospy.logerr("--------------------------")
            rospy.logerr(f"{hostname} failed to launch properly")
            rospy.logerr("--------------------------")
        self.beeper.publish(msg)

        timestamp = rospy.get_time()
        first_update_done = False
        while not self.ctrl_c:
            if (rospy.get_time() - timestamp) > 5:
                first_update_done = True
                if self.check_active_nodes() and not self.robot_error:
                    self.print_status_msg()
                else:
                    self.print_error_msg()
                timestamp = rospy.get_time()
            elif first_update_done:
                print(".", end="")
                sys.stdout.flush()

            self.rate.sleep()

if __name__ == '__main__':
    node = tb3Status()
    try:
        node.main()
    except rospy.ROSInterruptException:
        pass