#!/usr/bin/env python3

import os
import sys
import rospy
import rosnode
from socket import gethostname
import datetime as dt
from turtlebot3_msgs.msg import Sound
from sensor_msgs.msg import BatteryState

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
        self.active_nodes = []
        
        rospy.init_node(self.node_name, anonymous=True)
        self.rate = rospy.Rate(1) # hz

        self.beeper = rospy.Publisher("/sound", Sound, queue_size = 10)
        self.battery = rospy.Subscriber("/battery_state", BatteryState, self.battery_callback)
                
        self.ctrl_c = False
        rospy.on_shutdown(self.shutdownhook) 

        timestamp = rospy.get_time()
        self.starttime = rospy.get_time()
        while (rospy.get_time() - timestamp) < 10:
            continue
        self.startup = False

    def shutdownhook(self):
        self.ctrl_c = True
    
    def battery_callback(self, topic_data: BatteryState):
        self.battery_voltage = topic_data.voltage
        self.capacity = int(min((60 * self.battery_voltage) - 650, 100)) # Approx. percentage (capped at 100%)
    
    def check_active_nodes(self):
        self.active_nodes = rosnode.get_node_names()
        return True if all([i in self.active_nodes for i in core_nodes]) else False

    def main(self):
        timestamp = rospy.get_time()
        while self.startup:
            continue
        
        if self.check_active_nodes():
            msg = Sound()
            msg.value = 1
            rospy.loginfo("--------------------------")
            rospy.loginfo(f"{hostname} is up and running!")
            rospy.loginfo("--------------------------")
            self.beeper.publish(msg)
        
        first_update = False
        while not self.ctrl_c:
            if (rospy.get_time() - timestamp) > 10:
                first_update = True
                
                ts = dt.datetime.now().strftime("%d/%m/%Y %H:%M:%S")
                if self.check_active_nodes():
                    rts = rospy.get_time() - self.starttime
                    if rts > 60:
                        runtimestring = f"{rts / 60:.1f}"
                        minsecs="minutes"
                    else:
                        runtimestring = f"{rts:.0f}"
                        minsecs="seconds"
                    os.system('clear')
                    print(f"{f'{ts: ^21}':#^32}")
                    print(f"{'Device: ':>16}{hostname}\n"
                          f"{'Status: ':>16}OK\n"
                          f"{'Active Nodes: ':>16}{len(self.active_nodes)}\n"
                          f"{'Up Time: ':>16}{runtimestring} {minsecs}\n"
                          f"{'Voltage: ':>16}{self.battery_voltage:.2f}V [~{self.capacity}%]"
                    )
                else:
                    print(
                        f"[{ts}] Waffle Status: ERROR\n"
                        f"Core node(s) not running!"
                        )
                timestamp = rospy.get_time()
            elif first_update:
                print(".", end="")
                sys.stdout.flush()

            self.rate.sleep()

if __name__ == '__main__':
    node = tb3Status()
    try:
        node.main()
    except rospy.ROSInterruptException:
        pass