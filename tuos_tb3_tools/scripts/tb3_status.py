#!/usr/bin/env python3

import os
import sys

import rclpy
from rclpy.node import Node

from socket import gethostname
import datetime as dt
import rclpy.signals
from turtlebot3_msgs.msg import Sound
from sensor_msgs.msg import BatteryState
from rcl_interfaces.msg import Log

hostname = gethostname()

core_nodes = [
    "/camera/camera",
    "/diff_drive_controller",
    "/hlds_laser_publisher",
    "/robot_state_publisher",
    "/turtlebot3_node",
]

class tb3Status(Node):
    
    def __init__(self):
        super().__init__("tb3_status")

        self.robot_status_string = "OK"
        self.startup_complete = False
        self.waffle_core_errors = False
        self.active_nodes = []
        
        node_rate = 1 # hz

        self.beeper = self.create_publisher(
            msg_type=Sound,
            topic="sound",
            qos_profile=10
        )
        self.battery = self.create_subscription(
            msg_type=BatteryState,
            topic="battery_state",
            callback=self.battery_callback,
            qos_profile=10,
        )
        self.ros_errors = self.create_subscription(
            topic="rosout",
            msg_type=Log,
            callback=self.rosout_callback,
            qos_profile=10,
        )

        self.stopped = False
        self.starttime = self.get_clock().now().seconds_nanoseconds()
        self.timestamp = self.get_clock().now().seconds_nanoseconds()

    def shutdownhook(self):
        self.stopped = True
    
    def battery_callback(self, msg_data: BatteryState):
        self.battery_voltage = msg_data.voltage
        self.capacity = int(min((60 * self.battery_voltage) - 650, 100)) # Approx. percentage (capped at 100%)
    
    def rosout_callback(self, rosout: Log):
        node = rosout.name
        msg = rosout.msg
        level = rosout.level

        if msg == "Run!" and node == "diff_drive_controller":
            print("startup complete")
            self.startup_complete = True
        
        if level > 20 and node in core_nodes and msg != "":
            print("error detected")
            self.waffle_core_errors = True

#     def check_active_nodes(self):
#         self.active_nodes = rosnode.get_node_names()
#         return True if all([i in self.active_nodes for i in core_nodes]) else False

#     def print_status_msg(self):
#         now = rospy.get_time()
#         if (now - self.timestamp) > 5:
#             msg_timestamp = dt.datetime.now().strftime("%d/%m/%Y %H:%M:%S")
#             runtime = now - self.starttime
#             if runtime > 60:
#                 runtimestring = f"{runtime / 60:.1f}"
#                 units="minutes"
#             else:
#                 runtimestring = f"{runtime:.0f}"
#                 units="seconds"
#             self.waffle_beeper(hush_if_ok=True)
#             os.system('clear')
#             print(f"{f'{msg_timestamp: ^21}':#^32}")
#             print(
#                 f"{'Device: ':>16}{hostname}\n"
#                 f"{'Status: ':>16}{self.robot_status_string}\n"
#                 f"{'Active Nodes: ':>16}{len(self.active_nodes)}\n"
#                 f"{'Up Time: ':>16}{runtimestring} {units}\n"
#                 f"{'Battery: ':>16}{self.battery_voltage:.2f}V [{self.capacity}%]"
#             )
#             self.timestamp = rospy.get_time()
#         else:
#             print(".", end="")
#             sys.stdout.flush()
    
#     def waffle_beeper(self, hush_if_ok = True):
#         robot_status_ok = self.check_active_nodes() and not self.waffle_core_errors
#         self.robot_status_string = "OK" if robot_status_ok else "ERRORS"
#         if robot_status_ok and hush_if_ok:
#             pass
#         else:
#             msg = Sound()
#             msg.value = 1 if robot_status_ok else 3
#             self.beeper.publish(msg)
#         return robot_status_ok

#     def main(self):
#         while not self.startup_complete:
#             continue
        
#         bringup_errors = False
#         if self.waffle_beeper(hush_if_ok=False):
#             rospy.loginfo("--------------------------")
#             rospy.loginfo(f"{hostname} is up and running!")
#             rospy.loginfo("--------------------------")
#         else:
#             rospy.logerr("--------------------------")
#             rospy.logerr(f"{hostname} failed to launch correctly, please try again.")
#             rospy.logerr("--------------------------")
#             bringup_errors = True
        
#         if not bringup_errors:
#             self.timestamp = rospy.get_time()
#             while not self.ctrl_c:
#                 self.print_status_msg()
#                 self.rate.sleep()
#         else:
#             rospy.spin()

def main(args=None):
    rclpy.init(
        args=args,
        signal_handler_options=rclpy.signals.SignalHandlerOptions.NO,
    )
    tb3_status_node = tb3Status()
    try:
        rclpy.spin(tb3_status_node)
    except KeyboardInterrupt:
        print("Ctrl+C detected")
    finally:
        tb3_status_node.shutdownhook()
        while not tb3_status_node.stopped:
            continue
        tb3_status_node.destroy_node()
        rclpy.shutdown() 

if __name__ == '__main__':
    main()