#!/usr/bin/env python3

import os
import sys
import datetime as dt
from turtlebot3_msgs.msg import Sound  # Assuming message type exists
from sensor_msgs.msg import BatteryState  # Assuming message type exists
import rclpy
from rclpy.node import Node
from rclpy.signals import SignalHandlerOptions
# from rclpy.qos import QoSProfile, QoSPreset
# from rclpy.utilities import get_node_uri
from rcl_interfaces.msg import Log  

# Replace with your actual robot name
# hostname = get_node_uri(use_localhost=False).split("/")[-1]  # Get robot name from URI

core_nodes = [
    'camera', 
    'robot_state_publisher', 
    'zenoh_bridge_ros2dds', 
    'turtlebot3_node', 
    'diff_drive_controller',
    'hlds_laser_publisher',
]

class Tb3Status(Node):

    def __init__(self):
        super().__init__('tb3_status')

        self.startup_complete = [False, False]
        self.startup_sound = False
        self.waffle_core_errors = False
        self.active_nodes = []
        self.shutdown = False

        self.rosout_sub = self.create_subscription(
            Log, '/rosout', self.rosout_cb, qos_profile=10)

        self.create_timer(timer_period_sec=2, callback=self.timer_cb)

        # self.rate = 1.0  # Hz
        # self.qos_profile = QoSProfile(qos_preset=QoSPreset.SENSOR_DATA)  # Adjust QoS parameters as needed

        self.beeper_pub = self.create_publisher(Sound, '/sound', qos_profile=10)
        self.battery_sub = self.create_subscription(
            BatteryState, '/battery_state', self.battery_callback, qos_profile=10)
        

        self.starttime = self.get_clock().now()
        self.timestamp = self.get_clock().now()

    def timer_cb(self):
        self.check_active_nodes()
        self.waffle_beeper()

    def on_shutdown(self):
        self.get_logger().info(
            "Shutting down"
        )
        self.shutdown = True

    def battery_callback(self, msg: BatteryState):
        self.battery_voltage = msg.voltage
        self.capacity = int(min((60 * self.battery_voltage) - 650, 100))  # Approx. percentage (capped at 100%)

    def rosout_cb(self, topic_data: Log):
        node = topic_data.name
        msg = topic_data.msg
        level = topic_data.level

        if msg == "Run!" and node == "diff_drive_controller":
            self.startup_complete[0] = True

        if msg == "Run!" and node == "turtlebot3_node":
            self.startup_complete[1] = True

        if level > 2 and node in core_nodes and msg != "":
            self.waffle_core_errors = True

    def check_active_nodes(self):
        self.active_nodes = self.get_node_names()
        core_nodes_up = all(node in self.active_nodes for node in core_nodes)
        self.get_logger().info(
            f"Node Check: {core_nodes_up}"
        )
        self.get_logger().info(
            f"startup status: {self.startup_complete}"
        )
        if all(self.startup_complete):
        
            self.get_logger().info(
                f"{'Battery: ':>16}{self.battery_voltage:.2f}V [{self.capacity}%]"
            )
        return core_nodes_up

    # def print_status_msg(self):
    #     now = self.get_clock().now()
    #     if (now - self.timestamp).nanoseconds() > 5e9:  # Check every 5 seconds
    #         msg_timestamp = dt.datetime.now().strftime("%d/%m/%Y %H:%M:%S")
    #         runtime = (now - self.starttime).seconds
    #         if runtime > 60:
    #             runtimestring = f"{runtime / 60:.1f}"
    #             units = "minutes"
    #         else:
    #             runtimestring = f"{runtime:.0f}"
    #             units = "seconds"
    #         self.waffle_beeper(hush_if_ok=True)
    #         os.system('clear')
    #         print(f"{f'{msg_timestamp: ^21}':#^32}")
    #         print(
    #             f"{'Device: ':>16}{hostname}\n"
    #             f"{'Status: ':>16}{self.robot_status_string}\n"
    #             f"{'Active Nodes: ':>16}{len(self.active_nodes)}\n"
    #             f"{'Up Time: ':>16}{runtimestring} {units}\n"
    #             f"{'Battery: ':>16}{self.battery_voltage:.2f}V [{self.capacity}%]"
    #         )
    #         self.timestamp = self.get_clock().now()
    #     else:
    #         print(".", end="")
    #         sys.stdout.flush()

    def waffle_beeper(self):
        # Beeping doesn't seem to work
        # (not using ros2 topic pub either...?)
        if not self.startup_sound and all(self.startup_complete):
            self.get_logger().info(
                "BEEPING!"
            )
            msg = Sound()
            msg.value = 1
            self.beeper_pub.publish(msg)
            self.startup_sound = True
        else:
            self.get_logger().info(
                "(Not Beeping)"
            )

    # def main(self):
    #     while not self.startup_complete:
    #         continue

    #     bringup_errors = False
    #     if self.waffle_beeper(hush_if_ok=False):
    #         self.get_logger().info("--------------------------")
    #         self.get_logger().info(f"{hostname} is up and running!")
    #         self.get_logger().info("--------------------------")
    #     else:
    #         self.get_logger().error("--------------------------")
    #         self.get_logger().error(f"{hostname} failed to launch correctly, please try again.")
    #         self.get_logger().error("--------------------------")
    #         bringup_errors = True

    #     if not bringup_errors:
    #         self.timestamp = self.get_clock().now()
    #         rclpy.spin(self)
    #     else:
    #         rclpy.shutdown()


def main(args=None):
    rclpy.init(
        args=args,
        signal_handler_options=SignalHandlerOptions.NO
    )
    node = Tb3Status()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        print(
            f"{node.get_name()} received a shutdown request (Ctrl+C)."
        )
    finally:
        node.on_shutdown()
        while not node.shutdown:
            continue
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()