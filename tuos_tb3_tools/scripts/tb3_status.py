#!/usr/bin/env python3

import os
import sys
import datetime as dt
from turtlebot3_msgs.msg import Sound  # Assuming message type exists
from sensor_msgs.msg import BatteryState  # Assuming message type exists
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSPreset
from rclpy.utilities import get_node_uri
from ros2_msgs.msg import Log  # Assuming message type exists

# Replace with your actual robot name
hostname = get_node_uri(use_localhost=False).split("/")[-1]  # Get robot name from URI

core_nodes = [
    '/turtlebot3_core',
    '/turtlebot3_lds',
    '/rosout',
    '/camera/realsense2_camera',
    '/camera/realsense2_camera_manager',
    '/turtlebot3_diagnostics',
]


class Tb3Status(Node):

    def __init__(self):
        super().__init__('tb3_status')

        self.startup_complete = False
        self.waffle_core_errors = False
        self.active_nodes = []

        self.rate = 1.0  # Hz
        self.qos_profile = QoSProfile(qos_preset=QoSPreset.SENSOR_DATA)  # Adjust QoS parameters as needed

        self.beeper_pub = self.create_publisher(Sound, '/sound', 10, self.qos_profile)
        self.battery_sub = self.create_subscription(
            BatteryState, '/battery_state', self.battery_callback, 10, self.qos_profile)
        self.rosout_sub = self.create_subscription(
            Log, '/rosout', self.rosout_cb, 10, self.qos_profile)

        self.ctrl_c = False
        self.get_logger().add_on_shutdown_callback(self.shutdownhook)

        self.starttime = self.get_clock().now()
        self.timestamp = self.get_clock().now()

    def shutdownhook(self):
        self.ctrl_c = True

    def battery_callback(self, msg: BatteryState):
        self.battery_voltage = msg.voltage
        self.capacity = int(min((60 * self.battery_voltage) - 650, 100))  # Approx. percentage (capped at 100%)

    def rosout_cb(self, msg: Log):
        node = msg.name
        msg = msg.msg
        level = msg.level

        if msg == "Calibration End" and node == "/turtlebot3_core":
            self.startup_complete = True

        if level > 2 and node in core_nodes and msg != "":
            self.waffle_core_errors = True

    def check_active_nodes(self):
        self.active_nodes = self.get_node_names()
        return all(node in self.active_nodes for node in core_nodes)

    def print_status_msg(self):
        now = self.get_clock().now()
        if (now - self.timestamp).nanoseconds() > 5e9:  # Check every 5 seconds
            msg_timestamp = dt.datetime.now().strftime("%d/%m/%Y %H:%M:%S")
            runtime = (now - self.starttime).seconds
            if runtime > 60:
                runtimestring = f"{runtime / 60:.1f}"
                units = "minutes"
            else:
                runtimestring = f"{runtime:.0f}"
                units = "seconds"
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
            self.timestamp = self.get_clock().now()
        else:
            print(".", end="")
            sys.stdout.flush()

    def waffle_beeper(self, hush_if_ok=True):
        robot_status_ok = self.check_active_nodes() and not self.waffle_core_errors
        self.robot_status_string = "OK" if robot_status_ok else "ERRORS"
        if robot_status_ok and hush_if_ok:
            pass
        else:
            msg = Sound()
            msg.value = 1 if robot_status_ok else 3
            self.beeper_pub.publish(msg)
        return robot_status_ok

    def main(self):
        while not self.startup_complete:
            continue

        bringup_errors = False
        if self.waffle_beeper(hush_if_ok=False):
            self.get_logger().info("--------------------------")
            self.get_logger().info(f"{hostname} is up and running!")
            self.get_logger().info("--------------------------")
        else:
            self.get_logger().error("--------------------------")
            self.get_logger().error(f"{hostname} failed to launch correctly, please try again.")
            self.get_logger().error("--------------------------")
            bringup_errors = True

        if not bringup_errors:
            self.timestamp = self.get_clock().now()
            rclpy.spin(self)
        else:
            rclpy.shutdown()


def main(args=None):
    rclpy.init(args=args)
    node = Tb3Status()
    node.main()
    rclpy.shutdown()


if __name__ == '__main__':
    main()