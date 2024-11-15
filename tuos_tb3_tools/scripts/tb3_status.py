#!/usr/bin/env python3

import os
import sys
import datetime as dt
from socket import gethostname
from turtlebot3_msgs.srv import Sound
from sensor_msgs.msg import BatteryState
import rclpy
from rclpy.node import Node
from rclpy.signals import SignalHandlerOptions
from rcl_interfaces.msg import Log  

hostname = gethostname()

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

        self.bringup_checks = [False, False, False]
        self.bringup_complete = False
        self.bringup_errors = False
        self.waffle_core_errors = False
        self.active_nodes = []
        self.shutdown = False
        self.robot_status = ""
        self.previous_battery_capacity = 0

        self.rosout_sub = self.create_subscription(
            Log, '/rosout', self.rosout_cb, qos_profile=10)

        timer_rate = 1 # Hz
        self.create_timer(timer_period_sec=1/timer_rate, callback=self.timer_cb)
        self.timer_counter = 0
        msg_period = 5 # seconds
        self.status_msg_trigger = msg_period * timer_rate 

        self.beep_srv = self.create_client(srv_type=Sound, srv_name="sound")
        self.get_logger().info("Waiting for the /sound service...")
        while not self.beep_srv.wait_for_service(timeout_sec=1.0):
            continue
        self.bringup_checks[2] = True
        self.get_logger().info("/sound service is ready.")

        self.battery_sub = self.create_subscription(
            BatteryState, '/battery_state', self.battery_callback, qos_profile=10)
        

        self.starttime = self.get_clock().now()
        self.timestamp = self.get_clock().now()

    def timer_cb(self):
        self.check_active_nodes()
        now = self.get_clock().now()
        self.get_logger().info(now)
        if self.timer_counter > self.status_msg_trigger: 
            msg_timestamp = dt.datetime.now().strftime("%d/%m/%Y %H:%M:%S")
            # runtime = (now - self.starttime).seconds_nanoseconds
            # if runtime > 60:
            #     runtimestring = f"{runtime / 60:.1f}"
            #     units = "minutes"
            # else:
            #     runtimestring = f"{runtime:.0f}"
            #     units = "seconds"
            # # self.waffle_beeper(hush_if_ok=True)
            self.get_logger().info(
                f"\n\n{f'{msg_timestamp: ^21}':#^32}\n"
                f"{'Device: ':>16}{hostname}\n"
                f"{'Status: ':>16}{self.robot_status}\n"
                f"{'Active Nodes: ':>16}{len(self.active_nodes)}\n"
                # f"{'Up Time: ':>16}{runtimestring} {units}\n"
                f"{'Battery: ':>16}{self.battery_voltage:.2f}V [{self.capacity}%]\n"
            )
            # self.timestamp = self.get_clock().now()
            self.timer_counter = 0
        else:
            print(".", end="")
            sys.stdout.flush()
            self.timer_counter += 1

    def on_shutdown(self):
        self.get_logger().info(
            "Shutting down"
        )
        self.shutdown = True

    def battery_callback(self, msg: BatteryState):
        self.battery_voltage = msg.voltage
        self.capacity = int(min((60 * self.battery_voltage) - 650, 100))  # Approx. percentage (capped at 100%)
        if self.capacity <= 15:
            if self.capacity != self.previous_battery_capacity:
                self.waffle_beeper(value=2) 
                self.get_logger().warn(
                    f"Battery Low. Replace BEFORE capacity reaches 10%."
                )
        self.previous_battery_capacity = self.capacity

    def rosout_cb(self, topic_data: Log):
        node = topic_data.name
        msg = topic_data.msg
        level = topic_data.level

        if not self.bringup_complete:
            if msg == "Run!" and node == "diff_drive_controller":
                self.bringup_checks[0] = True
            if msg == "Run!" and node == "turtlebot3_node":
                self.bringup_checks[1] = True
            self.bringup_complete = all(self.bringup_checks)
            if self.bringup_complete: 
                if not self.waffle_core_errors:
                    self.get_logger().info(
                        "\n\n"
                        f"{'':#^40}\n"
                        f"{f' {hostname} is up and running! ':#^40}\n"
                        f"{'':#^40}\n"
                    )
                    self.bringup_errors = False
                    self.waffle_beeper(value=1)
                else:
                    self.get_logger().error(
                        "\n\n"
                        f"{'':#^40}\n"
                        f"{f' Bringup error, please try again. ':#^40}\n"
                        f"{'':#^40}\n"
                    )
                    self.bringup_errors = True

        if level > 30 and node in core_nodes and msg != "":
            # Any logs greater than WARN from core nodes only:
            self.waffle_core_errors = True
            self.robot_status = "ERROR"
            # BEEP [value=3] (at rate X...?)
        else:
            self.robot_status = "OK"

    def check_active_nodes(self):
        self.active_nodes = self.get_node_names()
        return all(node in self.active_nodes for node in core_nodes)

    def waffle_beeper(self, value = 4):
        msg = Sound.Request()
        msg.value = value
        self.beep_srv.call_async(msg)

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