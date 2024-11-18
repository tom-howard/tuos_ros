#!/usr/bin/env python3

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
        self.batt_warn_levels = [15, 14, 13, 12, 11, 10]
        self.batt_warns = self.batt_warn_levels
        self.capacity = 100

        [self.starttime, _] = self.get_clock().now().seconds_nanoseconds()
        [self.err_timestamp, _] = self.get_clock().now().seconds_nanoseconds()
        
    def timer_cb(self):
        self.check_active_nodes()
        [now, _] = self.get_clock().now().seconds_nanoseconds()
        if self.timer_counter > self.status_msg_trigger: 
            self.check_battery_status()
            msg_timestamp = dt.datetime.now().strftime("%d/%m/%Y %H:%M:%S")
            runtime = now - self.starttime
            if runtime > 60:
                runtimestring = f"{runtime / 60:.1f}"
                units = "minutes"
            else:
                runtimestring = f"{runtime:.0f}"
                units = "seconds"
            self.get_logger().info(
                f"\n\n{f'{msg_timestamp: ^21}':#^32}\n"
                f"{'Device: ':>16}{hostname}\n"
                f"{'Status: ':>16}{self.robot_status}\n"
                f"{'Active Nodes: ':>16}{len(self.active_nodes)}\n"
                f"{'Up Time: ':>16}{runtimestring} {units}\n"
                f"{'Battery: ':>16}{self.battery_voltage:.2f}V [{self.capacity}%]"
                f"{'*'*sum([1 for x in self.batt_warns if x == -1])}\n"
            )
            self.timer_counter = 0
        else:
            self.timer_counter += 1

    def on_shutdown(self):
        self.get_logger().info(
            "Shutting down"
        )
        self.shutdown = True

    def battery_callback(self, msg: BatteryState):
        self.battery_voltage = msg.voltage
        self.capacity = int(min((60 * self.battery_voltage) - 650, 100))  # Approx. percentage (capped at 100%)
        if self.capacity < max(self.batt_warn_levels) + 5:
            self.get_logger().warn(
                f"Battery level is getting low!",
                throttle_duration_sec=30
            )

    def check_battery_status(self):
        if self.capacity < max(self.batt_warn_levels) + 5:
            if self.capacity in self.batt_warns:
                self.batt_warns[self.batt_warns.index(self.capacity)] = -1
                self.waffle_beeper(value=2) 
                self.get_logger().warn(
                    f"Battery capacity dropped to {self.capacity}%! Replace BEFORE capacity reaches 10%."
                ) 
        else:
            self.batt_warns = self.batt_warn_levels

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
            [timer, _] = self.get_clock().now().seconds_nanoseconds()
            if timer - self.err_timestamp > 10:
                self.waffle_beeper(value=3)
                [self.err_timestamp, _] = self.get_clock().now().seconds_nanoseconds()
        elif self.capacity <= 10:
            self.robot_status = "ERROR"
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