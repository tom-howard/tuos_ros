import rclpy
from rclpy.node import Node

from nav_msgs.msg import Odometry
# from tf_transformations import euler_from_quaternion
from math import degrees, atan2, asin
import pandas as pd
import os

pd.set_option("display.float_format", '{:0.3f}'.format)

def euler_from_quaternion(quaternions):
    """
    Convert a quaternion into euler angles (roll, pitch, yaw)
    taken from:
    https://automaticaddison.com/how-to-convert-a-quaternion-into-euler-angles-in-python/
    """
    x = quaternions.x
    y = quaternions.y
    z = quaternions.z
    w = quaternions.w
    
    t0 = +2.0 * (w * x + y * z)
    t1 = +1.0 - 2.0 * (x * x + y * y)
    roll = atan2(t0, t1)
    
    t2 = +2.0 * (w * y - z * x)
    t2 = +1.0 if t2 > +1.0 else t2
    t2 = -1.0 if t2 < -1.0 else t2
    pitch = asin(t2)
    
    t3 = +2.0 * (w * z + x * y)
    t4 = +1.0 - 2.0 * (y * y + z * z)
    yaw = atan2(t3, t4)
    
    return roll, pitch, yaw # in radians

class PoseSubscriber(Node):

    def __init__(self):
        super().__init__('waffle_odom_parser')
        
        self.startup = True
        self.up_and_running = False

        self.df = pd.DataFrame(index = ['linear_x','linear_y','linear_z',
                          'theta_x(Roll)','theta_y(Pitch)','theta_z(Yaw)'])
        self.df['units'] = ['meters','meters','meters',
                           'radians','radians','radians']
        
        self.subscription = self.create_subscription(
            Odometry,
            'odom',
            self.odom_callback,
            10)
        self.subscription  # prevent unused variable warning
        print_rate = 1  # Hz
        self.timer = self.create_timer(print_rate, self.print_callback)

    def odom_callback(self, msg: Odometry):

        pos_x = msg.pose.pose.position.x
        pos_y = msg.pose.pose.position.y
        pos_z = msg.pose.pose.position.z
        orientation_quaternions = msg.pose.pose.orientation

        (roll, pitch, yaw) = euler_from_quaternion(orientation_quaternions)

        waffle_odometry = [pos_x, pos_y, pos_z, roll, pitch, yaw]

        if self.startup:
            self.startup = False
            self.df['initial'] = waffle_odometry
        else:
            self.df['current'] = waffle_odometry
            self.up_and_running = True

    def print_callback(self):
        if self.up_and_running:
            self.df['delta'] = self.df['current'] - self.df['initial']
            df = self.df[['initial', 'current', 'delta', 'units']]
            time = self.get_clock().now().to_msg()
            os.system('clear')
            print(f"=========== ROS Time (s): {time.sec:>10.0f} ===========")
            print(df)        

def main(args=None):
    rclpy.init(args=args)

    minimal_subscriber = PoseSubscriber()

    rclpy.spin(minimal_subscriber)
     
    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    minimal_subscriber.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
