#!/usr/bin/env python3
import sys
import termios
import tty
import rclpy
from rclpy.node import Node

from pynput import keyboard
import numpy as np
# TODO: include needed ROS msg type headers and libraries
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Bool
from nav_msgs.msg import Odometry
from ackermann_msgs.msg import AckermannDriveStamped, AckermannDrive


class Keyboard(Node):
    """
    The class that handles emergency braking.
    """
    def __init__(self):
        super().__init__('safety_node')
        """
        One publisher should publish to the /drive topic with a AckermannDriveStamped drive message.

        You should also subscribe to the /scan topic to get the LaserScan messages and
        the /ego_racecar/odom topic to get the current speed of the vehicle.

        The subscribers should use the provided odom_callback and scan_callback as callback methods

        NOTE that the x component of the linear velocity in odom is the speed
        """

        print('press b to open or close AEB')
        # 创建AckermannDriveStamped发布者  
        self.kb_pub = self.create_publisher(  
            Bool,  
            'keyboard_listener',  
            10)
        self.run_pub=self.create_publisher(
            AckermannDriveStamped,
            'drive',
            10
        )  
        self.run_opp_pub=self.create_publisher(
            AckermannDriveStamped,
            'opp_drive',
            10
        ) 
        self.open_aeb=False
        print('start')
        while  rclpy.ok:  
            fd = sys.stdin.fileno()  
            old_settings = termios.tcgetattr(fd)  
            #不产生回显效果  
            old_settings[3] = old_settings[3] & ~termios.ICANON & ~termios.ECHO  
            try :  
                tty.setraw( fd )  
                ch = sys.stdin.read( 1 )
                if ch== 'b':
                    self.open_aeb=not self.open_aeb
                    open=Bool()
                    open.data=self.open_aeb
                    self.kb_pub.publish(open)
                    if self.open_aeb:
                        self.get_logger().info('you open the AEB\n')
                    else:
                        self.get_logger().info('you close the AEB\n')
                if ch=='s':
                    cmd=AckermannDriveStamped()
                    cmd.drive.speed=0.5
                    cmd.drive.steering_angle = 0.0  # 保持直线  
                    cmd.header.stamp = self.get_clock().now().to_msg()

                    cmd1=AckermannDriveStamped()
                    cmd1.drive.speed=0.1
                    cmd1.drive.steering_angle = 0.0  # 保持直线  
                    cmd1.header.stamp = self.get_clock().now().to_msg()

                    self.run_pub.publish(cmd)
                    self.run_opp_pub.publish(cmd1)
                    self.get_logger().info('car 1 started')
                if ch=='a':
                    cmd=AckermannDriveStamped()
                    cmd.drive.speed=-1.0
                    cmd.drive.steering_angle = 0.0  # 保持直线  
                    cmd.header.stamp = self.get_clock().now().to_msg()

                    cmd1=AckermannDriveStamped()
                    cmd1.drive.speed=0.1
                    cmd1.drive.steering_angle = 0.0  # 保持直线  
                    cmd1.header.stamp = self.get_clock().now().to_msg()

                    self.run_pub.publish(cmd)
                    self.run_opp_pub.publish(cmd1)
                    self.get_logger().info('car 1 started')
                if ch =='q':
                    termios.tcsetattr(fd, termios.TCSADRAIN, old_settings)
                    break
            finally :  
                termios.tcsetattr(fd, termios.TCSADRAIN, old_settings)

        
    def on_press(self,key):
        try:
            print('alphanumeric key {0} pressed'.format(key.char))
        except AttributeError:
            print('special key {0} pressed'.format(key))
        

    def on_release(self,key):
        if key == self.keys:
            self.open_aeb=not self.open_aeb
            self.kb_pub.publish(self.open_aeb)
            if self.open_aeb:
                self.get_logger.info('you open the AEB')
            else:
                self.get_logger.info('you close the AEB')




def main(args=None):
    rclpy.init(args=args)
    kb = Keyboard()
    rclpy.spin(kb)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    kb.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()