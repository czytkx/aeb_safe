#!/usr/bin/env python3
import rclpy
import math
from rclpy.node import Node

import numpy as np
# TODO: include needed ROS msg type headers and libraries
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Bool
from nav_msgs.msg import Odometry
from ackermann_msgs.msg import AckermannDriveStamped, AckermannDrive


class SafetyNode(Node):
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
        self.laserscan_subscription = self.create_subscription(
            LaserScan,
            'scan',
            self.laserscan_callback,
            10)

        # 订阅odometry话题（这里假设为base_odometry）  
        self.odometry_subscription = self.create_subscription(
            Odometry,
            'ego_racecar/odom',
            self.odometry_callback,
            10)

        self.kb_listener = self.create_subscription(
            Bool,
            'keyboard_listener',
            self.kb_callback,
            10
        )

        # 创建AckermannDriveStamped发布者  
        self.ackermann_pub = self.create_publisher(
            AckermannDriveStamped,
            'drive',
            10)

        self.ackermann_pub2 = self.create_publisher(
            AckermannDriveStamped,
            'opp_drive',
            10)

        # AEB相关参数  
        self.braking_threshold_ttc = 1  # TTC阈值，单位秒
        self.vehicle_speed = 0.0  # 初始速度，从odometry中更新  
        self.min_distance = 100000000  # 最小障碍物距离  
        self.brake_distance = 0.1
        self.open_aeb = False
        self.angle = 0.0
        self.distance = 0.0
        self.crash_ttc=0.01
        self.derivative_distance = 0.0
        self.min_ttc = 1000000

    def laserscan_callback(self, msg):
       # self.logger().info('i hear laser')
        # 遍历laserscan数据，查找最近的障碍物距离  
        
        for i in range(len(msg.ranges)):
            self.distance = msg.ranges[i]
            
            if (self.distance < msg.range_min or self.distance > msg.range_max):
                continue
            self.angle = msg.angle_min + msg.angle_increment * i
            self.derivative_distance = self.vehicle_speed * math.cos(self.angle)
            if (self.derivative_distance > 0 and (self.distance / self.derivative_distance) < self.min_ttc):
                self.min_ttc = self.distance / max(self.derivative_distance, 0.00001)       
            self.min_distance=min(self.min_distance,self.derivative_distance)
        self.get_logger().info('get min_ttc:%f'% self.min_ttc)
        # cmd=AckermannDriveStamped()
        # cmd.drive.speed=0.5
        # cmd.drive.steering_angle = 0.0  # 保持直线  
        # cmd.header.stamp = self.get_clock().now().to_msg()
        # self.ackermann_pub.publish(cmd) 
        # 计算TTC  


        if self.open_aeb:
            self.check_ttc(self.min_ttc)
        else:
            if self.min_ttc < 0.01:
                self.get_logger().info('Crash!')
                self.crash()

    def odometry_callback(self, msg):
        # 从odometry消息中更新车辆速度  
        self.vehicle_speed = msg.twist.twist.linear.x
        if(self.vehicle_speed>=0):
            self.crash_ttc=0.01
        else:
            self.crash_ttc=0.025

    def kb_callback(self, msg):
        if msg:
            self.open_aeb = True
        else:
            self.open_aeb = False

    def check_ttc(self, ttc):
        # 检查TTC是否小于阈值  
        if ttc < self.braking_threshold_ttc:
            self.brake()

    def brake(self):
        # 创建AckermannDriveStamped消息并设置制动命令  
        brake_cmd = AckermannDriveStamped()
        brake_cmd.drive.speed = 0.0  # 停止前进  
        brake_cmd.drive.steering_angle = 0.0  # 保持直线  
        brake_cmd.header.stamp = self.get_clock().now().to_msg()  # 设置时间戳  
        self.get_logger().info('the car stopped successfully,the distance is %f m' % self.min_distance)
        # 发布制动命令  
        self.ackermann_pub.publish(brake_cmd)

        run_cmd = AckermannDriveStamped()
        run_cmd.drive.speed = 0.1
        run_cmd.drive.steering_angle = 0.0
        run_cmd.header.stamp = self.get_clock().now().to_msg()  # 设置时间戳  
        self.ackermann_pub2.publish(run_cmd)
        exit(0)

    def crash(self):
        brake_cmd = AckermannDriveStamped()
        brake_cmd.drive.speed = 0.0  # 停止前进  
        brake_cmd.drive.steering_angle = 0.0  # 保持直线  
        brake_cmd.header.stamp = self.get_clock().now().to_msg()  # 设置时间戳  
        # 发布制动命令  
        self.ackermann_pub.publish(brake_cmd)
        self.ackermann_pub2.publish(brake_cmd)
        exit(0)


def main(args=None):
    rclpy.init(args=args)
    safety_node = SafetyNode()
    rclpy.spin(safety_node)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    safety_node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
