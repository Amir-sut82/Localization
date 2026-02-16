#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64
from geometry_msgs.msg import Twist
import math


class MotorCommandNode(Node):
    def __init__(self):
        super().__init__('motor_command_node')
        
        self.wheel_radius_ = 0.1
        self.wheel_separation_ = 0.46
        
        # Publishers
        self.left_motor_rpm_pub_ = self.create_publisher(Float64, '/left_motor_rpm', 10)
        self.right_motor_rpm_pub_ = self.create_publisher(Float64, '/right_motor_rpm', 10)
        
        # Subscriber to /cmd_vel (Twist)
        self.cmd_vel_sub_ = self.create_subscription(
            Twist,
            '/cmd_vel',
            self.cmd_vel_callback,
            10
        )
        
        self.get_logger().info('Motor Command Node started.')
    
    def cmd_vel_callback(self, msg):
        vx = msg.linear.x
        wz = msg.angular.z
        
        self.get_logger().info(f'Received cmd_vel → vx: {vx} , wz: {wz}')
        
        left_motor_rad_per_sec = (vx - (self.wheel_separation_ * wz) / 2.0) / self.wheel_radius_
        right_motor_rad_per_sec = (vx + (self.wheel_separation_ * wz) / 2.0) / self.wheel_radius_
        
        self.get_logger().info(f'Left rad/s: {left_motor_rad_per_sec} , Right rad/s: {right_motor_rad_per_sec}')
        
        # rad/s to RPM conversion
        left_motor_rpm = left_motor_rad_per_sec * (60.0 / (2.0 * math.pi))
        right_motor_rpm = right_motor_rad_per_sec * (60.0 / (2.0 * math.pi))
        
        self.get_logger().info(f'Left RPM: {left_motor_rpm} , Right RPM: {right_motor_rpm}')
        
        left_msg = Float64()
        right_msg = Float64()
        left_msg.data = left_motor_rpm
        right_msg.data = right_motor_rpm
        
        self.left_motor_rpm_pub_.publish(left_msg)
        self.right_motor_rpm_pub_.publish(right_msg)


def main(args=None):
    rclpy.init(args=args)
    node = MotorCommandNode()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == '__main__':
    main()