#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, TransformStamped
from nav_msgs.msg import Odometry
import tf2_ros
import math
import numpy as np

class BicycleModelNode(Node):
    def __init__(self):
        super().__init__('bicycle_model_odom')

        self.declare_parameter('wheelbase', 0.165)
        self.L = self.get_parameter('wheelbase').value

        self.subscription = self.create_subscription(
            Twist, '/cmd_vel', self.cmd_callback, 10)

        self.odom_pub = self.create_publisher(Odometry, 'odom', 10)
        self.tf_broadcaster = tf2_ros.TransformBroadcaster(self)

        self.timer = self.create_timer(0.05, self.update_pose)

        self.x = 0.0
        self.y = 0.0
        self.theta = 0.0

        self.v = 0.0
        self.steer_input = 0.0
        self.last_time = self.get_clock().now()

    def cmd_callback(self, msg):
        self.v = msg.linear.x * 1.8
        self.steer_input = msg.angular.z * -2.0  # [-1.0, 1.0] command

    def update_pose(self):
        now = self.get_clock().now()
        dt = (now - self.last_time).nanoseconds * 1e-9
        self.last_time = now

        delta = self.map_steering_input(self.steer_input)

        # Bicycle model equations
        self.x += self.v * math.cos(self.theta) * dt
        self.y += self.v * math.sin(self.theta) * dt
        self.theta += (self.v / self.L) * math.tan(delta) * dt

        # Quaternion conversion
        qx, qy, qz, qw = self.euler_to_quaternion(0, 0, self.theta)

        # Publish TF
        t = TransformStamped()
        t.header.stamp = now.to_msg()
        t.header.frame_id = 'odom'
        t.child_frame_id = 'base_link'
        t.transform.translation.x = self.x
        t.transform.translation.y = self.y
        t.transform.translation.z = 0.0
        t.transform.rotation.x = qx
        t.transform.rotation.y = qy
        t.transform.rotation.z = qz
        t.transform.rotation.w = qw
        self.tf_broadcaster.sendTransform(t)

        # Publish Odometry
        odom = Odometry()
        odom.header.stamp = now.to_msg()
        odom.header.frame_id = 'odom'
        odom.child_frame_id = 'base_link'
        odom.pose.pose.position.x = self.x
        odom.pose.pose.position.y = self.y
        odom.pose.pose.orientation.x = qx
        odom.pose.pose.orientation.y = qy
        odom.pose.pose.orientation.z = qz
        odom.pose.pose.orientation.w = qw
        odom.twist.twist.linear.x = self.v
        odom.twist.twist.angular.z = (self.v / self.L) * math.tan(delta)
        self.odom_pub.publish(odom)

    def map_steering_input(self, input_val):
        # [-1, 1] input from left to right
        # Bias and asymmetric gains
        bias = 0.05  # Adjust this based on real drift
        adjusted_input = input_val + bias
        adjusted_input = max(-1.0, min(1.0, adjusted_input))

        if adjusted_input >= 0:
            return adjusted_input * math.radians(35)
        else:
            return adjusted_input * math.radians(25)

    def euler_to_quaternion(self, roll, pitch, yaw):
        cy = math.cos(yaw * 0.5)
        sy = math.sin(yaw * 0.5)
        cp = math.cos(pitch * 0.5)
        sp = math.sin(pitch * 0.5)
        cr = math.cos(roll * 0.5)
        sr = math.sin(roll * 0.5)

        qx = sr * cp * cy - cr * sp * sy
        qy = cr * sp * cy + sr * cp * sy
        qz = cr * cp * sy - sr * sp * cy
        qw = cr * cp * cy + sr * sp * sy
        return (qx, qy, qz, qw)

def main(args=None):
    rclpy.init(args=args)
    node = BicycleModelNode()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
