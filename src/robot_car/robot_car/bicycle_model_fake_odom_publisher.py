#!/usr/bin/env python3          # added by Scott, tells C interpreter to use python

import math
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, TransformStamped
from nav_msgs.msg import Odometry
from tf2_ros import TransformBroadcaster
from rclpy.qos import QoSProfile
import tf_transformations

class BicycleModelFakeOdomPublisher(Node):
    def __init__(self):
        super().__init__('bicycle_model_fake_odom_publisher')

        # Robot parameters
        self.declare_parameter('wheel_base', 0.165)  # 16.5 cm wheelbase
        self.declare_parameter('odom_frame', 'odom')
        self.declare_parameter('base_frame', 'base_link')

        self.wheel_base = self.get_parameter('wheel_base').value
        self.odom_frame = self.get_parameter('odom_frame').value
        self.base_frame = self.get_parameter('base_frame').value

        # Robot state
        self.x = 0.0
        self.y = 0.0
        self.theta = 0.0

        # Velocity commands
        self.linear_velocity = 0.0  # m/s
        self.angular_velocity = 0.0  # rad/s

        # ROS interfaces
        qos = QoSProfile(depth=10)
        self.create_subscription(Twist, '/cmd_vel', self.cmd_vel_callback, qos)
        self.odom_pub = self.create_publisher(Odometry, '/odom', qos)
        self.tf_broadcaster = TransformBroadcaster(self)

        self.last_time = self.get_clock().now()
        self.timer = self.create_timer(0.02, self.update)  # 50Hz updates

    def cmd_vel_callback(self, msg):
        self.linear_velocity = msg.linear.x
        self.angular_velocity = msg.angular.z

    def update(self):
        current_time = self.get_clock().now()
        dt = (current_time - self.last_time).nanoseconds * 1e-9
        self.last_time = current_time

        if dt == 0:
            return

        # Estimate steering angle from angular velocity
        if abs(self.linear_velocity) > 1e-5:
            steering_angle = math.atan2(self.angular_velocity * self.wheel_base, self.linear_velocity)
        else:
            steering_angle = 0.0

        # Bicycle model integration
        if abs(steering_angle) > 1e-5:
            turning_radius = self.wheel_base / math.tan(steering_angle)
            omega = self.linear_velocity / turning_radius
        else:
            omega = 0.0

        delta_x = self.linear_velocity * math.cos(self.theta) * dt
        delta_y = self.linear_velocity * math.sin(self.theta) * dt
        delta_theta = omega * dt

        self.x += delta_x
        self.y += delta_y
        self.theta += delta_theta

        # Create Odometry message
        odom_msg = Odometry()
        odom_msg.header.stamp = current_time.to_msg()
        odom_msg.header.frame_id = self.odom_frame
        odom_msg.child_frame_id = self.base_frame

        odom_msg.pose.pose.position.x = self.x
        odom_msg.pose.pose.position.y = self.y
        odom_msg.pose.pose.position.z = 0.0

        quat = tf_transformations.quaternion_from_euler(0, 0, self.theta)
        odom_msg.pose.pose.orientation.x = quat[0]
        odom_msg.pose.pose.orientation.y = quat[1]
        odom_msg.pose.pose.orientation.z = quat[2]
        odom_msg.pose.pose.orientation.w = quat[3]

        odom_msg.twist.twist.linear.x = self.linear_velocity
        odom_msg.twist.twist.angular.z = omega

        self.odom_pub.publish(odom_msg)

        # Broadcast TF
        t = TransformStamped()
        t.header.stamp = current_time.to_msg()
        t.header.frame_id = self.odom_frame
        t.child_frame_id = self.base_frame
        t.transform.translation.x = self.x
        t.transform.translation.y = self.y
        t.transform.translation.z = 0.0
        t.transform.rotation.x = quat[0]
        t.transform.rotation.y = quat[1]
        t.transform.rotation.z = quat[2]
        t.transform.rotation.w = quat[3]

        self.tf_broadcaster.sendTransform(t)

def main(args=None):
    rclpy.init(args=args)
    node = BicycleModelFakeOdomPublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
