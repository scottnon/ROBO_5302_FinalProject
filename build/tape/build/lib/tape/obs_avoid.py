import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
from cv_bridge import CvBridge
import cv2
import time
import numpy as np
import os


class ObsAvoid(Node):
    def __init__(self):
        super().__init__('obs_avoid')
        # self.subscription = self.create_subscription(Image, '/image_raw', self.listener_callback, 10)
        self.subscription = self.create_subscription(LaserScan,'scan', self.listener_callback, 10)
        self.br = CvBridge() #establish a CV2 bridge to convert ros raw images to usable data for openCV
        self.publisher_ = self.create_publisher(Twist, '/cmd_vel', 10) #will publish motion commands
        
        # self.previous_error = 0.0
        # self.integral = 0.0
        # self.previous_time = time.time()
        # self.last_error = 0 #initialize the last error to zero
        # self.linear_velocity_gain = 0.14 #will stay constant but tune here
        # self.proportional_gain = 0.1 #will be used to steer towards centroid of black tape
        # self.integral_gain = 0.1
        # self.derivative_gain = 0.3

    def is_object_front(self, data):
        # Check if there's an object close to us based on laser scan data
        threshold_distance = 1.0  # Set a threshold distance (in meters)
        min_range = max(min(data.ranges[330:390]), data.range_min)
        # self.get_logger().info('I heard min: "%s"' % min_range)	# echo minimum range seen
        if min_range < threshold_distance:
            return True
        return False

    def is_object_left(self, data):
        threshold_distance = 1.0  # Set a threshold distance (in meters)
        min_range = max(min(data.ranges[60:120]), data.range_min)
        # self.get_logger().info('I heard min: "%s"' % min_range)	# echo minimum range seen
        if min_range < threshold_distance:
            return True
        return False

    def turn_right(self):
        # Create a Twist message to turn right
        msg = Twist()
        msg.linear.x = 0.0
        msg.angular.z = -0.5
        self.publisher_.publish(msg)
        self.get_logger().info('Turning right')
        time.sleep(0.5)

    def turn_left(self):
        # Create a Twist message to turn right
        msg = Twist()
        msg.linear.x = 0.0
        msg.angular.z = 0.5
        self.publisher_.publish(msg)
        self.get_logger().info('Turning right')
        time.sleep(0.5)

    def move_forward(self):
        # Create a Twist message to move forward
        msg = Twist()
        msg.linear.x = 0.2
        msg.angular.z = 0.0
        self.publisher_.publish(msg)
        self.get_logger().info('Moving forward')
        time.sleep(0.5)


    def listener_callback(self, data):
        #write a bug-like algorith that moves forward until it senses an object in front, then turns right around the object
        if not self.is_object_front(data):
            self.move_forward()
            if self.is_object_front(data):
                self.get_logger().info('Object detected in front, turning right')
                self.turn_right()
                while self.is_object_left(data):
                    self.move_forward()
                    if not self.is_object_left(data):
                        self.turn_left()
                        break   
        cv2.waitKey(1)

def main(args=None):
    rclpy.init(args=args)
    obs_avoid = ObsAvoid()
    try:
        rclpy.spin(obs_avoid)
    except:
        pass
    finally: 
        # Publish stop message using existing publisher
        stop_msg = Twist()  # All fields default to zero
        obs_avoid.publisher_.publish(stop_msg)

        # Allow time for message to send
        rclpy.spin_once(obs_avoid, timeout_sec=0.4)

        obs_avoid.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
