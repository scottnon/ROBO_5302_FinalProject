import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from geometry_msgs.msg import Twist
from cv_bridge import CvBridge
import cv2
import time
import numpy as np
import os


class SchoolZone(Node):
    def __init__(self):
        super().__init__('school_zone')
        self.subscription = self.create_subscription(Image, '/image_raw', self.listener_callback, 10)
        self.br = CvBridge() #establish a CV2 bridge to convert ros raw images to usable data for openCV
        self.publisher_ = self.create_publisher(Twist, '/cmd_vel', 10) #will publish motion commands
        self.fast_velocity = 0.2
        self.slow_velocity = 0.15
        self.mask_thresh = 50 #min number of pixels to qualify "seeing" a color


    def listener_callback(self, data):
        img = self.br.imgmsg_to_cv2(data, 'bgr8') #convert to grayscale image
        img_rgb = cv2.cvtColor(img, cv2.COLOR_BGR2RGB)
        #NOTE: true value of #ed0018 is 237, 0, 24
        enter_zone_range_lower = [230, 0, 20] 
        enter_zone_range_upper = [245, 10, 28]
        #NOTE: true value of #cbfb04 is 203, 251, 4
        exit_zone_range_lower = [200, 248, 0] 
        exit_zone_range_upper = [208, 254, 8]

        #make a mask
        mask_red = cv2.inRange(img_rgb, enter_zone_range_lower, enter_zone_range_upper)
        mask_green = cv2.inRange(img_rgb, exit_zone_range_lower, exit_zone_range_upper)

        #act based on the contents of each mask, first checking for the red mask
        found_red = cv2.countNonZero(mask_red)
        found_green = cv2.countNonZero(mask_green)
        
        if found_red > self.found_thresh:
            self.get_logger().info("(slowing for the school zone)") 
            msg = Twist()
            msg.linear.x = self.slow_velocity
            msg.linear.y = 0.0
            msg.linear.z = 0.0
            msg.angular.x = 0.0
            msg.angular.y = 0.0
            msg.angular.z = 0.0
            self.publisher_.publish(msg)
        elif found_green > self.found_thresh:
            self.get_logger().info("(speeding back for exit school zone)") 
            msg = Twist()
            msg.linear.x = self.fast_velocity
            msg.linear.y = 0.0
            msg.linear.z = 0.0
            msg.angular.x = 0.0
            msg.angular.y = 0.0
            msg.angular.z = 0.0
            self.publisher_.publish(msg)


    #UNCOMMENT THESE FOR VISUALIZATION        
        cv2.imshow("Red mask", mask_red)
        cv2.imshow("Green mask", mask_green)
        cv2.imshow("Raw Image", img)
        cv2.waitKey(1)


def main(args=None):
    rclpy.init(args=args)
    school_zone = SchoolZone()
    try:
        rclpy.spin(school_zone)
    except:
        pass
    finally: 
        # Publish stop message using existing publisher
        stop_msg = Twist()  # All fields default to zero
        school_zone.publisher_.publish(stop_msg)

        # Allow time for message to send
        rclpy.spin_once(school_zone, timeout_sec=0.4)

        school_zone.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
