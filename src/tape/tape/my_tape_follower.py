import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from geometry_msgs.msg import Twist
from cv_bridge import CvBridge
import cv2
import numpy as np

class my_tape_follower(Node):
    def __init__(self):
    	super().__init__(my_tape_follower)
    	self.subscription = self.create_subscription(Image, '/image_raw', self.listener_callback, 10)
        self.subscription
        self.br = CvBridge() #establish a CV2 bridge to convert ros raw images to usable data for openCV
        self.publisher_ = self.create_publisher(Twist, '/cmd_vel', 10) #will publish motion commands
        
        self.linear_velocity_gain = 0.1 #will stay constant but tune here
        self.angular_velocity_gain = 0.3 #will be used to steer towards centroid of black tape
        
    def listener_callback(self, data):
    	self.get_logger().info("receiving video frame")
    	img = self.br.imgmsg_to_cv2(data, 'bgr8')
    	img = cv2.resize(img, None, 1, 0.25, 0.25, cv2.INTER_CUBIC)
    	

