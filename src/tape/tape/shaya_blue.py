import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from geometry_msgs.msg import Twist
from cv_bridge import CvBridge
import cv2
import time
import numpy as np


class ShayaBlue(Node):
    def __init__(self):
        super().__init__('shaya_blue')
        self.subscription = self.create_subscription(Image, '/image_raw', self.listener_callback, 10)
        
        self.br = CvBridge()
        self.publisher_ = self.create_publisher(Twist, '/cmd_vel', 10)

        # Set the desired linear and angular velocity gains
        self.linear_velocity_gain = 0.17
        self.kp = 0.3
        self.kd = -0.1
        self.previous_time = time.time()
        self.previous_error = 0.0 #initialize the last error to zero


    def listener_callback(self, data):
        self.get_logger().info('Receiving video frame')
        # Convert ROS Image message to OpenCV image
        img = self.br.imgmsg_to_cv2(data, 'bgr8') 
        # Resize image (e.g., scale height by 0.25)
        img = cv2.resize(img, None, fx=1, fy=0.25, interpolation=cv2.INTER_CUBIC)

        gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)

        # Apply edge detection method on the image
        edges = cv2.Canny(gray, 50, 150, apertureSize=3)

        lines = cv2.HoughLinesP(edges, 1, np.pi / 180, 50, minLineLength=50, maxLineGap=20)

        for line in lines:
            x1, y1, x2, y2 = line[0]
            # Calculate the angle using atan2
            angle = np.degrees(np.arctan2(y2 - y1, x2 - x1))
            print(f"Angle: {angle}")

        self.get_logger().info('Lines: {}'.format(lines))
        cv2.imshow('edge', edges)
        cv2.imshow('gray', gray)
        cv2.waitKey(1)


def main(args=None):
    rclpy.init(args=args)
    node = ShayaBlue()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

