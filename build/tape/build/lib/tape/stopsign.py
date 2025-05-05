import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from geometry_msgs.msg import Twist
from cv_bridge import CvBridge
import cv2
import time
import numpy as np

class StopSign(Node):
    def __init__(self):
        super().__init__('stop_sign')
        self.subscription = self.create_subscription(Image, '/image_raw', self.listener_callback, 10)
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


    def listener_callback(self, data):
        # img = self.br.imgmsg_to_cv2(data, 'mono8') #convert to grayscale image
        img_rgb = cv2.cvtColor(data, cv2.COLOR_BGR2RGB)
        img_gray = cv2.cvtColor(data, cv2.COLOR_BGR2GRAY)


        # img = cv2.resize(img, None, 1, 0.25, 0.25, cv2.INTER_CUBIC) #resizes by cubic interpolation
		
        # imgHSV = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)  may not be needed bceause we are using grayscale
        # img = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)

        stop_cascade = cv2.CascadeClassifier('stop_data.xml')

        found = stop_cascade.detectMultiScale(img_gray, minSize=(20, 20)) #tune the 20x20 if we want to recognize smaller/larger

        for (x, y, w, h) in found:
            cv2.rectangle(img_rgb, (x, y), (x + w, y + h), (0, 255, 0), 5)
        
        cv2.imshow("Stop Sign Detected", img_rgb)
        # cv2.imshow("Raw Image", img_rgb)
        cv2.waitKey(1)

def main(args=None):
    rclpy.init(args=args)
    stop_sign = StopSign()
    try:
        rclpy.spin(stop_sign)
    except:
        pass
    finally: 
        # Publish stop message using existing publisher
        stop_msg = Twist()  # All fields default to zero
        stop_sign.publisher_.publish(stop_msg)

        # Allow time for message to send
        rclpy.spin_once(stop_sign, timeout_sec=0.4)

        stop_sign.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
