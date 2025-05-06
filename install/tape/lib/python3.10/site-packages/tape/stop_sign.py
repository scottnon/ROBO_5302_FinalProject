import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from geometry_msgs.msg import Twist
from cv_bridge import CvBridge
import cv2
import time
import numpy as np
import os
import datetime


class StopSign(Node):
    def __init__(self):
        super().__init__('stop_sign')
        self.subscription = self.create_subscription(Image, '/image_raw', self.listener_callback, 1)
        self.br = CvBridge() #establish a CV2 bridge to convert ros raw images to usable data for openCV
        self.publisher_ = self.create_publisher(Twist, '/cmd_vel', 1) #will publish motion commands
        self.stopped = 0
        self.starttime = datetime.datetime.now()


    def listener_callback(self, data):
        img = self.br.imgmsg_to_cv2(data, 'bgr8') #convert to grayscale image
        img_rgb = cv2.cvtColor(img, cv2.COLOR_BGR2RGB)
        img_gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)

        # img = cv2.resize(img, None, 1, 0.25, 0.25, cv2.INTER_CUBIC) #resizes by cubic interpolation
		
        # imgHSV = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)  may not be needed bceause we are using grayscale
        # img = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
        current_directory = os.getcwd()
        self.get_logger().info('CURRENT DIR' + str(current_directory))

        stop_cascade = cv2.CascadeClassifier('src/tape/tape/stop_data.xml')
        # /home/kingpooper/deepRacerWS/src/tape/tape/stop_data.xml

        found = stop_cascade.detectMultiScale(img_gray, minSize=(65, 65), maxSize=(800, 800)) #tune the 20x20 if we want to recognize smaller/larger

        # self.get_logger().info('STOP SIGNS FOUND' + str(len(found)))
        # self.get_logger().info('has stopped alreadry' + str(self.stopped))
        time_since_start = (datetime.datetime.now() - self.starttime).total_seconds()
        # self.get_logger().info('time since start' + str(time_since_start))

        #red filtering
        enter_zone_range_lower = np.array([150, 0, 15] )
        enter_zone_range_upper = np.array([255, 40, 65])
        img_rgb = cv2.cvtColor(img, cv2.COLOR_BGR2RGB)
        mask_red = cv2.inRange(img_rgb, enter_zone_range_lower, enter_zone_range_upper)
        width = 0
        if mask_red is None:
            pass
        else:
            column_sums = np.sum(mask_red, axis=0)
            width = np.count_nonzero(column_sums)

        if (len(found) > 0 or width > 60) and self.stopped == 0:
            self.get_logger().info("(stopping for the stop sign)") 
            msg = Twist()
            msg.linear.y = 0.0
            msg.linear.z = 0.0
            msg.angular.x = 0.0
            msg.angular.y = 0.0
            msg.linear.x = 0.0
            msg.angular.z = 0.0
            self.publisher_.publish(msg)
            time.sleep(3.3)
            msg = Twist()
            msg.linear.x = 0.18
            self.publisher_.publish(msg)
            self.stopped = 1
        
        for (x, y, w, h) in found:
            cv2.rectangle(img_rgb, (x, y), (x + w, y + h), (0, 255, 0), 5)
            self.get_logger().info("(STOP SIGN SIZE: )" + str(w) + "by" + str(h)) 
        
        # cv2.imshow("Stop Sign Detected", img_rgb)
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
