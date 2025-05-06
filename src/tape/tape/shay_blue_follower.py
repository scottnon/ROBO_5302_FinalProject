import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from geometry_msgs.msg import Twist
from cv_bridge import CvBridge
import cv2
import time
import numpy as np
import datetime


class ShayaFollower(Node):
    def __init__(self):
        super().__init__('shaya_follower')
        self.subscription = self.create_subscription(Image, '/image_raw', self.listener_callback, 1)
        
        self.br = CvBridge()
        self.publisher_ = self.create_publisher(Twist, '/cmd_vel', 10)

        # Set the desired linear and angular velocity gains
        self.linear_velocity_gain = 0.17 # 0.155
        self.kp = 0.2
        self.kd = -0.10
        self.previous_time = time.time()
        self.previous_error = 0.0 #initialize the last error to zero
        self.last_saw_blue = datetime.datetime.now()
        self.flag = "fwd"
        self.detect_tape = True


    # def turn(self):
    #     #this code is all messed up 
    #     #stop
    #     msg = Twist()
    #     msg.linear.x = 0.0
    #     msg.angular.z = 0.0
    #     self.publisher_.publish(msg)
    #     time.sleep(1)
    #     #back up straight
    #     msg = Twist()
    #     msg.linear.x = 0.15
    #     msg.angular.z = 0
    #     self.publisher_.publish(msg)
    #     time.sleep(.3)
    #     #back up right
    #     msg = Twist()
    #     msg.linear.x = 0.15
    #     msg.angular.z = 5.0
    #     self.publisher_.publish(msg)
    #     self.get_logger().info('Turning right')
    #     time.sleep(1.2)
    #     #stop again
    #     msg = Twist()
    #     msg.linear.x = 0.0
    #     msg.angular.z = 0.0
    #     self.publisher_.publish(msg)

    def backup(self):
        #stop
        msg = Twist()
        msg.linear.x = 0.0
        msg.angular.z = 0.0
        self.publisher_.publish(msg)
        time.sleep(.5)
        #back up straight
        msg = Twist()
        msg.linear.x = 0.15
        msg.angular.z = 0.0
        self.publisher_.publish(msg)
        time.sleep(.6)
        #stop again
        msg = Twist()
        msg.linear.x = 0.0
        msg.angular.z = 0.0
        self.publisher_.publish(msg)
        time.sleep(0.5)
        #back up turning
        msg = Twist()
        msg.linear.x = -0.14
        msg.angular.z = -5.0
        self.publisher_.publish(msg)
        self.get_logger().info('Turning right')
        time.sleep(1.5)
        #stop again
        msg = Twist()
        msg.linear.x = 0.0
        msg.angular.z = 0.0
        self.publisher_.publish(msg)
        time.sleep(0.5)
        #go forward just a bit
        msg = Twist()
        msg.linear.x = 0.14
        msg.angular.z = 0.0
        self.publisher_.publish(msg)
        time.sleep(.5)

    
    def listener_callback(self, data):
        self.get_logger().info('Receiving video frame')
        # Convert ROS Image message to OpenCV image
        img = self.br.imgmsg_to_cv2(data, 'bgr8')
        # Resize image (e.g., scale height by 0.25)
        img = cv2.resize(img, None, fx=1, fy=0.25, interpolation=cv2.INTER_CUBIC)

        # Convert to HSV and threshold for blue line
        img_hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
        lower_blue = np.array([100, 70, 105]) #lower_blue = np.array([100, 50, 10])
        upper_blue = np.array([120, 240, 220]) #upper_blue = np.array([140, 255, 255])
        mask = cv2.inRange(img_hsv, lower_blue, upper_blue)

        # Define ROI: lower half of the mask
        full_h, full_w = mask.shape[:2]
        self.get_logger().info(f"Image width: {full_w}")
        start_row = full_h // 2
        roi = mask[start_row:, :]

        #now, take a mask of the upper half of the image
        #shaya's new code
        height = img_hsv.shape[0]
        width = img_hsv.shape[1]
        cropped_img = img_hsv[0:height//2, 0:width]
        far_mask = cv2.inRange(cropped_img, lower_blue, upper_blue)
        farM = cv2.moments(far_mask)
        time_since_last_blue = (datetime.datetime.now() - self.last_saw_blue).total_seconds()
        if farM['m00'] < 200 and time_since_last_blue >= 0.15: #then we probably don't see the line anymore
            self.get_logger().info("Line not found, turning")
            #self.backup()

            #back up turning if tape is not detected for more than 0.3 seconds
            msg = Twist()
            msg.linear.x = -0.1155 # -0.14
            msg.angular.z = -5.0
            self.publisher_.publish(msg)
            self.get_logger().info('Turning right')
            self.detect_tape = False

        # keep turning for 0.2 seconds after tape is detected again
        if self.detect_tape == False:
            time_notape = datetime.datetime.now()
            while (datetime.datetime.now() - time_notape).total_seconds() < 0.04:
                msg = Twist()
                msg.linear.x = -0.1155 # -0.14
                msg.angular.z = -5.0
                self.publisher_.publish(msg)
                self.get_logger().info('Continued turning right')
            
        self.detect_tape = True

        # Compute image moments on the ROI
        M = cv2.moments(roi)
        if M['m00'] > 80000:
            self.last_saw_blue = datetime.datetime.now()
            # Centroid in ROI coordinates
            cx_roi = int(M['m10'] / M['m00'])
            cy_roi = int(M['m01'] / M['m00'])
            # Shift centroid to full image coordinates
            cx = cx_roi
            # cy = cy_roi + start_row
            # centroid = (cx, cy)

            # Draw centroid on original image for visualization
            # radius = 8
            # # Compute color intensity based on distance from image center
            img_center = (full_w // 2, full_h // 2)
            # distance = np.linalg.norm((cx - img_center[0], cy - img_center[1]))
            # max_dist = np.linalg.norm(img_center)
            # intensity = int(255 * (1 - distance / max_dist))
            # color = (intensity, intensity, 0)
            # cv2.circle(img, centroid, radius, color, -1)

            raw_error = (cx - img_center[0])
            max_possible_error = full_w/2  # use the larger region moment as normalization reference
            error = raw_error / max_possible_error if max_possible_error != 0 else 0.0  # normalize error to be between -1 and 1
            
            d = self.kd * ((self.previous_error -  error) / (time.time() - self.previous_time))
            p = error * self.kp
            angular_velocity = p + d

            # Clamp to avoid extreme turns
            angular_velocity = np.clip(angular_velocity, -3.0, 3.0)

            # if error < 0 :
                #     angular_velocity *= 1.4
                # if error > 0: 
                #     angular_velocity *= 0.4

            # Constant forward speed
            self.previous_time = time.time()
            self.previous_error = error

            # Publish Twist message
            self.get_logger().info("angular velocity: " + str(angular_velocity))
            msg = Twist()
            msg.linear.x = self.linear_velocity_gain  # Linear velocity in x-axis
            msg.linear.y = 0.0
            msg.linear.z = 0.0
            msg.angular.x = 0.0
            msg.angular.y = 0.0
            msg.angular.z = angular_velocity
            self.publisher_.publish(msg)

        # Visualization windows
        cv2.imshow("Line Threshold", mask)
        cv2.imshow("Region of Interest", roi)
        cv2.imshow("Centroid Indicator", img)
        cv2.waitKey(1)


def main(args=None):
    rclpy.init(args=args)
    node = ShayaFollower()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

