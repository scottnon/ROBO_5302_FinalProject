import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from geometry_msgs.msg import Twist
from cv_bridge import CvBridge
import cv2
import time
import numpy as np

class MySexyTapeFollower(Node):
    def __init__(self):
        super().__init__('my_sexy_tape_follower')
        self.subscription = self.create_subscription(Image, '/image_raw', self.listener_callback, 10)
        self.br = CvBridge() #establish a CV2 bridge to convert ros raw images to usable data for openCV
        self.publisher_ = self.create_publisher(Twist, '/cmd_vel', 10) #will publish motion commands
        
        self.previous_error = 0.0
        self.integral = 0.0
        self.previous_time = time.time()
        self.last_error = 0 #initialize the last error to zero
        self.linear_velocity_gain = 0.29 #will stay constant but tune here
        self.proportional_gain = 0.004 #will be used to steer towards centroid of black tape
        self.integral_gain = 0.001
        self.derivative_gain = 0.0


    def listener_callback(self, data):
        self.get_logger().info("receiving video frame") 
        img = self.br.imgmsg_to_cv2(data, 'mono8') #convert to grayscale image
        # img = self.br.imgmsg_to_cv2(data, 'bgr8') #convert to BGR8 Image
        #note: the reason it's not converting to a grayscale image right way is because we later use the cvtColor function 
        img = cv2.resize(img, None, 1, 0.25, 0.25, cv2.INTER_CUBIC) #resizes by cubic interpolation
		
        #imgHSV = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)  may not be needed bceause we are using grayscale
        # img = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)


        lower_threshold = 0 #darkest gray we accept
        upper_threshold = 20 #lightest gray we accept

        mask = cv2.inRange(img, lower_threshold, upper_threshold) #gets rid of everything that isnt in that color range
        # self.get_logger().info("image moments before masking: ", cv2.moments(mask)['m00']) #print the moments of the image before masking
        roi_height = mask.shape[0] // 2 #take only (a hot dog) half of the image
        #will make two rois, one for the left and one for the right
        roi_left = mask[mask.shape[0] - roi_height:, :mask.shape[1] // 2] #cut out the left region   
        roi_right = mask[mask.shape[0] - roi_height:, mask.shape[1] // 2:] #cut out the right region
        # roi = mask[mask.shape[0] - roi_height:, :] #old ROI

        
        
        MR = cv2.moments(roi_right)
        ML = cv2.moments(roi_left)
        if MR['m00'] > 0 or ML['m00'] > 0: #'m00' is all of the points that are in the mask (within the gray range)
            #want to compare the number of black pixels in the left and right regions
            #if the right region has more black pixels, turn right
            #if the left region has more black pixels, turn left
            #if both regions have black pixels, go straight
            threshold = 0.01 * min(MR['m00'], ML['m00'])
            error = MR['m00'] - ML['m00']
            # self.get_logger().info("here is the error") 
            if np.abs(error) > threshold:
                #turn a direction
                self.get_logger().info("turning") 

                p = self.proportional_gain * error

                self.integral += error * (time.time() - self.previous_time)
                i = self.integral_gain * self.integral
                
                d = self.integral_gain * ((self.previous_error -  error) / (time.time() - self.previous_time))

                angular_velocity =  p + i + d
                self.previous_time = time.time()
                self.previous_error = error
                
                if abs(error) > 20:
                #To avoid extreme turning when the error is large, the angular velocity is clamped between -0.3 and 0.3
                    angular_velocity = np.clip(angular_velocity, -1, 1) 
                self.get_logger().info("angular velocity: " + str(angular_velocity))
                msg = Twist()
                msg.linear.x = self.linear_velocity_gain  # Linear velocity in x-axis
                msg.linear.y = 0.0
                msg.linear.z = 0.0
                msg.angular.x = 0.0
                msg.angular.y = 0.0
                msg.angular.z = angular_velocity
                self.publisher_.publish(msg)
            else:
                # go straight
                self.get_logger().info("going straight") 
                msg = Twist()
                msg.linear.y = 0.0
                msg.linear.z = 0.0
                msg.angular.x = 0.0
                msg.angular.y = 0.0
                msg.linear.x = self.linear_velocity_gain
                msg.angular.z = 0.0
                self.publisher_.publish(msg)
        else:
            # If no tape is detected, go straight
            self.get_logger().info("(not sensing any black)") 
            msg = Twist()
            msg.linear.y = 0.0
            msg.linear.z = 0.0
            msg.angular.x = 0.0
            msg.angular.y = 0.0
            msg.linear.x = self.linear_velocity_gain
            msg.angular.z = 0.0
            self.publisher_.publish(msg)

        #UNCOMMENT THESE FOR VISUALIZATION        
        # cv2.imshow("Line Threshold", mask)
        # cv2.imshow("Left Region of Interest", roi_left)
        # cv2.imshow("Right Region of Interest", roi_right)
        #cv2.imshow("Raw Image", img)
        cv2.waitKey(1)

def main(args=None):
    rclpy.init(args=args)
    my_sexy_tape_follower = MySexyTapeFollower()
    try:
        rclpy.spin(my_sexy_tape_follower)
    except:
        pass
    finally: 
        # Publish stop message using existing publisher
        stop_msg = Twist()  # All fields default to zero
        my_sexy_tape_follower.publisher_.publish(stop_msg)

        # Allow time for message to send
        rclpy.spin_once(my_sexy_tape_follower, timeout_sec=0.4)

        my_sexy_tape_follower.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
