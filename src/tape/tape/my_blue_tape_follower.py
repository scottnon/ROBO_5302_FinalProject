import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from geometry_msgs.msg import Twist
from cv_bridge import CvBridge
import cv2
import time
import numpy as np

class MyBlueTapeFollower(Node):
    def __init__(self):
        super().__init__('my_sexy_tape_follower')
        self.subscription = self.create_subscription(Image, '/image_raw', self.listener_callback, 10)
        self.br = CvBridge() #establish a CV2 bridge to convert ros raw images to usable data for openCV
        self.publisher_ = self.create_publisher(Twist, '/cmd_vel', 10) #will publish motion commands
        
        self.previous_error = 0.0
        self.integral = 0.0
        self.previous_time = time.time()
        self.last_error = 0 #initialize the last error to zero
        self.linear_velocity_gain = 0.14 #will stay constant but tune here
        self.proportional_gain = 0.000001 #will be used to steer towards centroid of black tape
        self.integral_gain =     0.0 #0.0000001
        self.derivative_gain =   0.0 # 0.000001


    def listener_callback(self, data):
        # img = self.br.imgmsg_to_cv2(data, 'mono8') #convert to grayscale image
        img = self.br.imgmsg_to_cv2(data, 'bgr8') #convert to BGR8 Image
        #note: the reason it's not converting to a grayscale image right way is because we later use the cvtColor function 
        img = cv2.resize(img, None, 1, 0.25, 0.25, cv2.INTER_CUBIC) #resizes by cubic interpolation
		
        #imgHSV = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)  may not be needed bceause we are using grayscale
        img = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)


        lower_blue = np.array([100, 50, 10])
        upper_blue = np.array([140, 255, 255])

        mask = cv2.inRange(img, lower_blue, upper_blue) #gets rid of everything that isnt in that color range
        # self.get_logger().info("image moments before masking: ", cv2.moments(mask)['m00']) #print the moments of the image before masking
        vertical_cutoff = 1.5
        roi_height = int(mask.shape[0] // vertical_cutoff) #take only (a hot dog) half of the image
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
            threshold = 0.03 * max(MR['m00'], ML['m00']) #this is using 3% of the number of pixels
            clip_threshold = 0.5 * max(MR['m00'], ML['m00']) #this is the condition used to clip angles
            #clip threshold represents like "we think it's pointed really away from the line"

            #explanation of moments:
            # m00 = sum of all pixels in the image
            # m10 = sum of all x coordinates of the pixels in the image


            # change centroid values to x values only (suspicious code)
            # left_cx = int(ML["m10"] - ML["m00"]) #from shaya: im not sure what this is doing 
            # right_cx = int(MR["m10"] - MR["m00"])
            # error = right_cx - left_cx

            #use x centroid (shaya rewrite)
            # left_cx = int(ML["m10"]/ML["m00"]) #this divides the sum of all x coordinates by the number of pixels in the image
            #theoretically should be the mean x value
            # right_cx = int(MR["m10"]/MR["m00"])
            # error = right_cx - left_cx

            # #compare number of pixels 
            # right_pixels = cv2.countNonZero(roi_right) #counts nonzero pixels in the mask
            # left_pixels = cv2.countNonZero(roi_left)
            # error = right_pixels - left_pixels

            #compare number of pixels 
            right_pixels = ML['m00'] #counts nonzero pixels in the mask, but a different way
            left_pixels = MR['m00']
            error = (left_pixels - right_pixels)
            self.get_logger().info("error: " + str(error)) 
            self.get_logger().info("threshold: " + str(threshold))
            self.get_logger().info("clip: " + str(clip_threshold))

            # self.get_logger().info("here is the error") 
            if np.abs(error) > clip_threshold:
                #turn a direction
                self.get_logger().info("turning") 

                p = self.proportional_gain * error

                self.integral += error * (time.time() - self.previous_time)
                i = self.integral_gain * self.integral
                
                d = self.derivative_gain * ((self.previous_error -  error) / (time.time() - self.previous_time))

                angular_velocity =  p + i + d
                self.previous_time = time.time()
                self.previous_error = error

                linear_velocity = self.linear_velocity_gain
                
                
                if abs(error) > threshold:
                #To avoid extreme turning when the error is large, the angular velocity is clamped between -0.3 and 0.3
                    self.get_logger().info("clipping") 
                    angular_velocity = np.clip(angular_velocity, -0.2, 0.2) # was -0.3,0.3
                    # linear_velocity = 0.133

                if error < 0 :
                    angular_velocity *= 1.4
                if error > 0: 
                    angular_velocity *= 0.4

                self.get_logger().info("angular velocity: " + str(angular_velocity))
                msg = Twist()
                msg.linear.x = linear_velocity  # Linear velocity in x-axis
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
                msg.angular.z = -0.28
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
            msg.angular.z = -0.28
            self.publisher_.publish(msg)

        #UNCOMMENT THESE FOR VISUALIZATION        
        cv2.imshow("Line Threshold", mask)
        cv2.imshow("Left Region of Interest", roi_left)
        cv2.imshow("Right Region of Interest", roi_right)
        cv2.imshow("Raw Image", img)
        cv2.waitKey(1)

def main(args=None):
    rclpy.init(args=args)
    my_blue_tape_follower = MyBlueTapeFollower()
    try:
        rclpy.spin(my_blue_tape_follower)
    except:
        pass
    finally: 
        # Publish stop message using existing publisher
        stop_msg = Twist()  # All fields default to zero
        my_blue_tape_follower.publisher_.publish(stop_msg)

        # Allow time for message to send
        rclpy.spin_once(my_blue_tape_follower, timeout_sec=0.4)

        my_blue_tape_follower.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
