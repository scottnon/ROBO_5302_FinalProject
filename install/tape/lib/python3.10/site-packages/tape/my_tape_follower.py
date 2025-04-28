import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from geometry_msgs.msg import Twist
from cv_bridge import CvBridge
import cv2
import numpy as np

class MyTapeFollower(Node):
    def __init__(self):
        super().__init__('my_tape_follower')
        self.subscription = self.create_subscription(Image, '/image_raw', self.listener_callback, 10)
        self.br = CvBridge() #establish a CV2 bridge to convert ros raw images to usable data for openCV
        self.publisher_ = self.create_publisher(Twist, '/cmd_vel', 10) #will publish motion commands
        
        self.last_error = 0 #initialize the last error to zero
        self.linear_velocity_gain = 0.1 #will stay constant but tune here
        self.angular_velocity_gain = 0.3 #will be used to steer towards centroid of black tape
        self.derivative_gain = 0.1 #used to smooth out the steering


    def listener_callback(self, data):
        self.get_logger().info("receiving video frame") 
        # img = self.br.imgmsg_to_cv2(data, 'mono8') #convert to grayscale image
        img = self.br.imgmsg_to_cv2(data, 'bgr8') #convert to grayscale image
        img = cv2.resize(img, None, 1, 0.25, 0.25, cv2.INTER_CUBIC) #resizes by cubic interpolation
		
        #imgHSV = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)  may not be needed bceause we are using grayscale
        gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)


        lower_threshold = 10 #darkest gray we accept
        upper_threshold = 30 #lightest gray we accept

        mask = cv2.inRange(gray, lower_threshold, upper_threshold) #gets rid of everything that isnt in that color range
        # self.get_logger().info("image moments before masking: ", cv2.moments(mask)['m00']) #print the moments of the image before masking
        roi_height = mask.shape[0] // 2 #take only (a hot dog) half of the image
        roi = mask[mask.shape[0] - roi_height:, :] #cut out the desired region
        
        M = cv2.moments(roi)
        if M['m00'] > 0: #'m00' is all of the points that are in the mask (within the gray range)
            cx = int(M['m10'] / M['m00']) #use the image moment to find the centroid
            cy = int(M['m01'] / M['m00']) + mask.shape[0] - roi_height
            centroid = (cx, cy) #The center of those points

            img_center = (mask.shape[1] // 2, mask.shape[0] // 2) #center of the actual image
            
            #prob remove the below line 
            #distance_from_center = np.sqrt((cx - img_center[0])**2 + (cy - img_center[1])**2) # how far the centroid is from the center of the image
            #above line is euclidean distance

            #now, visualize the centroid on the screen
            radius = 5 #sets the radius (in pixels) of the circle that will be drawn at the centroid
            cv2.circle(img, centroid, radius, [255, 0, 0], -1) #plot a simple red circle

    	
            # Calculate linear and angular velocities based on centroid position
            error = img_center[0] - cx #positive if centroid is on the left part of the image, and vice versa
            linear_velocity = self.linear_velocity_gain #constant

            #derivative control
            if error != 0:
                derivative = (error - self.last_error) / 0.1 #should the 0.1 be derivative gain?
                self.last_error = error
            else:
                derivative = 0
                self.last_error = 0
            angular_velocity = self.angular_velocity_gain * error + self.derivative_gain * derivative

            if abs(error) > 20:
                #To avoid extreme turning when the error is large, the angular velocity is clamped between -0.3 and 0.3
                angular_velocity = np.clip(angular_velocity, -0.3, 0.3) 

            # Publish the velocity commands
            twist = Twist()
            self.get_logger().info("publishing nonzero commands") 
            twist.linear.x = linear_velocity
            twist.angular.z = angular_velocity
            self.publisher_.publish(twist)
        else:
            # If no tape is detected, stop the robot
            self.get_logger().info("(not sensing any black)") 
            # twist = Twist()
            # twist.linear.x = 0.0
            # twist.angular.z = 0.0
            # self.publisher_.publish(twist)
                
        #cv2.imshow("Line Threshold", mask)
        #cv2.imshow("Region of Interest", roi)
        #cv2.imshow("Centroid Indicator", img)
        cv2.waitKey(1)

def main(args=None):
    rclpy.init(args=args)
    my_tape_follower = MyTapeFollower()
    rclpy.spin(my_tape_follower)
    my_tape_follower.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
