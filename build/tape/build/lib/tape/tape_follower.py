import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from geometry_msgs.msg import Twist
from cv_bridge import CvBridge
import cv2
import numpy as np

class LineFollower(Node):
    def __init__(self):
        super().__init__('line_follower')
        self.subscription = self.create_subscription(Image, '/image_raw', self.listener_callback, 10)
        self.subscription
        self.br = CvBridge()
        self.publisher_ = self.create_publisher(Twist, '/cmd_vel', 10)

        # Set the desired linear and angular velocity gains
        self.linear_velocity_gain = 0.1
        self.angular_velocity_gain = 0.3

    def listener_callback(self, data):
        self.get_logger().info('Receiving video frame')
        img = self.br.imgmsg_to_cv2(data, 'bgr8')
        img = cv2.resize(img, None, 1, 0.25, 0.25, cv2.INTER_CUBIC)

        imgHSV = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)

        lower_yellow = np.array([30, 30, 30])
        upper_yellow = np.array([90, 90, 90])

        mask = cv2.inRange(imgHSV, lower_yellow, upper_yellow) #gets rid of everything that isnt in that color range

        roi_height = mask.shape[0] // 2
        roi = mask[mask.shape[0] - roi_height:, :]
        #roi = mask[:,50:]

        M = cv2.moments(roi)
        if M['m00'] > 0: #'m00' is all of the points that are in the mask (within the color range)
            cx = int(M['m10'] / M['m00'])
            cy = int(M['m01'] / M['m00']) + mask.shape[0] - roi_height
            centroid = (cx, cy) #The center of those points

            radius = 8 #sets the radius (in pixels) of the circle that will be drawn at the centroid
            img_center = (mask.shape[1] // 2, mask.shape[0] // 2)
            distance_from_center = np.sqrt((cx - img_center[0])**2 + (cy - img_center[1])**2) # how far the centroid is from the center of the image
            #The next four lines are just for the visualizer
            max_distance = np.sqrt(img_center[0]**2 + img_center[1]**2) 
            color_intensity = int(255 * (1 - distance_from_center / max_distance))
            color = (color_intensity, color_intensity, 0)
            cv2.circle(img, centroid, radius, color, -1)

            # Calculate linear and angular velocities based on centroid position
            error = img_center[0] - cx #positive if centroid is on the left part of the image, and vice versa
            linear_velocity = self.linear_velocity_gain #constant
            angular_velocity = self.angular_velocity_gain * error #this is just proportional control

            # Keep the centroid in the center of the ROI
            if abs(error) > 20:
                angular_velocity = np.clip(angular_velocity, -0.3, 0.3) #To avoid extreme turning when the error is large, the angular velocity is clamped between -0.3 and 0.3

            # Publish the velocity commands
            twist = Twist()
            twist.linear.x = linear_velocity
            twist.angular.z = angular_velocity
            self.publisher_.publish(twist)

        #cv2.imshow("Line Threshold", mask)
        #cv2.imshow("Region of Interest", roi)
        #cv2.imshow("Centroid Indicator", img)
        cv2.waitKey(1)

def main(args=None):
    rclpy.init(args=args)
    line_follower = LineFollower()
    rclpy.spin(line_follower)
    line_follower.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
