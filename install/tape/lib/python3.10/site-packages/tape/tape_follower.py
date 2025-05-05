# import rclpy
# from rclpy.node import Node
# from sensor_msgs.msg import Image
# from geometry_msgs.msg import Twist
# from cv_bridge import CvBridge
# import cv2
# import time
# import numpy as np

# class LineFollower(Node):
#     def __init__(self):
#         super().__init__('line_follower')
#         self.subscription = self.create_subscription(Image, '/image_raw', self.listener_callback, 10)
#         self.subscription   # idk what this does
#         self.br = CvBridge()
#         self.publisher_ = self.create_publisher(Twist, '/cmd_vel', 10)

#         # Set the desired linear and angular velocity gains
#         self.linear_velocity_gain = 0.14
#         self.kp = 0.1
#         self.ki = 0.01
#         self.kd = 0.2
#         self.integral = 0.0
#         self.previous_time = time.time()
#         self.previous_error = 0.0 #initialize the last error to zero


#     def listener_callback(self, data):
#         self.get_logger().info('Receiving video frame')
#         img = self.br.imgmsg_to_cv2(data, 'bgr8')
#         img = cv2.resize(img, None, 1, 0.25, 0.25, cv2.INTER_CUBIC)

#         imgHSV = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)

#         lower_blue = np.array([100, 50, 10])
#         upper_blue = np.array([140, 255, 255])

#         mask = cv2.inRange(imgHSV, lower_blue, upper_blue) #gets rid of everything that isnt in that color range

#         roi_height = mask.shape[0] // 2 #take only (a hot dog) half of the image
#         roi = mask[mask.shape[0] - roi_height:, :] #cut out the desired region
#         roi = mask[:,50:] # what is this?

#         M = cv2.moments(roi)
#         if M['m00'] > 0: #'m00' is all of the points that are in the mask (within the color range)
#             cx = int(M['m10'] / M['m00']) +50 
#             cy = int(M['m01'] / M['m00']) + mask.shape[0] - roi_height
#             centroid = (cx, cy) #The center of those points

#             radius = 8 #sets the radius (in pixels) of the circle that will be drawn at the centroid
#             img_center = (mask.shape[1] // 2, mask.shape[0] // 2) #center of the actual image
#             distance_from_center = np.sqrt((cx - img_center[0])**2 + (cy - img_center[1])**2) # how far the centroid is from the center of the image
#             #The next four lines are just for the visualizer
#             max_distance = np.sqrt(img_center[0]**2 + img_center[1]**2) 
#             color_intensity = int(255 * (1 - distance_from_center / max_distance))
#             color = (color_intensity, color_intensity, 0)
#             cv2.circle(img, centroid, radius, color, -1)

#             # Calculate linear and angular velocities based on centroid position
#             error = img_center[0] - cx #positive if centroid is on the left part of the image, and vice versa
#             linear_velocity = self.linear_velocity_gain #constant

#             p = self.kp * error

#             self.integral += error * (time.time() - self.previous_time)
#             i = self.ki * self.integral
               
#             d = self.kd * ((self.previous_error -  error) / (time.time() - self.previous_time))

#             angular_velocity =  p + i + d
#             self.previous_time = time.time()
#             self.previous_error = error
#             angular_velocity = self.kp * error #this is just proportional control

#             # Keep the centroid in the center of the ROI
#             if abs(error) > 20:
#                 angular_velocity = np.clip(angular_velocity, -0.3, 0.3) #To avoid extreme turning when the error is large, the angular velocity is clamped between -0.3 and 0.3

#             # Publish the velocity commands
#             msg = Twist()
#             msg.linear.x = linear_velocity  # Linear velocity in x-axis
#             msg.linear.y = 0.0
#             msg.linear.z = 0.0
#             msg.angular.x = 0.0
#             msg.angular.y = 0.0
#             msg.angular.z = angular_velocity
#             self.publisher_.publish(msg)

#         cv2.imshow("Line Threshold", mask)
#         cv2.imshow("Region of Interest", roi)
#         cv2.imshow("Centroid Indicator", img)
#         cv2.waitKey(1)

# def main(args=None):
#     rclpy.init(args=args)
#     line_follower = LineFollower()
#     rclpy.spin(line_follower)
#     line_follower.destroy_node()
#     rclpy.shutdown()

# if __name__ == '__main__':
#     main()


import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from geometry_msgs.msg import Twist
from cv_bridge import CvBridge
import cv2
import time
import numpy as np

class LineFollower(Node):
    def __init__(self):
        super().__init__('line_follower')
        self.subscription = self.create_subscription(Image, '/image_raw', self.listener_callback, 10)
        
        self.br = CvBridge()
        self.publisher_ = self.create_publisher(Twist, '/cmd_vel', 10)

        # Set the desired linear and angular velocity gains
        self.linear_velocity_gain = 0.14
        self.kp = 0.1
        self.ki = 0.00
        self.kd = 0.0
        self.integral = 0.0
        self.previous_time = time.time()
        self.previous_error = 0.0  # initialize the last error to zero

    def listener_callback(self, data):
        self.get_logger().info('Receiving video frame')
        # Convert ROS Image message to OpenCV image
        img = self.br.imgmsg_to_cv2(data, 'bgr8')
        # Resize image (e.g., scale height by 0.25)
        img = cv2.resize(img, None, fx=1, fy=0.25, interpolation=cv2.INTER_CUBIC)

        # Convert to HSV and threshold for blue line
        img_hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
        lower_blue = np.array([100, 50, 10])
        upper_blue = np.array([140, 255, 255])
        mask = cv2.inRange(img_hsv, lower_blue, upper_blue)

        # Define ROI: lower half of the mask
        full_h, full_w = mask.shape[:2]
        start_row = full_h // 2
        roi = mask[start_row:, :]

        # Compute image moments on the ROI
        M = cv2.moments(roi)
        if M['m00'] > 0:
            # Centroid in ROI coordinates
            cx_roi = int(M['m10'] / M['m00'])
            cy_roi = int(M['m01'] / M['m00'])
            # Shift centroid to full image coordinates
            cx = cx_roi
            cy = cy_roi + start_row
            centroid = (cx, cy)

            # Draw centroid on original image for visualization
            radius = 8
            # Compute color intensity based on distance from image center
            img_center = (full_w // 2, full_h // 2)
            distance = np.linalg.norm((cx - img_center[0], cy - img_center[1]))
            max_dist = np.linalg.norm(img_center)
            intensity = int(255 * (1 - distance / max_dist))
            color = (intensity, intensity, 0)
            cv2.circle(img, centroid, radius, color, -1)

            # PID control for angular velocity
            error = img_center[0] - cx
            current_time = time.time()
            dt = current_time - self.previous_time if self.previous_time else 0.0

            # Proportional
            p = self.kp * error
            # Integral
            self.integral += error * dt
            i = self.ki * self.integral
            # Derivative
            d = self.kd * ((error - self.previous_error) / dt) if dt > 0 else 0.0

            angular_velocity = p + i + d
            # Clamp to avoid extreme turns
            angular_velocity = np.clip(angular_velocity, -0.5, 0.5)

            # Constant forward speed
            linear_velocity = self.linear_velocity_gain

            # Publish Twist message
            twist = Twist()
            twist.linear.x = linear_velocity
            twist.angular.z = angular_velocity
            self.publisher_.publish(twist)

            # Update time and error
            self.previous_time = current_time
            self.previous_error = error

        # Visualization windows
        cv2.imshow("Line Threshold", mask)
        cv2.imshow("Region of Interest", roi)
        cv2.imshow("Centroid Indicator", img)
        cv2.waitKey(1)


def main(args=None):
    rclpy.init(args=args)
    node = LineFollower()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
