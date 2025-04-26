import rclpy
from rclpy.node import Node

from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist


class MinimalSubscriber(Node):

    def __init__(self):
        super().__init__('minimal_subscriber')
        self.subscription = self.create_subscription(
            LaserScan,
            'scan',
            self.listener_callback,
            10)
        self.subscription  # prevent unused variable warning
        self.publisher_ = self.create_publisher(Twist, 'cmd_vel', 10)

    def listener_callback(self, msg): #this is what gets called when we hear from the topic
    	min_range = max(min(msg.ranges[330:390]), msg.range_min) # minimum detected value above range_min value
    	self.get_logger().info('I heard min: "%s"' % min_range)	# echo minimum range seen
    	#ranges array is 720 length
    	self.get_logger().info('Central reading: "%s"' % msg.ranges[360])
    	
    	
    	
    	if min_range < 1.0:
            msg = Twist()
            msg.linear.x = 0.0  # Linear velocity in x-axis
            msg.linear.y = 0.0
            msg.linear.z = 0.0
            msg.angular.x = 0.0
            msg.angular.y = 0.0
            msg.angular.z = 0.0 # Angular velocity around z-axis
            self.publisher_.publish(msg)
            self.get_logger().info('Publishing stop: "%s"' % msg.linear.x)
            # self.get_logger().info('Publishing range length: "%s" ' % len(msg.ranges))
        
        


def main(args=None):
    rclpy.init(args=args)

    minimal_subscriber = MinimalSubscriber()

    rclpy.spin(minimal_subscriber)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    minimal_subscriber.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
    
   
   # this is the format for our output data from the /scan topic
   # output type is std_msgs/msg/LaserScan
   # https://docs.ros2.org/foxy/api/sensor_msgs/msg/LaserScan.html
    
# std_msgs/msg/Header header
# float angle_min
# float angle_max
# float angle_increment
# float time_increment
# float scan_time
# float range_min
# float range_max
# float[] ranges
# float[] intensities
