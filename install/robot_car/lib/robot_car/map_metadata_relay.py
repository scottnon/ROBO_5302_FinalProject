#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from nav_msgs.msg import OccupancyGrid, MapMetaData

class MapMetadataRelay(Node):
    def __init__(self):
        super().__init__('map_metadata_relay')
        self.subscription = self.create_subscription(
            OccupancyGrid,
            '/map',
            self.map_callback,
            10
        )
        self.publisher = self.create_publisher(
            OccupancyGrid, # MapMetaData,
            '/map_metadata',
            10
        )
        self.get_logger().info('Waiting for /map...')

    def map_callback(self, msg):
        self.publisher.publish(msg.info)
        self.get_logger().info('Publishing /map_metadata from /map.')

def main(args=None):
    rclpy.init(args=args)
    node = OccupancyGrid() #MapMetadataRelay()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()