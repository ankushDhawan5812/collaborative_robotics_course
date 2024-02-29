#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from visualization_msgs.msg import Marker

class CubeLocatorSubscriber(Node):
    def __init__(self):
        super().__init__('cube_locator_subscriber')
        self.subscription = self.create_subscription(
            Marker,
            '/locobot/camera_cube_locator',
            self.listener_callback,
            10)
        self.subscription  # prevent unused variable warning

    def listener_callback(self, msg):
        self.get_logger().info('Received marker: "%s"' % msg)
        position = msg.pose.position
        
        # self.get_logger().info(f'Received marker position: x={position.x}, y={position.y}, z={position.z}')



def main(args=None):
    rclpy.init(args=args)
    cube_locator_subscriber = CubeLocatorSubscriber()
    rclpy.spin(cube_locator_subscriber)
    cube_locator_subscriber.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
