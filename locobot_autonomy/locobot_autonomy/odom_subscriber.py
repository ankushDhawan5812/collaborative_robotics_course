#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry  # Import the Odometry message type

class OdomSubscriber(Node):
    def __init__(self):
        super().__init__('odom_subscriber')
        self.subscription = self.create_subscription(
            Odometry,  # Use the Odometry message type
            '/odom',  # Subscribe to the /odom topic
            self.odom_callback,
            10)
        self.subscription  # prevent unused variable warning

    def odom_callback(self, msg):
        # Log some basic info from the Odometry message
        position = msg.pose.pose.position
        orientation = msg.pose.pose.orientation
        self.get_logger().info(f'Position: [{position.x}, {position.y}, {position.z}], Orientation: [{orientation.x}, {orientation.y}, {orientation.z}, {orientation.w}]')

def main(args=None):
    rclpy.init(args=args)
    odom_subscriber = OdomSubscriber()
    rclpy.spin(odom_subscriber)
    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    odom_subscriber.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
