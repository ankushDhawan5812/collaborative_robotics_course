#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import os
from datetime import datetime

class ImageSubscriber(Node):
    def __init__(self):
        super().__init__('image_subscriber')

        self.subscription = self.create_subscription(
            Image,
            '/locobot/camera_frame_sensor/image_raw',
            self.image_callback,
            10)
        self.get_logger().info(f"Starting the image subscriber")
        self.subscription  # prevent unused variable warning
        self.bridge = CvBridge()
        self.image_counter = 0  # Initialize a counter for image filenames

    def image_callback(self, msg):
        cv_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
        # Generate a unique filename for each image
        filename = f"camera_frame_{self.image_counter}.png"
        # Alternatively, you can use a timestamp to generate unique filenames
        # filename = f"camera_frame_{datetime.now().strftime('%Y%m%d_%H%M%S')}.png"
        
        # Save the image
        cv2.imwrite(filename, cv_image)
        self.get_logger().info(f"Saved {filename}")
        self.image_counter += 1  # Increment the counter

def main(args=None):
    rclpy.init(args=args)
    image_subscriber = ImageSubscriber()
    rclpy.spin(image_subscriber)
    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    image_subscriber.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()