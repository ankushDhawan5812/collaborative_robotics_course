#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
from rclpy.qos import QoSProfile, ReliabilityPolicy


class ImageSubscriber(Node):
    def __init__(self):
        super().__init__('image_subscriber')
        # qos_profile = QoSProfile(depth=10, reliability=ReliabilityPolicy.RELIABLE)
        qos_profile = QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT)
        self.get_logger().info(f"Starting the image subscriber")
        # topic = '/locobot/camera_frame_sensor/block_color_filt_img'
        topic = '/locobot/camera_frame_sensor/image_raw'
        self.subscription = self.create_subscription(
            Image,
            topic,
            self.image_callback,
            qos_profile)
        self.subscription  # prevent unused variable warning
        self.bridge = CvBridge()

    def image_callback(self, msg):
        self.get_logger().info(f"Calling callback")
        cv_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
        cv2.imshow("Camera Frame", cv_image)
        cv2.waitKey(1)

def main(args=None):
    rclpy.init(args=args)
    image_subscriber = ImageSubscriber()
    rclpy.spin(image_subscriber)
    image_subscriber.get_logger().info(f"Node spun up")
    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    image_subscriber.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
