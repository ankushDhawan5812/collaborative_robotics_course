#!/usr/bin/env python3

# import rclpy
# from rclpy.node import Node
# from visualization_msgs.msg import Marker

# class CubeLocatorSubscriber(Node):
#     def __init__(self):
#         super().__init__('cube_locator_subscriber')
#         self.subscription = self.create_subscription(
#             Marker,
#             '/locobot/camera_cube_locator',
#             self.listener_callback,
#             10)
#         self.subscription  # prevent unused variable warning

#     def listener_callback(self, msg):
#         self.get_logger().info('Received marker: "%s"' % msg)
#         position = msg.pose.position
        
#         # self.get_logger().info(f'Received marker position: x={position.x}, y={position.y}, z={position.z}')



# def main(args=None):
#     rclpy.init(args=args)
#     cube_locator_subscriber = CubeLocatorSubscriber()
#     rclpy.spin(cube_locator_subscriber)
#     cube_locator_subscriber.destroy_node()
#     rclpy.shutdown()

# if __name__ == '__main__':
#     main()

import rclpy
from rclpy.node import Node
from visualization_msgs.msg import MarkerArray  # Changed from Marker to MarkerArray

class CubeLocatorSubscriber(Node):
    def __init__(self):
        super().__init__('cube_locator_subscriber')
        self.subscription = self.create_subscription(
            MarkerArray,  # Changed from Marker to MarkerArray
            '/locobot/camera_cube_locator',
            self.listener_callback,
            10)
        self.subscription  # prevent unused variable warning

    def listener_callback(self, msg):
        self.get_logger().info(f'Received {len(msg.markers)} markers')  # Logging the number of markers received
        for marker in msg.markers:  # Iterate through each marker in the MarkerArray
            position = marker.pose.position
            color = get_color(marker)
            self.get_logger().info(f'Marker ID: {marker.id}, Color: {color}, Position: x={position.x}, y={position.y}, z={position.z}')

def get_color(marker):
    color_arr = marker.color
    if color_arr.r == 1 and color_arr.g == 1: # red and green means yellow
        color = "y"
    elif color_arr.r == 1:
        color = "r"
    elif color_arr.g == 1:
        color = "g"
    else:
        color = "b"
    return color

def main(args=None):
    rclpy.init(args=args)
    cube_locator_subscriber = CubeLocatorSubscriber()
    rclpy.spin(cube_locator_subscriber)
    cube_locator_subscriber.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
