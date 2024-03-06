#!/usr/bin/env python3
import sys

import rclpy
from rclpy.node import Node
from la_msgs.srv import Ptps  # Adjusted import to match the service type

class PixToPointClientAsync(Node):

    def __init__(self):
        super().__init__('pix_to_point_client_async')
        self.cli = self.create_client(Ptps, '/pix_to_point_cpp')  # Adjusted to Ptps service type and service name
        while not self.cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Service not available, waiting again...')
        self.req = Ptps.Request()

    def send_request(self, desired_frame):
        self.req.desired_frame = desired_frame  # Set the desired frame in the request
        self.future = self.cli.call_async(self.req)
        rclpy.spin_until_future_complete(self, self.future)
        return self.future.result()

def main(args=None):
    rclpy.init(args=args)

    client = PixToPointClientAsync()
    if len(sys.argv) > 1:
        desired_frame = sys.argv[1]
    else:
        desired_frame = 'locobot/base_link'  # Default value if not provided via command line
    response = client.send_request(desired_frame)
    
    # Assuming the response contains a field 'point' that you want to log.
    # You need to adjust this according to the actual service definition.
    red_point = response.red_points[0].point
    client.get_logger().info(
        f'Result of pix_to_point_cpp for desired_frame {desired_frame}: {red_point}')

    client.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()