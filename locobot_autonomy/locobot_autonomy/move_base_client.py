#!/usr/bin/env python3

import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from locobot_autonomy.action import MoveBase
from geometry_msgs.msg import Pose
from visualization_msgs.msg import Marker
from la_msgs.srv import Ptps  # Adjusted import to match the service type
import sys
import time


class MoveBaseClient(Node):
    def __init__(self):
        super().__init__('move_base_client')
        self.base_cli = ActionClient(self, MoveBase, '/movebase')
        # self.subscription = self.create_subscription(
        #     Marker,
        #     '/locobot/camera_cube_locator',
        #     self.cube_locator_callback,
        #     10)
        self.point_cli = self.create_client(Ptps, '/pix_to_point_cpp')  # Adjusted to Ptps service type and service name
        while not self.point_cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Service not available, waiting again...')
        self.req = Ptps.Request()
    
    def send_request(self, desired_frame):
        self.req.desired_frame = desired_frame  # Set the desired frame in the request
        self.future = self.point_cli.call_async(self.req)
        rclpy.spin_until_future_complete(self, self.future)
        return self.future.result()



    def send_goal(self, point):
        goal_msg = MoveBase.Goal()
        goal_msg.target_pose.position.x = (point.x)
        goal_msg.target_pose.position.y = (point.y)
        goal_msg.target_pose.position.z = 0.5
        goal_msg.target_pose.orientation.x = 0.0
        goal_msg.target_pose.orientation.y = 0.0
        goal_msg.target_pose.orientation.z = 0.0
        goal_msg.target_pose.orientation.w = 1.0

        #Set angle to true always
        goal_msg.control_base_angle_bool = True
        
        self.base_cli.wait_for_server()
        self.send_goal_future = self.base_cli.send_goal_async(goal_msg)
        self.send_goal_future.add_done_callback(self.goal_response_callback)

    def goal_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().info('Goal rejected :(')
            return

        self.get_logger().info('Goal accepted :)')

        self.result_future = goal_handle.get_result_async()
        self.result_future.add_done_callback(self.get_result_callback)

    def get_result_callback(self, future):
        result = future.result().result
        self.get_logger().info('Result: {}'.format(result))

    def cube_locator_callback(self, msg):
        self.get_logger().info('Received marker: "%s"' % msg)
        position = msg.pose.position
        self.get_logger().info('\n Received Position: "%s"' % position)
        self.send_goal(position)

def main(args=None):
    rclpy.init(args=args)
    move_base_client = MoveBaseClient()
    #move_base_client.send_goal(1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0)

    if len(sys.argv) > 1:
        desired_frame = sys.argv[1]
    else:
        desired_frame = 'locobot/base_link'  # Default value if not provided via command line
    response = move_base_client.send_request(desired_frame)

    red_point = response.red_points[0].point
    move_base_client.get_logger().info(
        f'Result of pix_to_point_cpp for desired_frame {desired_frame}: {red_point}')

    move_base_client.get_logger().info('Sleeping...')
    time.sleep(5)
    move_base_client.send_goal(red_point)

    rclpy.spin(move_base_client)
    move_base_client.destroy_node()
    rclpy.shutdown()

# if __name__ == '__main__':
#     main()
