#!/usr/bin/env python3

import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from locobot_autonomy.action import MoveBase
from geometry_msgs.msg import Pose
from visualization_msgs.msg import Marker

class MoveBaseClient(Node):
    def __init__(self):
        super().__init__('move_base_client')
        self.client = ActionClient(self, MoveBase, '/movebase')
        self.subscription = self.create_subscription(
            Marker,
            '/locobot/camera_cube_locator',
            self.cube_locator_callback,
            10)


    def send_goal(self, position):
        goal_msg = MoveBase.Goal()
        goal_msg.target_pose.position.x = position.z
        goal_msg.target_pose.position.y = position.y
        goal_msg.target_pose.position.z = 0.0
        goal_msg.target_pose.orientation.x = 0.0
        goal_msg.target_pose.orientation.y = 0.0
        goal_msg.target_pose.orientation.z = 0.0
        goal_msg.target_pose.orientation.w = 0.0

        self.client.wait_for_server()
        self.send_goal_future = self.client.send_goal_async(goal_msg)
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
    rclpy.spin(move_base_client)
    move_base_client.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
