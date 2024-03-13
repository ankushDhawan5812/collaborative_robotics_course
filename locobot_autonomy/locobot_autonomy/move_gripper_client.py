#!/usr/bin/env python3

import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from locobot_autonomy.action import MoveGripper

class MoveGripperActionClient(Node):
    def __init__(self):
        super().__init__('gripper_action_client')
        self.client = ActionClient(self, MoveGripper, 'movegripper')
        self.action_complete = False

    def send_goal(self, command):
        goal_msg = MoveGripper.Goal(command=command)
        self.client.wait_for_server()

        self.get_logger().info(f'Sending goal to {command} the gripper')
        self._send_goal_future = self.client.send_goal_async(goal_msg)
        self._send_goal_future.add_done_callback(self.goal_response_callback)

    def goal_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().info('Goal rejected')
            return

        self.get_logger().info('Goal accepted')
        self._get_result_future = goal_handle.get_result_async()
        self._get_result_future.add_done_callback(self.get_result_callback)

    def get_result_callback(self, future):
        result = future.result().result
        if result.success:
            self.get_logger().info('Action completed successfully')
        else:
            self.get_logger().info('Action failed')

def main(args=None):
    rclpy.init(args=args)
    gripper_client = MoveGripperActionClient()
    command = 'close'  # or 'open', based on the desired action

    gripper_client.send_goal(command)

    rclpy.spin(gripper_client)
    gripper_client.get_logger().info('Interrupted by user, shutting down...')
    gripper_client.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()