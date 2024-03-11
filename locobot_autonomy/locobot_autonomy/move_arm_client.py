#!/usr/bin/env python3

import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from la_msgs.action import MoveArm  # Import the correct action type

class MoveArmActionClient(Node):
    def __init__(self):
        super().__init__('arm_movement_action_client')
        self.arm_client = ActionClient(self, MoveArm, '/movearm')

    def send_goal(self, pose):
        goal_msg = MoveArm.Goal(pose=pose)
        self.arm_client.wait_for_server()
        self.send_goal_future = self.arm_client.send_goal_async(goal_msg, feedback_callback=self.feedback_callback)
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
        self.get_logger().info(f'Final Arm Position reached')
        self.get_logger().info('Movement completed successfully.')

    def feedback_callback(self, feedback_msg):
        feedback = feedback_msg.feedback
        self.get_logger().info(f'Current Arm Position: {feedback.current_pose}')

# def main(args=None):
#     rclpy.init(args=args)
#     client = MoveArmActionClient()

#     # Example pose, replace with your desired pose
#     desired_pose = [0.5, 0, 0.1, 0, 90, 0]  # Example pose values, adjust accordingly
    
#     client.get_logger().info("setting pose")
#     client.send_goal(desired_pose)
#     rclpy.spin(client)
#     client.destroy_node()
#     rclpy.shutdown()

if __name__ == '__main__':
    main()