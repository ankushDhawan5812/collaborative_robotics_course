#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.action import ActionServer
from rclpy.duration import Duration
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from std_msgs.msg import String
from control_msgs.msg import JointTrajectoryControllerState
from locobot_autonomy.action import MoveGripper
import time

class LocobotGrip(Node):
    def __init__(self):
        super().__init__('locobot_gripper')
        self.gripper_state = "unknown"

        # Gripper open and close positions
        self.open_positions = [1.0, -1.0]
        self.close_positions =  [0.0, 0.0]

        self.get_logger().info("Gripper Action Server started")

        self.grip_publisher = self.create_publisher(JointTrajectory, '/locobot/gripper_controller/joint_trajectory', 10)
        self.grip_subscriber = self.create_subscription(JointTrajectoryControllerState, '/locobot/gripper_controller/state', self.set_actual_positions_callback, 10)
        self.state_publisher = self.create_publisher(String, '/locobot/gripper_state', 10)
        self.start_time = None
        self.error = None
        self.actual_positions = [0.0, 0.0]
        self.loop_timeout = 10

        # Action Server
        self._action_server = ActionServer(
            self,
            MoveGripper,
            'movegripper',
            self.execute_callback
        )

    def set_actual_positions_callback(self, msg):
        self.actual_positions[0] = msg.actual.positions[0]
        self.actual_positions[1] = msg.actual.positions[1]

    def set_error(self, positions_desired):
        self.get_logger().info("Setting error")
        self.get_logger().info(f"Desired {positions_desired}")
        self.get_logger().info(f"Actual {self.actual_positions}")
        current_positions = self.actual_positions
        self.error = ((positions_desired[0] - current_positions[0]) ** 2 + (positions_desired[1] - current_positions[1]) ** 2) ** 0.5
        self.get_logger().info(f"Error {self.error}")
        return self.error

    def execute_callback(self, goal_handle):
        goal = goal_handle.request
        self.get_logger().info("here")
        feedback_msg = MoveGripper.Feedback()
        feedback_msg.feedback = f"Processing command: {goal.command}"
        goal_handle.publish_feedback(feedback_msg)
        self.start_time = time.time()
        self.error = None # set to none when the action is called

        while self.error is None or self.error > 0.005:
            self.get_logger().info(f"Error {self.error}")
            if time.time() - self.start_time > self.loop_timeout: # action shoudl not take more than 5 seconds to complete
                self.get_logger().info("Timeout")
                result = MoveGripper.Result()
                result.done = False
                goal_handle.abort()
                return result
            elif goal.command.lower() == "open":
                self.get_logger().info("Opening...")
                self.gripper_state = "open"
                self.publish_gripper_action(self.open_positions)
                self.get_logger().info(f"publishing {self.open_positions}")
                self.set_error([0.0145, -0.015])
            elif goal.command.lower() == "close":
                self.get_logger().info("Closing...")
                self.gripper_state = "close"
                self.publish_gripper_action(self.close_positions)
                self.get_logger().info(f"publishing {self.close_positions}")
                self.set_error([0.037, -0.037])
            else:
                self.get_logger().info(f"Invalid command: {goal.command}")
                result = MoveGripper.Result()
                result.done = False
                goal_handle.abort()
                return result

        result = MoveGripper.Result()

        # determine if gripper actually did the command according to the error
        if self.error <= 0.005:
            result.done = True
            goal_handle.succeed()
        else:
            result.done = False
        return result

    def publish_gripper_action(self, positions):
        gripAction = JointTrajectory()
        gripAction.header.frame_id = ''
        gripAction.joint_names = ['left_finger', 'right_finger']
        
        point = JointTrajectoryPoint()
        point.positions = positions
        point.velocities = [0.0, 0.0]
        point.accelerations = [0.0, 0.0]
        
        gripAction.points = [point]
        self.grip_publisher.publish(gripAction)

    def publish_state(self, state_message):
        state_msg = String()
        state_msg.data = state_message
        self.state_publisher.publish(state_msg)
        self.get_logger().info(state_message)

def main(args=None):
    rclpy.init(args=args)
    locobot_grip = LocobotGrip()
    rclpy.spin(locobot_grip)
    locobot_grip.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
