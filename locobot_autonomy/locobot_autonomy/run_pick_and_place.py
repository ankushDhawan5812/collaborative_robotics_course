#!/usr/bin/env python3

import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from geometry_msgs.msg import Pose
# from some_interfaces.action import MoveBase, MoveArm, OperateGripper, DetectBlock
from locobot_autonomy.action import MoveBase, MoveArm, MoveGripper
from locobot_autonomy.move_base_client import MoveBaseClient
from locobot_autonomy.move_arm_client import MoveArmActionClient
from locobot_autonomy.move_gripper_client import MoveGripperActionClient

import time

class LoCoBotPickPlace(Node):
    def __init__(self):
        super().__init__('locobot_pick_place')
        # self.detect_block_client = ActionClient(self, DetectBlock, 'detect_block')
        
        self.move_base_client = MoveBaseClient()
        self.camera_client = self.move_base_client.point_cli # send cube points
        self.base_client = self.move_base_client.base_cli # move base
        self.move_arm_client = MoveArmActionClient() # move arm
        self.move_gripper_client = MoveGripperActionClient()
        # self.move_arm_and_grip_gripper_client = ActionClient(self, MoveGripper, 'operate_gripper') # move gripper

        # self.start_detection()

    def start_detection(self, desired_frame):
        self.get_logger().info('Starting block detection...')
        # goal = DetectBlock.Goal()
        # self.detect_block_client.wait_for_server()
        # self.detect_block_client.send_goal_async(goal, feedback_callback=self.detect_feedback_callback)
        response = self.move_base_client.send_request(desired_frame)

        #Choose which block to detect here. For now, lets default to the first red-point
        self.red_point = response.red_points[0].point
        self.move_base_client.get_logger().info(
            f'Result of pix_to_point_cpp for desired_frame {desired_frame}: {self.red_point}')
        
        # Move the gripper to the goal
        # gripper_state = "open"
        # self.move_gripper(gripper_state)
        # self.move_base_to_block(red_point)
        # arm_pickup_pose = [0.44, 0, 0, 0, 90, 0]
        # self.move_arm(arm_pickup_pose)

    def move_base_to_block(self, position):
        self.get_logger().info("Moving base to the block")
        self.move_base_client.send_goal(position)

    def move_arm(self, arm_pose):
        # Sequence to move the arm above the block, grip it, and lift
        self.get_logger().info("Moving arm to the block")
        self.move_arm_client.send_goal(arm_pose)
    
    def move_gripper(self, gripper_state):
        self.get_logger().info(f"Gripper Action: {gripper_state}")
        self.move_gripper_client.send_goal(gripper_state)
        while self.move_gripper_client.action_complete is None or self.move_gripper_client.action_complete is False:
            rclpy.spin_once(self.move_gripper_client)
            self.get_logger().info('Gripper action not complete...')
        self.get_logger().info('Gripper action complete...')
        

def main(args=None):
    rclpy.init(args=args)
    pickplace = LoCoBotPickPlace()

    # Test the gripper
    pickplace.move_gripper("close") # gripper action should fully complete because of while loop in move_gripper()
    pickplace.move_gripper("close")
    pickplace.move_gripper("open") # gripper action should fully complete because of while loop in move_gripper()

    # Clean up resources
    pickplace.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
