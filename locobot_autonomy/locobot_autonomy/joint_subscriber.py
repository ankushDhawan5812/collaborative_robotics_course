#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from trajectory_msgs.msg import JointTrajectory

class JointTrajectorySubscriber(Node):
    def __init__(self):
        super().__init__('joint_trajectory_subscriber')
        self.subscription = self.create_subscription(
            JointTrajectory,
            '/locobot/arm_controller/joint_trajectory',
            self.joint_trajectory_callback,
            10)
        self.subscription  # prevent unused variable warning

    def joint_trajectory_callback(self, msg):
        self.get_logger().info('Received Joint Trajectory:')
        for point in msg.points:
            positions_str = ', '.join([str(pos) for pos in point.positions])
            self.get_logger().info(f'  Positions: [{positions_str}]')
            # Add more details from `msg` as needed

def main(args=None):
    rclpy.init(args=args)
    joint_trajectory_subscriber = JointTrajectorySubscriber()
    rclpy.spin(joint_trajectory_subscriber)
    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    joint_trajectory_subscriber.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()