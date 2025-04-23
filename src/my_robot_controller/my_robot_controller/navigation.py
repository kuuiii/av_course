#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped
import tf_transformations
import math
import time

class TurtleNavigationNode(Node):
    def __init__(self):
        super().__init__("navigation")
        self.get_logger().info("Navigation Node started")

        # Define goal positions and orientations (x, y, yaw in degrees)
        self.goal_poses = [
            {'x': 4.8, 'y': 4.4, 'yaw': 45},
            {'x': 0.3, 'y': -3.0, 'yaw': 90},
            {'x': -7.5, 'y': 4.0, 'yaw': -90},
            {'x': 3.5, 'y': -6.5, 'yaw': 180}
        ]
        self.current_goal_index = 0

        # Optional: Set initial pose
        self.initial_pose = {'x': 0.0, 'y': 0.0, 'yaw': 0.0}

        # Publishers
        self.initial_pose_publisher = self.create_publisher(
            PoseWithCovarianceStamped, "/initialpose", 10)
        self.goal_pose_publisher = self.create_publisher(
            PoseStamped, "/goal_pose", 10)

        # Subscriber
        self.odom_listener = self.create_subscription(
            Odometry, "/odom", self.odom_callback, 10)

        # Delay to allow simulation and Nav2 stack to start
        time.sleep(5)
        self.publish_initial_pose()
        time.sleep(5)
        self.publish_goal()

    def publish_initial_pose(self):
        self.get_logger().info("Publishing initial pose...")
        initial_pose = PoseWithCovarianceStamped()
        initial_pose.header.frame_id = 'map'
        initial_pose.pose.pose.position.x = self.initial_pose['x']
        initial_pose.pose.pose.position.y = self.initial_pose['y']
        quaternion = tf_transformations.quaternion_from_euler(0, 0, math.radians(self.initial_pose['yaw']))
        initial_pose.pose.pose.orientation.x = quaternion[0]
        initial_pose.pose.pose.orientation.y = quaternion[1]
        initial_pose.pose.pose.orientation.z = quaternion[2]
        initial_pose.pose.pose.orientation.w = quaternion[3]
        self.initial_pose_publisher.publish(initial_pose)

    def odom_callback(self, msg: Odometry):
        current_pose = msg.pose.pose
        goal_pose = self.goal_poses[self.current_goal_index]
        distance_to_goal = math.sqrt(
            (current_pose.position.x - goal_pose['x']) ** 2 +
            (current_pose.position.y - goal_pose['y']) ** 2
        )

        self.get_logger().info(f"Distance to goal {self.current_goal_index + 1}: {distance_to_goal:.2f}m")

        if distance_to_goal < 0.3:  # Threshold to consider the goal reached
            self.get_logger().info(f"Reached goal {self.current_goal_index + 1}")
            self.publish_next_goal()

    def publish_next_goal(self):
        if self.current_goal_index < len(self.goal_poses) - 1:
            self.current_goal_index += 1
            time.sleep(2)  # optional pause between goals
            self.publish_goal()
        else:
            self.get_logger().info("All goals reached! Navigation complete.")
            rclpy.shutdown()

    def publish_goal(self):
        goal = self.goal_poses[self.current_goal_index]
        pose_msg = PoseStamped()
        pose_msg.header.frame_id = 'map'
        pose_msg.pose.position.x = goal['x']
        pose_msg.pose.position.y = goal['y']
        quaternion = tf_transformations.quaternion_from_euler(0, 0, math.radians(goal['yaw']))
        pose_msg.pose.orientation.x = quaternion[0]
        pose_msg.pose.orientation.y = quaternion[1]
        pose_msg.pose.orientation.z = quaternion[2]
        pose_msg.pose.orientation.w = quaternion[3]
        time.sleep(0.5)
        self.goal_pose_publisher.publish(pose_msg)
        self.get_logger().info(f"Published goal {self.current_goal_index + 1}: "
                               f"({goal['x']}, {goal['y']}, yaw={goal['yaw']}°)")

def main(args=None):
    rclpy.init(args=args)
    node = TurtleNavigationNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("Navigation Node stopped by user")
    finally:
        rclpy.shutdown()
