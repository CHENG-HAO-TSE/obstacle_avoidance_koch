#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from moveit_msgs.msg import MotionPlanRequest
from moveit_ros_planning_interface._moveit_move_group_interface import MoveGroupInterface

class SimpleMoveItControl(Node):
    def __init__(self):
        super().__init__("simple_moveit_control")

        self.group = MoveGroupInterface("interbotix_arm", "robot_description")

        self.get_logger().info("=== Simple MoveIt Control ===")
        self.get_logger().info("MoveIt group initialized.")

        target_pose = PoseStamped()
        target_pose.header.frame_id = "base_link"
        target_pose.pose.position.x = 0.20
        target_pose.pose.position.y = 0.00
        target_pose.pose.position.z = 0.15
        target_pose.pose.orientation.w = 1.0

        self.get_logger().info("Sending target pose...")
        result = self.group.go_to_pose(target_pose)

        if result:
            self.get_logger().info("Motion SUCCESS!")
        else:
            self.get_logger().error("Motion FAILED!")

def main():
    rclpy.init()
    node = SimpleMoveItControl()
    rclpy.spin(node)

if __name__ == "__main__":
    main()
