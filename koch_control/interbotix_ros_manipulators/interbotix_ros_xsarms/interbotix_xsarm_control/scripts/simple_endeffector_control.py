#!/usr/bin/env python3
"""
簡單的末端執行器控制 - 直接發送目標位置
就像在 RViz 中拖動 Interactive Marker 一樣！
"""

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from moveit_msgs.action import MoveGroup
from geometry_msgs.msg import PoseStamped, Pose
from moveit_msgs.msg import Constraints, PositionConstraint, OrientationConstraint
from shape_msgs.msg import SolidPrimitive
import sys

class SimpleEndEffectorControl(Node):
    def __init__(self):
        super().__init__('simple_endeffector_control')
        
        self.get_logger().info("===========================================")
        self.get_logger().info("Simple End Effector Control")
        self.get_logger().info("直接發送目標位置給 MoveIt")
        self.get_logger().info("===========================================")
        
        # 創建 Action Client
        self._action_client = ActionClient(
            self, 
            MoveGroup, 
            '/low_cost_robot/move_action'
        )
        
        self.get_logger().info("Waiting for MoveIt action server...")
        if not self._action_client.wait_for_server(timeout_sec=10.0):
            self.get_logger().error("MoveIt action server not available!")
            return
        
        self.get_logger().info("✓ Connected to MoveIt!")
    
    def send_goal(self, x, y, z, qx=0.0, qy=0.0, qz=0.0, qw=1.0):
        """
        發送目標位置給 MoveIt
        
        參數:
            x, y, z: 目標位置 (米)
            qx, qy, qz, qw: 目標方向 (四元數)
        """
        goal_msg = MoveGroup.Goal()
        
        # 設定規劃組
        goal_msg.request.group_name = "arm"
        goal_msg.request.num_planning_attempts = 10
        goal_msg.request.allowed_planning_time = 5.0
        goal_msg.request.max_velocity_scaling_factor = 0.3
        goal_msg.request.max_acceleration_scaling_factor = 0.3
        
        # 創建位置約束
        pose_stamped = PoseStamped()
        pose_stamped.header.frame_id = "world"
        pose_stamped.header.stamp = self.get_clock().now().to_msg()
        pose_stamped.pose.position.x = x
        pose_stamped.pose.position.y = y
        pose_stamped.pose.position.z = z
        pose_stamped.pose.orientation.x = qx
        pose_stamped.pose.orientation.y = qy
        pose_stamped.pose.orientation.z = qz
        pose_stamped.pose.orientation.w = qw
        
        # 設定約束
        constraint = Constraints()
        
        # 位置約束
        pos_constraint = PositionConstraint()
        pos_constraint.header = pose_stamped.header
        pos_constraint.link_name = "link5"  # end effector link
        pos_constraint.target_point_offset.x = 0.0
        pos_constraint.target_point_offset.y = 0.0
        pos_constraint.target_point_offset.z = 0.0
        
        # 創建一個小球體區域
        primitive = SolidPrimitive()
        primitive.type = SolidPrimitive.SPHERE
        primitive.dimensions = [0.0001]  # 1mm 容差
        
        pos_constraint.constraint_region.primitives.append(primitive)
        pos_constraint.constraint_region.primitive_poses.append(pose_stamped.pose)
        pos_constraint.weight = 1.0
        
        constraint.position_constraints.append(pos_constraint)
        
        # 方向約束
        ori_constraint = OrientationConstraint()
        ori_constraint.header = pose_stamped.header
        ori_constraint.link_name = "link5"
        ori_constraint.orientation = pose_stamped.pose.orientation
        ori_constraint.absolute_x_axis_tolerance = 0.1
        ori_constraint.absolute_y_axis_tolerance = 0.1
        ori_constraint.absolute_z_axis_tolerance = 0.1
        ori_constraint.weight = 1.0
        
        constraint.orientation_constraints.append(ori_constraint)
        
        goal_msg.request.goal_constraints.append(constraint)
        
        self.get_logger().info("-------------------------------------------")
        self.get_logger().info(f"Target Position: [{x:.3f}, {y:.3f}, {z:.3f}]")
        self.get_logger().info(f"Target Orientation: [{qx:.3f}, {qy:.3f}, {qz:.3f}, {qw:.3f}]")
        self.get_logger().info("-------------------------------------------")
        self.get_logger().info("Sending goal to MoveIt...")
        
        # 發送目標
        send_goal_future = self._action_client.send_goal_async(
            goal_msg,
            feedback_callback=self.feedback_callback
        )
        
        rclpy.spin_until_future_complete(self, send_goal_future)
        goal_handle = send_goal_future.result()
        
        if not goal_handle.accepted:
            self.get_logger().error("✗ Goal rejected by MoveIt")
            return False
        
        self.get_logger().info("✓ Goal accepted, planning...")
        
        # 等待結果
        get_result_future = goal_handle.get_result_async()
        rclpy.spin_until_future_complete(self, get_result_future)
        
        result = get_result_future.result().result
        
        if result.error_code.val == 1:  # SUCCESS
            self.get_logger().info("===========================================")
            self.get_logger().info("✓ Motion SUCCEEDED!")
            self.get_logger().info("===========================================")
            return True
        else:
            self.get_logger().error(f"✗ Motion FAILED with error code: {result.error_code.val}")
            return False
    
    def feedback_callback(self, feedback_msg):
        feedback = feedback_msg.feedback
        self.get_logger().info(f"State: {feedback.state}")


def main(args=None):
    rclpy.init(args=args)
    
    node = SimpleEndEffectorControl()
    
    # 從命令行讀取目標位置，或使用默認值
    if len(sys.argv) >= 4:
        x = float(sys.argv[1])
        y = float(sys.argv[2])
        z = float(sys.argv[3])
    else:
        # 默認目標位置
        x, y, z = 0.18, 0.0, 0.15
    
    # 可選：從命令行讀取方向（四元數）
    if len(sys.argv) >= 8:
        qx = float(sys.argv[4])
        qy = float(sys.argv[5])
        qz = float(sys.argv[6])
        qw = float(sys.argv[7])
    else:
        # 默認方向（直向下）
        qx, qy, qz, qw = 0.0, 0.0, 0.0, 1.0
    
    # 發送目標
    success = node.send_goal(x, y, z, qx, qy, qz, qw)
    
    node.destroy_node()
    rclpy.shutdown()
    
    return 0 if success else 1


if __name__ == '__main__':
    sys.exit(main())
