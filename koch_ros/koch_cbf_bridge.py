#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Koch CBF Bridge Node
將 koch.CBF.py 的避障控制結果透過 ROS2 傳送到實體機械臂

功能:
- 從 koch.CBF.py 獲取 CBF 計算後的關節速度指令
- 轉換為 ROS2 JointState 消息
- 發布到實體機械臂的控制話題

使用方式:
1. 先啟動實體機械臂控制節點: ros2 run koch_ros2_wrapper koch_follower
2. 再啟動此橋接節點: python3 koch_cbf_bridge.py
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64MultiArray
import numpy as np
import threading
import time

# 匯入 koch.CBF 的核心邏輯
import sys
sys.path.append('/workspace/Neural-JSDF/learning/nn-learning')

class KochCBFBridge(Node):
    def __init__(self):
        super().__init__('koch_cbf_bridge')
        
        # 參數設定
        self.declare_parameter('arm_name', 'follower')
        self.declare_parameter('control_rate', 50.0)  # Hz
        self.declare_parameter('use_position_mode', False)  # True=位置控制, False=速度控制
        
        self.arm_name = self.get_parameter('arm_name').value
        self.control_rate = self.get_parameter('control_rate').value
        self.use_position_mode = self.get_parameter('use_position_mode').value
        
        # ROS2 發布者
        cmd_topic = f'/{self.arm_name}/joint_states_control'
        self.joint_cmd_pub = self.create_publisher(JointState, cmd_topic, 10)
        
        # ROS2 訂閱者 (接收實際關節狀態)
        state_topic = f'/{self.arm_name}/joint_states'
        self.joint_state_sub = self.create_subscription(
            JointState, state_topic, self.joint_state_callback, 10
        )
        
        # 內部狀態
        self.current_joint_pos = None
        self.current_joint_vel = None
        self.joint_names = ['joint1', 'joint2', 'joint3', 'joint4', 'joint5', 'joint_gripper']
        self.n_joints = 6
        
        # 控制指令緩衝
        self.cmd_velocity = np.zeros(self.n_joints)
        self.cmd_position = None
        self.lock = threading.Lock()
        
        self.get_logger().info(f'🤖 Koch CBF Bridge 已啟動')
        self.get_logger().info(f'📡 發布控制指令到: {cmd_topic}')
        self.get_logger().info(f'📡 訂閱關節狀態: {state_topic}')
        self.get_logger().info(f'⚙️  控制模式: {"位置控制" if self.use_position_mode else "速度控制"}')
        
    def joint_state_callback(self, msg):
        """接收實體機械臂的當前關節狀態"""
        with self.lock:
            self.current_joint_pos = np.array(msg.position[:self.n_joints])
            self.current_joint_vel = np.array(msg.velocity[:self.n_joints]) if msg.velocity else np.zeros(self.n_joints)
    
    def update_command(self, velocity_cmd, position_cmd=None):
        """
        更新控制指令 (由 CBF 控制循環調用)
        velocity_cmd: [6] 關節速度 (rad/s)
        position_cmd: [6] 目標關節位置 (rad), 可選
        """
        with self.lock:
            self.cmd_velocity = velocity_cmd.copy()
            if position_cmd is not None:
                self.cmd_position = position_cmd.copy()
    
    def publish_command(self):
        """發布控制指令到實體機械臂"""
        if self.current_joint_pos is None:
            self.get_logger().warn('⚠️  尚未收到關節狀態，跳過發布', throttle_duration_sec=1.0)
            return
        
        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.name = self.joint_names
        
        with self.lock:
            if self.use_position_mode and self.cmd_position is not None:
                # 位置控制模式
                msg.position = self.cmd_position.tolist()
            else:
                # 速度控制模式: 積分速度得到目標位置
                dt = 1.0 / self.control_rate
                target_pos = self.current_joint_pos + self.cmd_velocity * dt
                msg.position = target_pos.tolist()
        
        self.joint_cmd_pub.publish(msg)
    
    def get_current_state(self):
        """獲取當前關節狀態 (供 CBF 控制器使用)"""
        with self.lock:
            return self.current_joint_pos, self.current_joint_vel


def main(args=None):
    rclpy.init(args=args)
    bridge = KochCBFBridge()
    
    try:
        # 啟動 ROS2 spin 執行緒
        spin_thread = threading.Thread(target=rclpy.spin, args=(bridge,), daemon=True)
        spin_thread.start()
        
        # 主循環: 執行 CBF 控制 + 發布指令
        rate = bridge.control_rate
        dt = 1.0 / rate
        
        bridge.get_logger().info('🚀 開始 CBF 控制循環...')
        
        while rclpy.ok():
            start_time = time.time()
            
            # 獲取當前狀態
            q_current, qd_current = bridge.get_current_state()
            
            if q_current is not None:
                # ============ 這裡整合您的 koch.CBF.py 邏輯 ============
                # 目前先用簡單的測試指令
                # TODO: 替換為實際的 CBF 計算
                velocity_cmd = np.zeros(6)  # 替換為 CBF 輸出
                
                # 更新指令
                bridge.update_command(velocity_cmd)
                
                # 發布到機械臂
                bridge.publish_command()
            
            # 控制週期
            elapsed = time.time() - start_time
            sleep_time = max(0, dt - elapsed)
            time.sleep(sleep_time)
            
    except KeyboardInterrupt:
        bridge.get_logger().info('🛑 收到中斷信號，正在關閉...')
    finally:
        bridge.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
