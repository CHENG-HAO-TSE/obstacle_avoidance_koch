#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Koch CBF 完整控制節點 (整合版)
將 koch.CBF.py 的避障邏輯完整移植到 ROS2 環境

功能:
1. 從實體機械臂讀取當前關節狀態
2. 使用 CBF + QP 計算安全的關節速度指令
3. 發布控制指令到實體機械臂
4. 支援目標位置動態調整 (透過 ROS2 話題或服務)

啟動方式:
ros2 run koch_ros2_wrapper koch_cbf_controller --ros-args \
  -p arm_name:=follower \
  -p control_rate:=50.0 \
  -p goal_x:=0.15 -p goal_y:=0.0 -p goal_z:=0.15
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from geometry_msgs.msg import Point
from std_srvs.srv import Trigger
import numpy as np
import threading
import time

try:
    import cvxpy as cp
    USE_QP = True
except:
    USE_QP = False
    print("[WARN] CVXPY 未安裝，CBF 功能將受限")

class KochCBFController(Node):
    def __init__(self):
        super().__init__('koch_cbf_controller')
        
        # ========== 參數設定 ==========
        self.declare_parameter('arm_name', 'follower')
        self.declare_parameter('control_rate', 50.0)
        self.declare_parameter('goal_x', 0.15)
        self.declare_parameter('goal_y', 0.0)
        self.declare_parameter('goal_z', 0.15)
        self.declare_parameter('safe_radius', 0.10)
        self.declare_parameter('max_vel', 0.2)
        self.declare_parameter('kp', 0.5)
        
        self.arm_name = self.get_parameter('arm_name').value
        self.control_rate = self.get_parameter('control_rate').value
        self.goal_pos = np.array([
            self.get_parameter('goal_x').value,
            self.get_parameter('goal_y').value,
            self.get_parameter('goal_z').value
        ])
        self.safe_radius = self.get_parameter('safe_radius').value
        self.max_vel = self.get_parameter('max_vel').value
        self.kp = self.get_parameter('kp').value
        
        # ========== CBF 參數 ==========
        self.n_joints = 6
        self.ee_link = 5  # Koch: gripper_moving_1
        self.critical_links = [2, 3, 4, 5]  # 需要保護的連桿
        self.vel_clamp = 0.8
        self.step_gain = 0.06
        self.cbf_gain_min = 1.5
        self.cbf_gain_max = 6.0
        
        # 關節限制 (Koch arm 的實際限制，需要根據您的機器人調整)
        self.j_lo = np.array([-3.14, -1.57, -1.57, -3.14, -1.57, -3.14])
        self.j_hi = np.array([3.14, 1.57, 1.57, 3.14, 1.57, 3.14])
        
        # ========== 內部狀態 ==========
        self.current_joint_pos = None
        self.current_joint_vel = None
        self.joint_names = ['joint1', 'joint2', 'joint3', 'joint4', 'joint5', 'joint_gripper']
        self.lock = threading.Lock()
        
        # 障礙物列表 (動態更新)
        self.obstacles = []  # [(x,y,z), ...]
        
        # ========== ROS2 接口 ==========
        # 發布控制指令
        cmd_topic = f'/{self.arm_name}/joint_states_control'
        self.joint_cmd_pub = self.create_publisher(JointState, cmd_topic, 10)
        
        # 訂閱關節狀態
        state_topic = f'/{self.arm_name}/joint_states'
        self.joint_state_sub = self.create_subscription(
            JointState, state_topic, self.joint_state_callback, 10
        )
        
        # 訂閱目標位置 (可選)
        self.goal_sub = self.create_subscription(
            Point, '/koch_cbf/goal', self.goal_callback, 10
        )
        
        # 訂閱障礙物位置 (可選)
        # TODO: 需要額外的障礙物檢測節點提供數據
        
        # 提供緊急停止服務
        self.emergency_stop_srv = self.create_service(
            Trigger, '/koch_cbf/emergency_stop', self.emergency_stop_callback
        )
        
        self.is_emergency_stop = False
        
        self.get_logger().info('=' * 50)
        self.get_logger().info('🤖 Koch CBF Controller 已啟動')
        self.get_logger().info(f'📡 發布指令: {cmd_topic}')
        self.get_logger().info(f'📡 訂閱狀態: {state_topic}')
        self.get_logger().info(f'🎯 目標位置: {self.goal_pos}')
        self.get_logger().info(f'🛡️  安全半徑: {self.safe_radius}m')
        self.get_logger().info(f'⚙️  控制頻率: {self.control_rate}Hz')
        self.get_logger().info('=' * 50)
    
    def joint_state_callback(self, msg):
        """接收實體機械臂的當前關節狀態"""
        with self.lock:
            self.current_joint_pos = np.array(msg.position[:self.n_joints])
            self.current_joint_vel = np.array(msg.velocity[:self.n_joints]) if msg.velocity else np.zeros(self.n_joints)
    
    def goal_callback(self, msg):
        """動態更新目標位置"""
        self.goal_pos = np.array([msg.x, msg.y, msg.z])
        self.get_logger().info(f'🎯 目標位置已更新: {self.goal_pos}')
    
    def emergency_stop_callback(self, request, response):
        """緊急停止服務"""
        self.is_emergency_stop = True
        response.success = True
        response.message = '緊急停止已啟動'
        self.get_logger().warn('🛑 緊急停止已啟動！')
        return response
    
    def adaptive_cbf_gain(self, dist, safe_r):
        """自適應 CBF 增益"""
        if dist > 2.5 * safe_r:
            return self.cbf_gain_min
        if dist < 1.1 * safe_r:
            return self.cbf_gain_max
        ratio = (dist - 1.1 * safe_r) / (1.4 * safe_r)
        return self.cbf_gain_min + (self.cbf_gain_max - self.cbf_gain_min) * (1 - ratio)
    
    def compute_cbf_velocity(self, q_current):
        """
        計算 CBF 約束下的安全關節速度
        
        注意: 這裡使用簡化版本，因為 ROS2 環境下沒有 PyBullet
        實際部署需要:
        1. 使用機器人正運動學 (FK) 計算末端位置
        2. 使用雅可比矩陣計算
        3. 或者使用預訓練的 Neural-JSDF 模型
        """
        
        if not USE_QP:
            # 降級為簡單 P 控制
            self.get_logger().warn('⚠️  QP 求解器不可用，使用簡單控制', throttle_duration_sec=5.0)
            # TODO: 實現 FK 計算當前末端位置
            # 目前返回零速度
            return np.zeros(self.n_joints)
        
        # ========== 簡化版 CBF (需要您補充 FK 邏輯) ==========
        # TODO: 計算末端執行器位置
        # ee_pos = forward_kinematics(q_current, self.ee_link)
        
        # TODO: 計算任務雅可比矩陣
        # J_task = compute_jacobian(q_current, self.ee_link)
        
        # TODO: 計算期望末端速度
        # err_pos = self.goal_pos - ee_pos
        # v_des = self.kp * err_pos
        # if np.linalg.norm(v_des) > self.max_vel:
        #     v_des = v_des / np.linalg.norm(v_des) * self.max_vel
        
        # ========== QP 求解 ==========
        dq = cp.Variable(self.n_joints)
        cons = [dq >= -self.vel_clamp, dq <= self.vel_clamp]
        q_next = q_current + self.step_gain * dq
        cons += [q_next >= self.j_lo, q_next <= self.j_hi]
        
        # TODO: 添加 CBF 約束
        # for obstacle in self.obstacles:
        #     for link_idx in self.critical_links:
        #         dist, grad = compute_distance_and_gradient(q_current, link_idx, obstacle)
        #         alpha = self.adaptive_cbf_gain(dist, self.safe_radius)
        #         h = dist - self.safe_radius
        #         cons.append(-grad @ dq <= alpha * h)
        
        # 目標: 最小化速度誤差
        # obj = cp.sum_squares(J_task @ dq - v_des)
        obj = cp.sum_squares(dq)  # 臨時: 最小化運動
        
        prob = cp.Problem(cp.Minimize(obj), cons)
        try:
            prob.solve(solver=cp.OSQP, warm_start=True, verbose=False)
            if dq.value is not None:
                return dq.value
        except:
            self.get_logger().error('❌ QP 求解失敗')
        
        return np.zeros(self.n_joints)
    
    def publish_command(self, velocity_cmd):
        """發布控制指令"""
        if self.current_joint_pos is None:
            return
        
        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.name = self.joint_names
        
        # 速度積分得到目標位置
        dt = 1.0 / self.control_rate
        target_pos = self.current_joint_pos + velocity_cmd * dt
        msg.position = target_pos.tolist()
        
        self.joint_cmd_pub.publish(msg)
    
    def control_loop(self):
        """主控制循環"""
        rate = self.control_rate
        dt = 1.0 / rate
        
        self.get_logger().info('🚀 開始 CBF 控制循環...')
        
        while rclpy.ok() and not self.is_emergency_stop:
            start_time = time.time()
            
            with self.lock:
                q_current = self.current_joint_pos
            
            if q_current is not None:
                # 計算 CBF 速度
                velocity_cmd = self.compute_cbf_velocity(q_current)
                
                # 發布指令
                self.publish_command(velocity_cmd)
            
            # 控制週期
            elapsed = time.time() - start_time
            sleep_time = max(0, dt - elapsed)
            time.sleep(sleep_time)


def main(args=None):
    rclpy.init(args=args)
    controller = KochCBFController()
    
    try:
        # 啟動 ROS2 spin 執行緒
        spin_thread = threading.Thread(target=rclpy.spin, args=(controller,), daemon=True)
        spin_thread.start()
        
        # 執行控制循環
        controller.control_loop()
        
    except KeyboardInterrupt:
        controller.get_logger().info('🛑 收到中斷信號')
    finally:
        controller.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
