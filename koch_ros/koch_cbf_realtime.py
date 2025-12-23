#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Koch CBF Real-time Controller
實時避障控制節點 - 針對實體機械臂優化

特點:
1. 使用 Pinocchio 實現高效 FK/Jacobian 計算
2. 多線程設計: ROS2 通訊與控制計算分離
3. 低延遲: 目標 < 5ms 計算時間
4. 實時安全監控

啟動:
python3 koch_cbf_realtime.py
"""

import rclpy
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
from rclpy.callback_groups import ReentrantCallbackGroup
from sensor_msgs.msg import JointState
from geometry_msgs.msg import Point, PointStamped
from std_msgs.msg import Float64MultiArray
from std_srvs.srv import Trigger
import numpy as np
import threading
import time
from collections import deque

try:
    import pinocchio as pin
    USE_PINOCCHIO = True
except:
    USE_PINOCCHIO = False
    print("[ERROR] Pinocchio 未安裝！請執行: pip install pin")
    exit(1)

try:
    import cvxpy as cp
    USE_QP = True
except:
    USE_QP = False
    print("[WARN] CVXPY 未安裝，CBF 功能將受限")

class KochCBFRealtime(Node):
    def __init__(self):
        super().__init__('koch_cbf_realtime')
        
        # 使用 ReentrantCallbackGroup 允許並行回調
        self.callback_group = ReentrantCallbackGroup()
        
        # ========== 參數配置 ==========
        self.declare_parameter('arm_name', 'follower')
        self.declare_parameter('control_rate', 100.0)  # 提高到 100Hz
        self.declare_parameter('urdf_path', '/workspace/model/koch_arm/koch.urdf')
        
        # 目標與控制參數
        self.declare_parameter('goal_x', 0.15)
        self.declare_parameter('goal_y', 0.0)
        self.declare_parameter('goal_z', 0.15)
        self.declare_parameter('kp', 0.5)
        self.declare_parameter('max_vel', 0.2)
        
        # CBF 參數
        self.declare_parameter('safe_radius', 0.10)
        self.declare_parameter('cbf_gain_min', 1.5)
        self.declare_parameter('cbf_gain_max', 6.0)
        
        # 讀取參數
        self.arm_name = self.get_parameter('arm_name').value
        self.control_rate = self.get_parameter('control_rate').value
        self.urdf_path = self.get_parameter('urdf_path').value
        
        self.goal_pos = np.array([
            self.get_parameter('goal_x').value,
            self.get_parameter('goal_y').value,
            self.get_parameter('goal_z').value
        ])
        
        self.kp = self.get_parameter('kp').value
        self.max_vel = self.get_parameter('max_vel').value
        self.safe_radius = self.get_parameter('safe_radius').value
        self.cbf_gain_min = self.get_parameter('cbf_gain_min').value
        self.cbf_gain_max = self.get_parameter('cbf_gain_max').value
        
        # ========== Pinocchio 初始化 ==========
        self.model = pin.buildModelFromUrdf(self.urdf_path)
        self.data = self.model.createData()
        self.n_joints = 6
        self.ee_frame_id = self.model.getFrameId("gripper_moving_1")  # Koch 末端執行器
        
        # 關節限制
        self.q_min = self.model.lowerPositionLimit[:self.n_joints]
        self.q_max = self.model.upperPositionLimit[:self.n_joints]
        
        # CBF 保護連桿
        self.critical_frames = []
        for link_name in ["link3_1", "link4_1", "gripper_static_1", "gripper_moving_1"]:
            try:
                frame_id = self.model.getFrameId(link_name)
                self.critical_frames.append(frame_id)
            except:
                self.get_logger().warn(f'⚠️  找不到連桿: {link_name}')
        
        # ========== 內部狀態 (線程安全) ==========
        self.lock = threading.Lock()
        self.current_q = None
        self.current_dq = None
        self.obstacles = []  # [(x,y,z), ...]
        self.is_emergency_stop = False
        self.joint_names = ['joint1', 'joint2', 'joint3', 'joint4', 'joint5', 'joint_gripper']
        
        # 性能監控
        self.compute_times = deque(maxlen=100)
        
        # ========== ROS2 接口 ==========
        # 訂閱關節狀態 (高優先級)
        self.joint_state_sub = self.create_subscription(
            JointState,
            f'/{self.arm_name}/joint_states',
            self.joint_state_callback,
            10,
            callback_group=self.callback_group
        )
        
        # 發布控制指令
        self.joint_cmd_pub = self.create_publisher(
            JointState,
            f'/{self.arm_name}/joint_states_control',
            10
        )
        
        # 訂閱目標位置
        self.goal_sub = self.create_subscription(
            Point,
            '/koch_cbf/goal',
            self.goal_callback,
            10,
            callback_group=self.callback_group
        )
        
        # 訂閱障礙物
        self.obstacle_sub = self.create_subscription(
            PointStamped,
            '/koch_cbf/obstacle',
            self.obstacle_callback,
            10,
            callback_group=self.callback_group
        )
        
        # 發布調試信息
        self.debug_pub = self.create_publisher(
            Float64MultiArray,
            '/koch_cbf/debug',
            10
        )
        
        # 緊急停止服務
        self.emergency_stop_srv = self.create_service(
            Trigger,
            '/koch_cbf/emergency_stop',
            self.emergency_stop_callback,
            callback_group=self.callback_group
        )
        
        # 重置服務
        self.reset_srv = self.create_service(
            Trigger,
            '/koch_cbf/reset',
            self.reset_callback,
            callback_group=self.callback_group
        )
        
        self.get_logger().info('=' * 60)
        self.get_logger().info('🚀 Koch CBF Real-time Controller 已啟動')
        self.get_logger().info(f'📡 控制頻率: {self.control_rate} Hz')
        self.get_logger().info(f'🎯 目標位置: [{self.goal_pos[0]:.3f}, {self.goal_pos[1]:.3f}, {self.goal_pos[2]:.3f}]')
        self.get_logger().info(f'🛡️  安全半徑: {self.safe_radius} m')
        self.get_logger().info(f'⚙️  保護連桿數: {len(self.critical_frames)}')
        self.get_logger().info('=' * 60)
    
    def joint_state_callback(self, msg):
        """接收關節狀態 (實時更新)"""
        with self.lock:
            self.current_q = np.array(msg.position[:self.n_joints])
            self.current_dq = np.array(msg.velocity[:self.n_joints]) if msg.velocity else np.zeros(self.n_joints)
    
    def goal_callback(self, msg):
        """動態更新目標"""
        self.goal_pos = np.array([msg.x, msg.y, msg.z])
        self.get_logger().info(f'🎯 目標更新: [{msg.x:.3f}, {msg.y:.3f}, {msg.z:.3f}]')
    
    def obstacle_callback(self, msg):
        """接收障礙物位置"""
        obs_pos = np.array([msg.point.x, msg.point.y, msg.point.z])
        with self.lock:
            # 簡單策略: 只保留最近的 5 個障礙物
            self.obstacles.append(obs_pos)
            if len(self.obstacles) > 5:
                self.obstacles.pop(0)
    
    def emergency_stop_callback(self, request, response):
        """緊急停止"""
        self.is_emergency_stop = True
        response.success = True
        response.message = '🛑 緊急停止已啟動'
        self.get_logger().error(response.message)
        return response
    
    def reset_callback(self, request, response):
        """重置控制器"""
        self.is_emergency_stop = False
        with self.lock:
            self.obstacles.clear()
        response.success = True
        response.message = '✅ 控制器已重置'
        self.get_logger().info(response.message)
        return response
    
    def forward_kinematics(self, q, frame_id):
        """計算正運動學 (Pinocchio 加速)"""
        pin.framesForwardKinematics(self.model, self.data, q)
        return self.data.oMf[frame_id].translation.copy()
    
    def compute_jacobian(self, q, frame_id):
        """計算雅可比矩陣"""
        pin.computeFrameJacobian(self.model, self.data, q, frame_id, pin.LOCAL_WORLD_ALIGNED)
        J_full = pin.getFrameJacobian(self.model, self.data, frame_id, pin.LOCAL_WORLD_ALIGNED)
        return J_full[:3, :self.n_joints]  # 只取位置部分
    
    def adaptive_cbf_gain(self, dist):
        """自適應 CBF 增益"""
        safe_r = self.safe_radius
        if dist > 2.5 * safe_r:
            return self.cbf_gain_min
        if dist < 1.1 * safe_r:
            return self.cbf_gain_max
        ratio = (dist - 1.1 * safe_r) / (1.4 * safe_r)
        return self.cbf_gain_min + (self.cbf_gain_max - self.cbf_gain_min) * (1.0 - ratio)
    
    def compute_cbf_control(self, q, dq):
        """
        計算 CBF 約束下的關節速度
        返回: (velocity_cmd, success, debug_info)
        """
        start_time = time.perf_counter()
        
        # 1. 計算末端位置和雅可比
        ee_pos = self.forward_kinematics(q, self.ee_frame_id)
        J_ee = self.compute_jacobian(q, self.ee_frame_id)
        
        # 2. 計算期望末端速度
        err_pos = self.goal_pos - ee_pos
        dist_to_goal = np.linalg.norm(err_pos)
        
        v_des = self.kp * err_pos
        v_norm = np.linalg.norm(v_des)
        if v_norm > self.max_vel:
            v_des = v_des / v_norm * self.max_vel
        
        # 3. 設置 QP 問題
        vel_clamp = 0.8
        step_gain = 0.06
        
        dq_var = cp.Variable(self.n_joints)
        constraints = [
            dq_var >= -vel_clamp,
            dq_var <= vel_clamp
        ]
        
        # 關節限制
        q_next = q + step_gain * dq_var
        constraints.extend([
            q_next >= self.q_min + 0.05,
            q_next <= self.q_max - 0.05
        ])
        
        # 4. 添加 CBF 約束 (針對所有保護連桿和障礙物)
        cbf_active = 0
        with self.lock:
            obstacles = self.obstacles.copy()
        
        for obs_pos in obstacles:
            for frame_id in self.critical_frames:
                link_pos = self.forward_kinematics(q, frame_id)
                dist_vec = link_pos - obs_pos
                dist = np.linalg.norm(dist_vec)
                
                if dist < 2.0 * self.safe_radius:  # 只在接近時計算
                    # 計算梯度
                    J_link = self.compute_jacobian(q, frame_id)
                    grad = (dist_vec / (dist + 1e-6)) @ J_link
                    
                    # CBF 約束
                    alpha = self.adaptive_cbf_gain(dist)
                    h = dist - self.safe_radius
                    constraints.append(-grad @ dq_var <= alpha * h)
                    cbf_active += 1
        
        # 5. 目標函數: 最小化末端速度誤差
        objective = cp.sum_squares(J_ee @ dq_var - v_des)
        
        # 6. 求解 QP
        prob = cp.Problem(cp.Minimize(objective), constraints)
        
        try:
            prob.solve(solver=cp.OSQP, warm_start=True, verbose=False, eps_abs=1e-4, eps_rel=1e-4)
            
            if dq_var.value is not None:
                velocity_cmd = dq_var.value
                success = True
            else:
                velocity_cmd = np.zeros(self.n_joints)
                success = False
                self.get_logger().warn('⚠️  QP 求解失敗，返回零速度', throttle_duration_sec=1.0)
        
        except Exception as e:
            velocity_cmd = np.zeros(self.n_joints)
            success = False
            self.get_logger().error(f'❌ QP 異常: {e}', throttle_duration_sec=1.0)
        
        # 計算時間
        compute_time = (time.perf_counter() - start_time) * 1000  # ms
        self.compute_times.append(compute_time)
        
        debug_info = {
            'compute_time_ms': compute_time,
            'dist_to_goal': dist_to_goal,
            'cbf_active': cbf_active,
            'num_obstacles': len(obstacles)
        }
        
        return velocity_cmd, success, debug_info
    
    def publish_command(self, velocity_cmd):
        """發布控制指令"""
        with self.lock:
            if self.current_q is None:
                return
            q = self.current_q.copy()
        
        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.name = self.joint_names
        
        # 速度積分
        dt = 1.0 / self.control_rate
        target_pos = q + velocity_cmd * dt
        msg.position = target_pos.tolist()
        
        self.joint_cmd_pub.publish(msg)
    
    def publish_debug_info(self, debug_info):
        """發布調試信息"""
        msg = Float64MultiArray()
        msg.data = [
            debug_info['compute_time_ms'],
            debug_info['dist_to_goal'],
            float(debug_info['cbf_active']),
            float(debug_info['num_obstacles'])
        ]
        self.debug_pub.publish(msg)
    
    def control_loop(self):
        """實時控制主循環"""
        rate = self.control_rate
        dt = 1.0 / rate
        
        self.get_logger().info('🚀 啟動實時控制循環...')
        
        loop_count = 0
        last_report_time = time.time()
        
        while rclpy.ok() and not self.is_emergency_stop:
            loop_start = time.perf_counter()
            
            # 獲取當前狀態
            with self.lock:
                if self.current_q is None:
                    time.sleep(dt)
                    continue
                q = self.current_q.copy()
                dq = self.current_dq.copy()
            
            # 計算 CBF 控制
            velocity_cmd, success, debug_info = self.compute_cbf_control(q, dq)
            
            # 發布控制指令
            if success:
                self.publish_command(velocity_cmd)
            
            # 發布調試信息
            self.publish_debug_info(debug_info)
            
            loop_count += 1
            
            # 每秒報告性能
            if time.time() - last_report_time > 1.0:
                avg_time = np.mean(self.compute_times) if self.compute_times else 0
                max_time = np.max(self.compute_times) if self.compute_times else 0
                self.get_logger().info(
                    f'⏱️  性能 | 平均: {avg_time:.2f}ms | 最大: {max_time:.2f}ms | '
                    f'到目標: {debug_info["dist_to_goal"]:.3f}m | CBF: {debug_info["cbf_active"]}',
                    throttle_duration_sec=1.0
                )
                last_report_time = time.time()
            
            # 控制週期
            elapsed = time.perf_counter() - loop_start
            sleep_time = max(0, dt - elapsed)
            if sleep_time > 0:
                time.sleep(sleep_time)
            else:
                self.get_logger().warn(
                    f'⚠️  控制循環超時! 耗時: {elapsed*1000:.2f}ms',
                    throttle_duration_sec=2.0
                )


def main(args=None):
    rclpy.init(args=args)
    
    controller = KochCBFRealtime()
    
    # 使用多線程執行器
    executor = MultiThreadedExecutor(num_threads=4)
    executor.add_node(controller)
    
    # 在獨立線程中啟動 ROS2 spin
    spin_thread = threading.Thread(target=executor.spin, daemon=True)
    spin_thread.start()
    
    try:
        # 主線程執行控制循環
        controller.control_loop()
    except KeyboardInterrupt:
        controller.get_logger().info('🛑 收到中斷信號')
    finally:
        controller.destroy_node()
        rclpy.shutdown()
        spin_thread.join(timeout=1.0)


if __name__ == '__main__':
    main()
