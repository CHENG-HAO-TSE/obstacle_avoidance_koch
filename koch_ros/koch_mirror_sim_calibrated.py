#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Koch PyBullet Mirror Controller - 支援校正檔案
===============================================
修改重點：
1. 載入 follower_calibration.json
2. 發送關節指令時自動套用校正 offset
3. 確保虛擬手臂和實體手臂精確同步

使用方法：
  python3 koch_mirror_sim_calibrated.py
  
校正檔位置：
  /workspace/koch_ros/calibration/follower_calibration.json
"""

import time, math, numpy as np, json
from pathlib import Path
import pybullet as p
import pybullet_data
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import Bool
import threading

try:
    import cvxpy as cp
    USE_QP = True
except:
    USE_QP = False
    print("[WARN] CVXPY 未安裝，無法使用 CBF")

# ========== PyBullet CBF 設定（參照 koch_CBF.py）==========
SAFE_RADIUS_M = 0.12  # Koch 安全範圍，稍微放大一點
CRITICAL_LINKS = [2, 3, 4, 5]  # 保護後段連桿 link3_1(2), link4_1(3), gripper_static_1(4), gripper_moving_1(5)
MAX_EE_VEL = 0.10  # 最大末端速度 (m/s) - 降低以減少抖動
Kp, STEP_GAIN, VEL_CLAMP = 0.6, 0.04, 1.0  # Kp提高以確保能追蹤目標，步長減小避免震盪
BALL_RADIUS = 0.015  # 障礙物球體半徑
EE_LINK = 5  # Koch arm: gripper_moving_1 作為末端執行器
N_J = 6  # Koch arm 有 6 個可動關節

# 目標位置 - 更靠近手臂且更高
# Koch URDF 座標系: X=右, Y=前, Z=上
# 手臂伸直時末端大約在 Y=0.28~0.30m 的位置
goal = np.array([0.00, 0.15, 0.10], dtype=np.float32)  # X=0（中央），Y=0.15（靠近手臂），Z=0.10（更高）

CBF_GAIN_MIN, CBF_GAIN_MAX = 4.0, 12.0  # 提高 CBF 增益讓避障更積極

def adaptive_cbf_gain(dist, safe_r):
    """自適應 CBF 增益（參照 koch_CBF.py）"""
    if dist > 2.5*safe_r: return CBF_GAIN_MIN
    if dist < 1.1*safe_r: return CBF_GAIN_MAX
    ratio = (dist - 1.1*safe_r)/(1.4*safe_r)
    return CBF_GAIN_MIN + (CBF_GAIN_MAX - CBF_GAIN_MIN)*(1 - ratio)

# ========== 校正檔案載入 ==========
def load_calibration(calib_file="/workspace/koch_ros/calibration/zero_point_with_direction.json"):
    """
    載入零點校正檔案（含方向設定）
    
    Returns:
        dict: {motor_id: zero_tick}，例如 {1: 2048, 2: 3009, ...}
        注意：這裡返回的是 tick 值，不是弧度偏移
    """
    calib_path = Path(calib_file)
    
    if not calib_path.exists():
        print(f"⚠️  零點校正檔不存在: {calib_file}")
        print("   將使用預設零點 2048（可能導致虛實不同步）")
        print("   請先執行: python3 /workspace/koch_ros/calibration/simple_calibrate_v2.py")
        return {i: 2048 for i in range(1, 7)}
    
    try:
        with open(calib_path, 'r') as f:
            data = json.load(f)
        
        # 從 JSON 提取零點 tick
        zero_ticks = {}
        if "zero_positions_tick" in data:
            for motor_id_str, tick in data["zero_positions_tick"].items():
                motor_id = int(motor_id_str)
                zero_ticks[motor_id] = int(tick)
        else:
            print("⚠️  校正檔格式錯誤，缺少 'zero_positions_tick'")
            return {i: 2048 for i in range(1, 7)}
        
        print("✅ 已載入零點校正檔:")
        for motor_id, tick in zero_ticks.items():
            angle_deg = ((tick - 2048) / 4096.0) * 360.0
            print(f"   馬達 {motor_id}: zero = {tick:4d} tick ({angle_deg:+7.2f}°)")
        
        return zero_ticks
        
    except Exception as e:
        print(f"❌ 載入校正檔失敗: {e}")
        return {i: 2048 for i in range(1, 7)}

# ========== ROS2 發布節點（支援零點校正）==========
class KochMirrorPublisher(Node):
    def __init__(self, zero_ticks):
        super().__init__('koch_mirror_publisher_zero')
        
        # 儲存零點 tick（不再是偏移，而是參考位置）
        self.zero_ticks = zero_ticks
        
        # 參數
        self.declare_parameter('arm_name', 'follower')
        self.declare_parameter('publish_rate', 50.0)
        self.declare_parameter('enabled', True)
        
        self.arm_name = self.get_parameter('arm_name').value
        self.publish_rate = self.get_parameter('publish_rate').value
        self.enabled = self.get_parameter('enabled').value
        
        # 發布關節指令
        self.joint_cmd_pub = self.create_publisher(
            JointState,
            f'/{self.arm_name}/joint_states_control',
            10
        )
        
        # 訂閱啟用/停用
        self.enable_sub = self.create_subscription(
            Bool,
            '/koch_mirror/enable',
            self.enable_callback,
            10
        )
        
        self.joint_names = ['joint1', 'joint2', 'joint3', 'joint4', 'joint5', 'joint_gripper']
        
        self.get_logger().info('=' * 60)
        self.get_logger().info('🪞 Koch Mirror Publisher (零點校正版) 已啟動')
        self.get_logger().info(f'📡 發布到: /{self.arm_name}/joint_states_control')
        self.get_logger().info(f'📊 頻率: {self.publish_rate} Hz')
        self.get_logger().info(f'🎮 狀態: {"啟用" if self.enabled else "停用"}')
        self.get_logger().info(f'🔧 零點校正: {"已載入" if any(v != 2048 for v in zero_ticks.values()) else "使用預設"}')
        self.get_logger().info('=' * 60)
    
    def enable_callback(self, msg):
        self.enabled = msg.data
        status = "啟用" if self.enabled else "停用"
        self.get_logger().info(f'🎮 鏡像同步: {status}')
    
    def publish_joint_state(self, virtual_joint_positions):
        """
        發布關節位置到實體機械臂
        
        Args:
            virtual_joint_positions: PyBullet 虛擬手臂的關節角度 (numpy array, 6個值)
                                    這些是相對於 PyBullet 初始姿態 (q_home) 的角度
        """
        if not self.enabled:
            return
        
        # 直接發布 PyBullet 的角度（相對於參考姿態）
        # 實體控制器會負責轉換為實際的 tick 值
        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.name = self.joint_names
        msg.position = virtual_joint_positions.tolist()
        
        self.joint_cmd_pub.publish(msg)

# ========== PyBullet 場景設置 ==========
def setup_pybullet(use_gui=True):
    import os
    import subprocess
    
    # 檢查 X11 是否可用
    can_use_gui = False
    if use_gui and os.environ.get('DISPLAY'):
        try:
            # 測試是否可以連接到 X server
            result = subprocess.run(['xdpyinfo'], capture_output=True, timeout=1)
            can_use_gui = (result.returncode == 0)
        except:
            can_use_gui = False
        
        if not can_use_gui:
            print("[WARN] 無法連接到 X server，使用無 GUI 模式")
    
    if can_use_gui:
        try:
            cid = p.connect(p.GUI)
            if cid < 0:
                print("[WARN] GUI 模式失敗，切換到 DIRECT 模式")
                cid = p.connect(p.DIRECT)
        except Exception as e:
            print(f"[WARN] GUI 錯誤: {e}，切換到 DIRECT 模式")
            cid = p.connect(p.DIRECT)
    else:
        cid = p.connect(p.DIRECT)
    
    if cid < 0:
        raise RuntimeError("PyBullet 連接失敗")
    
    if p.getConnectionInfo()['connectionMethod'] == p.GUI:
        p.configureDebugVisualizer(p.COV_ENABLE_GUI, 1)
        p.configureDebugVisualizer(p.COV_ENABLE_MOUSE_PICKING, 1)
        p.resetDebugVisualizerCamera(
            cameraDistance=0.8,
            cameraYaw=45,
            cameraPitch=-30,
            cameraTargetPosition=[0, 0, 0.1]
        )
    
    p.setAdditionalSearchPath(pybullet_data.getDataPath())
    p.setGravity(0, 0, -9.81)
    p.setTimeStep(1./240.)
    
    # 地面
    p.loadURDF("plane.urdf", [0, 0, -0.01])
    
    # 載入 Koch 手臂
    urdf_path = "/workspace/model/koch_arm/koch.urdf"
    if not Path(urdf_path).exists():
        raise FileNotFoundError(f"找不到 URDF: {urdf_path}")
    
    robot_id = p.loadURDF(
        urdf_path,
        basePosition=[0, 0, 0],
        baseOrientation=p.getQuaternionFromEuler([0, 0, 0]),
        useFixedBase=True
    )
    
    # 初始姿態：鉤子形狀
    # - Joint2 往上 90° (shoulder向上)
    # - Joint3 向下 90° (elbow向下彎曲) → 修正：用 +90°
    # - Joint4 向下 90° (wrist向下彎曲) → 用 +90°
    # - 其他關節保持 0°
    # 
    # 根據測試結果：
    # - Motor4(方向=-1): +90° → tick=2100 ✅ (在範圍內，Joint4會向下彎)
    # - Motor4(方向=-1): -90° → tick=4146 ❌ (超出範圍)
    q_home = [
        0.0,                # joint1: 0° (waist 不轉)
        +np.pi/2,           # joint2: +90° (shoulder向上)
        +np.pi/2,           # joint3: +90° (elbow向下) [修正：方向反了，改用正號]
        +np.pi/2,           # joint4: +90° (因為 motor4 反向，+90° 會讓 joint4 向下彎)
        0.0,                # joint5: 0° (保持中立)
        0.0                 # joint6: 0° (gripper中立)
    ]
    
    print("\n🎯 設定 PyBullet 初始姿勢:")
    for i, q in enumerate(q_home):
        p.resetJointState(robot_id, i, q)
        print(f"   Joint{i+1}: {q:+7.3f} rad ({np.degrees(q):+7.2f}°)")
    
    # 目標球（綠色）- 可以拖動來控制手臂（無碰撞，僅視覺）
    goal_id = p.createMultiBody(
        baseMass=0,
        baseCollisionShapeIndex=-1,  # 無碰撞形狀
        baseVisualShapeIndex=p.createVisualShape(p.GEOM_SPHERE, radius=BALL_RADIUS*0.8, rgbaColor=[0, 1, 0, 0.7]),
        basePosition=goal
    )
    
    # 移動障礙物（紅色球）- 會左右移動通過目標位置
    obstacle_start_pos = np.array([0.25, 0.15, 0.10])  # 從右側開始（+25cm），Y與目標一致
    obstacle_id = p.createMultiBody(
        baseMass=0,
        baseCollisionShapeIndex=p.createCollisionShape(p.GEOM_SPHERE, radius=BALL_RADIUS),
        baseVisualShapeIndex=p.createVisualShape(p.GEOM_SPHERE, radius=BALL_RADIUS, rgbaColor=[1, 0, 0, 0.8]),
        basePosition=obstacle_start_pos
    )
    
    return robot_id, goal_id, q_home, obstacle_id

# ========== CBF QP Solver ==========
def solve_cbf_qp(grad_h, dist, d_safe, u_ref, alpha):
    """CBF QP 最佳化
    
    Args:
        grad_h: CBF 梯度 (1D array, 長度 N_J)，表示關節速度對距離的影響
        dist: 當前最小距離
        d_safe: 安全距離
        u_ref: 參考控制輸入（目標追蹤速度）
        alpha: CBF 增益
    
    約束: grad_h @ u >= -alpha * (dist - d_safe)
    意思是：關節運動必須讓距離不會減少得太快
    """
    if not USE_QP:
        return u_ref
    
    try:
        u = cp.Variable(N_J)
        h = dist - d_safe  # 障礙函數值
        Lgh = grad_h @ u   # 障礙函數對時間的導數
        
        objective = cp.Minimize(cp.sum_squares(u - u_ref))
        constraints = [Lgh >= -alpha * h]  # CBF 約束
        prob = cp.Problem(objective, constraints)
        prob.solve(solver=cp.OSQP, verbose=False, warm_start=True)
        
        if prob.status in ["optimal", "optimal_inaccurate"]:
            return np.array(u.value).flatten()
        else:
            return u_ref
    except:
        return u_ref

# ========== 主迴圈 ==========
def main_loop(robot_id, goal_id, q_home, ros_publisher, obstacle_id, warmup_time=5.0):
    """PyBullet 主迴圈 + ROS2 發布（含預熱階段和移動障礙物）
    
    Args:
        warmup_time: 預熱時間(秒)，在此期間保持初始姿勢，讓實體手臂慢慢移動到位
        obstacle_id: 移動障礙物的 ID
    """
    q_current = np.array(q_home, dtype=np.float32)
    
    # 障礙物移動參數（與目標位置對齊）
    obstacle_amplitude = 0.25  # 左右移動幅度 (±25cm) - 增加移動距離
    obstacle_period = 8.0      # 週期 8 秒
    obstacle_center_x = 0.00   # 中心 X 位置（通過目標）
    obstacle_y = 0.15          # Y 位置（與目標一致，Y=0.15m）
    obstacle_z = 0.10          # Z 高度（與目標同高度 0.10m）
    
    loop_count = 0
    last_pub_time = time.time()
    pub_interval = 1.0 / ros_publisher.publish_rate
    
    # 預熱階段
    print("\n⏳ 預熱階段: 讓實體手臂移動到初始姿勢...")
    print(f"   預計需要 {warmup_time:.1f} 秒")
    warmup_start = time.time()
    warmup_loops = 0
    
    while rclpy.ok() and (time.time() - warmup_start) < warmup_time:
        warmup_loops += 1
        
        # 預熱期間保持初始姿勢不變
        for i in range(N_J):
            p.resetJointState(robot_id, i, q_current[i])
        
        # 持續發布初始姿勢給實體手臂
        current_time = time.time()
        if current_time - last_pub_time >= pub_interval:
            ros_publisher.publish_joint_state(q_current)
            last_pub_time = current_time
        
        # 顯示倒數
        remaining = warmup_time - (time.time() - warmup_start)
        if warmup_loops % 50 == 0:
            print(f"   ⏱️  剩餘時間: {remaining:.1f}s")
        
        p.stepSimulation()
        time.sleep(1./240.)
    
    print("✅ 預熱完成! 開始跟隨目標移動\n")
    
    try:
        sim_start_time = time.time()
        
        while rclpy.ok():
            loop_count += 1
            current_sim_time = time.time() - sim_start_time
            
            # 更新移動障礙物位置（左右來回移動）
            obstacle_x = obstacle_center_x + obstacle_amplitude * np.sin(2 * np.pi * current_sim_time / obstacle_period)
            obstacle_pos = [obstacle_x, obstacle_y, obstacle_z]
            p.resetBasePositionAndOrientation(obstacle_id, obstacle_pos, [0, 0, 0, 1])
            
            # 更新目標位置（從 PyBullet 讀取，可以用滑鼠拖動）
            goal_pos, _ = p.getBasePositionAndOrientation(goal_id)
            goal[:] = goal_pos
            
            # 獲取當前狀態
            ee_state = p.getLinkState(robot_id, EE_LINK, computeLinkVelocity=1)
            ee_pos = np.array(ee_state[0], dtype=np.float32)
            
            # 計算目標速度
            err = goal - ee_pos
            err_norm = np.linalg.norm(err)
            v_des = np.clip(Kp * err, -MAX_EE_VEL, MAX_EE_VEL)
            
            # 獲取當前關節狀態
            q_current = np.array([p.getJointState(robot_id, i)[0] for i in range(N_J)], dtype=np.float32)
            
            # Jacobian
            J_full, _ = p.calculateJacobian(
                robot_id, EE_LINK,
                localPosition=[0, 0, 0],
                objPositions=q_current.tolist(),
                objVelocities=[0]*N_J,
                objAccelerations=[0]*N_J
            )
            J = np.array(J_full, dtype=np.float32)[:3, :N_J]
            
            # 參考控制指令（逆運動學）
            u_ref = np.linalg.pinv(J) @ v_des
            u_ref = np.clip(u_ref, -VEL_CLAMP, VEL_CLAMP)
            
            # CBF 避障：檢查所有關鍵連桿與障礙物的距離
            min_dist = float('inf')
            closest_link = None
            closest_link_pos = None
            closest_J = None
            
            for link_id in CRITICAL_LINKS:
                link_state = p.getLinkState(robot_id, link_id)
                link_pos = np.array(link_state[0], dtype=np.float32)
                
                # 計算連桿到障礙物的距離
                dist_to_obs = np.linalg.norm(link_pos - np.array(obstacle_pos))
                
                if dist_to_obs < min_dist:
                    min_dist = dist_to_obs
                    closest_link = link_id
                    closest_link_pos = link_pos
                    
                    # 計算這個連桿的 Jacobian
                    J_link_full, _ = p.calculateJacobian(
                        robot_id, link_id,
                        localPosition=[0, 0, 0],
                        objPositions=q_current.tolist(),
                        objVelocities=[0]*N_J,
                        objAccelerations=[0]*N_J
                    )
                    closest_J = np.array(J_link_full, dtype=np.float32)[:3, :N_J]
            
            # 應用 CBF
            if min_dist < 2.5 * SAFE_RADIUS_M and closest_J is not None:
                # 計算從障礙物指向連桿的方向（遠離障礙物的方向）
                obs_pos = np.array(obstacle_pos, dtype=np.float32)
                direction = (closest_link_pos - obs_pos) / (min_dist + 1e-6)
                
                # CBF 梯度：∇h = direction^T * J
                # 這個梯度表示關節速度如何影響連桿遠離障礙物
                grad_h = direction @ closest_J
                
                alpha = adaptive_cbf_gain(min_dist, SAFE_RADIUS_M)
                u_safe = solve_cbf_qp(grad_h, min_dist, SAFE_RADIUS_M, u_ref, alpha)
            else:
                u_safe = u_ref
            
            u_safe = np.clip(u_safe, -VEL_CLAMP, VEL_CLAMP)
            
            # 更新關節 - 使用 setJointMotorControl2 而不是 resetJointState
            q_new = q_current + STEP_GAIN * u_safe
            for i in range(N_J):
                p.setJointMotorControl2(
                    robot_id, i,
                    p.POSITION_CONTROL,
                    targetPosition=q_new[i],
                    force=500
                )
            
            q_current = q_new
            
            # ROS2 發布（按頻率）
            current_time = time.time()
            if current_time - last_pub_time >= pub_interval:
                ros_publisher.publish_joint_state(q_current)
                last_pub_time = current_time
            
            # 顯示資訊
            if loop_count % 100 == 0:
                cbf_active = min_dist < 2.0 * SAFE_RADIUS_M
                cbf_status = "🛡️ CBF" if cbf_active else "✅ Safe"
                threshold = 2.0 * SAFE_RADIUS_M
                print(f"[{loop_count:6d}] EE: ({ee_pos[0]:.3f}, {ee_pos[1]:.3f}, {ee_pos[2]:.3f}) | "
                      f"Goal: ({goal[0]:.3f}, {goal[1]:.3f}, {goal[2]:.3f}) | "
                      f"Obs: ({obstacle_pos[0]:.3f}, {obstacle_pos[1]:.3f}, {obstacle_pos[2]:.3f}) | "
                      f"Dist: {min_dist:.3f}m (閾值:{threshold:.3f}m) Link:{closest_link} {cbf_status}")
            
            p.stepSimulation()
            time.sleep(1./240.)
            
    except KeyboardInterrupt:
        print("\n⏹️  使用者中斷")

# ========== 主程式 ==========
def main():
    import sys
    
    print("=" * 60)
    print("🚀 Koch PyBullet Mirror (零點校正版) 啟動中...")
    print("=" * 60)
    
    # 載入零點校正檔
    zero_ticks = load_calibration()
    
    # 檢查是否使用 GUI
    use_gui = not ('--no-gui' in sys.argv or '--headless' in sys.argv)
    
    # 初始化 PyBullet
    print("\n🎮 初始化 PyBullet...")
    robot_id, goal_id, q_home, obstacle_id = setup_pybullet(use_gui)
    print("✅ PyBullet 場景已建立（含移動障礙物）")
    
    # 初始化 ROS2
    print("\n📡 初始化 ROS2...")
    rclpy.init()
    ros_publisher = KochMirrorPublisher(zero_ticks)
    
    # 在背景執行 ROS2 spin
    ros_thread = threading.Thread(
        target=lambda: rclpy.spin(ros_publisher),
        daemon=True
    )
    ros_thread.start()
    print("✅ ROS2 節點已啟動")
    
    # 主迴圈
    print("\n▶️  開始控制迴圈...")
    print("=" * 60)
    print("💡 操作說明:")
    print("   - 系統會先進行 5 秒預熱，讓實體手臂移動到初始姿勢")
    print("   - 預熱完成後，紅色球會左右移動通過目標位置")
    print("   - 手臂會使用 CBF 自動避開移動的紅色障礙物")
    print("   - 可以在 PyBullet 視窗中拖動綠色球（目標）")
    print("   - 實體手臂會即時同步虛擬手臂的動作")
    print("   - 按 Ctrl+C 停止")
    print("=" * 60)
    
    try:
        main_loop(robot_id, goal_id, q_home, ros_publisher, obstacle_id, warmup_time=5.0)
    finally:
        print("\n🛑 清理資源...")
        ros_publisher.destroy_node()
        rclpy.shutdown()
        p.disconnect()
        print("✅ 已安全退出")

if __name__ == "__main__":
    main()
