#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Koch PyBullet Mirror Controller
PyBullet 虛擬控制 + 實時同步到實體機械臂

功能:
1. 在 PyBullet 中運行完整的 CBF 避障控制
2. 實時將虛擬機械臂的關節狀態同步到實體機械臂
3. 支持鍵盤/滑鼠控制虛擬目標點
4. 動態障礙物可視化

啟動:
  # GUI 模式 (需要 X11 顯示器)
  python3 koch_mirror_sim.py
  
  # 無頭模式 (Docker/SSH 環境)
  python3 koch_mirror_sim.py --no-gui
  python3 koch_mirror_sim.py --headless
"""

import time, math, numpy as np
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
    print("[WARN] CVXPY 未安裝")

# ========== PyBullet CBF 設定 (來自 koch.CBF.py) ==========
SAFE_RADIUS_M = 0.10
CRITICAL_LINKS = [2, 3, 4, 5]
MAX_EE_VEL = 0.2
Kp, STEP_GAIN, VEL_CLAMP = 0.5, 0.06, 0.8
BALL_RADIUS = 0.03
EE_LINK = 5
N_J = 6

# 預設目標位置
goal = np.array([0.15, 0.00, 0.15], dtype=np.float32)

# CBF 增益
CBF_GAIN_MIN, CBF_GAIN_MAX = 1.5, 6.0

def adaptive_cbf_gain(dist, safe_r):
    if dist > 2.5*safe_r: return CBF_GAIN_MIN
    if dist < 1.1*safe_r: return CBF_GAIN_MAX
    ratio = (dist - 1.1*safe_r)/(1.4*safe_r)
    return CBF_GAIN_MIN + (CBF_GAIN_MAX - CBF_GAIN_MIN)*(1 - ratio)

# ========== ROS2 發布節點 ==========
class KochMirrorPublisher(Node):
    def __init__(self):
        super().__init__('koch_mirror_publisher')
        
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
        self.get_logger().info('🪞 Koch Mirror Publisher 已啟動')
        self.get_logger().info(f'📡 發布到: /{self.arm_name}/joint_states_control')
        self.get_logger().info(f'📊 頻率: {self.publish_rate} Hz')
        self.get_logger().info(f'🎮 狀態: {"啟用" if self.enabled else "停用"}')
        self.get_logger().info('=' * 60)
        self.get_logger().info('💡 控制說明:')
        self.get_logger().info('   - 在 PyBullet 視窗中用滑鼠拖動綠色目標球')
        self.get_logger().info('   - 虛擬機械臂會避障移動到目標')
        self.get_logger().info('   - 實體機械臂會實時跟隨虛擬機械臂')
        self.get_logger().info('   - 按 Ctrl+C 停止')
        self.get_logger().info('=' * 60)
    
    def enable_callback(self, msg):
        self.enabled = msg.data
        status = "啟用" if self.enabled else "停用"
        self.get_logger().info(f'🎮 鏡像同步: {status}')
    
    def publish_joint_state(self, joint_positions):
        """發布關節位置到實體機械臂"""
        if not self.enabled:
            return
        
        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.name = self.joint_names
        msg.position = joint_positions.tolist()
        
        self.joint_cmd_pub.publish(msg)

# ========== PyBullet 場景設置 ==========
def setup_pybullet(use_gui=True):
    """
    設置 PyBullet 環境
    
    Args:
        use_gui: True=GUI模式(需要顯示器), False=DIRECT模式(無頭)
    """
    # 嘗試連接 GUI，失敗則降級到 DIRECT 模式
    if use_gui:
        try:
            cid = p.connect(p.GUI)
            if cid < 0:
                print("[WARN] GUI 模式連接失敗，切換到 DIRECT 模式")
                cid = p.connect(p.DIRECT)
        except Exception as e:
            print(f"[WARN] GUI 模式錯誤: {e}")
            print("[INFO] 切換到 DIRECT 無頭模式")
            cid = p.connect(p.DIRECT)
    else:
        cid = p.connect(p.DIRECT)
    
    if cid < 0:
        raise RuntimeError("PyBullet 連接失敗")
    
    # 只在 GUI 模式下配置視覺化
    if p.getConnectionInfo()['connectionMethod'] == p.GUI:
        p.configureDebugVisualizer(p.COV_ENABLE_GUI, 1)
        p.configureDebugVisualizer(p.COV_ENABLE_MOUSE_PICKING, 1)
        p.resetDebugVisualizerCamera(
            cameraDistance=0.8,
            cameraYaw=50,
            cameraPitch=-20,
            cameraTargetPosition=[0.15, 0.05, 0.15]
        )
        print("[INFO] PyBullet GUI 模式已啟動")
    else:
        print("[INFO] PyBullet DIRECT 無頭模式已啟動")
    
    p.setAdditionalSearchPath(pybullet_data.getDataPath())
    p.setGravity(0, 0, -9.81)
    p.setRealTimeSimulation(0)
    
    plane = p.loadURDF("plane.urdf")
    robot = p.loadURDF("/workspace/model/koch_arm/koch.urdf", useFixedBase=True)
    
    return robot

# ========== 幾何計算 ==========
def get_link_pos(robot, link_idx):
    return np.array(p.getLinkState(robot, link_idx)[4])

def get_link_jacobian(robot, q_arm, link_idx, total_dof):
    q_full = list(q_arm) + [0.0] * (total_dof - N_J)
    z = [0.0] * total_dof
    Jlin = p.calculateJacobian(robot, link_idx, [0, 0, 0], q_full, z, z)[0]
    return np.array(Jlin)[:, :N_J]

# ========== QP 求解器 ==========
def solve_qp(J_task, v_des, q, d_list, g_list, r_list, j_lo, j_hi):
    dq = cp.Variable(N_J)
    cons = [dq >= -VEL_CLAMP, dq <= VEL_CLAMP]
    q_next = q + STEP_GAIN * dq
    cons += [q_next >= j_lo, q_next <= j_hi]
    
    for d, g, r in zip(d_list, g_list, r_list):
        if np.linalg.norm(g) < 1e-6:
            continue
        alpha = adaptive_cbf_gain(d, r)
        h = d - r
        cons.append(-g @ dq <= alpha * h)
    
    prob = cp.Problem(cp.Minimize(cp.sum_squares(J_task @ dq - v_des)), cons)
    
    try:
        prob.solve(solver=cp.OSQP, warm_start=True, max_iter=5000, verbose=False)
    except:
        pass
    
    if prob.status in ["optimal", "optimal_inaccurate"] and dq.value is not None:
        return dq.value
    else:
        return np.zeros(N_J)

# ========== 主程序 ==========
def main():
    import sys
    
    # 檢查命令行參數
    use_gui = '--no-gui' not in sys.argv and '--headless' not in sys.argv
    
    if not use_gui:
        print("[INFO] 🖥️  無頭模式 (DIRECT) - 不使用 GUI")
    else:
        print("[INFO] 🖥️  嘗試啟動 GUI 模式...")
    
    # 初始化 ROS2
    rclpy.init()
    ros_node = KochMirrorPublisher()
    
    # 在獨立線程中運行 ROS2
    def ros_spin():
        rclpy.spin(ros_node)
    
    ros_thread = threading.Thread(target=ros_spin, daemon=True)
    ros_thread.start()
    
    # 設置 PyBullet
    print("[INFO] 正在啟動 PyBullet 模擬...")
    robot = setup_pybullet(use_gui=use_gui)
    
    # 偵測關節
    J_IDX_MOVABLE = [i for i in range(p.getNumJoints(robot)) 
                     if p.getJointInfo(robot, i)[2] != p.JOINT_FIXED]
    TOTAL_DOF = len(J_IDX_MOVABLE)
    J_IDX = list(range(N_J))
    
    # 關節限制 (從 URDF 讀取)
    j_lo = np.array([p.getJointInfo(robot, j)[8] for j in J_IDX])
    j_hi = np.array([p.getJointInfo(robot, j)[9] for j in J_IDX])
    
    # 顯示真實關節限制
    print("\n[INFO] 🔧 關節限制 (rad):")
    joint_names = ['joint1', 'joint2', 'joint3', 'joint4', 'joint5', 'joint_gripper']
    for i, (name, lo, hi) in enumerate(zip(joint_names, j_lo, j_hi)):
        print(f"  {name:15s}: [{lo:6.3f}, {hi:6.3f}] rad = [{np.degrees(lo):6.1f}°, {np.degrees(hi):6.1f}°]")
    
    # 安全初始位置 (手臂向前完全伸直，容易擺放和校正)
    # joint1=0°(朝前), joint2=90°(水平), joint3=180°(伸直), joint4=0°, joint5=0°, gripper=0.5
    q_home = [0.0, 1.57, 3.14, 0.0, 0.0, 0.5]
    # 驗證並夾限初始位置
    q_home = np.clip(q_home, j_lo, j_hi)
    print(f"\n[INFO] 🏠 初始位置 (手臂完全向前伸直): {[f'{q:.3f}' for q in q_home]}")
    
    for j, q0 in zip(J_IDX, q_home):
        p.resetJointState(robot, j, q0)
    
    # 創建障礙物球
    def create_ball(color, pos):
        vis = p.createVisualShape(p.GEOM_SPHERE, radius=BALL_RADIUS, rgbaColor=color)
        col = p.createCollisionShape(p.GEOM_SPHERE, radius=BALL_RADIUS)
        return p.createMultiBody(baseMass=0, baseCollisionShapeIndex=col,
                                baseVisualShapeIndex=vis, basePosition=pos)
    
    ball1 = create_ball((1, 0, 0, 1), (0.18, 0.00, 0.18))
    ball2 = create_ball((0, 0, 1, 1), (0.22, 0.08, 0.16))
    
    # 創建目標球 (可拖動)
    goal_vis = p.createVisualShape(p.GEOM_SPHERE, radius=0.02, rgbaColor=(0, 1, 0, 0.6))
    goal_ball = p.createMultiBody(baseMass=0, baseVisualShapeIndex=goal_vis, basePosition=goal)
    
    # 障礙物運動函數
    def centers_fn(t):
        A, v = 0.08, 0.10
        w = v / A
        c1 = np.array([0.18, -0.10, 0.20 + A * math.sin(w * t)], dtype=np.float32)
        c2 = np.array([0.18, 0.10, 0.20 + A * math.sin(w * t + math.pi)], dtype=np.float32)
        return [c1, c2]
    
    def upd_balls(cs):
        p.resetBasePositionAndOrientation(ball1, cs[0].tolist(), [0, 0, 0, 1])
        p.resetBasePositionAndOrientation(ball2, cs[1].tolist(), [0, 0, 0, 1])
    
    print("[INFO] ✅ PyBullet 場景已設置")
    print("[INFO] 🚀 開始控制循環...")
    print("[INFO] 💡 提示: 用滑鼠拖動綠色球改變目標位置")
    
    # 控制循環
    t = 0.0
    dt = 1.0 / 240.0
    publish_counter = 0
    publish_interval = int(240.0 / ros_node.publish_rate)  # 發布頻率轉換
    
    try:
        while p.isConnected():
            t += dt
            
            # 更新障礙物
            centers = centers_fn(t)
            upd_balls(centers)
            
            # 獲取目標位置 (可能被用戶拖動)
            goal_pos_current = np.array(p.getBasePositionAndOrientation(goal_ball)[0])
            
            # 獲取當前關節狀態
            joint_states = [p.getJointState(robot, i) for i in J_IDX]
            q = np.array([s[0] for s in joint_states])
            
            # 計算末端位置
            ee_pos = get_link_pos(robot, EE_LINK)
            err = goal_pos_current - ee_pos
            dist_to_goal = np.linalg.norm(err)
            
            # 計算期望速度
            v_des = Kp * err
            v_norm = np.linalg.norm(v_des)
            if v_norm > MAX_EE_VEL:
                v_des = (MAX_EE_VEL / v_norm) * v_des
            
            # 計算雅可比
            J_ee = get_link_jacobian(robot, q, EE_LINK, TOTAL_DOF)
            
            # 收集 CBF 約束
            d_list, g_list, r_list = [], [], []
            
            for center in centers:
                for link_idx in CRITICAL_LINKS:
                    link_pos = get_link_pos(robot, link_idx)
                    dist_vec = link_pos - center
                    dist = np.linalg.norm(dist_vec)
                    
                    if dist < 2.0 * SAFE_RADIUS_M:  # 只在接近時計算
                        J_link = get_link_jacobian(robot, q, link_idx, TOTAL_DOF)
                        grad = (dist_vec / (dist + 1e-6)) @ J_link
                        
                        d_list.append(dist)
                        g_list.append(grad)
                        r_list.append(SAFE_RADIUS_M)
            
            # QP 求解
            if USE_QP and len(d_list) > 0:
                dq_cmd = solve_qp(J_ee, v_des, q, d_list, g_list, r_list, j_lo, j_hi)
            elif USE_QP:
                # 無障礙物，簡單 IK
                dq_cmd = np.linalg.pinv(J_ee) @ v_des
                dq_cmd = np.clip(dq_cmd, -VEL_CLAMP, VEL_CLAMP)
            else:
                dq_cmd = np.zeros(N_J)
            
            # 應用速度控制到虛擬機械臂
            for i, joint_idx in enumerate(J_IDX):
                p.setJointMotorControl2(
                    robot, joint_idx,
                    p.VELOCITY_CONTROL,
                    targetVelocity=dq_cmd[i],
                    force=100.0
                )
            
            # 步進模擬
            p.stepSimulation()
            
            # 發布到實體機械臂 (降低頻率)
            publish_counter += 1
            if publish_counter >= publish_interval:
                # 安全檢查：確保關節位置在限制內
                q_safe = np.clip(q, j_lo, j_hi)
                
                # 檢查是否有關節超出限制
                if not np.allclose(q, q_safe, atol=0.01):
                    exceeded = []
                    for i, (name, q_val, q_s, lo, hi) in enumerate(zip(joint_names, q, q_safe, j_lo, j_hi)):
                        if abs(q_val - q_s) > 0.01:
                            exceeded.append(f"{name}={np.degrees(q_val):.1f}° (限制: [{np.degrees(lo):.1f}°, {np.degrees(hi):.1f}°])")
                    print(f"[WARN] ⚠️  關節超限已夾限: {', '.join(exceeded)}")
                
                ros_node.publish_joint_state(q_safe)
                publish_counter = 0
            
            time.sleep(dt)
    
    except KeyboardInterrupt:
        print("\n[INFO] 🛑 收到中斷信號")
    
    finally:
        print("[INFO] 正在關閉...")
        p.disconnect()
        ros_node.destroy_node()
        rclpy.shutdown()
        ros_thread.join(timeout=1.0)
        print("[INFO] ✅ 已關閉")

if __name__ == '__main__':
    main()
