#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Koch 真實世界障礙物避障控制器
===============================================
功能：
1. 使用 RealSense 相機檢測真實世界的彩色障礙物
2. 將檢測到的障礙物同步到 PyBullet 模擬環境
3. 使用 CBF 演算法進行避障控制
4. 支援零點校正檔案
5. 即時同步虛擬手臂和實體手臂

使用方法：
  # 有 RealSense 相機
  python3 koch_real_obstacle_avoidance.py
  
  # 無相機（使用模擬障礙物）
  python3 koch_real_obstacle_avoidance.py --no-camera

依賴：
  pip install pyrealsense2 opencv-python

校正檔位置：
  /workspace/koch_ros/calibration/zero_point_with_direction.json
  /workspace/avoid_real/avoid_real/camera_calibration.json
"""

import time, math, numpy as np, json, sys, os
from pathlib import Path
import pybullet as p
import pybullet_data
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import Bool
import threading
import cv2

# 導入真實障礙物檢測模組
sys.path.append('/workspace/avoid_real/avoid_real')
try:
    from realsense_camera import RealSenseCamera
    from obstacle_detector import ObstacleDetector
    from coordinate_transform import CoordinateTransform
    REALSENSE_AVAILABLE = True
except Exception as e:
    print(f"⚠️  無法載入 RealSense 模組: {e}")
    REALSENSE_AVAILABLE = False

try:
    import cvxpy as cp
    USE_QP = True
except:
    USE_QP = False
    print("[WARN] CVXPY 未安裝，無法使用 CBF")

# ========== CBF 設定 ==========
SAFE_RADIUS_M = 0.12  # Koch 安全範圍
CRITICAL_LINKS = [2, 3, 4, 5]  # 保護連桿
MAX_EE_VEL = 0.10  # 最大末端速度
Kp, STEP_GAIN, VEL_CLAMP = 0.6, 0.04, 1.0
BALL_RADIUS = 0.015  # 障礙物可視化半徑（實際半徑從相機檢測）
EE_LINK = 5
N_J = 6

# 目標位置
goal = np.array([0.00, 0.15, 0.10], dtype=np.float32)

CBF_GAIN_MIN, CBF_GAIN_MAX = 4.0, 12.0

def adaptive_cbf_gain(dist, safe_r):
    """自適應 CBF 增益"""
    if dist > 2.5*safe_r: return CBF_GAIN_MIN
    if dist < 1.1*safe_r: return CBF_GAIN_MAX
    ratio = (dist - 1.1*safe_r)/(1.4*safe_r)
    return CBF_GAIN_MIN + (CBF_GAIN_MAX - CBF_GAIN_MIN)*(1 - ratio)

# ========== 校正檔案載入 ==========
def load_calibration(calib_file="/workspace/koch_ros/calibration/zero_point_with_direction.json"):
    """載入零點校正檔案"""
    calib_path = Path(calib_file)
    
    if not calib_path.exists():
        print(f"⚠️  零點校正檔不存在: {calib_file}")
        print("   將使用預設零點 2048")
        return {i: 2048 for i in range(1, 7)}
    
    try:
        with open(calib_path, 'r') as f:
            data = json.load(f)
        
        zero_ticks = {}
        if "zero_positions_tick" in data:
            for motor_id_str, tick in data["zero_positions_tick"].items():
                motor_id = int(motor_id_str)
                zero_ticks[motor_id] = int(tick)
        else:
            print("⚠️  校正檔格式錯誤")
            return {i: 2048 for i in range(1, 7)}
        
        print("✅ 已載入零點校正檔:")
        for motor_id, tick in zero_ticks.items():
            angle_deg = ((tick - 2048) / 4096.0) * 360.0
            print(f"   馬達 {motor_id}: zero = {tick:4d} tick ({angle_deg:+7.2f}°)")
        
        return zero_ticks
        
    except Exception as e:
        print(f"❌ 載入校正檔失敗: {e}")
        return {i: 2048 for i in range(1, 7)}

# ========== ROS2 發布節點 ==========
class KochMirrorPublisher(Node):
    def __init__(self, zero_ticks):
        super().__init__('koch_real_obstacle_publisher')
        
        self.zero_ticks = zero_ticks
        
        self.declare_parameter('arm_name', 'follower')
        self.declare_parameter('publish_rate', 50.0)
        self.declare_parameter('enabled', True)
        
        self.arm_name = self.get_parameter('arm_name').value
        self.publish_rate = self.get_parameter('publish_rate').value
        self.enabled = self.get_parameter('enabled').value
        
        self.joint_cmd_pub = self.create_publisher(
            JointState,
            f'/{self.arm_name}/joint_states_control',
            10
        )
        
        self.enable_sub = self.create_subscription(
            Bool,
            '/koch_mirror/enable',
            self.enable_callback,
            10
        )
        
        self.joint_names = ['joint1', 'joint2', 'joint3', 'joint4', 'joint5', 'joint_gripper']
        
        self.get_logger().info('=' * 60)
        self.get_logger().info('🎯 Koch 真實障礙物避障控制器已啟動')
        self.get_logger().info(f'📡 發布到: /{self.arm_name}/joint_states_control')
        self.get_logger().info(f'📊 頻率: {self.publish_rate} Hz')
        self.get_logger().info('=' * 60)
    
    def enable_callback(self, msg):
        self.enabled = msg.data
        status = "啟用" if self.enabled else "停用"
        self.get_logger().info(f'🎮 控制器: {status}')
    
    def publish_joint_state(self, virtual_joint_positions):
        """發布關節位置到實體機械臂"""
        if not self.enabled:
            return
        
        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.name = self.joint_names
        msg.position = virtual_joint_positions.tolist()
        
        self.joint_cmd_pub.publish(msg)

# ========== PyBullet 場景設置 ==========
def setup_pybullet(use_gui=True):
    import subprocess
    
    # 檢查 X11
    can_use_gui = False
    if use_gui and os.environ.get('DISPLAY'):
        try:
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
                cid = p.connect(p.DIRECT)
        except:
            cid = p.connect(p.DIRECT)
    else:
        cid = p.connect(p.DIRECT)
    
    if cid < 0:
        raise RuntimeError("PyBullet 連接失敗")
    
    if p.getConnectionInfo()['connectionMethod'] == p.GUI:
        p.configureDebugVisualizer(p.COV_ENABLE_GUI, 1)
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
    q_home = [
        0.0,
        +np.pi/2,
        +np.pi/2,
        +np.pi/2,
        0.0,
        0.0
    ]
    
    print("\n🎯 設定 PyBullet 初始姿勢:")
    for i, q in enumerate(q_home):
        p.resetJointState(robot_id, i, q)
        print(f"   Joint{i+1}: {q:+7.3f} rad ({np.degrees(q):+7.2f}°)")
    
    # 目標球（綠色）
    goal_id = p.createMultiBody(
        baseMass=0,
        baseCollisionShapeIndex=-1,
        baseVisualShapeIndex=p.createVisualShape(
            p.GEOM_SPHERE, 
            radius=BALL_RADIUS*0.8, 
            rgbaColor=[0, 1, 0, 0.7]
        ),
        basePosition=goal
    )
    
    return robot_id, goal_id, q_home

# ========== 真實障礙物管理器 ==========
class RealObstacleManager:
    """管理從 RealSense 檢測到的真實障礙物"""
    
    def __init__(self, use_camera=True):
        self.use_camera = use_camera
        self.obstacles = {}  # color -> pybullet_id
        self.obstacle_positions = {}  # color -> position
        self.obstacle_radii = {}  # color -> radius
        
        self.color_map = {
            'red': [1, 0, 0, 0.8],
            'blue': [0, 0, 1, 0.8],
            'green': [0, 1, 0, 0.8],
            'yellow': [1, 1, 0, 0.8]
        }
        
        if use_camera and REALSENSE_AVAILABLE:
            try:
                print("\n📷 初始化 RealSense 相機...")
                self.camera = RealSenseCamera()
                self.detector = ObstacleDetector()
                
                # 載入相機外參
                calib_file = "/workspace/avoid_real/avoid_real/camera_calibration.json"
                self.transform = CoordinateTransform(calib_file)
                
                print("✅ RealSense 系統已就緒")
                self.camera_ready = True
            except Exception as e:
                print(f"❌ 相機初始化失敗: {e}")
                print("   將使用模擬障礙物")
                self.camera_ready = False
        else:
            self.camera_ready = False
            print("📦 使用模擬障礙物模式")
        
        # 模擬障礙物（無相機時使用）
        if not self.camera_ready:
            self.sim_obstacle_time = 0
            self.create_simulated_obstacles()
    
    def create_simulated_obstacles(self):
        """創建模擬的移動障礙物"""
        # 一個左右移動的紅色障礙物
        pos = [0.25, 0.15, 0.10]
        radius = 0.04
        
        obstacle_id = p.createMultiBody(
            baseMass=0,
            baseCollisionShapeIndex=p.createCollisionShape(p.GEOM_SPHERE, radius=radius),
            baseVisualShapeIndex=p.createVisualShape(
                p.GEOM_SPHERE, 
                radius=radius, 
                rgbaColor=self.color_map['red']
            ),
            basePosition=pos
        )
        
        self.obstacles['red'] = obstacle_id
        self.obstacle_positions['red'] = np.array(pos)
        self.obstacle_radii['red'] = radius
    
    def update_obstacles(self):
        """更新障礙物位置"""
        if self.camera_ready:
            # 從相機檢測障礙物
            return self._update_from_camera()
        else:
            # 更新模擬障礙物
            return self._update_simulated()
    
    def _update_from_camera(self):
        """從相機更新障礙物"""
        try:
            # 獲取影像
            color_image, depth_image = self.camera.get_frames()
            if color_image is None or depth_image is None:
                return False
            
            # 檢測障礙物
            detected_obstacles = self.detector.detect_obstacles(color_image, depth_image)
            
            # 轉換座標並更新 PyBullet
            detected_colors = set()
            for obs in detected_obstacles:
                color = obs.color
                detected_colors.add(color)
                
                # 轉換到世界座標系
                pos_world = self.transform.camera_to_world(obs.position_camera)
                
                # 更新或創建障礙物
                if color in self.obstacles:
                    # 更新位置
                    p.resetBasePositionAndOrientation(
                        self.obstacles[color],
                        pos_world,
                        [0, 0, 0, 1]
                    )
                else:
                    # 創建新障礙物
                    obstacle_id = p.createMultiBody(
                        baseMass=0,
                        baseCollisionShapeIndex=p.createCollisionShape(
                            p.GEOM_SPHERE, 
                            radius=obs.radius
                        ),
                        baseVisualShapeIndex=p.createVisualShape(
                            p.GEOM_SPHERE,
                            radius=obs.radius,
                            rgbaColor=self.color_map.get(color, [0.5, 0.5, 0.5, 0.8])
                        ),
                        basePosition=pos_world
                    )
                    self.obstacles[color] = obstacle_id
                
                self.obstacle_positions[color] = pos_world
                self.obstacle_radii[color] = obs.radius
            
            # 移除不再檢測到的障礙物
            colors_to_remove = [c for c in self.obstacles.keys() if c not in detected_colors]
            for color in colors_to_remove:
                p.removeBody(self.obstacles[color])
                del self.obstacles[color]
                del self.obstacle_positions[color]
                del self.obstacle_radii[color]
            
            # 顯示檢測結果
            vis_image = self.detector.visualize_detections(
                color_image,
                self.detector.detect_colored_objects(color_image)
            )
            cv2.imshow("RealSense - 障礙物檢測", vis_image)
            cv2.waitKey(1)
            
            return True
            
        except Exception as e:
            print(f"⚠️  相機更新錯誤: {e}")
            return False
    
    def _update_simulated(self):
        """更新模擬障礙物"""
        self.sim_obstacle_time += 1.0 / 240.0
        
        # 左右移動
        x = 0.0 + 0.25 * np.sin(2 * np.pi * self.sim_obstacle_time / 8.0)
        pos = [x, 0.15, 0.10]
        
        if 'red' in self.obstacles:
            p.resetBasePositionAndOrientation(
                self.obstacles['red'],
                pos,
                [0, 0, 0, 1]
            )
            self.obstacle_positions['red'] = np.array(pos)
        
        return True
    
    def get_closest_obstacle(self, point):
        """
        找到距離給定點最近的障礙物
        
        Returns:
            (distance, position, radius) 或 (None, None, None)
        """
        if not self.obstacle_positions:
            return None, None, None
        
        min_dist = float('inf')
        closest_pos = None
        closest_radius = SAFE_RADIUS_M
        
        for color, obs_pos in self.obstacle_positions.items():
            dist = np.linalg.norm(point - obs_pos)
            if dist < min_dist:
                min_dist = dist
                closest_pos = obs_pos
                closest_radius = self.obstacle_radii.get(color, SAFE_RADIUS_M)
        
        return min_dist, closest_pos, closest_radius
    
    def cleanup(self):
        """清理資源"""
        if self.camera_ready:
            self.camera.stop()
            cv2.destroyAllWindows()
        
        for obstacle_id in self.obstacles.values():
            p.removeBody(obstacle_id)

# ========== CBF QP Solver ==========
def solve_cbf_qp(grad_h, dist, d_safe, u_ref, alpha):
    """CBF QP 最佳化"""
    if not USE_QP:
        return u_ref
    
    try:
        u = cp.Variable(N_J)
        h = dist - d_safe
        Lgh = grad_h @ u
        
        objective = cp.Minimize(cp.sum_squares(u - u_ref))
        constraints = [Lgh >= -alpha * h]
        prob = cp.Problem(objective, constraints)
        prob.solve(solver=cp.OSQP, verbose=False, warm_start=True)
        
        if prob.status in ["optimal", "optimal_inaccurate"]:
            return np.array(u.value).flatten()
        else:
            return u_ref
    except:
        return u_ref

# ========== 主迴圈 ==========
def main_loop(robot_id, goal_id, q_home, ros_publisher, obstacle_manager, warmup_time=5.0):
    """主控制迴圈"""
    q_current = np.array(q_home, dtype=np.float32)
    
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
        
        for i in range(N_J):
            p.resetJointState(robot_id, i, q_current[i])
        
        current_time = time.time()
        if current_time - last_pub_time >= pub_interval:
            ros_publisher.publish_joint_state(q_current)
            last_pub_time = current_time
        
        remaining = warmup_time - (time.time() - warmup_start)
        if warmup_loops % 50 == 0:
            print(f"   ⏱️  剩餘時間: {remaining:.1f}s")
        
        p.stepSimulation()
        time.sleep(1./240.)
    
    print("✅ 預熱完成! 開始運行\n")
    
    try:
        while rclpy.ok():
            loop_count += 1
            
            # 更新真實障礙物
            obstacle_manager.update_obstacles()
            
            # 更新目標位置
            goal_pos, _ = p.getBasePositionAndOrientation(goal_id)
            goal[:] = goal_pos
            
            # 獲取當前狀態
            ee_state = p.getLinkState(robot_id, EE_LINK, computeLinkVelocity=1)
            ee_pos = np.array(ee_state[0], dtype=np.float32)
            
            # 計算目標速度
            err = goal - ee_pos
            v_des = np.clip(Kp * err, -MAX_EE_VEL, MAX_EE_VEL)
            
            # 獲取關節狀態
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
            
            # 參考控制
            u_ref = np.linalg.pinv(J) @ v_des
            u_ref = np.clip(u_ref, -VEL_CLAMP, VEL_CLAMP)
            
            # CBF 避障
            min_dist = float('inf')
            closest_link = None
            closest_link_pos = None
            closest_J = None
            obs_safe_radius = SAFE_RADIUS_M
            
            for link_id in CRITICAL_LINKS:
                link_state = p.getLinkState(robot_id, link_id)
                link_pos = np.array(link_state[0], dtype=np.float32)
                
                # 找最近的障礙物
                dist_to_obs, obs_pos, obs_radius = obstacle_manager.get_closest_obstacle(link_pos)
                
                if dist_to_obs is not None and dist_to_obs < min_dist:
                    min_dist = dist_to_obs
                    closest_link = link_id
                    closest_link_pos = link_pos
                    obs_safe_radius = obs_radius + 0.05  # 加上安全邊距
                    
                    J_link_full, _ = p.calculateJacobian(
                        robot_id, link_id,
                        localPosition=[0, 0, 0],
                        objPositions=q_current.tolist(),
                        objVelocities=[0]*N_J,
                        objAccelerations=[0]*N_J
                    )
                    closest_J = np.array(J_link_full, dtype=np.float32)[:3, :N_J]
            
            # 應用 CBF
            if min_dist < 2.5 * obs_safe_radius and closest_J is not None:
                _, obs_pos, _ = obstacle_manager.get_closest_obstacle(closest_link_pos)
                direction = (closest_link_pos - obs_pos) / (min_dist + 1e-6)
                grad_h = direction @ closest_J
                
                alpha = adaptive_cbf_gain(min_dist, obs_safe_radius)
                u_safe = solve_cbf_qp(grad_h, min_dist, obs_safe_radius, u_ref, alpha)
            else:
                u_safe = u_ref
            
            u_safe = np.clip(u_safe, -VEL_CLAMP, VEL_CLAMP)
            
            # 更新關節
            q_new = q_current + STEP_GAIN * u_safe
            for i in range(N_J):
                p.setJointMotorControl2(
                    robot_id, i,
                    p.POSITION_CONTROL,
                    targetPosition=q_new[i],
                    force=500
                )
            
            q_current = q_new
            
            # ROS2 發布
            current_time = time.time()
            if current_time - last_pub_time >= pub_interval:
                ros_publisher.publish_joint_state(q_current)
                last_pub_time = current_time
            
            # 顯示資訊
            if loop_count % 100 == 0:
                cbf_active = min_dist < 2.0 * obs_safe_radius
                cbf_status = "🛡️ CBF" if cbf_active else "✅ Safe"
                num_obstacles = len(obstacle_manager.obstacles)
                print(f"[{loop_count:6d}] 障礙物數: {num_obstacles} | "
                      f"最近距離: {min_dist:.3f}m | "
                      f"Link: {closest_link} | "
                      f"{cbf_status}")
            
            p.stepSimulation()
            time.sleep(1./240.)
            
    except KeyboardInterrupt:
        print("\n⏹️  使用者中斷")
    finally:
        obstacle_manager.cleanup()

# ========== 主程式 ==========
def main():
    print("=" * 60)
    print("🚀 Koch 真實障礙物避障控制器啟動中...")
    print("=" * 60)
    
    # 載入校正檔
    zero_ticks = load_calibration()
    
    # 檢查參數
    use_gui = not ('--no-gui' in sys.argv or '--headless' in sys.argv)
    use_camera = not '--no-camera' in sys.argv
    
    # 初始化 PyBullet
    print("\n🎮 初始化 PyBullet...")
    robot_id, goal_id, q_home = setup_pybullet(use_gui)
    print("✅ PyBullet 場景已建立")
    
    # 初始化障礙物管理器
    print("\n🎯 初始化障礙物檢測系統...")
    obstacle_manager = RealObstacleManager(use_camera=use_camera)
    
    # 初始化 ROS2
    print("\n📡 初始化 ROS2...")
    rclpy.init()
    ros_publisher = KochMirrorPublisher(zero_ticks)
    
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
    if use_camera and obstacle_manager.camera_ready:
        print("   - 📷 使用 RealSense 相機檢測真實障礙物")
        print("   - 🔴 將彩色球體放在相機視野內")
        print("   - 支援顏色: 紅、藍、綠、黃")
    else:
        print("   - 📦 使用模擬障礙物（紅色球左右移動）")
    print("   - 🟢 可在 PyBullet 中拖動綠色球（目標）")
    print("   - 🤖 實體手臂會即時同步")
    print("   - 🛡️ CBF 自動避障")
    print("   - ⌨️  按 Ctrl+C 停止")
    print("=" * 60)
    
    try:
        main_loop(robot_id, goal_id, q_home, ros_publisher, obstacle_manager, warmup_time=5.0)
    finally:
        print("\n🛑 清理資源...")
        obstacle_manager.cleanup()
        ros_publisher.destroy_node()
        rclpy.shutdown()
        p.disconnect()
        print("✅ 已安全退出")

if __name__ == "__main__":
    main()
