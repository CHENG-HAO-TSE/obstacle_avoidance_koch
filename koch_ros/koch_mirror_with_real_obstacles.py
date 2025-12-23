#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Koch PyBullet Mirror Controller - 整合真實障礙物檢測
===============================================
功能：
1. 使用 RealSense 相機檢測真實世界的彩色障礙物
2. 將檢測到的障礙物即時同步到 PyBullet 模擬環境進行可視化
3. 使用 CBF 演算法對真實障礙物進行避障
4. 支援零點校正檔案
5. 即時同步虛擬手臂和實體手臂

使用方法：
  # 有 RealSense 相機（推薦）
  python3 koch_mirror_with_real_obstacles.py
  
  # 無相機（使用模擬移動障礙物）
  python3 koch_mirror_with_real_obstacles.py --no-camera
  
  # 無 GUI 模式
  python3 koch_mirror_with_real_obstacles.py --no-gui

依賴：
  pip install pyrealsense2 opencv-python cvxpy

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
MAX_EE_VEL = 0.15  # 最大末端速度（提高到 0.15）
Kp = 0.8  # 大幅提高增益加快反應
STEP_GAIN = 0.035  # 大幅提高步進增益
VEL_CLAMP = 1.2  # 大幅提高速度限制
EE_LINK = 5
N_J = 6

# 自適應速度平滑參數
VELOCITY_SMOOTHING_SAFE = 0.0   # 安全時完全無平滑（最快反應）
VELOCITY_SMOOTHING_DANGER = 0.25  # 接近障礙物時的平滑（稍微降低以提高反應）

# 目標位置
goal = np.array([0.00, 0.15, 0.10], dtype=np.float32)

CBF_GAIN_MIN, CBF_GAIN_MAX = 3.0, 8.0  # 降低 CBF 增益避免過度反應

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

# ========== 真實障礙物管理器 ==========
class RealObstacleManager:
    """管理 RealSense 相機檢測到的真實障礙物"""
    
    def __init__(self, use_camera=True, show_camera_view=True):
        self.use_camera = use_camera
        self.show_camera_view = show_camera_view  # 是否顯示相機視窗
        self.camera = None
        self.detector = None
        self.transform = None
        self.obstacles_pybullet_ids = {}  # {color: pybullet_id}
        self.latest_color_image = None  # 保存最新的彩色影像
        self.latest_detected_obstacles = []  # 保存最新檢測到的障礙物
        
        # 顏色對應的 RGBA (PyBullet)
        self.color_rgba = {
            'red': [1, 0, 0, 0.8],
            'blue': [0, 0, 1, 0.8],
            'green': [0, 1, 0, 0.8],
            'yellow': [1, 1, 0, 0.8]
        }
        
        # 顏色對應的 BGR (OpenCV)
        self.color_bgr = {
            'red': (0, 0, 255),
            'blue': (255, 0, 0),
            'green': (0, 255, 0),
            'yellow': (0, 255, 255)
        }
        
        if use_camera and REALSENSE_AVAILABLE:
            try:
                print("🔧 初始化 RealSense 相機...")
                self.camera = RealSenseCamera()
                print(f"✅ 相機已連接: {self.camera.device_name}")
                
                print("🔍 初始化障礙物檢測器...")
                self.detector = ObstacleDetector()
                
                print("📐 初始化座標轉換...")
                calib_file = "/workspace/avoid_real/avoid_real/camera_calibration.json"
                self.transform = CoordinateTransform(calib_file)
                
                print("✅ 真實障礙物檢測系統已就緒")
                
                # 創建 OpenCV 視窗
                if self.show_camera_view:
                    cv2.namedWindow("RealSense Camera View", cv2.WINDOW_NORMAL)
                    cv2.resizeWindow("RealSense Camera View", 640, 480)
                
            except Exception as e:
                print(f"❌ 相機初始化失敗: {e}")
                print("   將使用模擬障礙物")
                self.use_camera = False
                self.show_camera_view = False
        else:
            self.use_camera = False
            self.show_camera_view = False
            
        if not self.use_camera:
            print("📦 使用模擬移動障礙物模式")
            # 模擬障礙物參數
            self.sim_obstacle_amplitude = 0.25
            self.sim_obstacle_period = 8.0
            self.sim_obstacle_start_time = time.time()
    
    def create_obstacle_in_pybullet(self, color, position, radius=0.02):
        """在 PyBullet 中創建障礙物球體"""
        rgba = self.color_rgba.get(color, [0.5, 0.5, 0.5, 0.8])
        
        obstacle_id = p.createMultiBody(
            baseMass=0,
            baseCollisionShapeIndex=p.createCollisionShape(p.GEOM_SPHERE, radius=radius),
            baseVisualShapeIndex=p.createVisualShape(p.GEOM_SPHERE, radius=radius, rgbaColor=rgba),
            basePosition=position
        )
        
        return obstacle_id
    
    def draw_obstacle_on_image(self, image, obs):
        """在影像上繪製障礙物標記"""
        u, v = int(obs.position_2d[0]), int(obs.position_2d[1])
        radius_pixel = int(obs.radius * 100)
        
        color_bgr = self.color_bgr.get(obs.color, (128, 128, 128))
        
        # 繪製圓圈
        cv2.circle(image, (u, v), radius_pixel, color_bgr, 2)
        cv2.circle(image, (u, v), 3, color_bgr, -1)
        
        # 繪製十字
        cross_size = 10
        cv2.line(image, (u - cross_size, v), (u + cross_size, v), color_bgr, 1)
        cv2.line(image, (u, v - cross_size), (u, v + cross_size), color_bgr, 1)
        
        # 顯示座標
        x_cam, y_cam, z_cam = obs.position_camera
        text = f"{obs.color.upper()} Z:{z_cam:.2f}m"
        cv2.putText(image, text, (u + 10, v - 10),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.4, color_bgr, 1)
    
    def update_camera_view(self):
        """更新相機視窗顯示"""
        if not self.show_camera_view or self.latest_color_image is None:
            return
        
        vis_image = self.latest_color_image.copy()
        
        # 繪製所有檢測到的障礙物
        for obs in self.latest_detected_obstacles:
            self.draw_obstacle_on_image(vis_image, obs)
        
        # 顯示資訊
        info_text = f"Obstacles: {len(self.latest_detected_obstacles)}"
        cv2.putText(vis_image, info_text, (10, 30),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 255), 2)
        cv2.putText(vis_image, "Press 'q' in terminal to quit", (10, 460),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.5, (200, 200, 200), 1)
        
        cv2.imshow("RealSense Camera View", vis_image)
        cv2.waitKey(1)  # 非阻塞更新
    
    def update_obstacles(self, force_update=False):
        """
        更新障礙物位置
        
        Args:
            force_update: 強制更新（忽略幀率限制）
        
        Returns:
            list: 障礙物位置列表 [(x, y, z, radius, color), ...]
        """
        obstacles = []
        
        if self.use_camera:
            # 從相機獲取影像（相機本身是 30Hz，但這裡每次都嘗試獲取最新幀）
            color_image, depth_image = self.camera.get_frames()
            if color_image is not None and depth_image is not None:
                # 保存影像供視覺化使用
                self.latest_color_image = color_image.copy()
                
                # 檢測障礙物
                detected_obstacles = self.detector.detect_obstacles(color_image, depth_image, self.camera)
                self.latest_detected_obstacles = detected_obstacles
                
                # 轉換到世界座標系
                for obs in detected_obstacles:
                    world_pos = self.transform.camera_to_world(obs.position_camera)
                    obstacles.append({
                        'position': world_pos,
                        'radius': obs.radius,
                        'color': obs.color
                    })
                    
                    # 在 PyBullet 中更新或創建障礙物
                    if obs.color not in self.obstacles_pybullet_ids:
                        # 創建新障礙物
                        obs_id = self.create_obstacle_in_pybullet(obs.color, world_pos, obs.radius)
                        self.obstacles_pybullet_ids[obs.color] = obs_id
                    else:
                        # 立即更新現有障礙物位置（適合動態障礙物）
                        p.resetBasePositionAndOrientation(
                            self.obstacles_pybullet_ids[obs.color],
                            world_pos,
                            [0, 0, 0, 1]
                        )
                
                # 移除不再檢測到的障礙物
                detected_colors = {obs.color for obs in detected_obstacles}
                colors_to_remove = []
                for color, obs_id in self.obstacles_pybullet_ids.items():
                    if color not in detected_colors:
                        p.removeBody(obs_id)
                        colors_to_remove.append(color)
                
                for color in colors_to_remove:
                    del self.obstacles_pybullet_ids[color]
                
                # 更新相機視窗
                self.update_camera_view()
        
        else:
            # 模擬移動障礙物
            current_sim_time = time.time() - self.sim_obstacle_start_time
            obstacle_x = 0.0 + self.sim_obstacle_amplitude * np.sin(2 * np.pi * current_sim_time / self.sim_obstacle_period)
            obstacle_pos = np.array([obstacle_x, 0.15, 0.10])
            
            obstacles.append({
                'position': obstacle_pos,
                'radius': 0.015,
                'color': 'red'
            })
            
            # 更新或創建模擬障礙物
            if 'sim_red' not in self.obstacles_pybullet_ids:
                obs_id = self.create_obstacle_in_pybullet('red', obstacle_pos, 0.015)
                self.obstacles_pybullet_ids['sim_red'] = obs_id
            else:
                p.resetBasePositionAndOrientation(
                    self.obstacles_pybullet_ids['sim_red'],
                    obstacle_pos,
                    [0, 0, 0, 1]
                )
        
        return obstacles
    
    def cleanup(self):
        """清理資源"""
        if self.camera is not None:
            self.camera.stop()
        
        # 關閉 OpenCV 視窗
        if self.show_camera_view:
            cv2.destroyAllWindows()
        
        # 移除所有 PyBullet 障礙物
        for obs_id in self.obstacles_pybullet_ids.values():
            try:
                p.removeBody(obs_id)
            except:
                pass
        self.obstacles_pybullet_ids.clear()

# ========== ROS2 發布節點 ==========
class KochMirrorPublisher(Node):
    def __init__(self, zero_ticks):
        super().__init__('koch_mirror_publisher_real_obs')
        
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
        self.get_logger().info('🪞 Koch Mirror Publisher (真實障礙物版) 已啟動')
        self.get_logger().info(f'📡 發布到: /{self.arm_name}/joint_states_control')
        self.get_logger().info(f'📊 頻率: {self.publish_rate} Hz')
        self.get_logger().info(f'🎮 狀態: {"啟用" if self.enabled else "停用"}')
        self.get_logger().info('=' * 60)
    
    def enable_callback(self, msg):
        self.enabled = msg.data
        status = "啟用" if self.enabled else "停用"
        self.get_logger().info(f'🎮 鏡像同步: {status}')
    
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
    import os
    import subprocess
    
    # 檢查 X11 是否可用
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
    q_home = [
        0.0,                # joint1: 0° (waist 不轉)
        +np.pi/2,           # joint2: +90° (shoulder向上)
        +np.pi/2,           # joint3: +90° (elbow向下)
        +np.pi/2,           # joint4: +90° (wrist向下彎)
        0.0,                # joint5: 0° (保持中立)
        0.0                 # joint6: 0° (gripper中立)
    ]
    
    print("\n🎯 設定 PyBullet 初始姿勢:")
    for i, q in enumerate(q_home):
        p.resetJointState(robot_id, i, q)
        print(f"   Joint{i+1}: {q:+7.3f} rad ({np.degrees(q):+7.2f}°)")
    
    # 目標球（綠色）- 可拖動
    goal_id = p.createMultiBody(
        baseMass=0,
        baseCollisionShapeIndex=-1,
        baseVisualShapeIndex=p.createVisualShape(p.GEOM_SPHERE, radius=0.012, rgbaColor=[0, 1, 0, 0.7]),
        basePosition=goal
    )
    
    return robot_id, goal_id, q_home

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
    """PyBullet 主迴圈 + ROS2 發布（整合真實障礙物）"""
    q_current = np.array(q_home, dtype=np.float32)
    u_safe_prev = np.zeros(N_J, dtype=np.float32)  # 上一次的控制指令（用於平滑）
    
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
        
        # 預熱期間保持初始姿勢
        for i in range(N_J):
            p.resetJointState(robot_id, i, q_current[i])
        
        # 持續發布初始姿勢
        current_time = time.time()
        if current_time - last_pub_time >= pub_interval:
            ros_publisher.publish_joint_state(q_current)
            last_pub_time = current_time
        
        # 更新障礙物（即使在預熱階段也要顯示）
        obstacle_manager.update_obstacles()
        
        # 顯示倒數
        remaining = warmup_time - (time.time() - warmup_start)
        if warmup_loops % 50 == 0:
            print(f"   ⏱️  剩餘時間: {remaining:.1f}s")
        
        p.stepSimulation()
        time.sleep(1./240.)
    
    print("✅ 預熱完成! 開始避障控制\n")
    
    try:
        while rclpy.ok():
            loop_count += 1
            
            # 更新真實障礙物位置
            detected_obstacles = obstacle_manager.update_obstacles()
            
            # 更新目標位置（可從 PyBullet 拖動綠球）
            goal_pos, _ = p.getBasePositionAndOrientation(goal_id)
            goal[:] = goal_pos
            
            # 獲取末端執行器狀態
            ee_state = p.getLinkState(robot_id, EE_LINK, computeLinkVelocity=1)
            ee_pos = np.array(ee_state[0], dtype=np.float32)
            
            # 計算目標速度
            err = goal - ee_pos
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
            
            # 參考控制指令
            u_ref = np.linalg.pinv(J) @ v_des
            u_ref = np.clip(u_ref, -VEL_CLAMP, VEL_CLAMP)
            
            # CBF 避障：檢查所有關鍵連桿與所有障礙物的距離
            min_dist = float('inf')
            closest_link = None
            closest_link_pos = None
            closest_J = None
            closest_obs_pos = None
            closest_obs_color = None
            
            for obs_data in detected_obstacles:
                obs_pos = np.array(obs_data['position'], dtype=np.float32)
                obs_radius = obs_data['radius']
                
                for link_id in CRITICAL_LINKS:
                    link_state = p.getLinkState(robot_id, link_id)
                    link_pos = np.array(link_state[0], dtype=np.float32)
                    
                    # 計算距離（考慮障礙物半徑）
                    dist_to_obs = np.linalg.norm(link_pos - obs_pos) - obs_radius
                    
                    if dist_to_obs < min_dist:
                        min_dist = dist_to_obs
                        closest_link = link_id
                        closest_link_pos = link_pos
                        closest_obs_pos = obs_pos
                        closest_obs_color = obs_data['color']
                        
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
                # 計算遠離障礙物的方向
                direction = (closest_link_pos - closest_obs_pos) / (np.linalg.norm(closest_link_pos - closest_obs_pos) + 1e-6)
                
                # CBF 梯度
                grad_h = direction @ closest_J
                
                alpha = adaptive_cbf_gain(min_dist, SAFE_RADIUS_M)
                u_safe = solve_cbf_qp(grad_h, min_dist, SAFE_RADIUS_M, u_ref, alpha)
            else:
                u_safe = u_ref
            
            # 自適應速度平滑（接近障礙物時才平滑，遠離時快速反應）
            if min_dist < 2.0 * SAFE_RADIUS_M:
                # 接近障礙物：使用較大的平滑係數（減少抖動）
                smoothing = VELOCITY_SMOOTHING_DANGER
            else:
                # 安全距離：使用較小的平滑係數（快速反應）
                smoothing = VELOCITY_SMOOTHING_SAFE
            
            u_safe = smoothing * u_safe_prev + (1 - smoothing) * u_safe
            u_safe_prev = u_safe.copy()
            
            u_safe = np.clip(u_safe, -VEL_CLAMP, VEL_CLAMP)
            
            # 更新關節
            q_new = q_current + STEP_GAIN * u_safe
            for i in range(N_J):
                p.setJointMotorControl2(
                    robot_id, i,
                    p.POSITION_CONTROL,
                    targetPosition=q_new[i],
                    force=500,  # 提高力度以加快反應
                    maxVelocity=1.5  # 大幅提高最大速度限制
                )
            
            q_current = q_new
            
            # ROS2 發布
            current_time = time.time()
            if current_time - last_pub_time >= pub_interval:
                ros_publisher.publish_joint_state(q_current)
                last_pub_time = current_time
            
            # 顯示資訊
            if loop_count % 100 == 0:
                cbf_active = min_dist < 2.0 * SAFE_RADIUS_M
                cbf_status = "🛡️ CBF" if cbf_active else "✅ Safe"
                obs_info = f"{closest_obs_color}({closest_obs_pos[0]:.2f},{closest_obs_pos[1]:.2f},{closest_obs_pos[2]:.2f})" if closest_obs_pos is not None else "None"
                print(f"[{loop_count:6d}] EE: ({ee_pos[0]:.3f},{ee_pos[1]:.3f},{ee_pos[2]:.3f}) | "
                      f"Goal: ({goal[0]:.3f},{goal[1]:.3f},{goal[2]:.3f}) | "
                      f"Obstacles: {len(detected_obstacles)} | "
                      f"Closest: {obs_info} | Dist: {min_dist:.3f}m Link:{closest_link} {cbf_status}")
            
            p.stepSimulation()
            time.sleep(1./240.)
            
    except KeyboardInterrupt:
        print("\n⏹️  使用者中斷")

# ========== 主程式 ==========
def main():
    import sys
    
    print("=" * 70)
    print("🚀 Koch PyBullet Mirror - 真實障礙物避障版")
    print("=" * 70)
    
    # 載入零點校正檔
    zero_ticks = load_calibration()
    
    # 檢查命令列參數
    use_gui = not ('--no-gui' in sys.argv or '--headless' in sys.argv)
    use_camera = not '--no-camera' in sys.argv
    show_camera_view = '--show-camera' in sys.argv or use_camera  # 預設有相機就顯示
    
    # 初始化 PyBullet
    print("\n🎮 初始化 PyBullet...")
    robot_id, goal_id, q_home = setup_pybullet(use_gui)
    print("✅ PyBullet 場景已建立")
    
    # 初始化障礙物管理器
    print("\n📷 初始化障礙物檢測系統...")
    obstacle_manager = RealObstacleManager(use_camera=use_camera, show_camera_view=show_camera_view)
    
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
    print("=" * 70)
    print("💡 操作說明:")
    print("   - 系統會先進行 5 秒預熱，讓實體手臂移動到初始姿勢")
    if use_camera:
        print("   - 將彩色球體（紅/藍/綠/黃）放在相機前")
        print("   - 檢測到的障礙物會即時顯示在 PyBullet 中")
        if show_camera_view:
            print("   - RealSense 相機視窗會顯示檢測畫面")
    else:
        print("   - 使用模擬移動障礙物（紅色球）")
    print("   - 手臂會使用 CBF 自動避開障礙物")
    print("   - 可以在 PyBullet 視窗中拖動綠色球（目標）")
    print("   - 實體手臂會即時同步虛擬手臂的動作")
    print("   - 按 Ctrl+C 停止")
    print("=" * 70)
    
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
