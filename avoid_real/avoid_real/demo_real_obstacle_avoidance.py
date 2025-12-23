"""
真實世界障礙物避障整合示例
結合 RealSense 相機、障礙物檢測、座標轉換和 PyBullet 同步
"""

import sys
import os
import numpy as np
import cv2
import pybullet as p
import time

# 導入自訂模組
from realsense_camera import RealSenseCamera
from obstacle_detector import ObstacleDetector
from coordinate_transform import CoordinateTransform
from pybullet_sync import PyBulletObstacleSync, RealTimeObstacleSynchronizer


def setup_pybullet_scene(use_gui: bool = True):
    """
    設置 PyBullet 模擬場景
    
    Args:
        use_gui: 是否使用 GUI
        
    Returns:
        physics_client: PyBullet 客戶端 ID
    """
    # 連接 PyBullet
    if use_gui:
        physics_client = p.connect(p.GUI)
        p.configureDebugVisualizer(p.COV_ENABLE_GUI, 1)
        p.configureDebugVisualizer(p.COV_ENABLE_SHADOWS, 1)
    else:
        physics_client = p.connect(p.DIRECT)
    
    # 設置重力
    p.setGravity(0, 0, -9.8)
    
    # 創建地面
    p.createMultiBody(
        baseMass=0,
        baseCollisionShapeIndex=p.createCollisionShape(p.GEOM_PLANE),
        baseVisualShapeIndex=p.createVisualShape(
            p.GEOM_PLANE,
            rgbaColor=[0.8, 0.8, 0.8, 1]
        ),
        basePosition=[0, 0, 0]
    )
    
    # 添加座標軸指示器
    p.addUserDebugLine([0, 0, 0], [0.5, 0, 0], [1, 0, 0], lineWidth=3)  # X軸 紅色
    p.addUserDebugLine([0, 0, 0], [0, 0.5, 0], [0, 1, 0], lineWidth=3)  # Y軸 綠色
    p.addUserDebugLine([0, 0, 0], [0, 0, 0.5], [0, 0, 1], lineWidth=3)  # Z軸 藍色
    
    # 設置相機視角
    p.resetDebugVisualizerCamera(
        cameraDistance=1.5,
        cameraYaw=45,
        cameraPitch=-30,
        cameraTargetPosition=[0, 0, 0.3]
    )
    
    print("✅ PyBullet 場景已設置")
    return physics_client


def load_robot_model(physics_client: int = None):
    """
    載入機器人模型（可選）
    
    Args:
        physics_client: PyBullet 客戶端 ID
        
    Returns:
        robot_id: 機器人 ID，如果沒有模型則返回 None
    """
    # 這裡可以載入你的機器人 URDF
    # 例如: robot_id = p.loadURDF("path/to/robot.urdf")
    
    # 暫時創建一個簡單的機器人底座作為參考
    robot_id = p.createMultiBody(
        baseMass=1,
        baseCollisionShapeIndex=p.createCollisionShape(p.GEOM_CYLINDER, radius=0.1, height=0.1),
        baseVisualShapeIndex=p.createVisualShape(
            p.GEOM_CYLINDER,
            radius=0.1,
            length=0.1,
            rgbaColor=[0.3, 0.3, 0.3, 1]
        ),
        basePosition=[0, 0, 0.05]
    )
    
    print("✅ 機器人模型已載入")
    return robot_id


def main():
    """主程式"""
    print("=" * 60)
    print("🚀 真實世界障礙物避障系統啟動")
    print("=" * 60)
    
    # 1. 初始化 RealSense 相機
    print("\n📷 初始化 RealSense 相機...")
    try:
        camera = RealSenseCamera()
    except Exception as e:
        print(f"❌ 相機初始化失敗: {e}")
        print("⚠️  將以離線模式運行（僅 PyBullet 模擬）")
        camera = None
    
    # 2. 初始化障礙物檢測器
    print("\n🔍 初始化障礙物檢測器...")
    detector = ObstacleDetector()
    
    # 3. 初始化座標轉換
    print("\n📐 初始化座標轉換...")
    calibration_file = "/workspace/koch_ros/avoid_real/camera_calibration.json"
    transform = CoordinateTransform(calibration_file)
    
    # 4. 設置 PyBullet 場景
    print("\n🎮 設置 PyBullet 場景...")
    physics_client = setup_pybullet_scene(use_gui=True)
    robot_id = load_robot_model(physics_client)
    
    # 5. 初始化 PyBullet 同步器
    print("\n🔄 初始化 PyBullet 同步器...")
    pybullet_sync = PyBulletObstacleSync(physics_client)
    
    # 6. 創建即時同步器（如果有相機）
    if camera is not None:
        print("\n⚡ 啟動即時同步...")
        synchronizer = RealTimeObstacleSynchronizer(
            camera=camera,
            detector=detector,
            coordinate_transform=transform,
            pybullet_sync=pybullet_sync,
            update_rate=10.0  # 10 Hz
        )
        
        # 主循環
        print("\n" + "=" * 60)
        print("🎯 系統就緒！")
        print("   - 將彩色球體放置在相機視野內")
        print("   - 障礙物將即時同步到 PyBullet")
        print("   - 按 Ctrl+C 停止")
        print("=" * 60 + "\n")
        
        try:
            frame_count = 0
            start_time = time.time()
            
            while True:
                # 更新障礙物檢測和同步
                obstacles = synchronizer.update()
                
                if obstacles is not None:
                    frame_count += 1
                    
                    # 顯示檢測結果
                    if len(obstacles) > 0:
                        print(f"\n📊 檢測統計 (幀 {frame_count}):")
                        for obs in obstacles:
                            print(f"   {obs.color.upper():8s} | "
                                  f"相機座標: {obs.position_camera} | "
                                  f"世界座標: {obs.position_world} | "
                                  f"半徑: {obs.radius:.3f}m")
                    
                    # 獲取相機影像並顯示檢測結果
                    color_image, depth_image = camera.get_frames()
                    if color_image is not None:
                        # 繪製檢測結果
                        vis_image = detector.visualize_detections(
                            color_image, 
                            detector.detect_colored_objects(color_image)
                        )
                        
                        # 顯示影像
                        cv2.imshow("RealSense - Obstacle Detection", vis_image)
                        
                        # 按 'q' 退出
                        if cv2.waitKey(1) & 0xFF == ord('q'):
                            print("\n👋 使用者按 'q' 退出")
                            break
                
                # PyBullet 模擬步進
                p.stepSimulation()
                time.sleep(0.001)
                
        except KeyboardInterrupt:
            print("\n⏸️  接收到停止信號")
        
        finally:
            # 計算統計資訊
            elapsed_time = time.time() - start_time
            avg_fps = frame_count / elapsed_time if elapsed_time > 0 else 0
            
            print("\n" + "=" * 60)
            print("📈 運行統計:")
            print(f"   總運行時間: {elapsed_time:.2f} 秒")
            print(f"   處理幀數: {frame_count}")
            print(f"   平均 FPS: {avg_fps:.2f}")
            print("=" * 60)
            
            # 清理資源
            print("\n🧹 清理資源...")
            synchronizer.stop()
            pybullet_sync.clear_all_obstacles()
            camera.stop()
            cv2.destroyAllWindows()
    
    else:
        # 離線模式：手動創建測試障礙物
        print("\n🧪 離線測試模式")
        print("   手動創建模擬障礙物...")
        
        # 創建測試障礙物
        test_obstacles = [
            {'position': [0.3, 0.2, 0.15], 'radius': 0.05, 'color': 'red'},
            {'position': [-0.2, 0.3, 0.12], 'radius': 0.04, 'color': 'blue'},
            {'position': [0.0, -0.3, 0.18], 'radius': 0.06, 'color': 'green'},
        ]
        
        for obs_data in test_obstacles:
            pybullet_sync.create_obstacle(
                position=obs_data['position'],
                radius=obs_data['radius'],
                color=obs_data['color']
            )
        
        print("\n✅ 已創建 3 個測試障礙物")
        print("   觀察 PyBullet 視窗中的障礙物")
        print("   按 Ctrl+C 停止...")
        
        try:
            while True:
                p.stepSimulation()
                time.sleep(0.01)
        except KeyboardInterrupt:
            print("\n⏸️  接收到停止信號")
    
    # 最終清理
    print("\n🏁 關閉 PyBullet...")
    p.disconnect()
    print("✅ 程式已結束\n")


if __name__ == "__main__":
    main()
