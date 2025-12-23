#!/usr/bin/env python3
"""
視覺化 RealSense 相機檢測的 3D 位置
即時顯示檢測到的障礙物在相機座標系中的 XYZ 位置
"""

import numpy as np
import cv2
import sys
import time

try:
    from realsense_camera import RealSenseCamera
    from obstacle_detector import ObstacleDetector
except ImportError:
    print("❌ 無法導入模組，請確保在正確目錄執行")
    sys.exit(1)


def draw_3d_axes(image, position_2d, scale=50):
    """
    在影像上繪製 3D 座標軸指示器
    
    Args:
        image: 影像
        position_2d: 物體的 2D 位置 (u, v)
        scale: 座標軸長度（像素）
    """
    u, v = int(position_2d[0]), int(position_2d[1])
    
    # X 軸（紅色，向右）
    cv2.arrowedLine(image, (u, v), (u + scale, v), (0, 0, 255), 2, tipLength=0.3)
    cv2.putText(image, 'X', (u + scale + 5, v), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 255), 2)
    
    # Y 軸（綠色，向下）
    cv2.arrowedLine(image, (u, v), (u, v + scale), (0, 255, 0), 2, tipLength=0.3)
    cv2.putText(image, 'Y', (u, v + scale + 15), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)
    
    # Z 軸（藍色，向前/深度方向用文字表示）
    cv2.putText(image, 'Z→', (u - 30, v - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 0, 0), 2)


def draw_info_panel(image, obstacles, fps):
    """
    繪製資訊面板
    
    Args:
        image: 影像
        obstacles: 檢測到的障礙物列表
        fps: 當前 FPS
    """
    h, w = image.shape[:2]
    panel_height = 150 + len(obstacles) * 60
    
    # 半透明背景
    overlay = image.copy()
    cv2.rectangle(overlay, (10, 10), (w - 10, panel_height), (0, 0, 0), -1)
    cv2.addWeighted(overlay, 0.6, image, 0.4, 0, image)
    
    # 標題
    y = 35
    cv2.putText(image, f"RealSense 3D Detection | FPS: {fps:.1f}", 
                (20, y), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 255), 2)
    
    # 座標系說明
    y += 35
    cv2.putText(image, "Camera Frame: X=Right, Y=Down, Z=Forward (depth)", 
                (20, y), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (200, 200, 200), 1)
    
    # 障礙物資訊
    y += 30
    if len(obstacles) == 0:
        cv2.putText(image, "No obstacles detected", 
                    (20, y), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (100, 100, 100), 1)
    else:
        for i, obs in enumerate(obstacles):
            y += 30
            
            # 顏色指示器
            color_bgr = {
                'red': (0, 0, 255),
                'blue': (255, 0, 0),
                'green': (0, 255, 0),
                'yellow': (0, 255, 255)
            }.get(obs.color, (128, 128, 128))
            
            cv2.circle(image, (30, y - 5), 8, color_bgr, -1)
            
            # 文字資訊
            x_cam, y_cam, z_cam = obs.position_camera
            text = f"{obs.color.upper():8s} | X:{x_cam:+6.3f}m  Y:{y_cam:+6.3f}m  Z:{z_cam:+6.3f}m  R:{obs.radius:.3f}m"
            cv2.putText(image, text, (50, y), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1)
    
    # 操作說明
    y += 50
    cv2.putText(image, "Press 'q' to quit | 's' to save screenshot", 
                (20, y), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (150, 150, 150), 1)


def main():
    print("=" * 70)
    print("📷 RealSense 3D 位置視覺化工具")
    print("=" * 70)
    print()
    
    # 初始化相機
    print("🔧 初始化 RealSense 相機...")
    try:
        camera = RealSenseCamera()
        print(f"✅ 相機已連接: {camera.device_name}")
    except Exception as e:
        print(f"❌ 相機初始化失敗: {e}")
        print("\n請確保:")
        print("  1. RealSense 相機已連接")
        print("  2. 執行 'realsense-viewer' 測試相機")
        print("  3. USB 權限正確")
        return
    
    # 初始化檢測器
    print("🔍 初始化障礙物檢測器...")
    detector = ObstacleDetector()
    print("✅ 檢測器已就緒")
    
    print()
    print("=" * 70)
    print("💡 使用說明:")
    print("  - 將彩色球體（紅/藍/綠/黃）放在相機前")
    print("  - 系統會即時顯示物體在相機座標系的 XYZ 位置")
    print("  - 座標系: X=右, Y=下, Z=前（深度）")
    print("  - 按 'q' 退出，按 's' 截圖")
    print("=" * 70)
    print()
    
    # 創建視窗
    window_name = "RealSense 3D Detection"
    cv2.namedWindow(window_name, cv2.WINDOW_NORMAL)
    cv2.resizeWindow(window_name, 1280, 720)
    
    # FPS 計算
    fps = 0
    frame_count = 0
    start_time = time.time()
    last_fps_update = start_time
    
    screenshot_count = 0
    
    try:
        print("▶️  開始檢測...\n")
        
        while True:
            # 獲取影像
            color_image, depth_image = camera.get_frames()
            if color_image is None or depth_image is None:
                print("⚠️  無法獲取影像")
                time.sleep(0.1)
                continue
            
            # 檢測障礙物
            obstacles = detector.detect_obstacles(color_image, depth_image, camera)
            
            # 繪製檢測結果
            vis_image = color_image.copy()
            
            for obs in obstacles:
                # 繪製邊界框
                u, v = int(obs.position_2d[0]), int(obs.position_2d[1])
                radius_pixel = int(obs.radius * 100)  # 簡單估計像素半徑
                
                color_bgr = {
                    'red': (0, 0, 255),
                    'blue': (255, 0, 0),
                    'green': (0, 255, 0),
                    'yellow': (0, 255, 255)
                }.get(obs.color, (128, 128, 128))
                
                # 繪製圓圈
                cv2.circle(vis_image, (u, v), radius_pixel, color_bgr, 3)
                cv2.circle(vis_image, (u, v), 5, color_bgr, -1)
                
                # 繪製 3D 座標軸
                draw_3d_axes(vis_image, obs.position_2d, scale=40)
                
                # 顯示 3D 座標
                x_cam, y_cam, z_cam = obs.position_camera
                text_lines = [
                    f"{obs.color.upper()}",
                    f"X: {x_cam:+.3f}m",
                    f"Y: {y_cam:+.3f}m",
                    f"Z: {z_cam:+.3f}m"
                ]
                
                # 文字位置（避免遮擋）
                text_x = u + radius_pixel + 10
                text_y = v - 40
                
                for i, line in enumerate(text_lines):
                    y_offset = text_y + i * 20
                    # 文字陰影
                    cv2.putText(vis_image, line, (text_x + 1, y_offset + 1),
                                cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 0), 2)
                    # 文字
                    cv2.putText(vis_image, line, (text_x, y_offset),
                                cv2.FONT_HERSHEY_SIMPLEX, 0.5, color_bgr, 2)
            
            # 計算 FPS
            frame_count += 1
            current_time = time.time()
            if current_time - last_fps_update >= 1.0:
                fps = frame_count / (current_time - last_fps_update)
                frame_count = 0
                last_fps_update = current_time
            
            # 繪製資訊面板
            draw_info_panel(vis_image, obstacles, fps)
            
            # 顯示影像
            cv2.imshow(window_name, vis_image)
            
            # 處理按鍵
            key = cv2.waitKey(1) & 0xFF
            if key == ord('q'):
                print("\n👋 使用者退出")
                break
            elif key == ord('s'):
                screenshot_count += 1
                filename = f"detection_screenshot_{screenshot_count}.png"
                cv2.imwrite(filename, vis_image)
                print(f"📸 截圖已儲存: {filename}")
            
            # 終端輸出（每秒一次）
            if frame_count == 0 and len(obstacles) > 0:
                print(f"[{current_time - start_time:6.1f}s] 檢測到 {len(obstacles)} 個障礙物:")
                for obs in obstacles:
                    x, y, z = obs.position_camera
                    print(f"  {obs.color:8s}: X={x:+7.3f}m  Y={y:+7.3f}m  Z={z:+7.3f}m  R={obs.radius:.3f}m")
    
    except KeyboardInterrupt:
        print("\n⏸️  接收到中斷信號")
    
    finally:
        # 統計資訊
        total_time = time.time() - start_time
        print()
        print("=" * 70)
        print("📊 運行統計:")
        print(f"  總運行時間: {total_time:.1f} 秒")
        print(f"  平均 FPS: {fps:.1f}")
        if screenshot_count > 0:
            print(f"  截圖數量: {screenshot_count}")
        print("=" * 70)
        
        # 清理
        camera.stop()
        cv2.destroyAllWindows()
        print("✅ 已清理資源")


if __name__ == "__main__":
    main()
