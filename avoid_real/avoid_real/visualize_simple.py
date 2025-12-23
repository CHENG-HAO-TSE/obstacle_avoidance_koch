#!/usr/bin/env python3
"""
簡化版 RealSense 3D 檢測視覺化
- 只偵測障礙物（紅/藍/綠/黃）
- 顯示相機座標和世界座標
- 不包含手臂檢測
"""

import numpy as np
import cv2
import sys
import time

try:
    from realsense_camera import RealSenseCamera
    from obstacle_detector import ObstacleDetector
    from coordinate_transform import CoordinateTransform
except ImportError:
    print("❌ 無法導入模組，請確保在正確目錄執行")
    sys.exit(1)


def draw_coordinate_axes(image, origin, scale=50):
    """繪製 3D 座標軸圖示（右手座標系）"""
    ox, oy = origin
    
    # X 軸 (紅色, 向右)
    cv2.arrowedLine(image, (ox, oy), (ox + scale, oy), (0, 0, 255), 2, tipLength=0.3)
    cv2.putText(image, "X", (ox + scale + 5, oy + 5), 
                cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 255), 2)
    
    # Y 軸 (綠色, 向下)
    cv2.arrowedLine(image, (ox, oy), (ox, oy + scale), (0, 255, 0), 2, tipLength=0.3)
    cv2.putText(image, "Y", (ox + 5, oy + scale + 15), 
                cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)
    
    # Z 軸 (藍色, 向外/斜上表示深度)
    z_end = (ox - int(scale * 0.5), oy - int(scale * 0.5))
    cv2.arrowedLine(image, (ox, oy), z_end, (255, 0, 0), 2, tipLength=0.3)
    cv2.putText(image, "Z", (z_end[0] - 15, z_end[1]), 
                cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 0, 0), 2)


def draw_info_panel(image, obstacles, fps, show_world_coords=True):
    """
    繪製資訊面板
    
    Args:
        image: 影像
        obstacles: 檢測到的障礙物列表
        fps: 當前 FPS
        show_world_coords: 是否顯示世界座標
    """
    h, w = image.shape[:2]
    panel_height = 200 + len(obstacles) * 80
    
    # 半透明背景
    overlay = image.copy()
    cv2.rectangle(overlay, (10, 10), (w - 10, panel_height), (0, 0, 0), -1)
    cv2.addWeighted(overlay, 0.7, image, 0.3, 0, image)
    
    # 標題
    y = 35
    cv2.putText(image, f"RealSense Obstacle Detection | FPS: {fps:.1f}", 
                (20, y), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (255, 255, 255), 2)
    
    # 座標系說明
    y += 40
    draw_coordinate_axes(image, (30, y - 5), scale=30)
    cv2.putText(image, "Camera Frame: X=Right, Y=Down, Z=Forward (depth)", 
                (80, y), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (200, 200, 200), 1)
    
    y += 30
    cv2.putText(image, "World Frame: Robot base coordinate system", 
                (20, y), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (150, 200, 150), 1)
    
    # 障礙物資訊
    y += 35
    if len(obstacles) == 0:
        cv2.putText(image, "No obstacles detected", 
                    (20, y), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (100, 100, 100), 1)
    else:
        for i, obs in enumerate(obstacles):
            y += 35
            
            # 顏色指示器
            color_bgr = {
                'red': (0, 0, 255),
                'blue': (255, 0, 0),
                'green': (0, 255, 0),
                'yellow': (0, 255, 255)
            }.get(obs.color, (128, 128, 128))
            
            cv2.circle(image, (30, y - 5), 10, color_bgr, -1)
            cv2.circle(image, (30, y - 5), 10, (255, 255, 255), 2)
            
            # 相機座標
            x_cam, y_cam, z_cam = obs.position_camera
            text1 = f"{obs.color.upper():8s} | Cam: X:{x_cam:+6.3f}m  Y:{y_cam:+6.3f}m  Z:{z_cam:+6.3f}m"
            cv2.putText(image, text1, (50, y), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1)
            
            # 世界座標（如果有）
            if show_world_coords and hasattr(obs, 'position_world'):
                y += 20
                x_world, y_world, z_world = obs.position_world
                text2 = f"         World: X:{x_world:+6.3f}m  Y:{y_world:+6.3f}m  Z:{z_world:+6.3f}m  R:{obs.radius:.3f}m"
                cv2.putText(image, text2, (50, y), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (150, 200, 150), 1)
            
            y += 5
    
    # 操作說明
    y += 40
    cv2.putText(image, "Press 'q' to quit | 's' to save screenshot | 'c' toggle coords", 
                (20, y), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (150, 150, 150), 1)


def draw_obstacle_info_on_image(image, obs, show_world=True):
    """在影像上繪製障礙物資訊"""
    u, v = int(obs.position_2d[0]), int(obs.position_2d[1])
    radius_pixel = int(obs.radius * 100)
    
    color_bgr = {
        'red': (0, 0, 255),
        'blue': (255, 0, 0),
        'green': (0, 255, 0),
        'yellow': (0, 255, 255)
    }.get(obs.color, (128, 128, 128))
    
    # 繪製圓圈
    cv2.circle(image, (u, v), radius_pixel, color_bgr, 3)
    cv2.circle(image, (u, v), 5, color_bgr, -1)
    
    # 繪製十字標記
    cross_size = 15
    cv2.line(image, (u - cross_size, v), (u + cross_size, v), color_bgr, 2)
    cv2.line(image, (u, v - cross_size), (u, v + cross_size), color_bgr, 2)
    
    # 顯示座標資訊
    x_cam, y_cam, z_cam = obs.position_camera
    text_lines = [
        f"{obs.color.upper()}",
        f"Cam:",
        f"X:{x_cam:+.3f}m",
        f"Y:{y_cam:+.3f}m",
        f"Z:{z_cam:+.3f}m"
    ]
    
    if show_world and hasattr(obs, 'position_world'):
        x_w, y_w, z_w = obs.position_world
        text_lines.extend([
            "World:",
            f"X:{x_w:+.3f}m",
            f"Y:{y_w:+.3f}m",
            f"Z:{z_w:+.3f}m"
        ])
    
    # 文字位置
    text_x = u + radius_pixel + 15
    text_y = v - len(text_lines) * 10
    
    # 繪製文字背景
    max_width = max([cv2.getTextSize(line, cv2.FONT_HERSHEY_SIMPLEX, 0.45, 1)[0][0] 
                     for line in text_lines])
    bg_y1 = text_y - 15
    bg_y2 = text_y + len(text_lines) * 18
    cv2.rectangle(image, (text_x - 5, bg_y1), (text_x + max_width + 5, bg_y2), 
                  (0, 0, 0), -1)
    cv2.rectangle(image, (text_x - 5, bg_y1), (text_x + max_width + 5, bg_y2), 
                  color_bgr, 2)
    
    # 繪製文字
    for i, line in enumerate(text_lines):
        y_offset = text_y + i * 18
        text_color = color_bgr if i == 0 else (255, 255, 255)
        cv2.putText(image, line, (text_x, y_offset),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.45, text_color, 1)


def main():
    print("=" * 70)
    print("📷 RealSense 障礙物檢測視覺化工具")
    print("=" * 70)
    print()
    
    # 初始化相機
    print("🔧 初始化 RealSense 相機...")
    try:
        camera = RealSenseCamera()
        print(f"✅ 相機已連接: {camera.device_name}")
    except Exception as e:
        print(f"❌ 相機初始化失敗: {e}")
        return
    
    # 初始化檢測器
    print("🔍 初始化障礙物檢測器...")
    detector = ObstacleDetector()
    print("✅ 檢測器已就緒")
    
    # 初始化座標轉換
    print("📐 初始化座標轉換...")
    calib_file = "/workspace/avoid_real/avoid_real/camera_calibration.json"
    transform = CoordinateTransform(calib_file)
    print("✅ 座標轉換已就緒")
    
    print()
    print("=" * 70)
    print("💡 使用說明:")
    print("  - 將彩色球體（紅/藍/綠/黃）放在相機前")
    print("  - 系統會顯示物體在相機座標系和世界座標系的位置")
    print("  - 按 'q' 退出，按 's' 截圖，按 'c' 切換座標顯示")
    print("=" * 70)
    print()
    print("📊 座標系說明:")
    print("  相機座標系 (Camera Frame):")
    print("    - X: 右為正")
    print("    - Y: 下為正")
    print("    - Z: 前為正（深度）")
    print("    - 原點: 相機光學中心")
    print()
    print("  世界座標系 (World Frame):")
    print("    - X, Y, Z: 機器人底座座標系")
    print("    - 原點: 機器人底座")
    print("=" * 70)
    print()
    
    # 創建視窗
    window_name = "RealSense Obstacle Detection"
    cv2.namedWindow(window_name, cv2.WINDOW_NORMAL)
    cv2.resizeWindow(window_name, 1280, 720)
    
    # 狀態變數
    fps = 0
    frame_count = 0
    start_time = time.time()
    last_fps_update = start_time
    screenshot_count = 0
    show_world_coords = True
    
    try:
        print("▶️  開始檢測...\n")
        
        while True:
            # 獲取影像
            color_image, depth_image = camera.get_frames()
            if color_image is None or depth_image is None:
                time.sleep(0.1)
                continue
            
            # 檢測障礙物
            obstacles = detector.detect_obstacles(color_image, depth_image, camera)
            
            # 轉換到世界座標系
            for obs in obstacles:
                obs.position_world = transform.camera_to_world(obs.position_camera)
            
            # 繪製視覺化
            vis_image = color_image.copy()
            
            # 繪製障礙物
            for obs in obstacles:
                draw_obstacle_info_on_image(vis_image, obs, show_world=show_world_coords)
            
            # 計算 FPS
            frame_count += 1
            current_time = time.time()
            if current_time - last_fps_update >= 1.0:
                fps = frame_count / (current_time - last_fps_update)
                frame_count = 0
                last_fps_update = current_time
            
            # 繪製資訊面板
            draw_info_panel(vis_image, obstacles, fps, show_world_coords)
            
            # 顯示影像
            cv2.imshow(window_name, vis_image)
            
            # 處理按鍵
            key = cv2.waitKey(1) & 0xFF
            if key == ord('q'):
                print("\n👋 使用者退出")
                break
            elif key == ord('s'):
                screenshot_count += 1
                filename = f"obstacle_screenshot_{screenshot_count}.png"
                cv2.imwrite(filename, vis_image)
                print(f"📸 截圖已儲存: {filename}")
            elif key == ord('c'):
                show_world_coords = not show_world_coords
                mode = "世界座標" if show_world_coords else "僅相機座標"
                print(f"🔄 切換顯示模式: {mode}")
    
    except KeyboardInterrupt:
        print("\n⏸️  接收到中斷信號")
    
    finally:
        # 清理
        camera.stop()
        cv2.destroyAllWindows()
        
        total_time = time.time() - start_time
        print()
        print("=" * 70)
        print("📊 運行統計:")
        print(f"  總運行時間: {total_time:.1f} 秒")
        print(f"  平均 FPS: {fps:.1f}")
        if screenshot_count > 0:
            print(f"  截圖數量: {screenshot_count}")
        print("=" * 70)
        print("✅ 已清理資源")


if __name__ == "__main__":
    main()
