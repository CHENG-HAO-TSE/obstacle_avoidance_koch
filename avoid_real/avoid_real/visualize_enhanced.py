#!/usr/bin/env python3
"""
增強版 RealSense 3D 檢測視覺化
- 顯示障礙物的 XYZ 座標（相機座標系）
- 顯示障礙物的 XYZ 座標（機器人世界座標系）
- 用 Bounding Box 框出手臂
- 同時顯示兩種座標系
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


class ArmDetector:
    """手臂檢測器（使用顏色和深度結合）"""
    
    def __init__(self):
        # 手臂的預期深度範圍（公尺）
        self.arm_depth_min = 0.15  # 15cm
        self.arm_depth_max = 0.70  # 70cm
        
        # 手臂的預期位置範圍（相機座標系，公尺）
        self.arm_x_range = (-0.35, 0.35)  # 左右範圍
        self.arm_y_range = (-0.25, 0.35)  # 上下範圍
        
        # 機械臂顏色範圍（黑色/深灰色金屬材質）
        # HSV 範圍：低飽和度、低亮度
        self.arm_color_ranges = [
            # 黑色/深灰色範圍
            {'lower': np.array([0, 0, 0]), 'upper': np.array([180, 100, 100])},
            # 深藍灰色
            {'lower': np.array([90, 0, 0]), 'upper': np.array([130, 80, 80])},
        ]
    
    def detect_arm_region(self, color_image, depth_image, camera):
        """
        檢測手臂區域（結合顏色和深度）
        
        Args:
            color_image: BGR 彩色影像
            depth_image: 深度影像
            camera: RealSenseCamera 實例
            
        Returns:
            list of bounding boxes: [(x1, y1, x2, y2), ...]
        """
        h, w = color_image.shape[:2]
        
        # 轉換為 HSV
        hsv = cv2.cvtColor(color_image, cv2.COLOR_BGR2HSV)
        
        # 創建顏色遮罩
        color_mask = np.zeros((h, w), dtype=np.uint8)
        for color_range in self.arm_color_ranges:
            mask = cv2.inRange(hsv, color_range['lower'], color_range['upper'])
            color_mask = cv2.bitwise_or(color_mask, mask)
        
        # 創建深度遮罩
        depth_mask = np.zeros((h, w), dtype=np.uint8)
        depth_m = depth_image * camera.depth_scale
        depth_mask[(depth_m >= self.arm_depth_min) & (depth_m <= self.arm_depth_max)] = 255
        
        # 結合顏色和深度遮罩
        combined_mask = cv2.bitwise_and(color_mask, depth_mask)
        
        # 形態學操作去噪
        kernel_small = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (5, 5))
        combined_mask = cv2.morphologyEx(combined_mask, cv2.MORPH_OPEN, kernel_small)
        
        # 形態學操作連接斷裂區域
        kernel_large = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (20, 20))
        combined_mask = cv2.morphologyEx(combined_mask, cv2.MORPH_CLOSE, kernel_large)
        
        # 找輪廓
        contours, _ = cv2.findContours(combined_mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        
        bboxes = []
        for contour in contours:
            area = cv2.contourArea(contour)
            
            # 過濾太小的區域
            if area < 2000:
                continue
            
            # 計算 bounding box
            x, y, w, h = cv2.boundingRect(contour)
            
            # 過濾不合理的長寬比（手臂應該是細長的）
            aspect_ratio = max(w, h) / (min(w, h) + 1e-6)
            if aspect_ratio < 1.2 or aspect_ratio > 10:
                continue
            
            # 檢查中心點的深度是否在合理範圍
            center_u, center_v = x + w // 2, y + h // 2
            if 0 <= center_v < h and 0 <= center_u < w:
                center_depth = depth_image[center_v, center_u] * camera.depth_scale
                if not (self.arm_depth_min <= center_depth <= self.arm_depth_max):
                    continue
            
            bboxes.append((x, y, x+w, y+h))
        
        return bboxes


def draw_coordinate_axes(image, origin=(50, 50), scale=40):
    """
    繪製座標系圖示
    
    Args:
        image: 影像
        origin: 原點位置
        scale: 座標軸長度
    """
    ox, oy = origin
    
    # 背景圓圈
    cv2.circle(image, (ox, oy), scale + 10, (40, 40, 40), -1)
    cv2.circle(image, (ox, oy), scale + 10, (255, 255, 255), 2)
    
    # X 軸（紅色）
    cv2.arrowedLine(image, (ox, oy), (ox + scale, oy), (0, 0, 255), 3, tipLength=0.3)
    cv2.putText(image, 'X', (ox + scale + 5, oy + 5), 
                cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 0, 255), 2)
    
    # Y 軸（綠色）
    cv2.arrowedLine(image, (ox, oy), (ox, oy + scale), (0, 255, 0), 3, tipLength=0.3)
    cv2.putText(image, 'Y', (ox + 5, oy + scale + 15), 
                cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)
    
    # Z 軸標記（藍色）
    cv2.circle(image, (ox, oy), 8, (255, 0, 0), -1)
    cv2.putText(image, 'Z', (ox - 40, oy + 5), 
                cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 0, 0), 2)


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
    cv2.putText(image, f"RealSense 3D Detection | FPS: {fps:.1f}", 
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


def draw_arm_bounding_box(image, bboxes):
    """繪製手臂 Bounding Box"""
    for bbox in bboxes:
        x1, y1, x2, y2 = bbox
        
        # 繪製邊框
        cv2.rectangle(image, (x1, y1), (x2, y2), (0, 255, 255), 3)
        
        # 繪製角落標記
        corner_len = 20
        # 左上
        cv2.line(image, (x1, y1), (x1 + corner_len, y1), (0, 255, 255), 4)
        cv2.line(image, (x1, y1), (x1, y1 + corner_len), (0, 255, 255), 4)
        # 右上
        cv2.line(image, (x2, y1), (x2 - corner_len, y1), (0, 255, 255), 4)
        cv2.line(image, (x2, y1), (x2, y1 + corner_len), (0, 255, 255), 4)
        # 左下
        cv2.line(image, (x1, y2), (x1 + corner_len, y2), (0, 255, 255), 4)
        cv2.line(image, (x1, y2), (x1, y2 - corner_len), (0, 255, 255), 4)
        # 右下
        cv2.line(image, (x2, y2), (x2 - corner_len, y2), (0, 255, 255), 4)
        cv2.line(image, (x2, y2), (x2, y2 - corner_len), (0, 255, 255), 4)
        
        # 標籤
        label = "Robot Arm"
        label_size = cv2.getTextSize(label, cv2.FONT_HERSHEY_SIMPLEX, 0.6, 2)[0]
        cv2.rectangle(image, (x1, y1 - label_size[1] - 10), 
                     (x1 + label_size[0] + 10, y1), (0, 255, 255), -1)
        cv2.putText(image, label, (x1 + 5, y1 - 5),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 0, 0), 2)


def main():
    print("=" * 70)
    print("📷 RealSense 3D 位置視覺化工具（增強版）")
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
    
    # 初始化手臂檢測器
    print("🤖 初始化手臂檢測器...")
    arm_detector = ArmDetector()
    print("✅ 手臂檢測器已就緒")
    
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
    print("  - 手臂會用黃色 Bounding Box 標示")
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
    window_name = "RealSense 3D Detection (Enhanced)"
    cv2.namedWindow(window_name, cv2.WINDOW_NORMAL)
    cv2.resizeWindow(window_name, 1280, 720)
    
    # 狀態變數
    fps = 0
    frame_count = 0
    start_time = time.time()
    last_fps_update = start_time
    screenshot_count = 0
    show_world_coords = True
    detect_arm = True
    
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
            
            # 檢測手臂
            arm_bboxes = []
            if detect_arm:
                arm_bboxes = arm_detector.detect_arm_region(color_image, depth_image, camera)
            
            # 繪製視覺化
            vis_image = color_image.copy()
            
            # 繪製手臂邊界框
            if arm_bboxes:
                draw_arm_bounding_box(vis_image, arm_bboxes)
            
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
                filename = f"detection_screenshot_{screenshot_count}.png"
                cv2.imwrite(filename, vis_image)
                print(f"📸 截圖已儲存: {filename}")
            elif key == ord('c'):
                show_world_coords = not show_world_coords
                mode = "世界座標" if show_world_coords else "僅相機座標"
                print(f"🔄 切換顯示模式: {mode}")
            elif key == ord('a'):
                detect_arm = not detect_arm
                mode = "啟用" if detect_arm else "停用"
                print(f"🤖 手臂檢測: {mode}")
    
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
