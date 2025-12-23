"""
障礙物偵測器
使用 OpenCV 偵測彩色標記物（如紅色球）並計算其 3D 位置
"""

import cv2
import numpy as np
from typing import List, Tuple, Optional
from dataclasses import dataclass


@dataclass
class Obstacle:
    """障礙物資訊"""
    position_camera: np.ndarray  # 相機座標系位置 [x, y, z]
    position_2d: Tuple[int, int]  # 影像中的 2D 位置 (u, v)
    radius: float  # 估計的半徑（公尺）
    color: str  # 顏色標籤
    confidence: float  # 置信度


class ObstacleDetector:
    """基於顏色的障礙物偵測器"""
    
    def __init__(self):
        """初始化偵測器"""
        # 顏色範圍定義（HSV 色彩空間）
        self.color_ranges = {
            'red': [
                # 紅色在 HSV 中分為兩段
                (np.array([0, 100, 100]), np.array([10, 255, 255])),
                (np.array([160, 100, 100]), np.array([180, 255, 255]))
            ],
            'blue': [
                (np.array([100, 100, 100]), np.array([130, 255, 255]))
            ],
            'green': [
                (np.array([40, 100, 100]), np.array([80, 255, 255]))
            ],
            'yellow': [
                (np.array([20, 100, 100]), np.array([40, 255, 255]))
            ]
        }
        
        # 最小/最大輪廓面積
        self.min_area = 100
        self.max_area = 50000
        
        # 圓形度閾值
        self.circularity_threshold = 0.7
    
    def detect_colored_objects(self, color_image: np.ndarray, 
                               target_color: str = 'red') -> List[Tuple[int, int, int]]:
        """
        偵測特定顏色的物體
        
        Args:
            color_image: BGR 彩色影像
            target_color: 目標顏色 ('red', 'blue', 'green', 'yellow')
            
        Returns:
            偵測到的物體列表 [(x, y, radius), ...]
        """
        if target_color not in self.color_ranges:
            print(f"不支援的顏色: {target_color}")
            return []
        
        # 轉換到 HSV 色彩空間
        hsv = cv2.cvtColor(color_image, cv2.COLOR_BGR2HSV)
        
        # 建立顏色遮罩
        mask = np.zeros(hsv.shape[:2], dtype=np.uint8)
        for lower, upper in self.color_ranges[target_color]:
            mask |= cv2.inRange(hsv, lower, upper)
        
        # 形態學操作：去除雜訊
        kernel = np.ones((5, 5), np.uint8)
        mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)
        mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)
        
        # 尋找輪廓
        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        
        detected_objects = []
        
        for contour in contours:
            area = cv2.contourArea(contour)
            
            # 過濾太小或太大的輪廓
            if area < self.min_area or area > self.max_area:
                continue
            
            # 計算圓形度
            perimeter = cv2.arcLength(contour, True)
            if perimeter == 0:
                continue
            circularity = 4 * np.pi * area / (perimeter * perimeter)
            
            # 只保留接近圓形的物體
            if circularity < self.circularity_threshold:
                continue
            
            # 計算最小外接圓
            (x, y), radius = cv2.minEnclosingCircle(contour)
            
            detected_objects.append((int(x), int(y), int(radius)))
        
        return detected_objects
    
    def detect_obstacles(self, color_image: np.ndarray, 
                        depth_image: np.ndarray,
                        camera,
                        target_color: str = 'red') -> List[Obstacle]:
        """
        偵測障礙物並計算其 3D 位置
        
        Args:
            color_image: 彩色影像
            depth_image: 深度影像
            camera: RealSenseCamera 實例
            target_color: 目標顏色
            
        Returns:
            偵測到的障礙物列表
        """
        obstacles = []
        
        # 偵測 2D 位置
        detected_2d = self.detect_colored_objects(color_image, target_color)
        
        for (u, v, radius_px) in detected_2d:
            # 獲取中心點的 3D 座標
            point_3d = camera.get_3d_point(u, v, depth_image)
            
            if point_3d is None:
                continue
            
            # 估算實際半徑（基於深度）
            # 使用相似三角形: radius_m / depth = radius_px / focal_length
            depth = point_3d[2]
            if depth > 0 and camera.intrinsics:
                radius_m = (radius_px * depth) / camera.intrinsics.fx
            else:
                radius_m = 0.05  # 預設值
            
            obstacle = Obstacle(
                position_camera=point_3d,
                position_2d=(u, v),
                radius=radius_m,
                color=target_color,
                confidence=1.0
            )
            
            obstacles.append(obstacle)
        
        return obstacles
    
    def visualize_detections(self, color_image: np.ndarray, 
                            obstacles: List[Obstacle]) -> np.ndarray:
        """
        在影像上繪製偵測結果
        
        Args:
            color_image: 原始彩色影像
            obstacles: 偵測到的障礙物
            
        Returns:
            標註後的影像
        """
        vis_image = color_image.copy()
        
        for obs in obstacles:
            u, v = obs.position_2d
            x, y, z = obs.position_camera
            
            # 繪製圓圈
            cv2.circle(vis_image, (u, v), 5, (0, 255, 0), -1)
            cv2.circle(vis_image, (u, v), int(obs.radius * 100), (0, 255, 0), 2)
            
            # 顯示 3D 座標
            text = f"({x:.2f}, {y:.2f}, {z:.2f})m"
            cv2.putText(vis_image, text, (u + 10, v - 10),
                       cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)
            
            # 顯示半徑
            text_r = f"R:{obs.radius:.3f}m"
            cv2.putText(vis_image, text_r, (u + 10, v + 10),
                       cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 0), 2)
        
        return vis_image


def test_detector():
    """測試障礙物偵測"""
    from realsense_camera import RealSenseCamera
    
    camera = RealSenseCamera()
    detector = ObstacleDetector()
    
    try:
        print("\n🎯 障礙物偵測測試")
        print("按 'q' 退出")
        print("按 '1' 偵測紅色物體")
        print("按 '2' 偵測藍色物體")
        print("按 '3' 偵測綠色物體")
        
        target_color = 'red'
        
        while True:
            color_img, depth_img = camera.get_frames()
            
            if color_img is None:
                continue
            
            # 偵測障礙物
            obstacles = detector.detect_obstacles(color_img, depth_img, camera, target_color)
            
            # 視覺化
            vis_img = detector.visualize_detections(color_img, obstacles)
            
            # 顯示偵測到的數量
            cv2.putText(vis_img, f"Detecting: {target_color.upper()}", (10, 30),
                       cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 255), 2)
            cv2.putText(vis_img, f"Found: {len(obstacles)} objects", (10, 60),
                       cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 255), 2)
            
            cv2.imshow('Obstacle Detection', vis_img)
            
            key = cv2.waitKey(1)
            if key == ord('q'):
                break
            elif key == ord('1'):
                target_color = 'red'
            elif key == ord('2'):
                target_color = 'blue'
            elif key == ord('3'):
                target_color = 'green'
            
            # 印出偵測結果
            if obstacles:
                print(f"\n偵測到 {len(obstacles)} 個 {target_color} 障礙物:")
                for i, obs in enumerate(obstacles):
                    print(f"  [{i}] 位置: ({obs.position_camera[0]:.3f}, "
                          f"{obs.position_camera[1]:.3f}, {obs.position_camera[2]:.3f})m, "
                          f"半徑: {obs.radius:.3f}m")
    
    finally:
        camera.stop()
        cv2.destroyAllWindows()


if __name__ == "__main__":
    test_detector()
