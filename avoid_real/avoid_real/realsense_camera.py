"""
RealSense 深度相機介面
用於偵測障礙物的 3D 位置
"""

import pyrealsense2 as rs
import numpy as np
import cv2
from typing import Tuple, List, Optional
import time


class RealSenseCamera:
    """RealSense 深度相機包裝類"""
    
    def __init__(self, width=640, height=480, fps=60):
        """
        初始化 RealSense 相機
        
        Args:
            width: 影像寬度
            height: 影像高度
            fps: 幀率（D435 支援: 30/60/90Hz，解析度越低支援幀率越高）
        """
        self.width = width
        self.height = height
        self.fps = fps
        
        # 建立 pipeline
        self.pipeline = rs.pipeline()
        self.config = rs.config()
        
        # 配置串流
        self.config.enable_stream(rs.stream.depth, width, height, rs.format.z16, fps)
        self.config.enable_stream(rs.stream.color, width, height, rs.format.bgr8, fps)
        
        # 啟動串流
        self.profile = self.pipeline.start(self.config)
        
        # 獲取設備資訊
        device = self.profile.get_device()
        self.device_name = device.get_info(rs.camera_info.name)
        
        # 獲取深度感測器的深度比例
        depth_sensor = device.first_depth_sensor()
        self.depth_scale = depth_sensor.get_depth_scale()
        print(f"深度比例: {self.depth_scale}")
        
        # 建立對齊物件（將深度圖對齊到彩色圖）
        self.align = rs.align(rs.stream.color)
        
        # 緩存最新的幀（用於非阻塞式獲取）
        self.last_color_image = None
        self.last_depth_image = None
        
        # 獲取相機內參
        self.intrinsics = None
        self._get_intrinsics()
        
        # 預熱相機
        print("相機預熱中...")
        for _ in range(30):
            frames = self.pipeline.wait_for_frames()
            aligned_frames = self.align.process(frames)
            aligned_depth_frame = aligned_frames.get_depth_frame()
            color_frame = aligned_frames.get_color_frame()
            if aligned_depth_frame and color_frame:
                self.last_depth_image = np.asanyarray(aligned_depth_frame.get_data())
                self.last_color_image = np.asanyarray(color_frame.get_data())
        print(f"✅ RealSense 相機已就緒 ({fps}Hz)")
    
    def _get_intrinsics(self):
        """獲取相機內參矩陣"""
        frames = self.pipeline.wait_for_frames()
        color_frame = frames.get_color_frame()
        
        if color_frame:
            self.intrinsics = color_frame.profile.as_video_stream_profile().intrinsics
            print(f"\n📷 相機內參:")
            print(f"   解析度: {self.intrinsics.width} x {self.intrinsics.height}")
            print(f"   焦距 (fx, fy): ({self.intrinsics.fx:.2f}, {self.intrinsics.fy:.2f})")
            print(f"   主點 (ppx, ppy): ({self.intrinsics.ppx:.2f}, {self.intrinsics.ppy:.2f})")
            print(f"   畸變模型: {self.intrinsics.model}")
            print(f"   畸變係數: {self.intrinsics.coeffs}")
    
    def get_frames(self, non_blocking=True) -> Tuple[Optional[np.ndarray], Optional[np.ndarray]]:
        """
        獲取對齊的彩色和深度影像
        
        Args:
            non_blocking: 如果為 True，使用 poll_for_frames()（不阻塞）
                         如果為 False，使用 wait_for_frames()（阻塞直到新幀）
        
        Returns:
            (color_image, depth_image) 或 (None, None) 如果失敗
        """
        try:
            # 根據模式選擇獲取方法
            if non_blocking:
                # 非阻塞模式：嘗試獲取新幀，如果沒有則返回緩存的幀
                frames = self.pipeline.poll_for_frames()
                
                if not frames:
                    # 沒有新幀，返回上次的幀
                    return self.last_color_image, self.last_depth_image
            else:
                # 阻塞模式：等待新幀
                frames = self.pipeline.wait_for_frames()
            
            # 對齊深度圖到彩色圖
            aligned_frames = self.align.process(frames)
            
            # 獲取對齊後的幀
            aligned_depth_frame = aligned_frames.get_depth_frame()
            color_frame = aligned_frames.get_color_frame()
            
            if not aligned_depth_frame or not color_frame:
                # 如果對齊失敗，返回緩存的幀
                return self.last_color_image, self.last_depth_image
            
            # 轉換為 numpy 陣列並緩存
            self.last_depth_image = np.asanyarray(aligned_depth_frame.get_data())
            self.last_color_image = np.asanyarray(color_frame.get_data())
            
            return self.last_color_image, self.last_depth_image
            
        except Exception as e:
            print(f"獲取影像失敗: {e}")
            # 出錯時返回緩存的幀
            return self.last_color_image, self.last_depth_image
    
    def get_3d_point(self, u: int, v: int, depth_image: np.ndarray) -> Optional[np.ndarray]:
        """
        將 2D 像素座標轉換為 3D 相機座標
        
        Args:
            u: 像素 x 座標
            v: 像素 y 座標
            depth_image: 深度影像
            
        Returns:
            3D 點 [x, y, z] (公尺) 或 None
        """
        if self.intrinsics is None:
            return None
        
        # 獲取深度值（單位：mm）
        depth_value = depth_image[v, u]
        
        if depth_value == 0:
            return None
        
        # 轉換為公尺
        depth_m = depth_value * self.depth_scale
        
        # 使用相機內參反投影到 3D 空間
        # x = (u - ppx) * depth / fx
        # y = (v - ppy) * depth / fy
        # z = depth
        x = (u - self.intrinsics.ppx) * depth_m / self.intrinsics.fx
        y = (v - self.intrinsics.ppy) * depth_m / self.intrinsics.fy
        z = depth_m
        
        return np.array([x, y, z], dtype=np.float32)
    
    def deproject_pixel_to_point(self, pixel: Tuple[int, int], 
                                  depth_image: np.ndarray) -> Optional[np.ndarray]:
        """
        使用 RealSense SDK 的反投影函數
        
        Args:
            pixel: (u, v) 像素座標
            depth_image: 深度影像
            
        Returns:
            3D 點 [x, y, z] (公尺)
        """
        u, v = pixel
        depth_value = depth_image[v, u]
        
        if depth_value == 0:
            return None
        
        # 使用 SDK 的反投影函數
        point_3d = rs.rs2_deproject_pixel_to_point(
            self.intrinsics, 
            [u, v], 
            depth_value * self.depth_scale
        )
        
        return np.array(point_3d, dtype=np.float32)
    
    def get_camera_matrix(self) -> np.ndarray:
        """
        獲取相機內參矩陣 K
        
        Returns:
            3x3 內參矩陣
        """
        if self.intrinsics is None:
            return None
        
        K = np.array([
            [self.intrinsics.fx, 0, self.intrinsics.ppx],
            [0, self.intrinsics.fy, self.intrinsics.ppy],
            [0, 0, 1]
        ], dtype=np.float32)
        
        return K
    
    def stop(self):
        """停止相機串流"""
        self.pipeline.stop()
        print("相機已停止")


def test_camera():
    """測試相機功能"""
    camera = RealSenseCamera()
    
    try:
        print("\n按 'q' 退出，按 's' 擷取中心點座標")
        
        while True:
            color_img, depth_img = camera.get_frames()
            
            if color_img is None:
                continue
            
            # 顯示深度圖（可視化）
            depth_colormap = cv2.applyColorMap(
                cv2.convertScaleAbs(depth_img, alpha=0.03), 
                cv2.COLORMAP_JET
            )
            
            # 繪製中心點
            h, w = color_img.shape[:2]
            center = (w//2, h//2)
            cv2.circle(color_img, center, 5, (0, 255, 0), -1)
            cv2.circle(depth_colormap, center, 5, (0, 255, 0), -1)
            
            # 獲取中心點的 3D 座標
            point_3d = camera.get_3d_point(center[0], center[1], depth_img)
            if point_3d is not None:
                text = f"Center: ({point_3d[0]:.3f}, {point_3d[1]:.3f}, {point_3d[2]:.3f})m"
                cv2.putText(color_img, text, (10, 30), 
                           cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)
            
            # 並排顯示
            images = np.hstack((color_img, depth_colormap))
            cv2.imshow('RealSense - Color | Depth', images)
            
            key = cv2.waitKey(1)
            if key == ord('q'):
                break
            elif key == ord('s'):
                if point_3d is not None:
                    print(f"\n📍 中心點座標: X={point_3d[0]:.3f}m, "
                          f"Y={point_3d[1]:.3f}m, Z={point_3d[2]:.3f}m")
    
    finally:
        camera.stop()
        cv2.destroyAllWindows()


if __name__ == "__main__":
    test_camera()
