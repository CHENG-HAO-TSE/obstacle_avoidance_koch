"""
座標轉換系統
將相機座標系轉換為 PyBullet 世界座標系
"""

import numpy as np
from typing import Tuple
import json
import os


class CoordinateTransform:
    """座標轉換類"""
    
    def __init__(self, calibration_file: str = None):
        """
        初始化座標轉換
        
        Args:
            calibration_file: 標定檔案路徑（包含相機外參）
        """
        # 預設相機外參（相機到機器人底座的轉換）
        # 這些參數需要透過手眼標定獲得
        self.camera_to_base = np.eye(4, dtype=np.float32)
        
        # 預設值：相機位於機器人前方 30cm，上方 50cm，向下看
        if calibration_file and os.path.exists(calibration_file):
            self.load_calibration(calibration_file)
        else:
            self.set_default_transform()
    
    def set_default_transform(self):
        """
        設置預設的相機到底座轉換
        
        假設相機配置：
        - 位於機器人底座前方 Y=0.50m（50公分）
        - 高度 Z=0.10m（10公分）
        - 鏡頭面向手臂（水平向後看，向下俯視）
        
        座標系定義：
        相機座標系（RealSense D435 標準）：
          X_cam: 右為正
          Y_cam: 下為正
          Z_cam: 前為正（深度方向）
        
        機器人座標系（PyBullet/Koch）：
          X_robot: 右為正
          Y_robot: 前為正
          Z_robot: 上為正
        
        轉換關係（相機朝向手臂，向下俯視）：
          X_robot = -X_cam  （相機朝後看，左右相反）
          Y_robot = -Z_cam  （相機的前方 = 機器人的後方）
          Z_robot = -Y_cam  （相機的下方 = 機器人的下方，但 Y_cam 向下為正，Z_robot 向上為正）
        
        旋轉矩陣 R = Rx(90°) * Rz(180°)
        """
        # 方法：手動構建旋轉矩陣
        # 1. 相機 X 軸（右）→ 機器人 -X 軸（左） [-1,0,0]  因為相機朝後看
        # 2. 相機 Y 軸（下）→ 機器人 -Z 軸（下） [0,0,-1]
        # 3. 相機 Z 軸（前）→ 機器人 -Y 軸（後） [0,-1,0]
        
        R_cam_to_robot = np.array([
            [-1,  0,  0],  # 機器人 X = -相機 X （相機朝後，左右鏡像）
            [ 0,  0, -1],  # 機器人 Y = -相機 Z （相機前方對應機器人後方）
            [ 0, -1,  0]   # 機器人 Z = -相機 Y （相機下方對應機器人下方，但符號相反）
        ], dtype=np.float32)
        
        # 相機在機器人座標系中的位置
        # X=0 (中央), Y=0.50 (前方50cm), Z=0.10 (高度10cm)
        t_cam_in_robot = np.array([0.0, 0.50, 0.10], dtype=np.float32)
        
        # 組合成齊次轉換矩陣
        self.camera_to_base = np.eye(4, dtype=np.float32)
        self.camera_to_base[:3, :3] = R_cam_to_robot
        self.camera_to_base[:3, 3] = t_cam_in_robot
        
        print("📐 使用預設相機外參:")
        print(f"   相機位置（機器人座標系）: X={t_cam_in_robot[0]:.2f}m, Y={t_cam_in_robot[1]:.2f}m (前方), Z={t_cam_in_robot[2]:.2f}m (高度)")
        print(f"   相機朝向: 向後俯視手臂")
        print(f"   座標映射: [X_robot, Y_robot, Z_robot] = R * [X_cam, Y_cam, Z_cam] + t")
        print(f"            X_robot = +X_cam (右)")
        print(f"            Y_robot = -Z_cam (後)")
        print(f"            Z_robot = -Y_cam (上)")
    
    def camera_to_world(self, point_camera: np.ndarray) -> np.ndarray:
        """
        將相機座標系的點轉換為世界座標系
        
        Args:
            point_camera: 相機座標系中的點 [x, y, z]
            
        Returns:
            世界座標系中的點 [x, y, z]
        """
        # 轉換為齊次座標
        point_homo = np.append(point_camera, 1.0)
        
        # 應用轉換矩陣
        point_world_homo = self.camera_to_base @ point_homo
        
        # 轉回 3D 座標
        return point_world_homo[:3]
    
    def world_to_camera(self, point_world: np.ndarray) -> np.ndarray:
        """
        將世界座標系的點轉換為相機座標系
        
        Args:
            point_world: 世界座標系中的點 [x, y, z]
            
        Returns:
            相機座標系中的點 [x, y, z]
        """
        # 計算逆轉換
        base_to_camera = np.linalg.inv(self.camera_to_base)
        
        # 轉換為齊次座標
        point_homo = np.append(point_world, 1.0)
        
        # 應用轉換矩陣
        point_camera_homo = base_to_camera @ point_homo
        
        # 轉回 3D 座標
        return point_camera_homo[:3]
    
    def set_camera_pose(self, position: np.ndarray, 
                       rotation_matrix: np.ndarray = None,
                       euler_angles: np.ndarray = None):
        """
        手動設置相機位姿
        
        Args:
            position: 相機位置 [x, y, z]
            rotation_matrix: 3x3 旋轉矩陣（可選）
            euler_angles: 歐拉角 [roll, pitch, yaw]（弧度，可選）
        """
        self.camera_to_base = np.eye(4, dtype=np.float32)
        
        # 設置旋轉
        if rotation_matrix is not None:
            self.camera_to_base[:3, :3] = rotation_matrix
        elif euler_angles is not None:
            R = self.euler_to_rotation_matrix(euler_angles)
            self.camera_to_base[:3, :3] = R
        
        # 設置平移
        self.camera_to_base[:3, 3] = position
        
        print(f"✅ 相機位姿已更新: 位置 = {position}")
    
    @staticmethod
    def euler_to_rotation_matrix(euler: np.ndarray) -> np.ndarray:
        """
        將歐拉角轉換為旋轉矩陣（ZYX 順序）
        
        Args:
            euler: [roll, pitch, yaw] 弧度
            
        Returns:
            3x3 旋轉矩陣
        """
        roll, pitch, yaw = euler
        
        # Roll (繞 X 軸)
        R_x = np.array([
            [1, 0, 0],
            [0, np.cos(roll), -np.sin(roll)],
            [0, np.sin(roll), np.cos(roll)]
        ])
        
        # Pitch (繞 Y 軸)
        R_y = np.array([
            [np.cos(pitch), 0, np.sin(pitch)],
            [0, 1, 0],
            [-np.sin(pitch), 0, np.cos(pitch)]
        ])
        
        # Yaw (繞 Z 軸)
        R_z = np.array([
            [np.cos(yaw), -np.sin(yaw), 0],
            [np.sin(yaw), np.cos(yaw), 0],
            [0, 0, 1]
        ])
        
        # 組合: R = R_z * R_y * R_x
        R = R_z @ R_y @ R_x
        return R
    
    def save_calibration(self, filepath: str):
        """
        儲存標定結果
        
        Args:
            filepath: 儲存路徑
        """
        calibration_data = {
            'camera_to_base': self.camera_to_base.tolist(),
            'timestamp': str(np.datetime64('now'))
        }
        
        with open(filepath, 'w') as f:
            json.dump(calibration_data, f, indent=4)
        
        print(f"✅ 標定結果已儲存: {filepath}")
    
    def load_calibration(self, filepath: str):
        """
        載入標定結果
        
        Args:
            filepath: 檔案路徑
        """
        with open(filepath, 'r') as f:
            calibration_data = json.load(f)
        
        self.camera_to_base = np.array(calibration_data['camera_to_base'], 
                                       dtype=np.float32)
        
        print(f"✅ 已載入標定檔案: {filepath}")
    
    def calibrate_using_markers(self, 
                                marker_positions_camera: list,
                                marker_positions_world: list):
        """
        使用標記點進行手眼標定
        
        Args:
            marker_positions_camera: 在相機座標系中的標記點列表
            marker_positions_world: 在世界座標系中的標記點列表
        
        Note:
            這是簡化版本，實際應用建議使用 OpenCV 的 solvePnP 或專門的手眼標定演算法
        """
        import cv2
        
        # 轉換為 numpy 陣列
        src_points = np.array(marker_positions_camera, dtype=np.float32)
        dst_points = np.array(marker_positions_world, dtype=np.float32)
        
        # 使用 Procrustes 分析計算剛體轉換
        # 這裡簡化處理，實際應該使用更完整的演算法
        
        # 計算質心
        src_center = np.mean(src_points, axis=0)
        dst_center = np.mean(dst_points, axis=0)
        
        # 中心化
        src_centered = src_points - src_center
        dst_centered = dst_points - dst_center
        
        # 計算旋轉矩陣（使用 SVD）
        H = src_centered.T @ dst_centered
        U, S, Vt = np.linalg.svd(H)
        R = Vt.T @ U.T
        
        # 確保是右手座標系
        if np.linalg.det(R) < 0:
            Vt[-1, :] *= -1
            R = Vt.T @ U.T
        
        # 計算平移
        t = dst_center - R @ src_center
        
        # 組合轉換矩陣
        self.camera_to_base = np.eye(4, dtype=np.float32)
        self.camera_to_base[:3, :3] = R
        self.camera_to_base[:3, 3] = t
        
        print("✅ 手眼標定完成")
        print(f"   旋轉矩陣:\n{R}")
        print(f"   平移向量: {t}")


def test_transform():
    """測試座標轉換"""
    transform = CoordinateTransform()
    
    # 測試點（相機座標系）
    point_camera = np.array([0.1, 0.2, 0.5])  # X右0.1m, Y下0.2m, Z前0.5m
    
    print(f"\n📍 相機座標系: {point_camera}")
    
    # 轉換到世界座標系
    point_world = transform.camera_to_world(point_camera)
    print(f"📍 世界座標系: {point_world}")
    
    # 反向轉換驗證
    point_camera_back = transform.world_to_camera(point_world)
    print(f"📍 反向轉換: {point_camera_back}")
    print(f"誤差: {np.linalg.norm(point_camera - point_camera_back):.6f}m")
    
    # 儲存標定
    transform.save_calibration('/workspace/koch_ros/avoid_real/camera_calibration.json')


if __name__ == "__main__":
    test_transform()
