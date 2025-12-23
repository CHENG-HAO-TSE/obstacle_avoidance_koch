# 真實世界障礙物避障系統

基於 RealSense 深度相機的即時障礙物檢測與 PyBullet 同步系統。

## 📋 目錄

- [系統架構](#系統架構)
- [功能特性](#功能特性)
- [安裝依賴](#安裝依賴)
- [快速開始](#快速開始)
- [模組說明](#模組說明)
- [相機標定](#相機標定)
- [使用範例](#使用範例)
- [疑難排解](#疑難排解)

---

## 🏗️ 系統架構

```
RealSense 相機 → 障礙物檢測 → 座標轉換 → PyBullet 同步
     ↓              ↓             ↓            ↓
  RGB-D 影像    彩色物體檢測   相機→世界    即時更新障礙物
```

### 資料流程

1. **影像獲取**: RealSense D435/D455 提供對齊的 RGB-D 影像
2. **物體檢測**: 基於 HSV 顏色空間檢測彩色球體
3. **3D 定位**: 結合深度資訊計算物體在相機座標系的 3D 位置
4. **座標轉換**: 將相機座標轉換為機器人世界座標
5. **模擬同步**: 在 PyBullet 中即時創建/更新障礙物

---

## ✨ 功能特性

- ✅ **即時檢測**: 10Hz 障礙物檢測頻率
- ✅ **多顏色支援**: 支援紅、藍、綠、黃四種顏色
- ✅ **自動半徑估算**: 根據深度資訊自動計算障礙物半徑
- ✅ **座標轉換**: 完整的相機-世界座標轉換
- ✅ **PyBullet 同步**: 即時創建、更新、移除障礙物
- ✅ **視覺化**: 同步顯示相機檢測與模擬場景
- ✅ **離線測試**: 無相機時可使用模擬資料

---

## 📦 安裝依賴

### 1. Python 套件

```bash
pip install pyrealsense2 opencv-python numpy pybullet
```

### 2. RealSense SDK (如果尚未安裝)

#### Ubuntu/Debian:
```bash
sudo apt-key adv --keyserver keyserver.ubuntu.com --recv-key F6E65AC044F831AC80A06380C8B3A55A6F3EFCDE
sudo add-apt-repository "deb https://librealsense.intel.com/Debian/apt-repo $(lsb_release -cs) main"
sudo apt update
sudo apt install librealsense2-dkms librealsense2-utils librealsense2-dev
```

### 3. 驗證安裝

```bash
# 檢查 RealSense 相機
realsense-viewer

# 測試 Python 綁定
python3 -c "import pyrealsense2 as rs; print(rs.__version__)"
```

---

## 🚀 快速開始

### 基本使用

```bash
cd /workspace/koch_ros/avoid_real
python3 demo_real_obstacle_avoidance.py
```

### 預期輸出

```
🚀 真實世界障礙物避障系統啟動
📷 初始化 RealSense 相機...
✅ RealSense 相機已初始化: Intel RealSense D435
🔍 初始化障礙物檢測器...
📐 初始化座標轉換...
🎮 設置 PyBullet 場景...
🔄 初始化 PyBullet 同步器...

🎯 系統就緒！
   - 將彩色球體放置在相機視野內
   - 障礙物將即時同步到 PyBullet
   - 按 Ctrl+C 停止

📊 檢測統計 (幀 1):
   RED      | 相機座標: [0.12 0.05 0.45] | 世界座標: [0.31 0.42 0.38] | 半徑: 0.035m
```

---

## 📚 模組說明

### 1. `realsense_camera.py`

**RealSense 相機介面**

```python
from realsense_camera import RealSenseCamera

# 初始化相機
camera = RealSenseCamera(width=640, height=480, fps=30)

# 獲取影像
color_image, depth_image = camera.get_frames()

# 獲取 3D 點
point_3d = camera.get_3d_point(u=320, v=240, depth_image=depth_image)

# 停止相機
camera.stop()
```

**主要方法:**
- `get_frames()`: 獲取對齊的 RGB-D 影像
- `get_3d_point(u, v, depth_image)`: 像素轉 3D 座標
- `get_camera_matrix()`: 獲取相機內參矩陣

---

### 2. `obstacle_detector.py`

**障礙物檢測器**

```python
from obstacle_detector import ObstacleDetector

# 初始化檢測器
detector = ObstacleDetector()

# 檢測 2D 物體
detections = detector.detect_colored_objects(color_image)

# 檢測 3D 障礙物
obstacles = detector.detect_obstacles(color_image, depth_image)

# 視覺化
vis_image = detector.visualize_detections(color_image, detections)
```

**支援顏色:**
- `red`: 紅色 (HSV: 0-10, 160-180)
- `blue`: 藍色 (HSV: 100-130)
- `green`: 綠色 (HSV: 40-80)
- `yellow`: 黃色 (HSV: 20-40)

**過濾條件:**
- 最小面積: 500 像素
- 圓度閾值: 0.7 (球形物體)

---

### 3. `coordinate_transform.py`

**座標轉換系統**

```python
from coordinate_transform import CoordinateTransform

# 載入標定檔案
transform = CoordinateTransform('camera_calibration.json')

# 相機 → 世界座標
point_world = transform.camera_to_world([0.1, 0.2, 0.5])

# 世界 → 相機座標
point_camera = transform.world_to_camera([0.3, 0.4, 0.2])

# 儲存標定
transform.save_calibration('my_calibration.json')
```

**預設相機位姿:**
- 位置: `[0.0, 0.30, 0.50]` (機器人前方 30cm，高度 50cm)
- 俯仰角: 45°

---

### 4. `pybullet_sync.py`

**PyBullet 障礙物同步**

```python
from pybullet_sync import PyBulletObstacleSync

# 初始化同步器
sync = PyBulletObstacleSync(physics_client)

# 同步障礙物
obstacle_ids = sync.sync_obstacles(detected_obstacles)

# 獲取障礙物資訊
info = sync.get_obstacle_info()

# 清除所有障礙物
sync.clear_all_obstacles()
```

**特性:**
- 自動創建/更新/移除障礙物
- 半徑變化檢測 (>10% 重新創建)
- 超時機制 (1 秒無更新自動移除)

---

## 🎯 相機標定

### 方法 1: 使用預設值

如果相機位置已知，直接設置:

```python
from coordinate_transform import CoordinateTransform
import numpy as np

transform = CoordinateTransform()
transform.set_camera_pose(
    position=[0.0, 0.30, 0.50],  # [X, Y, Z]
    euler_angles=[0, np.radians(45), 0]  # [roll, pitch, yaw]
)
transform.save_calibration('camera_calibration.json')
```

### 方法 2: 標記點標定 (推薦)

1. **準備標記點**:
   - 在機器人工作空間放置 3-4 個已知位置的標記物
   - 使用 ArUco 標記或其他易於檢測的物體

2. **記錄座標**:

```python
# 在相機座標系中測量的位置
marker_positions_camera = [
    [0.1, 0.2, 0.5],
    [-0.1, 0.3, 0.6],
    [0.2, -0.1, 0.4],
]

# 在世界座標系中的已知位置
marker_positions_world = [
    [0.3, 0.4, 0.1],
    [0.2, 0.5, 0.15],
    [0.4, 0.2, 0.08],
]

# 執行標定
transform.calibrate_using_markers(
    marker_positions_camera,
    marker_positions_world
)
transform.save_calibration('camera_calibration.json')
```

### 驗證標定

```python
# 測試點
test_point_camera = [0.0, 0.0, 0.5]
test_point_world = transform.camera_to_world(test_point_camera)
test_point_back = transform.world_to_camera(test_point_world)

error = np.linalg.norm(test_point_camera - test_point_back)
print(f"標定誤差: {error*1000:.2f} mm")  # 應該 < 5mm
```

---

## 💡 使用範例

### 範例 1: 獨立測試各模組

```bash
# 測試相機
python3 realsense_camera.py

# 測試障礙物檢測
python3 obstacle_detector.py

# 測試座標轉換
python3 coordinate_transform.py

# 測試 PyBullet 同步
python3 pybullet_sync.py
```

### 範例 2: 自訂檢測顏色

```python
from obstacle_detector import ObstacleDetector

detector = ObstacleDetector()

# 添加橙色檢測
detector.color_ranges['orange'] = {
    'lower': np.array([10, 100, 100]),
    'upper': np.array([20, 255, 255])
}

# 檢測橙色物體
obstacles = detector.detect_obstacles(color_image, depth_image)
```

### 範例 3: 整合到現有控制器

```python
from pybullet_sync import RealTimeObstacleSynchronizer

# 在控制迴圈中
synchronizer = RealTimeObstacleSynchronizer(
    camera, detector, transform, pybullet_sync,
    update_rate=20.0  # 20 Hz
)

while robot_running:
    # 更新障礙物
    obstacles = synchronizer.update()
    
    # 你的控制邏輯
    robot_control_step()
    
    # PyBullet 步進
    p.stepSimulation()
```

---

## 🔧 疑難排解

### 問題 1: 找不到 RealSense 相機

**症狀:**
```
RuntimeError: No device connected, please connect a RealSense device
```

**解決方法:**
1. 檢查 USB 連接: `lsusb | grep Intel`
2. 測試相機: `realsense-viewer`
3. 檢查權限: `sudo chmod 666 /dev/video*`

### 問題 2: X11 顯示錯誤 (Docker 環境)

**症狀:**
```
cannot connect to X server
```

**解決方法:**
```bash
# 在宿主機執行
xhost +local:docker

# 在容器內執行
export DISPLAY=:0
```

參考: [X11_GUI_FIX.md](../X11_GUI_FIX.md)

### 問題 3: 檢測不到物體

**檢查清單:**
- ✅ 光照條件良好
- ✅ 物體顏色飽和度高
- ✅ 物體在相機視野內 (0.3-3m)
- ✅ 物體形狀接近球形

**調試方法:**
```python
# 顯示 HSV 影像
detector.debug_mode = True
obstacles = detector.detect_obstacles(color_image, depth_image)
```

### 問題 4: 座標轉換不正確

**症狀:**
障礙物在 PyBullet 中的位置與現實不符

**解決方法:**
1. 重新標定相機外參
2. 檢查座標系定義 (右手/左手系統)
3. 驗證單位 (米 vs 毫米)

```python
# 可視化座標軸
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D

fig = plt.figure()
ax = fig.add_subplot(111, projection='3d')

# 繪製相機座標系
ax.quiver(0,0,0, 0.1,0,0, color='r', label='Camera X')
ax.quiver(0,0,0, 0,0.1,0, color='g', label='Camera Y')
ax.quiver(0,0,0, 0,0,0.1, color='b', label='Camera Z')

plt.show()
```

---

## 📊 效能指標

| 指標 | 數值 |
|------|------|
| 檢測頻率 | 10-20 Hz |
| 檢測延遲 | <100 ms |
| 位置精度 | ±5 mm (標定良好時) |
| 支援顏色數 | 4 (可擴展) |
| 最大障礙物數 | 無限制 (建議 <10) |

---

## 🎓 進階主題

### 1. 結合 CBF 控制器

修改 `koch_cbf_controller.py`:

```python
from avoid_real.pybullet_sync import RealTimeObstacleSynchronizer

# 在 CBFController.__init__ 中添加
self.obstacle_sync = RealTimeObstacleSynchronizer(...)

# 在控制迴圈中
def control_loop(self):
    # 更新真實障礙物
    self.obstacle_sync.update()
    
    # 原有的 CBF 計算
    u = self.compute_cbf_control(...)
```

### 2. 多相機融合

```python
# 使用多個相機
cameras = [
    RealSenseCamera(serial='123456'),
    RealSenseCamera(serial='654321'),
]

# 合併檢測結果
all_obstacles = []
for cam, transform in zip(cameras, transforms):
    obstacles = detector.detect_obstacles(...)
    for obs in obstacles:
        obs.position_world = transform.camera_to_world(obs.position_camera)
    all_obstacles.extend(obstacles)
```

---

## 📝 授權

本專案採用 MIT 授權。

---

## 🤝 貢獻

歡迎提交 Issue 和 Pull Request！

---

## 📧 聯絡方式

如有問題，請在 GitHub 上開啟 Issue。

---

**最後更新**: 2024
