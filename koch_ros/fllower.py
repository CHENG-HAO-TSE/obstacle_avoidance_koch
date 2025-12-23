#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Koch Follower 控制節點（Relay/外部控制用；新版 LeRobot API）
- 只管理 Follower 手臂
- 發佈:    /<arm>/joint_states          (sensor_msgs/JointState, 弧度)
- 訂閱:    /<arm>/joint_states_control  (sensor_msgs/JointState, 弧度) → 寫入 Goal_Position(度)
- 服務:    /<arm>_reset_to_home         (std_srvs/Trigger) → 依校正 homing_offset 回原點
- 啟用校正 (若 calibration JSON 存在會自動讀取)
啟動範例:
ros2 run koch_ros2_wrapper koch_follower --ros-args \
  -p config_file:=/path/to/single_follower.yaml \
  -p calibration_dir:=/path/to/calibration/koch \
  -p publish_rate:=50.0
"""
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_srvs.srv import Trigger
# LeRobot 新 API
from lerobot.motors.dynamixel import DynamixelMotorsBus
from lerobot.motors.motors_bus import Motor, MotorNormMode, MotorCalibration
import numpy as np
import yaml
import json
import os
class KochFollower(Node):
    def __init__(self):
        super().__init__('follower_control_node')
        self.get_logger().info(':機器人: Follower Control Node (new API) started')
        # 參數
        self.declare_parameter(
            'config_file',
            '/home/hrc/koch_robot_arm/ros2_ws/src/koch_ros2_wrapper/config/single_follower.yaml'
        )
        self.declare_parameter('calibration_dir', '/home/hrc/koch_robot_arm/calibration/koch')
        self.declare_parameter('publish_rate', 50.0)  # Hz
        config_file_path = self.get_parameter('config_file').get_parameter_value().string_value
        self.calibration_dir = self.get_parameter('calibration_dir').get_parameter_value().string_value
        publish_rate = self.get_parameter('publish_rate').get_parameter_value().double_value
        # 載入設定
        try:
            with open(config_file_path, 'r') as f:
                config = yaml.safe_load(f)
            self.get_logger().info(f':白色勾勾: Loaded configuration: {config_file_path}')
        except Exception as e:
            self.get_logger().error(f':x: Failed to load configuration: {e}')
            raise
        # 結構
        self.follower_arms = {}             # name -> DynamixelMotorsBus
        self.joint_state_publishers = {}    # name -> Publisher
        self.joint_state_subscribers = {}   # name -> Subscriber
        self.reset_home_services = {}       # name -> Service
        self.homing_deg = {}                # name -> dict[motor_name] = homing_offset(deg)
        # 初始化每支 follower arm
        for arm_cfg in config.get('follower_arms', []):
            arm_name = arm_cfg['name']
            port = arm_cfg['port']
            motors_cfg = arm_cfg['motors']
            self.get_logger().info(f':板手: Initializing follower arm: {arm_name} on {port}')
            # 建立 Motor 物件（用度數）
            motors = {}
            for mname, spec in motors_cfg.items():
                motor_id, motor_model = spec
                motors[mname] = Motor(
                    id=motor_id,
                    model=motor_model,
                    norm_mode=MotorNormMode.DEGREES
                )
            # 載入校正（同時取回 homing offsets）
            calibration, homing_map = self._load_calibration(arm_name)
            self.homing_deg[arm_name] = homing_map  # 可能為 {}
            # 建立 bus
            bus = DynamixelMotorsBus(
                port=port,
                motors=motors,
                calibration=calibration
            )
            self.follower_arms[arm_name] = bus
            # ROS 介面（絕對 topic）
            state_topic = f'/{arm_name}/joint_states'
            cmd_topic = f'/{arm_name}/joint_states_control'
            self.joint_state_publishers[arm_name] = self.create_publisher(JointState, state_topic, 10)
            self.joint_state_subscribers[arm_name] = self.create_subscription(
                JointState,
                cmd_topic,
                lambda msg, name=arm_name: self._cb_joint_state_command(msg, name),
                10
            )
            self.reset_home_services[arm_name] = self.create_service(
                Trigger,
                f'/{arm_name}_reset_to_home',
                lambda req, res, name=arm_name: self._cb_reset_home(req, res, name)
            )
            self.get_logger().info(f':衛星天線: Publishing: {state_topic}')
            self.get_logger().info(f':衛星天線: Subscribing: {cmd_topic}')
            self.get_logger().info(f':服務鈴:  Service: /{arm_name}_reset_to_home')
        # 連線並啟用扭矩（Follower 需要 hold）
        for arm_name, bus in self.follower_arms.items():
            try:
                bus.connect()
                bus.enable_torque()
                self.get_logger().info(f'  :白色勾勾: Follower {arm_name} connected (torque enabled)')
            except Exception as e:
                self.get_logger().error(f':x: Failed to connect follower {arm_name}: {e}')
        # 只發佈狀態（無自動鏡像）
        self.timer = self.create_timer(1.0 / publish_rate, self._publish_loop)
        self.get_logger().info(f':碼表:  Publish rate: {publish_rate} Hz')
    # ---- 校正載入：回傳 (calibration_dict, homing_deg_map) ----
    def _load_calibration(self, arm_name):
        """
        讀取校正 JSON 並轉成:
          - calibration: dict[str, MotorCalibration] | None（給 LeRobot）
          - homing_deg: dict[str, float]（我們自己存，單位: 度）
        路徑: <calibration_dir>/follower/<arm_name>/<arm_name}_calibration.json
        """
        calib_path = os.path.join(
            self.calibration_dir, 'follower', arm_name, f'{arm_name}_calibration.json'
        )
        if not os.path.exists(calib_path):
            self.get_logger().warning(f':警告:  No calibration file: {calib_path}')
            return None, {}
        try:
            with open(calib_path, 'r') as f:
                raw = json.load(f)
            calibration = {}
            homing_deg = {}
            for mname, md in raw.get('motors', {}).items():
                # 假設 homing_offset 以「度」儲存（建議校正檔就用度）
                homing = float(md.get('homing_offset', 0.0))
                homing_deg[mname] = homing
                calibration[mname] = MotorCalibration(
                    id=md.get('index', 0),
                    drive_mode=md.get('drive_mode', 0),
                    homing_offset=homing,            # 與 norm_mode=DEGREES 一致
                    range_min=md.get('range_min', 0),
                    range_max=md.get('range_max', 4095),
                )
            self.get_logger().info(f'  :直條圖: Loaded calibration for {arm_name} from {calib_path}')
            return calibration, homing_deg
        except Exception as e:
            self.get_logger().error(f':x: Failed to load calibration for {arm_name}: {e}')
            return None, {}
    # ---- 發佈迴圈（Follower 狀態）----
    def _publish_loop(self):
        for arm_name, bus in self.follower_arms.items():
            try:
                pos_deg_dict = bus.sync_read("Present_Position")  # dict[name]->deg
                names = list(pos_deg_dict.keys())
                pos_deg = [pos_deg_dict[n] for n in names]
                pos_rad = np.radians(pos_deg)
                msg = JointState()
                msg.header.stamp = self.get_clock().now().to_msg()
                msg.name = [f"{arm_name}_{n}" for n in names]  # 加前綴，便於下游對齊
                msg.position = pos_rad.tolist()
                self.joint_state_publishers[arm_name].publish(msg)
            except Exception as e:
                self.get_logger().debug(f'Follower {arm_name} read error: {e}')
    # ---- 指令 Callback（寫入目標角度）----
    def _cb_joint_state_command(self, msg, arm_name):
        """
        期望輸入:
        - msg.position 為弧度
        - 若 msg.name 與本地關節名一致，優先用名稱對齊；否則退回用順序對齊
        """
        try:
            bus = self.follower_arms[arm_name]
            target_deg = {}
            # 名稱對齊（建議上游附 name，避免對不到順序）
            if msg.name and len(msg.name) == len(msg.position):
                for name, pos_rad in zip(msg.name, msg.position):
                    # 允許上游用 "<arm>_<joint>" 或純 "<joint>"
                    joint_local = name.split(f"{arm_name}_")[-1] if name.startswith(f"{arm_name}_") else name
                    if joint_local in bus.motors:
                        target_deg[joint_local] = float(np.degrees(pos_rad))
            else:
                # 退回順序對齊
                positions_deg = np.degrees(np.array(msg.position))
                for i, mname in enumerate(list(bus.motors.keys())[:len(positions_deg)]):
                    target_deg[mname] = float(positions_deg[i])
            if not target_deg:
                self.get_logger().warning(f':警告: No matching joints in command for {arm_name}')
                return
            # TODO: 可在此加入夾限/速率限制/濾波（依校正的 range_min/max）
            bus.sync_write("Goal_Position", target_deg)
        except Exception as e:
            self.get_logger().error(f':x: Command callback error for {arm_name}: {e}')
    # ---- 回原點服務（依校正 homing_offset）----
    def _cb_reset_home(self, request, response, arm_name):
        try:
            bus = self.follower_arms[arm_name]
            homing_map = self.homing_deg.get(arm_name, {})
            targets = {}
            for mname in bus.motors.keys():
                deg = float(homing_map.get(mname, 0.0))  # 無校正則退回 0°
                targets[mname] = deg
            bus.sync_write("Goal_Position", targets)
            response.success = True
            response.message = f'{arm_name}: moved to homing offsets (deg)'
            self.get_logger().info(f':房屋: {response.message}')
        except Exception as e:
            response.success = False
            response.message = f'{arm_name}: failed to move to homing ({e})'
            self.get_logger().error(f':x: {response.message}')
        return response
    # ---- 收尾 ----
    def destroy_node(self):
        self.get_logger().info(':停止標誌: Shutting down follower node...')
        for arm_name, bus in self.follower_arms.items():
            try:
                bus.disable_torque()
            except Exception:
                pass
            try:
                bus.disconnect()
                self.get_logger().info(f'    :白色勾勾: Follower {arm_name} disconnected')
            except Exception as e:
                self.get_logger().error(f'    :x: Disconnect error on {arm_name}: {e}')
        super().destroy_node()
def main(args=None):
    rclpy.init(args=args)
    node = KochFollower()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if 'node' in locals():
            node.destroy_node()
        rclpy.shutdown()
if __name__ == '__main__':
    main()
9:39
#!/usr/bin/env python3
"""
視覺檢測節點：整合 YOLO 和 RealSense 深度信息
用於檢測胡蘿蔔並輸出 3D 空間座標
支持壓縮圖像格式以節省帶寬
"""
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, CompressedImage, CameraInfo
from geometry_msgs.msg import PointStamped, Point
from std_msgs.msg import Header
from cv_bridge import CvBridge
import cv2
import numpy as np
from ultralytics import YOLO
import os
class CarrotDetectionNode(Node):
    def __init__(self):
        super().__init__('carrot_detection_node')
        # 參數
        self.declare_parameter('yolo_model_path', '/home/hrc/Koch_IL/model/yolo/carrot/best.pt')
        self.declare_parameter('confidence_threshold', 0.5)
        self.declare_parameter('depth_scale', 0.001)  # 深度單位轉換 (mm to m)
        self.declare_parameter('use_compressed', True)  # 是否使用壓縮圖像
        self.declare_parameter('jpeg_quality', 80)  # JPEG 壓縮品質 (1-100)
        yolo_model_path = self.get_parameter('yolo_model_path').value
        self.confidence_threshold = self.get_parameter('confidence_threshold').value
        self.depth_scale = self.get_parameter('depth_scale').value
        self.use_compressed = self.get_parameter('use_compressed').value
        self.jpeg_quality = self.get_parameter('jpeg_quality').value
        # 初始化 YOLO 模型
        self.get_logger().info(f'Loading YOLO model from: {yolo_model_path}')
        if not os.path.exists(yolo_model_path):
            self.get_logger().error(f'YOLO model not found at: {yolo_model_path}')
            raise FileNotFoundError(f'YOLO model not found at: {yolo_model_path}')
        self.yolo_model = YOLO(yolo_model_path)
        self.get_logger().info(':白色勾勾: YOLO model loaded successfully')
        # CV Bridge
        self.bridge = CvBridge()
        # 相機內參 (將從 camera_info 更新)
        self.camera_matrix = None
        self.dist_coeffs = None
        # 緩存最新的深度圖像
        self.latest_depth_image = None
        self.depth_image_lock = False
        # 發布頻率控制（避免過於頻繁發布）
        self.last_publish_time = None
        self.publish_interval = 1  # 每 1 秒最多發布一次檢測結果
        # 訂閱者 - 根據參數選擇壓縮或原始圖像
        if self.use_compressed:
            self.get_logger().info(':相機: Using compressed image format (saves bandwidth)')
            self.color_sub = self.create_subscription(
                CompressedImage,
                '/camera/camera_top/color/image_raw/compressed',
                self.compressed_callback,
                10
            )
        else:
            self.get_logger().info(':相機: Using raw image format')
            self.color_sub = self.create_subscription(
                Image,
                '/camera/camera_top/color/image_raw',
                self.color_callback,
                10
            )
        self.depth_sub = self.create_subscription(
            Image,
            '/camera/camera_top/aligned_depth_to_color/image_raw',
            self.depth_callback,
            10
        )
        self.camera_info_sub = self.create_subscription(
            CameraInfo,
            '/camera/camera_top/color/camera_info',
            self.camera_info_callback,
            10
        )
        # 發布者 - 使用壓縮圖像格式
        self.detection_viz_pub = self.create_publisher(
            CompressedImage,
            '/carrot_detection/image_compressed',
            10
        )
        self.position_3d_pub = self.create_publisher(
            PointStamped,
            '/carrot_detection/position_3d',
            10
        )
        self.get_logger().info(':火箭: Carrot Detection Node started')
        self.get_logger().info(':相機: Waiting for camera data...')
    def camera_info_callback(self, msg):
        """獲取相機內參"""
        if self.camera_matrix is None:
            self.camera_matrix = np.array(msg.k).reshape(3, 3)
            self.dist_coeffs = np.array(msg.d)
            self.get_logger().info(':白色勾勾: Camera intrinsics received')
            self.get_logger().info(f'Camera Matrix:\n{self.camera_matrix}')
    def depth_callback(self, msg):
        """接收深度圖像"""
        if not self.depth_image_lock:
            try:
                # 轉換深度圖像 (通常是 16-bit, 單位 mm)
                self.latest_depth_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='passthrough')
            except Exception as e:
                self.get_logger().error(f'Failed to convert depth image: {e}')
    def get_3d_position(self, u, v, depth_image):
        """
        從 2D 像素座標和深度值計算 3D 空間座標
        Args:
            u, v: 像素座標
            depth_image: 深度圖像
        Returns:
            x, y, z: 相機座標系中的 3D 座標 (單位: 米)
        """
        if self.camera_matrix is None:
            return None, None, None
        # 獲取深度值 (以像素為中心的小區域取平均，更穩定)
        h, w = depth_image.shape
        u, v = int(u), int(v)
        # 確保座標在圖像範圍內
        if u < 0 or u >= w or v < 0 or v >= h:
            return None, None, None
        # 取 5x5 區域的中位數深度
        half_size = 2
        u_min = max(0, u - half_size)
        u_max = min(w, u + half_size + 1)
        v_min = max(0, v - half_size)
        v_max = min(h, v + half_size + 1)
        depth_region = depth_image[v_min:v_max, u_min:u_max]
        depth_region = depth_region[depth_region > 0]  # 過濾無效深度值
        if len(depth_region) == 0:
            return None, None, None
        depth = np.median(depth_region) * self.depth_scale  # 轉換為米
        # 相機內參
        fx = self.camera_matrix[0, 0]
        fy = self.camera_matrix[1, 1]
        cx = self.camera_matrix[0, 2]
        cy = self.camera_matrix[1, 2]
        # 計算 3D 座標 (相機座標系)
        x = (u - cx) * depth / fx
        y = (v - cy) * depth / fy
        z = depth
        return x, y, z
    def compressed_callback(self, msg):
        """處理壓縮圖像並執行檢測"""
        try:
            # 解壓縮圖像
            np_arr = np.frombuffer(msg.data, np.uint8)
            cv_image = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)
            # 調用檢測邏輯
            self.process_detection(cv_image)
        except Exception as e:
            self.get_logger().error(f'Error in compressed callback: {e}')
            import traceback
            self.get_logger().error(traceback.format_exc())
    def color_callback(self, msg):
        """處理彩色圖像並執行檢測"""
        try:
            # 轉換 ROS Image 到 OpenCV
            cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
            # 調用檢測邏輯
            self.process_detection(cv_image)
        except Exception as e:
            self.get_logger().error(f'Error in color callback: {e}')
            import traceback
            self.get_logger().error(traceback.format_exc())
    def process_detection(self, cv_image):
        """統一的檢測處理邏輯"""
        try:
            # YOLO 檢測
            results = self.yolo_model(cv_image, conf=self.confidence_threshold, verbose=False)
            # 處理檢測結果
            detections = []
            for result in results:
                boxes = result.boxes
                for box in boxes:
                    # 獲取檢測框信息
                    x1, y1, x2, y2 = box.xyxy[0].cpu().numpy()
                    conf = box.conf[0].cpu().numpy()
                    cls = int(box.cls[0].cpu().numpy())
                    # 計算中心點
                    center_u = int((x1 + x2) / 2)
                    center_v = int((y1 + y2) / 2)
                    detections.append({
                        'bbox': (int(x1), int(y1), int(x2), int(y2)),
                        'center': (center_u, center_v),
                        'confidence': float(conf),
                        'class': cls
                    })
                    # 繪製檢測框
                    cv2.rectangle(cv_image, (int(x1), int(y1)), (int(x2), int(y2)), (0, 255, 0), 2)
                    cv2.circle(cv_image, (center_u, center_v), 5, (0, 0, 255), -1)
                    label = f'Carrot {conf:.2f}'
                    cv2.putText(cv_image, label, (int(x1), int(y1) - 10),
                              cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)
                    # 如果有深度圖像，計算 3D 位置
                    if self.latest_depth_image is not None:
                        x, y, z = self.get_3d_position(center_u, center_v, self.latest_depth_image)
                        if x is not None:
                            # 檢查發布頻率（避免過於頻繁）
                            current_time = self.get_clock().now()
                            should_publish = False
                            if self.last_publish_time is None:
                                should_publish = True
                            else:
                                time_diff = (current_time - self.last_publish_time).nanoseconds / 1e9
                                if time_diff >= self.publish_interval:
                                    should_publish = True
                            if should_publish:
                                # 發布 3D 位置
                                point_msg = PointStamped()
                                point_msg.header = Header()
                                point_msg.header.stamp = current_time.to_msg()
                                point_msg.header.frame_id = 'camera_top_color_optical_frame'
                                point_msg.point = Point(x=x, y=y, z=z)
                                self.position_3d_pub.publish(point_msg)
                                self.last_publish_time = current_time
                                self.get_logger().info(f':胡蘿蔔: Carrot detected at 3D position: '
                                                     f'x={x:.3f}m, y={y:.3f}m, z={z:.3f}m, conf={conf:.2f}')
                            # 在圖像上顯示 3D 座標（無論是否發布都顯示）
                            pos_text = f'3D: ({x:.3f}, {y:.3f}, {z:.3f})m'
                            cv2.putText(cv_image, pos_text, (int(x1), int(y2) + 20),
                                      cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 0, 0), 2)
            # 顯示檢測數量
            cv2.putText(cv_image, f'Detections: {len(detections)}', (10, 30),
                       cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 255), 2)
            # 壓縮並發布檢測結果圖像
            encode_param = [int(cv2.IMWRITE_JPEG_QUALITY), self.jpeg_quality]
            success, encoded_img = cv2.imencode('.jpg', cv_image, encode_param)
            if success:
                compressed_msg = CompressedImage()
                compressed_msg.header = Header()
                compressed_msg.header.stamp = self.get_clock().now().to_msg()
                compressed_msg.format = 'jpeg'
                compressed_msg.data = encoded_img.tobytes()
                self.detection_viz_pub.publish(compressed_msg)
        except Exception as e:
            self.get_logger().error(f'Error in color callback: {e}')
            import traceback
            self.get_logger().error(traceback.format_exc())
def main(args=None):
    rclpy.init(args=args)
    node = CarrotDetectionNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()
if __name__ == '__main__':
    main()