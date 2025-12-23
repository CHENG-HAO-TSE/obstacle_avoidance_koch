#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Koch 校正節點 - URDF 對齊版 (修正版)
--------------------------------
修正：自動偵測 MotorNormMode (TICKS/RAW) 避免 AttributeError
邏輯：
1. 位置 1 (伸直) -> 設定為 0 (Home)
2. 位置 2 (90度) -> 確認方向與刻度 (Align URDF)
"""

import sys
import time
import json
from pathlib import Path

import rclpy
from rclpy.node import Node
from std_srvs.srv import Trigger
import yaml
import select

# LeRobot 新 API
from lerobot.motors.dynamixel import DynamixelMotorsBus
from lerobot.motors.motors_bus import Motor, MotorNormMode

TICKS_PER_REV = 4096  # Dynamixel 一圈刻度
TICKS_90_DEG = int(TICKS_PER_REV / 4) # 90度約為 1024 ticks

class KochCalibrationURDF(Node):
    def __init__(self):
        super().__init__('koch_calibration_urdf')
        self.get_logger().info('🎯 Koch URDF-Alignment Calibration Node Started')

        # 參數設定
        self.declare_parameter('config_file', '/home/hrc/Koch_IL/ros2_ws/src/koch_ros2_wrapper/config/single_follower.yaml')
        self.declare_parameter('calibration_dir', '/home/hrc/Koch_IL/calibration/koch')
        self.declare_parameter('write_to_device', False)   # True: 寫入馬達 EEPROM; False: 僅輸出 JSON

        config_file_path = self.get_parameter('config_file').get_parameter_value().string_value
        self.calibration_dir = Path(self.get_parameter('calibration_dir').get_parameter_value().string_value)
        self.write_to_device = self.get_parameter('write_to_device').get_parameter_value().bool_value

        # 載入配置
        try:
            with open(config_file_path, 'r') as f:
                self.config = yaml.safe_load(f)
            self.get_logger().info(f'✅ Loaded config: {config_file_path}')
        except Exception as e:
            self.get_logger().error(f'❌ Failed to load config: {e}')
            raise

        # 初始化手臂
        self.leader_arms = {}
        self.follower_arms = {}
        self.initialize_arms()

        # 啟動校正
        self.start_calibration()

    def initialize_arms(self):
        # --- 修正開始: 自動偵測可用模式 ---
        # 不同的 lerobot 版本對 Raw/Ticks 的命名可能不同
        if hasattr(MotorNormMode, 'TICKS'):
            norm_mode = MotorNormMode.TICKS
        elif hasattr(MotorNormMode, 'RAW'):
            norm_mode = MotorNormMode.RAW
        else:
            # 如果都沒有，退回 DEGREES，但在讀取時我們依然會用 normalize=False 嘗試拿原始值
            norm_mode = MotorNormMode.DEGREES
        
        self.get_logger().info(f'⚙️  Motor Mode selected: {norm_mode}')
        # --- 修正結束 ---

        def build_motors(motors_cfg):
            d = {}
            for name, spec in motors_cfg.items():
                motor_id, model = spec
                d[name] = Motor(id=motor_id, model=model, norm_mode=norm_mode)
            return d

        for arm in self.config.get('leader_arms', []):
            name, port, motors_cfg = arm['name'], arm['port'], arm['motors']
            self.leader_arms[name] = DynamixelMotorsBus(port=port, motors=build_motors(motors_cfg))

        for arm in self.config.get('follower_arms', []):
            name, port, motors_cfg = arm['name'], arm['port'], arm['motors']
            self.follower_arms[name] = DynamixelMotorsBus(port=port, motors=build_motors(motors_cfg))

    def wait_for_enter(self):
        """等待使用者按下 Enter"""
        while True:
            if select.select([sys.stdin], [], [], 0) == ([sys.stdin], [], []):
                _ = input()
                return
            time.sleep(0.05)

    def start_calibration(self):
        self.get_logger().info('\n' + '='*60)
        self.get_logger().info('🎯 開始 URDF 對齊校正流程')
        self.get_logger().info('='*60)

        for name, bus in self.leader_arms.items():
            self.calibrate_arm(bus, name, 'leader')

        for name, bus in self.follower_arms.items():
            self.calibrate_arm(bus, name, 'follower')

        self.get_logger().info('\n✅ 所有手臂校正完成！')

    def calibrate_arm(self, motor_bus: DynamixelMotorsBus, arm_name: str, arm_type: str):
        self.get_logger().info(f'\n📍 正在校正手臂: {arm_name} ({arm_type})')
        
        try:
            motor_bus.connect()
            motor_bus.disable_torque()
            
            # 取得可用馬達
            motor_names = list(motor_bus.motors.keys())
            if not motor_names:
                self.get_logger().error('❌ 找不到馬達')
                return

            # ---------------------------------------------------------
            # 步驟 1: 定義 0 點 (伸直)
            # ---------------------------------------------------------
            self.get_logger().info('\n➡️  步驟 1/2: 歸零設定')
            self.get_logger().info('   請將手臂完全「伸直」，這將被定義為 0 度位置。')
            self.get_logger().info('   完成後請按 [ENTER]...')
            self.wait_for_enter()

            zero_positions = {}
            self.get_logger().info('📸 讀取 0 度位置 (Raw Ticks)...')
            for m in motor_names:
                # normalize=False 強制讀取原始 ticks，不受 norm_mode 影響
                val = motor_bus.read("Present_Position", m, normalize=False)
                zero_positions[m] = val
                self.get_logger().info(f'   {m}: {val} (設為零點)')

            # ---------------------------------------------------------
            # 步驟 2: 確認方向 (90度)
            # ---------------------------------------------------------
            self.get_logger().info('\n➡️  步驟 2/2: URDF 方向對齊')
            self.get_logger().info('   請將所有關節轉動到「+90 度」位置 (垂直於伸直方向)。')
            self.get_logger().info('   (請依照您 URDF 定義的正方向轉動)')
            self.get_logger().info('   完成後請按 [ENTER]...')
            self.wait_for_enter()

            ninety_positions = {}
            
            self.get_logger().info('📸 讀取 90 度位置並分析...')
            for m in motor_names:
                val_90 = motor_bus.read("Present_Position", m, normalize=False)
                val_0 = zero_positions[m]
                delta = val_90 - val_0
                
                ninety_positions[m] = val_90
                
                # 分析方向
                is_positive = delta > 0
                error_margin = abs(abs(delta) - TICKS_90_DEG)
                
                status_msg = ""
                if abs(delta) < 100:
                    status_msg = "⚠️ 沒動? (Delta too small)"
                elif is_positive:
                    status_msg = f"✅ 正向旋轉 (Delta: +{delta})"
                else:
                    status_msg = f"🔄 反向旋轉 (Delta: {delta}) -> 建議檢查 URDF 或 Drive Mode"

                self.get_logger().info(f'   {m}: {val_0} -> {val_90} | {status_msg}')
                
                # 簡單檢查誤差 (Dynamixel 90度約 1024 ticks)
                if error_margin > 200 and abs(delta) > 100:
                    self.get_logger().warn(f'      ⚠️ 角度誤差較大 (預期變化 ~1024, 實際 {abs(delta)})')

            # ---------------------------------------------------------
            # 寫入設定
            # ---------------------------------------------------------
            calibration = motor_bus.read_calibration()
            
            for m in motor_names:
                # 核心邏輯：Homing Offset = 伸直時的 Raw 值
                calibration[m].homing_offset = int(zero_positions[m])
                
                # 設定範圍：給予對稱的寬鬆範圍 (例如 +/- 半圈)
                safe_margin = 2048 # 半圈
                calibration[m].range_min = int(zero_positions[m] - safe_margin)
                calibration[m].range_max = int(zero_positions[m] + safe_margin)

            # 匯出
            if self.write_to_device:
                self.get_logger().info('💾 正在寫入馬達 EEPROM...')
                motor_bus.write_calibration(calibration)
            else:
                self.get_logger().info('📝 僅輸出 JSON (未寫入馬達)')

            self._export_to_json(motor_bus, arm_name, arm_type, calibration)
            
            # 恢復
            try:
                motor_bus.enable_torque()
            except:
                pass
            motor_bus.disconnect()

        except Exception as e:
            self.get_logger().error(f'❌ 校正失敗: {e}')
            # 嘗試斷開連線
            try:
                if motor_bus.is_connected:
                    motor_bus.disconnect()
            except:
                pass

    def _export_to_json(self, motor_bus, arm_name, arm_type, calibration_obj):
        calib_dir = self.calibration_dir / arm_type / arm_name
        calib_dir.mkdir(parents=True, exist_ok=True)

        data = {
            "uid": arm_name,
            "timestamp": time.strftime("%Y-%m-%d %H:%M:%S"),
            "method": "straight_is_zero_urdf_aligned",
            "motors": {}
        }
        for m, c in calibration_obj.items():
            motor = motor_bus.motors[m]
            data["motors"][m] = {
                "motor_id": getattr(motor, 'id', None),
                "homing_offset": int(getattr(c, 'homing_offset', 0)),
                "range_min": int(getattr(c, 'range_min', 0)),
                "range_max": int(getattr(c, 'range_max', 0)),
                "note": "Homing offset set to raw value at STRAIGHT position"
            }

        out = calib_dir / f"{arm_name}_calibration.json"
        with open(out, 'w') as f:
            json.dump(data, f, indent=2)
        self.get_logger().info(f'💾 校正檔已儲存: {out}')

def main(args=None):
    rclpy.init(args=args)
    try:
        KochCalibrationURDF()
    except KeyboardInterrupt:
        pass
    finally:
        if rclpy.ok():
            rclpy.shutdown()

if __name__ == '__main__':
    main()