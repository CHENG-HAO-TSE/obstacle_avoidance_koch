#!/usr/bin/env python3
"""
Koch 實體手臂控制器 - 零點校正版
-------------------------------
使用 zero_point_calibration.json 進行校正

校正邏輯：
1. 載入零點 tick 值（手臂在參考姿態時的位置）
2. 接收 PyBullet 的目標角度（相對於參考姿態）
3. 計算：目標 tick = 零點 tick + 角度轉 tick
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from dynamixel_sdk import *
import json
from pathlib import Path
import math

# Dynamixel 設定
DEVICENAME = '/dev/ttyACM0'  # Koch Arm 固定設備路徑
BAUDRATE = 1000000
PROTOCOL_VERSION = 2.0

# Dynamixel 地址
ADDR_TORQUE_ENABLE = 64
ADDR_GOAL_POSITION = 116
ADDR_PRESENT_POSITION = 132

# 馬達 ID
MOTOR_IDS = [1, 2, 3, 4, 5, 6]

class KochRealArmControllerZero(Node):
    def __init__(self):
        super().__init__('koch_real_arm_controller_zero')
        
        self.get_logger().info('=' * 60)
        self.get_logger().info('🤖 Koch 實體手臂控制器 (零點校正版)')
        self.get_logger().info('=' * 60)
        
        # 初始化 Dynamixel
        self.portHandler = PortHandler(DEVICENAME)
        self.packetHandler = PacketHandler(PROTOCOL_VERSION)
        
        # 開啟 port
        if not self.portHandler.openPort():
            self.get_logger().error(f'❌ 無法開啟 port: {DEVICENAME}')
            raise Exception("Failed to open port")
        
        self.get_logger().info(f'✅ 已連接到 {DEVICENAME}')
        
        # 設定 baudrate
        if not self.portHandler.setBaudRate(BAUDRATE):
            self.get_logger().error(f'❌ 無法設定 baudrate: {BAUDRATE}')
            raise Exception("Failed to set baudrate")
        
        self.get_logger().info(f'✅ Baudrate: {BAUDRATE}')
        
        # 載入零點校正（含方向）
        self.zero_positions, self.motor_directions = self.load_zero_calibration()
        
        # 啟用馬達 torque
        self.enable_motors()
        
        # 訂閱關節指令
        self.joint_cmd_sub = self.create_subscription(
            JointState,
            '/follower/joint_states_control',
            self.joint_command_callback,
            10
        )
        
        self.command_count = 0
        
        self.get_logger().info('📡 正在監聽: /follower/joint_states_control')
        self.get_logger().info('=' * 60)
        self.get_logger().info('✅ 準備就緒！')
        self.get_logger().info('=' * 60)
    
    def load_zero_calibration(self):
        """載入零點校正檔案（含方向）"""
        calib_file = Path("/workspace/koch_ros/calibration/zero_point_with_direction.json")
        
        if not calib_file.exists():
            self.get_logger().warn(f'⚠️  找不到校正檔: {calib_file}')
            self.get_logger().warn('   使用預設零點: 2048, 方向: +1')
            return {i: 2048 for i in MOTOR_IDS}, {i: 1 for i in MOTOR_IDS}
        
        try:
            with open(calib_file, 'r') as f:
                data = json.load(f)
            
            zero_positions = {}
            motor_directions = {}
            
            for motor_id_str, tick in data['zero_positions_tick'].items():
                motor_id = int(motor_id_str)
                zero_positions[motor_id] = int(tick)
            
            if 'motor_directions' in data:
                for motor_id_str, direction in data['motor_directions'].items():
                    motor_id = int(motor_id_str)
                    motor_directions[motor_id] = int(direction)
            else:
                # 沒有方向資訊，全部假設正向
                motor_directions = {i: 1 for i in MOTOR_IDS}
            
            self.get_logger().info('✅ 已載入零點校正檔（含方向）')
            self.get_logger().info('🔧 零點位置與方向:')
            for motor_id, tick in zero_positions.items():
                angle_deg = ((tick - 2048) / 4096.0) * 360.0
                direction = motor_directions.get(motor_id, 1)
                dir_str = "正向" if direction == 1 else "反向"
                self.get_logger().info(f'   馬達 {motor_id}: {tick:4d} tick ({angle_deg:+7.2f}°) | {dir_str}')
            
            return zero_positions, motor_directions
            
        except Exception as e:
            self.get_logger().error(f'❌ 載入校正檔失敗: {e}')
            return {i: 2048 for i in MOTOR_IDS}, {i: 1 for i in MOTOR_IDS}
    
    def enable_motors(self):
        """啟用所有馬達的 torque"""
        self.get_logger().info('🔌 正在啟用馬達 torque...')
        
        for motor_id in MOTOR_IDS:
            result, error = self.packetHandler.write1ByteTxRx(
                self.portHandler, motor_id, ADDR_TORQUE_ENABLE, 1
            )
            
            if result != COMM_SUCCESS:
                self.get_logger().warn(f'⚠️  馬達 {motor_id} 啟用失敗')
            else:
                self.get_logger().info(f'   馬達 {motor_id}: ✅')
        
        self.get_logger().info('✅ 所有馬達 torque 已啟用')
    
    def rad_to_tick(self, rad, zero_tick, direction):
        """
        將弧度（相對於參考姿態）轉換為 tick 值
        
        Args:
            rad: 相對於參考姿態的角度變化（弧度）
            zero_tick: 參考姿態的 tick 值
            direction: 馬達方向 (+1 或 -1)
        
        Returns:
            目標 tick 值
        """
        # 弧度 → 度數
        deg = rad * 57.29577951308232
        
        # 度數 → tick 變化量
        delta_tick = int(deg / 360.0 * 4096)
        
        # 目標 tick = 零點 + 方向 × 變化量
        target_tick = zero_tick + direction * delta_tick
        
        # 限制範圍 0-4095
        target_tick = max(0, min(4095, target_tick))
        
        return target_tick
    
    def joint_command_callback(self, msg):
        """接收關節指令並發送到馬達"""
        self.command_count += 1
        
        if len(msg.position) != 6:
            self.get_logger().warn(f'⚠️  指令長度錯誤: {len(msg.position)}')
            return
        
        # 轉換並發送到每個馬達
        for i, target_rad in enumerate(msg.position):
            motor_id = i + 1
            zero_tick = self.zero_positions.get(motor_id, 2048)
            direction = self.motor_directions.get(motor_id, 1)
            
            # 計算目標 tick
            target_tick = self.rad_to_tick(target_rad, zero_tick, direction)
            
            # 發送到馬達
            result, error = self.packetHandler.write4ByteTxRx(
                self.portHandler, motor_id, ADDR_GOAL_POSITION, target_tick
            )
            
            if result != COMM_SUCCESS:
                self.get_logger().warn(
                    f'⚠️  馬達 {motor_id} 寫入失敗: {self.packetHandler.getTxRxResult(result)}'
                )
        
        # 定期顯示狀態
        if self.command_count % 100 == 0:
            self.get_logger().info(f'📊 已接收指令: {self.command_count}')
    
    def disable_motors(self):
        """停用所有馬達的 torque"""
        self.get_logger().info('🔌 正在停用馬達 torque...')
        
        for motor_id in MOTOR_IDS:
            self.packetHandler.write1ByteTxRx(
                self.portHandler, motor_id, ADDR_TORQUE_ENABLE, 0
            )
        
        self.get_logger().info('✅ 所有馬達 torque 已停用')
    
    def __del__(self):
        """清理資源"""
        try:
            self.disable_motors()
            self.portHandler.closePort()
        except:
            pass

def main(args=None):
    rclpy.init(args=args)
    
    try:
        controller = KochRealArmControllerZero()
        rclpy.spin(controller)
    except KeyboardInterrupt:
        print('\n⏹️  使用者中斷')
    except Exception as e:
        print(f'❌ 錯誤: {e}')
    finally:
        if rclpy.ok():
            rclpy.shutdown()

if __name__ == '__main__':
    main()
