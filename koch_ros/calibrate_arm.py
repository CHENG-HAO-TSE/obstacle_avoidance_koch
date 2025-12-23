#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Koch 手臂校正工具
用於記錄真實手臂在 URDF 初始位置時的馬達數值

使用步驟：
1. 手動將真實手臂擺放到 URDF 中定義的初始姿態
2. 運行此腳本讀取當前馬達位置
3. 自動生成校正檔案
"""

import sys
from dynamixel_sdk import *
import json
import os

# Dynamixel 設定
DEVICE_PORT = '/dev/ttyACM0'
BAUDRATE = 1000000
PROTOCOL_VERSION = 2.0

# 地址
ADDR_PRESENT_POSITION = 132  # Protocol 2.0
ADDR_TORQUE_ENABLE = 64

# 馬達 ID
MOTOR_IDS = [1, 2, 3, 4, 5, 6]
JOINT_NAMES = ['joint1', 'joint2', 'joint3', 'joint4', 'joint5', 'joint_gripper']

# URDF 初始位置（弧度）- 手臂完全向前伸直
# 這個姿態容易擺放和校正
URDF_INITIAL_POSITIONS = {
    'joint1': 0.0,    # 0°   - 基座朝前
    'joint2': 1.57,   # 90°  - 水平向前
    'joint3': 3.14,   # 180° - 完全伸直
    'joint4': 0.0,    # 0°   - 手腕水平
    'joint5': 0.0,    # 0°   - 不旋轉
    'joint_gripper': 0.5  # 29° - 夾爪微開
}

def read_motor_position(port_handler, packet_handler, motor_id):
    """讀取馬達當前位置"""
    dxl_present_position, dxl_comm_result, dxl_error = packet_handler.read4ByteTxRx(
        port_handler, motor_id, ADDR_PRESENT_POSITION
    )
    
    if dxl_comm_result != COMM_SUCCESS:
        print(f"[ERROR] 讀取馬達 {motor_id} 失敗: {packet_handler.getTxRxResult(dxl_comm_result)}")
        return None
    elif dxl_error != 0:
        print(f"[WARN] 馬達 {motor_id} 錯誤: {packet_handler.getRxPacketError(dxl_error)}")
        return None
    
    return dxl_present_position

def position_to_radians(position_value):
    """將 Dynamixel 位置值轉換為弧度"""
    # 0-4095 對應 -π 到 π
    return (position_value / 4095.0) * 2.0 * 3.14159265359 - 3.14159265359

def main():
    print("=" * 60)
    print("🔧 Koch 手臂校正工具")
    print("=" * 60)
    print()
    print("📋 URDF 初始位置（目標姿態）：")
    for name, pos in URDF_INITIAL_POSITIONS.items():
        print(f"  {name:15s}: {pos:7.3f} rad = {pos*180/3.14159:7.2f}°")
    print()
    print("⚠️  請按照以下步驟操作：")
    print()
    print("步驟 1: 手動將真實手臂擺放到上述初始姿態")
    print("       （可以參考 PyBullet 虛擬手臂的初始姿態）")
    print()
    print("步驟 2: 確認手臂位置正確後，按 Enter 繼續...")
    print()
    
    input("按 Enter 開始讀取馬達位置...")
    print()
    
    # 初始化 Dynamixel
    print("🔌 連接到馬達...")
    port_handler = PortHandler(DEVICE_PORT)
    packet_handler = PacketHandler(PROTOCOL_VERSION)
    
    if not port_handler.openPort():
        print(f"[ERROR] 無法打開串口: {DEVICE_PORT}")
        print("請檢查：")
        print("  1. USB 線是否連接")
        print("  2. 設備權限: sudo chmod 666 /dev/ttyACM0")
        return 1
    
    if not port_handler.setBaudRate(BAUDRATE):
        print(f"[ERROR] 無法設定波特率: {BAUDRATE}")
        return 1
    
    print("✅ 已連接到馬達")
    print()
    
    # 讀取所有馬達位置
    print("📡 讀取馬達位置...")
    print()
    
    motor_positions = {}
    calibration_offsets = {}
    
    print(f"{'關節':<15} {'馬達ID':<8} {'原始值':<10} {'弧度':<12} {'URDF目標':<12} {'偏移量':<12}")
    print("-" * 80)
    
    for motor_id, joint_name in zip(MOTOR_IDS, JOINT_NAMES):
        position_value = read_motor_position(port_handler, packet_handler, motor_id)
        
        if position_value is None:
            print(f"[ERROR] 無法讀取馬達 {motor_id}")
            continue
        
        # 轉換為弧度
        position_rad = position_to_radians(position_value)
        
        # 計算偏移量（URDF目標 - 當前實際）
        urdf_target = URDF_INITIAL_POSITIONS[joint_name]
        offset = urdf_target - position_rad
        
        motor_positions[joint_name] = position_value
        calibration_offsets[joint_name] = offset
        
        print(f"{joint_name:<15} {motor_id:<8} {position_value:<10} {position_rad:>10.3f} {urdf_target:>10.3f} {offset:>10.3f}")
    
    print()
    print("=" * 60)
    
    # 詢問是否保存
    print()
    print("是否保存校正檔案？")
    save = input("輸入 'yes' 或 'y' 保存: ").lower()
    
    if save in ['yes', 'y']:
        # 創建校正檔案
        calibration_dir = '/workspace/koch_ros/calibration'
        os.makedirs(calibration_dir, exist_ok=True)
        
        calibration_file = os.path.join(calibration_dir, 'follower_calibration.json')
        
        calibration_data = {
            'robot_name': 'follower',
            'calibration_date': 'manual',
            'motor_ids': MOTOR_IDS,
            'joint_names': JOINT_NAMES,
            'urdf_initial_positions': URDF_INITIAL_POSITIONS,
            'motor_raw_positions': motor_positions,
            'calibration_offsets': calibration_offsets,
            'notes': '手動校正：將手臂擺放到 URDF 初始姿態並記錄馬達值'
        }
        
        with open(calibration_file, 'w') as f:
            json.dump(calibration_data, f, indent=2)
        
        print(f"✅ 校正檔案已保存: {calibration_file}")
        print()
        print("📝 校正偏移量：")
        for joint_name, offset in calibration_offsets.items():
            print(f"  {joint_name}: {offset:7.3f} rad = {offset*180/3.14159:7.2f}°")
        print()
        print("🎯 下一步：")
        print("  1. 重啟實體控制節點，它會自動加載校正檔案")
        print("  2. 啟動虛擬控制節點")
        print("  3. 虛擬和實體手臂應該會同步了")
    else:
        print("❌ 未保存校正檔案")
    
    # 關閉串口
    port_handler.closePort()
    print()
    print("✅ 完成")

if __name__ == '__main__':
    try:
        sys.exit(main())
    except KeyboardInterrupt:
        print("\n\n⚠️  校正已取消")
        sys.exit(1)
    except Exception as e:
        print(f"\n[ERROR] {e}")
        import traceback
        traceback.print_exc()
        sys.exit(1)
