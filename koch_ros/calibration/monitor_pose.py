#!/usr/bin/env python3
"""
即時監控手臂姿態 - 幫助校正時擺位
"""

import time
from dynamixel_sdk import *
import math

DEVICENAME = '/dev/ttyACM0'
BAUDRATE = 1000000
PROTOCOL_VERSION = 2.0
MOTOR_IDS = [1, 2, 3, 4, 5, 6]
ADDR_PRESENT_POSITION = 132

# 目標姿態（完全伸直）
TARGET_POSE_DEG = {
    1: 0.0,      # 基座朝前
    2: 90.0,     # 肩部水平
    3: 180.0,    # 肘部完全伸直
    4: 0.0,      # 腕部直線
    5: 0.0,      # 不旋轉
    6: 28.65     # 夾爪微開
}

JOINT_NAMES = {
    1: '基座旋轉',
    2: '肩部俯仰',
    3: '肘部俯仰',
    4: '腕部俯仰',
    5: '腕部旋轉',
    6: '夾爪開合'
}

def tick_to_deg(tick_value):
    """將 tick 轉換為度數"""
    rad = (tick_value - 2048) * (2 * math.pi) / 4096
    return rad * 57.2958

def main():
    # 連接馬達
    portHandler = PortHandler(DEVICENAME)
    packetHandler = PacketHandler(PROTOCOL_VERSION)
    
    if not portHandler.openPort():
        print(f"❌ 無法開啟 port: {DEVICENAME}")
        return
    
    if not portHandler.setBaudRate(BAUDRATE):
        print(f"❌ 無法設定 baudrate")
        portHandler.closePort()
        return
    
    print("✅ 已連接到手臂")
    print("=" * 80)
    print("📍 即時監控模式")
    print("=" * 80)
    print("目標：將手臂擺成「完全伸直向前」的姿態")
    print("提示：差距越小越好，目標是所有關節都 < ±5°")
    print("按 Ctrl+C 停止監控\n")
    
    try:
        while True:
            print("\033[2J\033[H")  # 清除螢幕
            print("=" * 80)
            print("📊 當前手臂姿態 vs 目標姿態")
            print("=" * 80)
            
            all_good = True
            
            for motor_id in MOTOR_IDS:
                # 讀取當前位置
                position, result, error = packetHandler.read4ByteTxRx(
                    portHandler, motor_id, ADDR_PRESENT_POSITION
                )
                
                if result != COMM_SUCCESS:
                    print(f"❌ 馬達 {motor_id} 讀取失敗")
                    continue
                
                current_deg = tick_to_deg(position)
                target_deg = TARGET_POSE_DEG[motor_id]
                diff = target_deg - current_deg
                
                # 判斷狀態
                if abs(diff) < 5:
                    status = "✅"
                elif abs(diff) < 15:
                    status = "⚠️ "
                    all_good = False
                else:
                    status = "❌"
                    all_good = False
                
                # 顯示
                joint_name = JOINT_NAMES[motor_id]
                print(f"{status} Joint {motor_id} ({joint_name:8s}): ", end="")
                print(f"當前 {current_deg:+7.2f}° → 目標 {target_deg:+7.2f}° | 差距 {diff:+7.2f}°")
                
                # 給建議
                if motor_id == 3 and abs(diff) > 15:
                    if diff > 0:
                        print(f"      💡 肘部需要再伸直一些（目前彎曲了 {abs(diff):.1f}°）")
                elif motor_id == 4 and abs(diff) > 15:
                    if diff < 0:
                        print(f"      💡 手腕往下壓一些")
                    else:
                        print(f"      💡 手腕往上抬一些")
                elif motor_id == 5 and abs(diff) > 15:
                    print(f"      💡 手腕旋轉調整")
            
            print("=" * 80)
            if all_good:
                print("🎉 姿態正確！可以執行校正了！")
                print("   執行: python3 simple_calibrate.py")
            else:
                print("⏳ 請繼續調整手臂姿態...")
            
            time.sleep(0.5)
            
    except KeyboardInterrupt:
        print("\n\n⏹️  監控已停止")
    finally:
        portHandler.closePort()

if __name__ == "__main__":
    main()
