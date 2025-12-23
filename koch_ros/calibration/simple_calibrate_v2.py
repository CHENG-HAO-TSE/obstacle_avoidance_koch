#!/usr/bin/env python3
"""
Koch 手臂校正工具 - 簡化版 v2
-----------------------------
邏輯：
1. 您手動將手臂擺到想要的「參考姿態」（例如完全伸直）
2. 記錄此時每個馬達的實際 tick 值
3. 這些值就是「零點」，儲存為 JSON

使用時：
- PyBullet 發送目標角度（相對於參考姿態的偏移）
- 實際馬達位置 = 零點 tick + 角度偏移
"""

import json
import time
from pathlib import Path
from dynamixel_sdk import *

# 馬達設定
DEVICENAME = '/dev/ttyACM0'
BAUDRATE = 1000000
PROTOCOL_VERSION = 2.0
MOTOR_IDS = [1, 2, 3, 4, 5, 6]
ADDR_PRESENT_POSITION = 132

def main():
    print("=" * 60)
    print("🎯 Koch 手臂校正工具 v2 - 超簡化版")
    print("=" * 60)
    
    # 初始化
    portHandler = PortHandler(DEVICENAME)
    packetHandler = PacketHandler(PROTOCOL_VERSION)
    
    if not portHandler.openPort():
        print(f"❌ 無法開啟 port: {DEVICENAME}")
        return
    
    print(f"✅ 已連接: {DEVICENAME}")
    
    if not portHandler.setBaudRate(BAUDRATE):
        print(f"❌ 無法設定 baudrate: {BAUDRATE}")
        portHandler.closePort()
        return
    
    print(f"✅ Baudrate: {BAUDRATE}")
    
    print("\n" + "=" * 60)
    print("📋 校正步驟：")
    print("=" * 60)
    print("\n1️⃣  將手臂擺到您想要的「參考姿態」")
    print("   (建議：完全伸直往前，像一支筆)")
    print("\n2️⃣  確認姿態穩定後，按 [ENTER]")
    print("\n   💡 此姿態將被定義為「零點」")
    print("   💡 之後所有動作都是相對於這個零點的偏移")
    print()
    
    input()
    
    print("\n📸 正在讀取馬達零點位置...")
    
    zero_positions = {}
    
    for motor_id in MOTOR_IDS:
        position, result, error = packetHandler.read4ByteTxRx(
            portHandler, motor_id, ADDR_PRESENT_POSITION
        )
        
        if result != COMM_SUCCESS:
            print(f"❌ 無法讀取馬達 {motor_id}: {packetHandler.getTxRxResult(result)}")
            continue
        elif error != 0:
            print(f"⚠️  馬達 {motor_id} 錯誤: {packetHandler.getRxPacketError(error)}")
            continue
        
        zero_positions[motor_id] = position
        
        # 計算角度（僅供參考）
        angle_deg = ((position - 2048) / 4096.0) * 360.0
        
        print(f"   馬達 {motor_id}: tick = {position:4d}  ({angle_deg:+7.2f}°)")
    
    portHandler.closePort()
    
    # 準備輸出 JSON
    output_data = {
        "calibration_type": "zero_point_reference",
        "method": "manual_positioning_v2",
        "timestamp": time.strftime("%Y-%m-%d %H:%M:%S"),
        "description": "記錄手臂在參考姿態時每個馬達的實際位置",
        "usage_note": "發送指令時：motor_tick = zero_tick + angle_to_tick(target_angle)",
        "zero_positions_tick": zero_positions,
        "motor_info": {
            str(motor_id): {
                "zero_tick": tick,
                "zero_angle_deg": ((tick - 2048) / 4096.0) * 360.0,
                "note": "此 tick 值對應手臂的參考姿態"
            }
            for motor_id, tick in zero_positions.items()
        }
    }
    
    # 儲存校正檔
    calib_dir = Path("/workspace/koch_ros/calibration")
    calib_dir.mkdir(parents=True, exist_ok=True)
    
    output_file = calib_dir / "zero_point_calibration.json"
    
    with open(output_file, 'w', encoding='utf-8') as f:
        json.dump(output_data, f, indent=2, ensure_ascii=False)
    
    print("\n" + "=" * 60)
    print(f"✅ 校正完成！")
    print("=" * 60)
    print(f"📁 校正檔已儲存至: {output_file}")
    
    print("\n📝 零點位置 (tick):")
    for motor_id, tick in zero_positions.items():
        angle_deg = ((tick - 2048) / 4096.0) * 360.0
        print(f"   馬達 {motor_id}: {tick:4d} tick  ({angle_deg:+7.2f}°)")
    
    print("\n🚀 使用方式：")
    print("   1. PyBullet 控制器載入此 JSON")
    print("   2. 發送目標角度（相對於參考姿態）")
    print("   3. 計算：實際 tick = 零點 tick + 角度轉換")
    print()
    print("   範例：")
    print("   如果 PyBullet 要讓 joint2 從參考姿態轉動 +30°")
    print(f"   zero_tick = {zero_positions.get(2, 2048)}")
    print("   delta_tick = int(30 / 360 * 4096) = 341")
    print(f"   target_tick = {zero_positions.get(2, 2048)} + 341 = {zero_positions.get(2, 2048) + 341}")
    print()

if __name__ == "__main__":
    main()
