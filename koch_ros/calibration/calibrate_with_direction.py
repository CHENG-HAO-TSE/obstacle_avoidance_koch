#!/usr/bin/env python3
"""
Koch 手臂校正工具 v3 - 含方向檢測
----------------------------------
改進：不僅記錄零點，還記錄每個關節的正方向

流程：
1. 擺到參考姿態 (position A) → 記錄 tick_A
2. 讓 joint2 向上抬 30° (position B) → 記錄 tick_B
3. 計算方向：direction = sign(tick_B - tick_A)
   - 如果 tick_B > tick_A → direction = +1 (正裝)
   - 如果 tick_B < tick_A → direction = -1 (反裝)
"""

import json
import time
from pathlib import Path
from dynamixel_sdk import *
import sys

DEVICENAME = '/dev/ttyACM0'
BAUDRATE = 1000000
PROTOCOL_VERSION = 2.0
MOTOR_IDS = [1, 2, 3, 4, 5, 6]
ADDR_PRESENT_POSITION = 132

def wait_for_enter(prompt="按 [ENTER] 繼續..."):
    """等待使用者按 ENTER"""
    print(f"\n{prompt}")
    input()

def read_motor_positions(portHandler, packetHandler):
    """讀取所有馬達的當前位置"""
    positions = {}
    
    for motor_id in MOTOR_IDS:
        position, result, error = packetHandler.read4ByteTxRx(
            portHandler, motor_id, ADDR_PRESENT_POSITION
        )
        
        if result != COMM_SUCCESS:
            print(f"❌ 無法讀取馬達 {motor_id}: {packetHandler.getTxRxResult(result)}")
            return None
        
        positions[motor_id] = position
    
    return positions

def main():
    print("=" * 70)
    print("🎯 Koch 手臂校正工具 v3 - 方向檢測版")
    print("=" * 70)
    
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
    
    # ========== 步驟 1: 記錄參考姿態 ==========
    print("\n" + "=" * 70)
    print("📍 步驟 1/2: 記錄參考姿態（零點）")
    print("=" * 70)
    print("\n請將手臂擺到「參考姿態」：")
    print("  - 完全伸直往前")
    print("  - 像一支筆水平指向前方")
    print("  - 這個姿態將被定義為「零點」")
    
    wait_for_enter()
    
    print("\n📸 正在讀取參考姿態...")
    zero_positions = read_motor_positions(portHandler, packetHandler)
    
    if zero_positions is None:
        print("❌ 讀取失敗")
        portHandler.closePort()
        return
    
    print("\n✅ 參考姿態記錄成功:")
    for motor_id, tick in zero_positions.items():
        angle_deg = ((tick - 2048) / 4096.0) * 360.0
        print(f"   馬達 {motor_id}: {tick:4d} tick  ({angle_deg:+7.2f}°)")
    
    # ========== 步驟 2: 檢測 joint2 方向 ==========
    print("\n" + "=" * 70)
    print("📍 步驟 2/2: 檢測關節方向")
    print("=" * 70)
    print("\n⚠️  重要！請仔細操作：")
    print("  1. 保持其他關節不動")
    print("  2. 只讓 joint2 (肩部) 向上抬起約 30°")
    print("  3. 確認手臂向上抬起了（不是向下）")
    print("")
    print("  💡 joint2 是控制肩部俯仰的關節")
    print("     向上抬 = 正方向")
    print("     向下壓 = 負方向")
    
    wait_for_enter("準備好後，按 [ENTER]，然後立即將肩部向上抬")
    
    print("\n📸 正在讀取 joint2 抬起後的位置...")
    time.sleep(0.5)  # 給使用者一點時間抬起
    
    test_positions = read_motor_positions(portHandler, packetHandler)
    
    if test_positions is None:
        print("❌ 讀取失敗")
        portHandler.closePort()
        return
    
    # 計算方向
    motor_directions = {}
    
    print("\n🔍 分析各馬達方向:")
    for motor_id in MOTOR_IDS:
        zero_tick = zero_positions[motor_id]
        test_tick = test_positions[motor_id]
        delta = test_tick - zero_tick
        
        # 判斷方向
        if abs(delta) < 50:
            # 幾乎沒變化，假設正向
            direction = 1
            status = "⚪ 無明顯變化 (預設正向)"
        elif delta > 0:
            direction = 1
            status = "✅ 正向 (tick 增加)"
        else:
            direction = -1
            status = "🔄 反向 (tick 減少)"
        
        motor_directions[motor_id] = direction
        
        print(f"   馬達 {motor_id}: {zero_tick:4d} → {test_tick:4d} (Δ={delta:+5d}) | {status}")
        
        # 特別提示 joint2
        if motor_id == 2 and abs(delta) > 100:
            expected_dir = "增加" if direction == 1 else "減少"
            print(f"      → Joint2 向上抬時，tick {expected_dir}")
    
    portHandler.closePort()
    
    # ========== 儲存校正檔 ==========
    output_data = {
        "calibration_type": "zero_point_with_direction",
        "method": "manual_positioning_v3",
        "timestamp": time.strftime("%Y-%m-%d %H:%M:%S"),
        "description": "記錄零點位置和各關節的運動方向",
        "usage_note": "發送指令時：motor_tick = zero_tick + direction * angle_to_tick(target_angle)",
        "zero_positions_tick": zero_positions,
        "motor_directions": motor_directions,
        "motor_info": {
            str(motor_id): {
                "zero_tick": zero_positions[motor_id],
                "direction": motor_directions[motor_id],
                "direction_note": "1=正裝(增加=正向), -1=反裝(減少=正向)",
                "zero_angle_deg": ((zero_positions[motor_id] - 2048) / 4096.0) * 360.0
            }
            for motor_id in MOTOR_IDS
        }
    }
    
    calib_dir = Path("/workspace/koch_ros/calibration")
    calib_dir.mkdir(parents=True, exist_ok=True)
    output_file = calib_dir / "zero_point_with_direction.json"
    
    with open(output_file, 'w', encoding='utf-8') as f:
        json.dump(output_data, f, indent=2, ensure_ascii=False)
    
    print("\n" + "=" * 70)
    print("✅ 校正完成！")
    print("=" * 70)
    print(f"📁 校正檔已儲存至: {output_file}")
    
    print("\n📝 零點位置 (tick):")
    for motor_id, tick in zero_positions.items():
        angle_deg = ((tick - 2048) / 4096.0) * 360.0
        direction = motor_directions[motor_id]
        dir_str = "正向" if direction == 1 else "反向"
        print(f"   馬達 {motor_id}: {tick:4d} tick ({angle_deg:+7.2f}°) | 方向: {dir_str}")
    
    print("\n🚀 使用說明:")
    print("   當 PyBullet 說「joint2 = +30°」時：")
    motor2_zero = zero_positions[2]
    motor2_dir = motor_directions[2]
    delta_tick = int(30 / 360 * 4096)
    target = motor2_zero + motor2_dir * delta_tick
    print(f"   計算：{motor2_zero} + ({motor2_dir}) × {delta_tick} = {target}")
    print()

if __name__ == "__main__":
    main()
