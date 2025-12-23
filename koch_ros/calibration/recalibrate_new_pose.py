#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
重新校正 - 記錄新的參考姿勢
============================
此腳本讓您可以更改 offset 的參考姿勢。

使用方法：
1. 手動移動手臂到您想要的新參考姿勢
2. 運行此腳本記錄該姿勢的 tick 值
3. 方向設定會保留原有的配置

作者：AI Assistant
日期：2025-12-16
"""

import json
import time
from pathlib import Path
from dynamixel_sdk import *

# ========== 設定 ==========
DEVICE_NAME = '/dev/ttyACM0'  # 設備路徑
BAUDRATE = 1000000
PROTOCOL_VERSION = 2.0

# Dynamixel 地址
ADDR_TORQUE_ENABLE = 64
ADDR_PRESENT_POSITION = 132

TORQUE_DISABLE = 0
TORQUE_ENABLE = 1

MOTOR_IDS = [1, 2, 3, 4, 5, 6]

# 校正檔路徑
CALIB_FILE = Path(__file__).parent / "zero_point_with_direction.json"

# ========== 初始化 Dynamixel ==========
def init_dynamixel():
    """初始化 Dynamixel 通訊"""
    port_handler = PortHandler(DEVICE_NAME)
    packet_handler = PacketHandler(PROTOCOL_VERSION)
    
    if not port_handler.openPort():
        raise Exception(f"❌ 無法開啟 {DEVICE_NAME}")
    print(f"✅ 已開啟 {DEVICE_NAME}")
    
    if not port_handler.setBaudRate(BAUDRATE):
        raise Exception(f"❌ 無法設定 Baudrate 為 {BAUDRATE}")
    print(f"✅ Baudrate: {BAUDRATE}")
    
    return port_handler, packet_handler

def read_motor_position(ph, pkt, motor_id):
    """讀取馬達當前位置（tick）"""
    pos, result, error = pkt.read4ByteTxRx(ph, motor_id, ADDR_PRESENT_POSITION)
    if result != COMM_SUCCESS:
        raise Exception(f"通訊失敗: {pkt.getTxRxResult(result)}")
    if error != 0:
        raise Exception(f"硬體錯誤: {pkt.getRxPacketError(error)}")
    return pos

def disable_all_torque(ph, pkt):
    """關閉所有馬達的 torque，讓手臂可以手動移動"""
    print("\n🔓 正在關閉所有馬達 torque（手臂可以自由移動）...")
    for motor_id in MOTOR_IDS:
        pkt.write1ByteTxRx(ph, motor_id, ADDR_TORQUE_ENABLE, TORQUE_DISABLE)
        print(f"   馬達 {motor_id}: ✅ Torque 已關閉")
    print("✅ 所有馬達 torque 已關閉，您現在可以手動移動手臂")

def read_all_positions(ph, pkt):
    """讀取所有馬達的當前位置"""
    positions = {}
    print("\n📊 當前馬達位置:")
    for motor_id in MOTOR_IDS:
        tick = read_motor_position(ph, pkt, motor_id)
        angle_deg = ((tick - 2048) / 4096.0) * 360.0
        positions[motor_id] = tick
        print(f"   馬達 {motor_id}: {tick:4d} tick ({angle_deg:+7.2f}°)")
    return positions

def load_existing_directions():
    """載入現有的方向設定"""
    if not CALIB_FILE.exists():
        print("⚠️  找不到現有校正檔，使用預設方向 (全部 +1)")
        return {str(i): 1 for i in MOTOR_IDS}
    
    with open(CALIB_FILE, 'r', encoding='utf-8') as f:
        data = json.load(f)
    
    if "motor_directions" in data:
        print("✅ 已載入現有的方向設定")
        return data["motor_directions"]
    else:
        print("⚠️  校正檔中沒有方向設定，使用預設 (全部 +1)")
        return {str(i): 1 for i in MOTOR_IDS}

def save_calibration(positions, directions):
    """保存新的校正資料（保留方向設定）"""
    calib_data = {
        "calibration_type": "zero_point_with_direction",
        "method": "manual_recalibration",
        "timestamp": time.strftime("%Y-%m-%d %H:%M:%S"),
        "description": "重新校正參考姿勢，保留原有方向設定",
        "usage_note": "發送指令時：motor_tick = zero_tick + direction * angle_to_tick(target_angle)",
        "zero_positions_tick": {str(k): int(v) for k, v in positions.items()},
        "motor_directions": directions
    }
    
    # 備份舊檔案
    if CALIB_FILE.exists():
        backup_file = CALIB_FILE.with_suffix('.json.backup')
        import shutil
        shutil.copy(CALIB_FILE, backup_file)
        print(f"📦 已備份舊校正檔到: {backup_file}")
    
    # 保存新檔案
    with open(CALIB_FILE, 'w', encoding='utf-8') as f:
        json.dump(calib_data, f, indent=2, ensure_ascii=False)
    
    print(f"\n✅ 新的校正檔已保存到: {CALIB_FILE}")

# ========== 主程式 ==========
def main():
    print("=" * 70)
    print("🔄 Koch 手臂重新校正 - 更換參考姿勢")
    print("=" * 70)
    
    # 初始化
    port_handler, packet_handler = init_dynamixel()
    
    # 載入現有的方向設定
    existing_directions = load_existing_directions()
    print("\n📋 當前方向設定:")
    for motor_id in MOTOR_IDS:
        direction = existing_directions.get(str(motor_id), 1)
        dir_text = "正向" if direction == 1 else "反向"
        print(f"   馬達 {motor_id}: {direction:+2d} ({dir_text})")
    
    # 關閉 torque
    disable_all_torque(port_handler, packet_handler)
    
    print("\n" + "=" * 70)
    print("📋 操作步驟:")
    print("   1. 手動移動手臂到您想要的新參考姿勢")
    print("   2. 常見的參考姿勢選擇:")
    print("      - 完全伸直向前 (原始姿勢)")
    print("      - 垂直向上")
    print("      - 收回到身體旁邊")
    print("      - 或任何您希望作為「零點」的姿勢")
    print("   3. 移動好後，按 Enter 繼續")
    print("=" * 70)
    
    input("\n👉 請手動調整手臂姿勢，完成後按 [Enter] 鍵記錄... ")
    
    # 讀取當前位置
    positions = read_all_positions(port_handler, packet_handler)
    
    # 確認
    print("\n" + "=" * 70)
    print("⚠️  確認資訊:")
    print("   - 這些位置將作為新的零點參考")
    print("   - 原有的方向設定會保留")
    print("   - 舊的校正檔會自動備份")
    print("=" * 70)
    
    confirm = input("\n❓ 確定要保存這個新的參考姿勢嗎？(yes/no): ").strip().lower()
    
    if confirm in ['yes', 'y']:
        save_calibration(positions, existing_directions)
        print("\n" + "=" * 70)
        print("✅ 校正完成！")
        print("\n📝 下一步:")
        print("   1. 重新啟動控制器以使用新的零點")
        print("   2. 測試虛擬手臂和實體手臂的同步")
        print("=" * 70)
    else:
        print("\n❌ 已取消，未保存變更")
    
    # 清理
    port_handler.closePort()
    print("\n✅ 已關閉連接")

if __name__ == "__main__":
    try:
        main()
    except KeyboardInterrupt:
        print("\n\n⏹️  使用者中斷")
    except Exception as e:
        print(f"\n❌ 錯誤: {e}")
        import traceback
        traceback.print_exc()
