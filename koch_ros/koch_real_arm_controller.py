#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Koch 實體手臂控制節點
訂閱虛擬手臂的關節狀態並控制真實手臂
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from interbotix_xs_modules.xs_robot.arm import InterbotixManipulatorXS

class KochRealArmController(Node):
    def __init__(self):
        super().__init__('koch_real_arm_controller')
        
        # 參數
        self.declare_parameter('robot_model', 'low_cost_robot')
        self.declare_parameter('robot_name', 'follower')
        
        robot_model = self.get_parameter('robot_model').value
        robot_name = self.get_parameter('robot_name').value
        
        self.get_logger().info('=' * 60)
        self.get_logger().info('🤖 Koch 實體手臂控制節點啟動中...')
        self.get_logger().info(f'📦 機器人型號: {robot_model}')
        self.get_logger().info(f'🏷️  機器人名稱: {robot_name}')
        
        # 初始化機械臂
        try:
            self.bot = InterbotixManipulatorXS(
                robot_model=robot_model,
                robot_name=robot_name
            )
            self.get_logger().info('✓ 機械臂初始化成功')
        except Exception as e:
            self.get_logger().error(f'✗ 機械臂初始化失敗: {e}')
            raise
        
        # 訂閱虛擬手臂的指令
        self.subscription = self.create_subscription(
            JointState,
            f'/{robot_name}/joint_states_control',
            self.joint_command_callback,
            10
        )
        
        self.get_logger().info(f'📡 訂閱: /{robot_name}/joint_states_control')
        self.get_logger().info('=' * 60)
        self.get_logger().info('💡 等待虛擬手臂指令...')
        self.get_logger().info('   確保 koch_mirror_sim.py 正在運行')
        self.get_logger().info('=' * 60)
        
        self.command_count = 0
    
    def joint_command_callback(self, msg):
        """接收關節指令並發送到真實手臂"""
        try:
            # 只控制手臂關節（不包括夾爪）
            if len(msg.position) >= 5:
                arm_positions = msg.position[:5]  # joint1-joint5
                
                # 發送到真實手臂
                self.bot.arm.set_joint_positions(arm_positions, blocking=False)
                
                self.command_count += 1
                if self.command_count % 50 == 0:  # 每 50 次打印一次
                    self.get_logger().info(
                        f'✓ 已發送 {self.command_count} 條指令 | '
                        f'當前位置: [{", ".join([f"{p:.2f}" for p in arm_positions])}]'
                    )
        
        except Exception as e:
            self.get_logger().error(f'發送指令失敗: {e}')
    
    def shutdown(self):
        """關閉時回到休眠姿態"""
        self.get_logger().info('正在關閉...')
        try:
            self.bot.arm.go_to_sleep_pose()
            self.get_logger().info('✓ 已回到休眠姿態')
        except:
            pass

def main(args=None):
    rclpy.init(args=args)
    
    try:
        controller = KochRealArmController()
        rclpy.spin(controller)
    
    except KeyboardInterrupt:
        print('\n[INFO] 收到中斷信號')
    
    except Exception as e:
        print(f'[ERROR] {e}')
    
    finally:
        if 'controller' in locals():
            controller.shutdown()
            controller.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
