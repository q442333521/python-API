#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
PAROL6 简单底座旋转控制程序（简化版）
只控制底座关节，小角度慢速旋转
"""

import time
from robot_api import *

class SimpleBaseController:
    def __init__(self):
        """初始化底座控制器"""
        self.base_joint_index = 0  # 底座是第1个关节（索引0）
        self.max_angle_step = 5.0  # 每次最大旋转角度（度）
        self.slow_speed = 15       # 慢速度（百分比）
        
        print("🤖 底座控制器已初始化")
        print(f"📊 设置：最大步长{self.max_angle_step}°，速度{self.slow_speed}%")

    def rotate_base(self, angle_degrees):
        """
        旋转底座指定角度
        
        参数:
        angle_degrees: 旋转角度（正数顺时针，负数逆时针）
        """
        try:
            # 限制角度范围
            if abs(angle_degrees) > self.max_angle_step:
                print(f"⚠️ 角度限制为±{self.max_angle_step}°")
                angle_degrees = self.max_angle_step if angle_degrees > 0 else -self.max_angle_step
            
            print(f"🔄 底座旋转 {angle_degrees}°...")
            
            # 确定方向
            if angle_degrees > 0:
                joint_index = self.base_joint_index  # 正方向 (0)
            else:
                joint_index = self.base_joint_index + 6  # 负方向 (6)
                angle_degrees = abs(angle_degrees)
            
            # 执行旋转
            result = jog_robot_joint(
                joint_index=joint_index,
                speed_percentage=self.slow_speed,
                distance_deg=angle_degrees,
                wait_for_ack=True,
                timeout=10.0
            )
            
            if result and result.get('status') == 'COMPLETED':
                print("✅ 底座旋转完成")
                return True
            else:
                print("❌ 底座旋转失败")
                print(f"   结果: {result}")
                return False
                
        except Exception as e:
            print(f"❌ 旋转错误: {e}")
            return False

    def get_base_angle(self):
        """获取当前底座角度"""
        try:
            angles = get_robot_joint_angles()
            if angles:
                base_angle = angles[0]
                print(f"📍 当前底座角度: {base_angle:.2f}°")
                return base_angle
            else:
                print("❌ 无法获取角度信息")
                return None
        except Exception as e:
            print(f"❌ 获取角度错误: {e}")
            return None

def main():
    """主程序"""
    print("=" * 40)
    print("🤖 PAROL6 简单底座控制")
    print("=" * 40)
    
    controller = SimpleBaseController()
    
    try:
        # 询问是否需要归零
        home_choice = input("是否需要机器人归零? (y/N): ").lower()
        if home_choice == 'y':
            print("🏠 机器人归零...")
            result = home_robot(wait_for_ack=True, timeout=30)
            if result:
                print("✅ 归零完成")
            else:
                print("⚠️ 归零可能未完成，请检查")
            
            # 等待稳定
            time.sleep(2)
        
        # 显示当前角度
        controller.get_base_angle()
        
        while True:
            print("\n" + "-" * 30)
            print("选择操作:")
            print("1. 顺时针旋转")
            print("2. 逆时针旋转")
            print("3. 查看当前角度")
            print("4. 查看所有关节角度")
            print("5. 退出")
            print("-" * 30)
            
            choice = input("请选择 (1-5): ").strip()
            
            if choice == '1':
                # 顺时针旋转
                try:
                    angle = float(input(f"输入角度 (1-{controller.max_angle_step}): ") or "2")
                    controller.rotate_base(abs(angle))
                except ValueError:
                    print("❌ 输入无效")
                    
            elif choice == '2':
                # 逆时针旋转
                try:
                    angle = float(input(f"输入角度 (1-{controller.max_angle_step}): ") or "2")
                    controller.rotate_base(-abs(angle))
                except ValueError:
                    print("❌ 输入无效")
                    
            elif choice == '3':
                # 查看底座角度
                controller.get_base_angle()
                
            elif choice == '4':
                # 查看所有关节角度
                try:
                    angles = get_robot_joint_angles()
                    if angles:
                        print("🔧 所有关节角度:")
                        joint_names = ["底座", "肩部", "肘部", "腕1", "腕2", "腕3"]
                        for i, angle in enumerate(angles):
                            print(f"   {joint_names[i]}: {angle:.2f}°")
                    else:
                        print("❌ 无法获取关节角度")
                except Exception as e:
                    print(f"❌ 获取角度错误: {e}")
                    
            elif choice == '5':
                # 退出
                print("👋 程序退出")
                break
                
            else:
                print("❌ 无效选择，请输入1-5")
                
    except KeyboardInterrupt:
        print("\n🛑 程序中断")
    except Exception as e:
        print(f"❌ 程序错误: {e}")

if __name__ == "__main__":
    main()