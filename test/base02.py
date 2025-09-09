#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
PAROL6 安全底座旋转控制程序
通过命令结果检测异常状态
"""
import sys
import os

# 获取当前文件的绝对路径
current_dir = os.path.dirname(os.path.abspath(__file__))

# 获取父目录路径
parent_dir = os.path.dirname(current_dir)

# 将父目录添加到模块搜索路径中
sys.path.append(parent_dir)

# 现在可以导入父目录中的模块
from robot_api import *
import time


class SafeBaseController:
    def __init__(self):
        """初始化底座控制器"""
        self.base_joint_index = 0  # 底座是第1个关节（索引0）
        self.max_angle_step = 3.0  # 减小最大角度为3度，更安全
        self.slow_speed = 10       # 进一步降低速度到10%
        self.last_successful_angles = None
        
        print("🛡️ 安全底座控制器已初始化")
        print(f"📊 安全设置：最大步长{self.max_angle_step}°，速度{self.slow_speed}%")
        print("⚠️ 注意：急停时请手动停止程序！")

    def safety_check_by_status(self):
        """通过状态查询进行安全检查"""
        try:
            # 尝试获取当前角度作为通信测试
            angles = get_robot_joint_angles()
            if angles is None:
                print("⚠️ 警告：无法获取机器人状态，可能存在问题")
                return False
            
            # 检查角度是否合理（不是异常值）
            base_angle = angles[0]
            if abs(base_angle) > 180:
                print(f"⚠️ 警告：底座角度异常 {base_angle}°")
                return False
            
            self.last_successful_angles = angles
            return True
            
        except Exception as e:
            print(f"⚠️ 状态检查失败: {e}")
            return False

    def safe_rotate_base(self, angle_degrees):
        """
        安全旋转底座，包含多重检查
        """
        # 预检查
        if not self.safety_check_by_status():
            print("❌ 安全检查失败，取消移动")
            return False
        
        try:
            # 限制角度
            if abs(angle_degrees) > self.max_angle_step:
                print(f"⚠️ 角度限制为±{self.max_angle_step}°")
                angle_degrees = self.max_angle_step if angle_degrees > 0 else -self.max_angle_step
            
            # 获取移动前状态
            before_angles = get_robot_joint_angles()
            if not before_angles:
                print("❌ 无法获取移动前状态")
                return False
            
            print(f"🔄 准备底座旋转 {angle_degrees}°...")
            print(f"   当前角度: {before_angles[0]:.2f}°")
            
            # 用户确认（对于大角度）
            if abs(angle_degrees) > 2.0:
                confirm = input(f"确认旋转 {angle_degrees}°? (y/N): ").lower()
                if confirm != 'y':
                    print("❌ 用户取消操作")
                    return False
            
            # 确定方向
            if angle_degrees > 0:
                joint_index = self.base_joint_index  # 正方向
            else:
                joint_index = self.base_joint_index + 6  # 负方向
                angle_degrees = abs(angle_degrees)
            
            # 执行旋转
            print(f"🔄 执行旋转...")
            result = jog_robot_joint(
                joint_index=joint_index,
                speed_percentage=self.slow_speed,
                distance_deg=angle_degrees,
                wait_for_ack=True,
                timeout=15.0  # 增加超时时间
            )
            
            # 检查结果
            if not result:
                print("❌ 旋转命令无响应，可能机器人被急停")
                return False
                
            if result.get('status') != 'COMPLETED':
                print(f"❌ 旋转失败: {result.get('status', '未知错误')}")
                print(f"   详情: {result}")
                return False
            
            # 移动后验证
            time.sleep(1)  # 等待稳定
            after_angles = get_robot_joint_angles()
            if not after_angles:
                print("⚠️ 移动后无法获取状态，请检查机器人")
                return False
            
            # 验证移动是否成功
            angle_change = after_angles[0] - before_angles[0]
            expected_change = angle_degrees if joint_index == 0 else -angle_degrees
            
            if abs(angle_change - expected_change) > 1.0:  # 允许1度误差
                print(f"⚠️ 移动结果异常:")
                print(f"   期望变化: {expected_change:.2f}°")
                print(f"   实际变化: {angle_change:.2f}°")
            else:
                print("✅ 底座旋转完成并验证成功")
                print(f"   新角度: {after_angles[0]:.2f}°")
            
            return True
                
        except Exception as e:
            print(f"❌ 旋转错误: {e}")
            return False

    def get_base_status(self):
        """获取详细的底座状态"""
        try:
            angles = get_robot_joint_angles()
            if angles:
                print(f"📍 底座状态:")
                print(f"   当前角度: {angles[0]:.2f}°")
                print(f"   所有关节: {[round(a, 2) for a in angles]}")
                
                # 显示变化（如果有之前的数据）
                if self.last_successful_angles:
                    change = angles[0] - self.last_successful_angles[0]
                    if abs(change) > 0.1:
                        print(f"   角度变化: {change:.2f}°")
                
                return angles[0]
            else:
                print("❌ 无法获取状态")
                return None
                
        except Exception as e:
            print(f"❌ 状态获取错误: {e}")
            return None

def main():
    """主程序"""
    print("=" * 50)
    print("🛡️ PAROL6 安全底座控制程序")
    print("=" * 50)
    print("⚠️ 安全提醒:")
    print("   1. 急停按钮随时可用")
    print("   2. 如按急停，请手动停止程序(Ctrl+C)")
    print("   3. 每次移动都会进行安全检查")
    print("   4. 大角度移动需要确认")
    print("=" * 50)
    
    controller = SafeBaseController()
    
    try:
        # 初始状态检查
        if not controller.safety_check_by_status():
            print("❌ 初始安全检查失败，程序退出")
            return
        
        # 询问是否需要归零
        home_choice = input("\n是否需要机器人归零? (y/N): ").lower()
        if home_choice == 'y':
            print("🏠 机器人归零...")
            result = home_robot(wait_for_ack=True, timeout=30)
            time.sleep(2)
            
        # 显示当前状态
        controller.get_base_status()
        
        while True:
            print("\n" + "=" * 30)
            print("🎮 控制选项:")
            print("1. 小角度顺时针 (+1°)")
            print("2. 小角度逆时针 (-1°)")
            print("3. 中角度顺时针 (+3°)")
            print("4. 中角度逆时针 (-3°)")
            print("5. 自定义角度")
            print("6. 查看状态")
            print("7. 安全检查")
            print("8. 退出")
            print("=" * 30)
            
            choice = input("请选择 (1-8): ").strip()
            
            if choice == '1':
                controller.safe_rotate_base(1.0)
            elif choice == '2':
                controller.safe_rotate_base(-1.0)
            elif choice == '3':
                controller.safe_rotate_base(3.0)
            elif choice == '4':
                controller.safe_rotate_base(-3.0)
            elif choice == '5':
                try:
                    angle = float(input(f"输入角度 (±{controller.max_angle_step}): "))
                    controller.safe_rotate_base(angle)
                except ValueError:
                    print("❌ 输入无效")
            elif choice == '6':
                controller.get_base_status()
            elif choice == '7':
                if controller.safety_check_by_status():
                    print("✅ 安全检查通过")
                else:
                    print("❌ 安全检查失败")
            elif choice == '8':
                print("👋 程序安全退出")
                break
            else:
                print("❌ 无效选择")
                
            # 每次操作后短暂暂停
            time.sleep(0.5)
                
    except KeyboardInterrupt:
        print("\n🛑 程序被用户中断")
        print("✅ 安全退出")
    except Exception as e:
        print(f"\n❌ 程序错误: {e}")
        print("🛑 建议检查机器人状态")

if __name__ == "__main__":
    main()