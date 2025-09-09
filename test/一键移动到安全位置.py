#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
PAROL6 快速安全移动脚本
一键移动到最安全的位置
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
from robot_api import *

def quick_safe_move():
    """快速移动到安全位置"""
    print("🚀 PAROL6 快速安全移动")
    print("=" * 30)
    
    # 获取当前状态
    try:
        current_pose = get_robot_pose()
        current_joints = get_robot_joint_angles()
        
        if not current_pose or not current_joints:
            print("❌ 无法获取机器人状态")
            return False
        
        print(f"📍 当前位姿: {[round(p, 2) for p in current_pose]}")
        print(f"🔧 当前关节: {[round(a, 2) for a in current_joints]}")
        
    except Exception as e:
        print(f"❌ 获取状态失败: {e}")
        return False
    
    # 定义最安全的位置（工作空间中心，中等高度）
    safe_joints = [0, -60, 150, 0, 30, 180]
    print(f"\n🎯 目标安全位置: {safe_joints}")
    
    # 确认移动
    if input("确认移动到安全位置? (y/N): ").lower() != 'y':
        print("❌ 用户取消")
        return False
    
    try:
        print("🚀 开始移动...")
        
        result = move_robot_joints(
            safe_joints,
            speed_percentage=8,  # 慢速
            wait_for_ack=True,
            timeout=20.0
        )
        
        if result:
            print("⏳ 等待移动完成...")
            
            # 等待完成
            start_time = time.time()
            while time.time() - start_time < 20:
                try:
                    if result.get('command_id'):
                        status = check_command_status(result['command_id'])
                        if status and status.get('completed'):
                            if status.get('status') == 'COMPLETED':
                                print("✅ 移动到安全位置成功！")
                                
                                # 显示新位置
                                time.sleep(1)
                                new_pose = get_robot_pose()
                                new_joints = get_robot_joint_angles()
                                if new_pose and new_joints:
                                    print(f"📍 新位姿: {[round(p, 2) for p in new_pose]}")
                                    print(f"🔧 新关节: {[round(a, 2) for a in new_joints]}")
                                
                                return True
                            else:
                                print(f"❌ 移动失败: {status.get('status')}")
                                return False
                    elif result.get('status') == 'COMPLETED':
                        print("✅ 移动完成！")
                        return True
                    
                    time.sleep(0.5)
                except:
                    pass
            
            print("⚠️ 移动可能超时")
            return False
        else:
            print("❌ 移动命令失败")
            return False
            
    except Exception as e:
        print(f"❌ 移动异常: {e}")
        return False

if __name__ == "__main__":
    if quick_safe_move():
        print("\n🎉 现在可以运行API测试程序了！")
        print("运行: python safe_api_test.py")
    else:
        print("\n❌ 移动失败，请检查机器人状态")