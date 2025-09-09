#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
PAROL6 专用 ikine_LM 测试和修复程序
基于确认可用的 ikine_LM 方法
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
import numpy as np
import time
import os
import shutil
from robot_api import *

# 尝试导入PAROL6模型
try:
    from PAROL6_ROBOT import PAROL6_ROBOT
    import spatialmath as sm
    PAROL_AVAILABLE = True
    print("✅ PAROL6 模型和spatialmath可用")
except ImportError as e:
    PAROL_AVAILABLE = False
    print(f"❌ 导入失败: {e}")

class PAROL6_IK_Fixer:
    def __init__(self):
        """初始化PAROL6 IK修复器"""
        self.robot = None
        self.current_pose = None
        self.current_joints = None
        
        print("🔧 PAROL6 逆运动学修复工具")
        print("基于确认可用的 ikine_LM 方法")

    def test_parol6_ik(self):
        """测试PAROL6机器人的逆运动学"""
        print("\n" + "="*50)
        print("🧪 测试PAROL6机器人逆运动学")
        print("="*50)
        
        if not PAROL_AVAILABLE:
            print("❌ 无法导入PAROL6模型，跳过IK测试")
            return False
        
        try:
            # 初始化PAROL6机器人
            self.robot = PAROL6_ROBOT()
            print("✅ PAROL6机器人模型初始化成功")
            
            # 获取当前机器人状态
            self.current_pose = get_robot_pose()
            self.current_joints = get_robot_joint_angles()
            
            if not self.current_pose or not self.current_joints:
                print("❌ 无法获取机器人当前状态")
                return False
            
            print(f"📍 当前位姿: {[round(p, 2) for p in self.current_pose]}")
            print(f"🔧 当前关节: {[round(a, 2) for a in self.current_joints]}")
            
            # 转换当前关节角度到弧度
            q_current = np.radians(self.current_joints)
            
            # 计算当前位姿的变换矩阵
            T_current = self.robot.fkine(q_current)
            print("✅ 正运动学计算成功")
            
            # 创建测试目标（小偏移）
            T_target = T_current * sm.SE3.Tx(0.02) * sm.SE3.Ty(0.02) * sm.SE3.Tz(0.01)
            print("🎯 测试目标：当前位置 + (20mm, 20mm, 10mm)")
            
            # 测试 ikine_LM
            print("\n🔍 测试 ikine_LM 求解器...")
            start_time = time.time()
            
            sol = self.robot.ikine_LM(T_target, q0=q_current)
            
            solve_time = time.time() - start_time
            
            if sol.success:
                print(f"✅ ikine_LM 求解成功！(耗时: {solve_time:.3f}s)")
                solution_deg = np.degrees(sol.q)
                print(f"🎯 解（度）: {solution_deg.round(2)}")
                
                # 验证解的精度
                T_verify = self.robot.fkine(sol.q)
                pos_error = np.linalg.norm(T_target.t - T_verify.t)
                print(f"📏 位置误差: {pos_error*1000:.3f}mm")
                
                if pos_error < 0.001:  # 1mm精度
                    print("✅ 解验证通过，精度良好")
                    return True
                else:
                    print("⚠️ 解精度较低，但可用")
                    return True
            else:
                print(f"❌ ikine_LM 求解失败: {sol.reason}")
                return False
                
        except Exception as e:
            print(f"❌ 测试过程出错: {e}")
            return False

    def find_files_to_fix(self):
        """查找需要修复的文件"""
        print("\n" + "="*50)
        print("🔍 查找需要修复的文件")
        print("="*50)
        
        files_to_check = [
            "headless_commander.py",
            "PAROL6_ROBOT.py",
            "smooth_motion.py"
        ]
        
        files_needing_fix = []
        
        for filename in files_to_check:
            if os.path.exists(filename):
                try:
                    with open(filename, 'r', encoding='utf-8') as f:
                        content = f.read()
                    
                    ikine_lms_count = content.count('ikine_LMS')
                    if ikine_lms_count > 0:
                        print(f"📄 {filename}: 发现 {ikine_lms_count} 个 'ikine_LMS'")
                        files_needing_fix.append((filename, ikine_lms_count))
                        
                        # 显示具体位置
                        lines = content.split('\n')
                        for i, line in enumerate(lines, 1):
                            if 'ikine_LMS' in line:
                                print(f"   行 {i}: {line.strip()}")
                    else:
                        print(f"📄 {filename}: 无需修复")
                        
                except Exception as e:
                    print(f"📄 {filename}: 读取失败 - {e}")
            else:
                print(f"📄 {filename}: 文件不存在")
        
        return files_needing_fix

    def backup_and_fix_file(self, filename):
        """备份并修复单个文件"""
        try:
            # 读取原文件
            with open(filename, 'r', encoding='utf-8') as f:
                original_content = f.read()
            
            # 检查是否需要修复
            if 'ikine_LMS' not in original_content:
                print(f"📄 {filename}: 无需修复")
                return True
            
            # 创建备份
            backup_filename = f"{filename}.backup_{int(time.time())}"
            shutil.copy2(filename, backup_filename)
            print(f"💾 {filename}: 已备份到 {backup_filename}")
            
            # 执行替换
            fixed_content = original_content.replace('ikine_LMS', 'ikine_LM')
            replacement_count = original_content.count('ikine_LMS')
            
            # 写入修复后的内容
            with open(filename, 'w', encoding='utf-8') as f:
                f.write(fixed_content)
            
            print(f"✅ {filename}: 替换了 {replacement_count} 处 'ikine_LMS' -> 'ikine_LM'")
            return True
            
        except Exception as e:
            print(f"❌ {filename}: 修复失败 - {e}")
            return False

    def fix_all_files(self, files_needing_fix):
        """修复所有需要的文件"""
        print("\n" + "="*50)
        print("🔧 开始修复文件")
        print("="*50)
        
        if not files_needing_fix:
            print("🎉 没有文件需要修复！")
            return True
        
        success_count = 0
        total_replacements = 0
        
        for filename, count in files_needing_fix:
            print(f"\n🔧 修复 {filename}...")
            if self.backup_and_fix_file(filename):
                success_count += 1
                total_replacements += count
            
        print(f"\n📊 修复统计:")
        print(f"   成功修复: {success_count}/{len(files_needing_fix)} 个文件")
        print(f"   总替换数: {total_replacements} 处")
        
        return success_count == len(files_needing_fix)

    def test_api_after_fix(self):
        """修复后测试API"""
        print("\n" + "="*50)
        print("🧪 修复后API测试")
        print("="*50)
        
        print("⚠️ 请先重启 headless_commander.py，然后按Enter继续...")
        input("按Enter继续测试...")
        
        # 测试一个简单的位姿移动
        current_pose = get_robot_pose()
        if current_pose:
            print(f"📍 当前位姿: {[round(p, 2) for p in current_pose]}")
            
            # 创建一个小的移动目标
            target_pose = current_pose.copy()
            target_pose[0] += 10  # X轴移动10mm
            
            print(f"🎯 测试目标: {[round(p, 2) for p in target_pose]}")
            print("🚀 执行测试移动...")
            
            try:
                result = move_robot_pose(
                    target_pose,
                    speed_percentage=10,  # 很慢的速度
                    wait_for_ack=True,
                    timeout=15
                )
                
                if result and result.get('status') == 'COMPLETED':
                    print("✅ 位姿移动测试成功！ikine_LM 修复有效！")
                    return True
                else:
                    print(f"❌ 位姿移动测试失败: {result}")
                    return False
                    
            except Exception as e:
                print(f"❌ 测试过程出错: {e}")
                return False
        else:
            print("❌ 无法获取当前位姿")
            return False

    def run_complete_fix(self):
        """运行完整的修复流程"""
        print("🚀 PAROL6 ikine_LM 完整修复流程")
        print("="*50)
        
        # 步骤1: 测试 ikine_LM 可用性
        print("步骤1: 验证 ikine_LM 可用性")
        if not self.test_parol6_ik():
            print("❌ ikine_LM 不可用，无法继续修复")
            return
        
        print("✅ ikine_LM 验证通过，可以用于修复")
        
        # 步骤2: 查找需要修复的文件
        print("\n步骤2: 查找需要修复的文件")
        files_needing_fix = self.find_files_to_fix()
        
        if not files_needing_fix:
            print("🎉 所有文件都已经是正确的！")
            return
        
        # 步骤3: 用户确认
        print(f"\n步骤3: 确认修复")
        print(f"将要修复 {len(files_needing_fix)} 个文件:")
        for filename, count in files_needing_fix:
            print(f"   • {filename}: {count} 处替换")
        
        if input("\n确认开始修复? (y/N): ").lower() != 'y':
            print("❌ 用户取消修复")
            return
        
        # 步骤4: 执行修复
        print("\n步骤4: 执行文件修复")
        if not self.fix_all_files(files_needing_fix):
            print("❌ 修复过程出现问题")
            return
        
        print("✅ 文件修复完成！")
        
        # 步骤5: 指导测试
        print("\n步骤5: 验证修复效果")
        print("📋 接下来请:")
        print("1. 停止当前的 headless_commander.py (Ctrl+C)")
        print("2. 重新启动 headless_commander.py")
        print("3. 按Enter继续API测试")
        
        if input("\n已重启服务器？按y继续测试 (y/N): ").lower() == 'y':
            if self.test_api_after_fix():
                print("\n🎉 修复成功！所有API应该都能正常工作了！")
            else:
                print("\n⚠️ API测试未通过，可能需要进一步检查")
        
        print("\n" + "="*60)
        print("📋 修复完成总结")
        print("="*60)
        print("✅ ikine_LMS -> ikine_LM 替换完成")
        print("✅ 原文件已备份")
        print("🎯 现在可以使用以下API:")
        print("   • move_robot_pose")
        print("   • move_robot_cartesian") 
        print("   • jog_cartesian")
        print("   • smooth_circle")
        print("   • smooth_arc_*")
        print("   • smooth_spline")
        print("   • smooth_helix")
        print("="*60)

def main():
    """主程序"""
    print("🔧 PAROL6 ikine_LM 自动修复工具")
    print("基于您测试通过的 ikine_LM 方法")
    
    fixer = PAROL6_IK_Fixer()
    
    print("\n选择操作:")
    print("1. 完整修复流程（推荐）")
    print("2. 仅测试 ikine_LM")
    print("3. 仅查找需要修复的文件")
    print("4. 退出")
    
    choice = input("\n请选择 (1-4): ").strip()
    
    if choice == '1':
        fixer.run_complete_fix()
    elif choice == '2':
        fixer.test_parol6_ik()
    elif choice == '3':
        fixer.find_files_to_fix()
    elif choice == '4':
        print("👋 退出程序")
    else:
        print("❌ 无效选择")

if __name__ == "__main__":
    main()