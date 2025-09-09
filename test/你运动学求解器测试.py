#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
PAROL6 逆运动学求解器测试程序
测试 ik_LM 和 ikine_LM 是否可用来替代 ikine_LMS
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
from robot_api import *

# 尝试导入机器人模型相关模块
try:
    import roboticstoolbox as rtb
    from spatialmath import SE3
    RTB_AVAILABLE = True
    print("✅ roboticstoolbox 可用")
except ImportError as e:
    RTB_AVAILABLE = False
    print(f"❌ roboticstoolbox 导入失败: {e}")

try:
    from PAROL6_ROBOT import PAROL6_ROBOT
    PAROL_MODEL_AVAILABLE = True
    print("✅ PAROL6_ROBOT 模型可用")
except ImportError as e:
    PAROL_MODEL_AVAILABLE = False
    print(f"❌ PAROL6_ROBOT 导入失败: {e}")

class IKTester:
    def __init__(self):
        """初始化逆运动学测试器"""
        self.robot = None
        self.current_pose = None
        self.current_joints = None
        
        print("🔬 PAROL6 逆运动学求解器测试")
        print("目标：验证 ik_LM 和 ikine_LM 可用性")
        
    def initialize_robot_model(self):
        """初始化机器人模型"""
        if not RTB_AVAILABLE or not PAROL_MODEL_AVAILABLE:
            print("❌ 缺少必要的库，无法初始化机器人模型")
            return False
        
        try:
            self.robot = PAROL6_ROBOT()
            print("✅ PAROL6 机器人模型初始化成功")
            print(f"📊 机器人信息: {self.robot.n}轴机器人")
            return True
        except Exception as e:
            print(f"❌ 机器人模型初始化失败: {e}")
            return False
    
    def get_current_robot_state(self):
        """获取当前机器人状态"""
        try:
            self.current_pose = get_robot_pose()
            self.current_joints = get_robot_joint_angles()
            
            if self.current_pose and self.current_joints:
                print(f"📍 当前位姿: {[round(p, 2) for p in self.current_pose]}")
                print(f"🔧 当前关节: {[round(a, 2) for a in self.current_joints]}")
                return True
            else:
                print("❌ 无法获取机器人当前状态")
                return False
        except Exception as e:
            print(f"❌ 获取状态失败: {e}")
            return False
    
    def test_ik_solvers(self):
        """测试不同的逆运动学求解器"""
        if not self.robot or not self.current_joints:
            print("❌ 机器人模型或状态未初始化")
            return
        
        print("\n" + "="*50)
        print("🧪 测试逆运动学求解器")
        print("="*50)
        
        # 将当前关节角度转换为弧度
        q_current = np.radians(self.current_joints)
        
        # 计算当前位姿的变换矩阵
        try:
            T_current = self.robot.fkine(q_current)
            print(f"✅ 正运动学计算成功")
        except Exception as e:
            print(f"❌ 正运动学计算失败: {e}")
            return
        
        # 创建一个稍微偏移的目标位姿
        T_target = T_current * SE3.Tx(0.01) * SE3.Ty(0.01)  # 偏移10mm
        print(f"🎯 目标位姿偏移: X+10mm, Y+10mm")
        
        # 测试不同的IK求解器
        solvers_to_test = [
            ("ik_LM", "C++版本LM求解器"),
            ("ikine_LM", "Python版本LM求解器"), 
            ("ikine_NR", "Newton-Raphson求解器"),
            ("ikine_min", "最小化求解器")
        ]
        
        for solver_name, description in solvers_to_test:
            print(f"\n🔍 测试 {solver_name} ({description})")
            self.test_single_solver(solver_name, T_target, q_current)
    
    def test_single_solver(self, solver_name, T_target, q_initial):
        """测试单个求解器"""
        try:
            # 检查求解器是否存在
            if not hasattr(self.robot, solver_name):
                print(f"   ❌ {solver_name} 方法不存在")
                return
            
            start_time = time.time()
            
            # 调用求解器
            solver_func = getattr(self.robot, solver_name)
            
            if solver_name in ['ik_LM', 'ikine_LM']:
                # LM求解器使用 q0 参数
                result = solver_func(T_target, q0=q_initial)
            else:
                # 其他求解器
                result = solver_func(T_target, q0=q_initial)
            
            solve_time = time.time() - start_time
            
            # 分析结果
            if result is not None:
                if hasattr(result, 'success'):
                    # 新版本RTB返回结果对象
                    success = result.success
                    if success:
                        q_solution = result.q
                        print(f"   ✅ 求解成功 (耗时: {solve_time:.3f}s)")
                        print(f"   🔧 解: {np.degrees(q_solution).round(2)}")
                        
                        # 验证解的准确性
                        self.verify_solution(q_solution, T_target)
                    else:
                        print(f"   ❌ 求解失败: {getattr(result, 'reason', '未知原因')}")
                else:
                    # 旧版本RTB直接返回关节角度
                    if len(result) == self.robot.n:
                        print(f"   ✅ 求解成功 (耗时: {solve_time:.3f}s)")
                        print(f"   🔧 解: {np.degrees(result).round(2)}")
                        self.verify_solution(result, T_target)
                    else:
                        print(f"   ❌ 返回结果异常: {result}")
            else:
                print(f"   ❌ 求解失败，返回None")
                
        except Exception as e:
            print(f"   ❌ {solver_name} 测试异常: {e}")
    
    def verify_solution(self, q_solution, T_target):
        """验证求解结果的准确性"""
        try:
            # 使用解算出的关节角度计算正运动学
            T_result = self.robot.fkine(q_solution)
            
            # 计算位置误差
            pos_target = T_target.t
            pos_result = T_result.t
            pos_error = np.linalg.norm(pos_target - pos_result)
            
            # 计算姿态误差
            R_error = T_target.R @ T_result.R.T
            angle_error = np.arccos(np.clip((np.trace(R_error) - 1) / 2, -1, 1))
            
            print(f"   📏 位置误差: {pos_error*1000:.3f}mm")
            print(f"   📐 姿态误差: {np.degrees(angle_error):.3f}°")
            
            if pos_error < 0.001 and angle_error < np.radians(1):  # 1mm, 1度
                print(f"   ✅ 解验证通过")
                return True
            else:
                print(f"   ⚠️ 解精度较低")
                return False
                
        except Exception as e:
            print(f"   ❌ 解验证失败: {e}")
            return False
    
    def test_file_replacement_needed(self):
        """检查需要替换的文件"""
        print("\n" + "="*50)
        print("📁 检查需要修改的文件")
        print("="*50)
        
        files_to_check = [
            "headless_commander.py",
            "PAROL6_ROBOT.py", 
            "smooth_motion.py"
        ]
        
        for filename in files_to_check:
            self.check_file_for_ikine_LMS(filename)
    
    def check_file_for_ikine_LMS(self, filename):
        """检查文件中是否包含 ikine_LMS"""
        try:
            with open(filename, 'r', encoding='utf-8') as f:
                content = f.read()
                
            count = content.count('ikine_LMS')
            if count > 0:
                print(f"📄 {filename}: 发现 {count} 个 'ikine_LMS' 需要替换")
                
                # 显示包含 ikine_LMS 的行
                lines = content.split('\n')
                for i, line in enumerate(lines):
                    if 'ikine_LMS' in line:
                        print(f"   行 {i+1}: {line.strip()}")
            else:
                print(f"📄 {filename}: 未发现 'ikine_LMS'")
                
        except FileNotFoundError:
            print(f"📄 {filename}: 文件不存在")
        except Exception as e:
            print(f"📄 {filename}: 检查失败 - {e}")
    
    def generate_replacement_script(self):
        """生成替换脚本"""
        print("\n" + "="*50)
        print("🔧 生成文件替换脚本")
        print("="*50)
        
        script_content = """#!/usr/bin/env python3
# PAROL6 ikine_LMS 替换脚本

import os
import re

def replace_in_file(filename, old_pattern, new_pattern):
    \"\"\"在文件中替换内容\"\"\"
    try:
        with open(filename, 'r', encoding='utf-8') as f:
            content = f.read()
        
        # 备份原文件
        backup_filename = filename + '.backup'
        with open(backup_filename, 'w', encoding='utf-8') as f:
            f.write(content)
        print(f"✅ 已备份: {backup_filename}")
        
        # 执行替换
        new_content = re.sub(old_pattern, new_pattern, content)
        changes = content.count(old_pattern.replace(r'\\b', ''))
        
        if changes > 0:
            with open(filename, 'w', encoding='utf-8') as f:
                f.write(new_content)
            print(f"✅ {filename}: 替换了 {changes} 处")
        else:
            print(f"📄 {filename}: 无需替换")
            
    except Exception as e:
        print(f"❌ {filename}: 替换失败 - {e}")

# 要修改的文件列表
files_to_modify = [
    "headless_commander.py",
    "PAROL6_ROBOT.py", 
    "smooth_motion.py"
]

print("🔧 开始替换 ikine_LMS -> ikine_LM")
for filename in files_to_modify:
    if os.path.exists(filename):
        replace_in_file(filename, r'\\bikine_LMS\\b', 'ikine_LM')
    else:
        print(f"⚠️ 文件不存在: {filename}")

print("\\n🎉 替换完成！")
print("💡 如果 ikine_LM 不工作，请尝试替换为 ik_LM")
"""
        
        with open("replace_ikine_LMS.py", "w", encoding='utf-8') as f:
            f.write(script_content)
        
        print("📝 已生成替换脚本: replace_ikine_LMS.py")
        print("🚀 使用方法: python replace_ikine_LMS.py")
    
    def run_comprehensive_test(self):
        """运行综合测试"""
        print("🚀 开始逆运动学求解器综合测试")
        
        # 1. 检查库可用性
        if not RTB_AVAILABLE:
            print("❌ 需要安装 roboticstoolbox-python")
            return
        
        # 2. 初始化机器人模型
        if not self.initialize_robot_model():
            print("❌ 无法初始化机器人模型，跳过IK测试")
        else:
            # 3. 获取当前状态
            if self.get_current_robot_state():
                # 4. 测试IK求解器
                self.test_ik_solvers()
        
        # 5. 检查文件
        self.test_file_replacement_needed()
        
        # 6. 生成替换脚本
        self.generate_replacement_script()
        
        print("\n" + "="*60)
        print("📋 测试总结与建议")
        print("="*60)
        print("1. 如果 ikine_LM 测试成功：")
        print("   → 运行 python replace_ikine_LMS.py 替换文件")
        print("   → 重启 headless_commander.py")
        print("   → 重新测试API")
        
        print("\n2. 如果 ikine_LM 失败但 ik_LM 成功：")
        print("   → 手动将文件中的 ikine_LMS 改为 ik_LM")
        
        print("\n3. 如果都失败：")
        print("   → 检查 roboticstoolbox 版本")
        print("   → pip install roboticstoolbox-python --upgrade")
        print("   → 检查 PAROL6 机器人配置")
        
        print("="*60)

def main():
    """主程序"""
    print("🔬 PAROL6 逆运动学求解器测试程序")
    
    tester = IKTester()
    
    if input("开始测试? (y/N): ").lower() == 'y':
        tester.run_comprehensive_test()
    else:
        print("❌ 用户取消测试")

if __name__ == "__main__":
    main()