#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
PAROL6 修复后API安全测试程序
专门测试修复后的位姿和平滑运动API
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
import numpy as np
from robot_api import *

class SafeAPITester:
    def __init__(self):
        """初始化安全测试器"""
        self.current_pose = None
        self.current_joints = None
        self.safe_workspace_center = None
        self.test_results = {}
        
        print("🛡️ PAROL6 修复后API安全测试器")
        print("专门测试位姿移动和平滑运动API")

    def initialize_safe_baseline(self):
        """初始化安全基线"""
        print("\n📊 初始化安全测试基线")
        print("-" * 40)
        
        try:
            # 获取当前状态
            self.current_pose = get_robot_pose()
            self.current_joints = get_robot_joint_angles()
            
            if not self.current_pose or not self.current_joints:
                print("❌ 无法获取机器人当前状态")
                return False
            
            print(f"📍 当前位姿: {[round(p, 2) for p in self.current_pose]}")
            print(f"🔧 当前关节: {[round(a, 2) for a in self.current_joints]}")
            
            # 设置安全工作空间中心（使用当前位置）
            self.safe_workspace_center = self.current_pose[:3].copy()  # 只取XYZ
            print(f"🎯 安全工作空间中心: {[round(p, 2) for p in self.safe_workspace_center]}")
            
            # 检查当前位置是否安全
            if self.is_pose_safe(self.current_pose):
                print("✅ 当前位置安全，可以开始测试")
                return True
            else:
                print("⚠️ 当前位置可能不安全，请手动移动到安全位置")
                return False
                
        except Exception as e:
            print(f"❌ 初始化失败: {e}")
            return False

    def is_pose_safe(self, pose):
        """检查位姿是否安全"""
        if not pose or len(pose) < 3:
            return False
        
        x, y, z = pose[:3]
        
        # 基本安全检查（根据PAROL6工作空间调整）
        if (50 <= x <= 400 and 
            -300 <= y <= 300 and 
            50 <= z <= 400):
            return True
        
        return False

    def create_safe_target_pose(self, offset_x=0, offset_y=0, offset_z=0):
        """创建安全的目标位姿"""
        if not self.current_pose:
            return None
        
        target_pose = self.current_pose.copy()
        target_pose[0] += offset_x
        target_pose[1] += offset_y
        target_pose[2] += offset_z
        
        # 检查目标是否安全
        if self.is_pose_safe(target_pose):
            return target_pose
        else:
            print(f"⚠️ 目标位姿不安全: {[round(p, 2) for p in target_pose]}")
            return None

    def wait_for_completion(self, result, timeout=15):
        """等待命令完成"""
        if not result or not isinstance(result, dict):
            return False, "无效结果"
        
        command_id = result.get('command_id')
        if not command_id:
            status = result.get('status')
            return status == 'COMPLETED', f"状态: {status}"
        
        start_time = time.time()
        last_status = None
        
        while time.time() - start_time < timeout:
            try:
                status_info = check_command_status(command_id)
                if status_info:
                    current_status = status_info.get('status')
                    if current_status != last_status:
                        print(f"   状态更新: {current_status}")
                        last_status = current_status
                    
                    if status_info.get('completed'):
                        final_status = status_info.get('status')
                        return final_status == 'COMPLETED', f"最终状态: {final_status}"
                
                time.sleep(0.3)
            except Exception as e:
                print(f"   状态检查异常: {e}")
                
        return False, "超时"

    def log_test_result(self, api_name, success, details=""):
        """记录测试结果"""
        self.test_results[api_name] = {
            'success': success,
            'details': details,
            'timestamp': time.time()
        }
        
        status = "✅ 成功" if success else "❌ 失败"
        print(f"{status} {api_name}: {details}")

    def test_move_robot_pose(self):
        """测试 move_robot_pose"""
        print("\n🤖 测试 move_robot_pose")
        print("-" * 30)
        
        # 创建安全的小幅移动目标
        target_pose = self.create_safe_target_pose(offset_x=10, offset_z=5)
        
        if not target_pose:
            self.log_test_result("move_robot_pose", False, "无法创建安全目标位姿")
            return
        
        print(f"🎯 目标位姿: {[round(p, 2) for p in target_pose]}")
        
        try:
            result = move_robot_pose(
                target_pose,
                speed_percentage=5,  # 非常慢的速度
                wait_for_ack=True,
                timeout=20
            )
            
            success, details = self.wait_for_completion(result, timeout=20)
            self.log_test_result("move_robot_pose", success, details)
            
            if success:
                time.sleep(1)  # 等待稳定
                
        except Exception as e:
            self.log_test_result("move_robot_pose", False, f"异常: {e}")

    def test_move_robot_cartesian(self):
        """测试 move_robot_cartesian"""
        print("\n📏 测试 move_robot_cartesian")
        print("-" * 30)
        
        target_pose = self.create_safe_target_pose(offset_y=8, offset_z=-3)
        
        if not target_pose:
            self.log_test_result("move_robot_cartesian", False, "无法创建安全目标位姿")
            return
        
        print(f"🎯 直线移动目标: {[round(p, 2) for p in target_pose]}")
        
        try:
            result = move_robot_cartesian(
                target_pose,
                speed_percentage=5,  # 非常慢的速度
                wait_for_ack=True,
                timeout=20
            )
            
            success, details = self.wait_for_completion(result, timeout=20)
            self.log_test_result("move_robot_cartesian", success, details)
            
            if success:
                time.sleep(1)
                
        except Exception as e:
            self.log_test_result("move_robot_cartesian", False, f"异常: {e}")

    def test_jog_cartesian(self):
        """测试 jog_cartesian"""
        print("\n🕹️ 测试 jog_cartesian")
        print("-" * 30)
        
        # 测试多个轴向的小幅点动
        test_axes = [
            ('X+', 'X轴正向'),
            ('Y+', 'Y轴正向'), 
            ('Z+', 'Z轴正向'),
            ('Z-', 'Z轴负向')
        ]
        
        for axis, desc in test_axes:
            print(f"🎮 测试 {desc} 点动...")
            
            try:
                result = jog_cartesian(
                    frame='WRF',  # 世界坐标系
                    axis=axis,
                    speed_percentage=8,  # 很慢的速度
                    duration=1.0,  # 短时间
                    wait_for_ack=True,
                    timeout=10
                )
                
                success, details = self.wait_for_completion(result, timeout=10)
                self.log_test_result(f"jog_cartesian_{axis}", success, details)
                
                if success:
                    time.sleep(0.5)  # 短暂停顿
                else:
                    break  # 如果失败，停止后续测试
                    
            except Exception as e:
                self.log_test_result(f"jog_cartesian_{axis}", False, f"异常: {e}")
                break

    def test_smooth_circle(self):
        """测试 smooth_circle"""
        print("\n⭕ 测试 smooth_circle")
        print("-" * 30)
        
        if not self.safe_workspace_center:
            self.log_test_result("smooth_circle", False, "无安全工作空间中心")
            return
        
        # 在当前位置附近画小圆
        center = self.safe_workspace_center.copy()
        radius = 8  # 很小的半径
        
        print(f"🎯 圆心: {[round(c, 2) for c in center]}, 半径: {radius}mm")
        
        try:
            result = smooth_circle(
                center=center,
                radius=radius,
                plane='XY',
                frame='WRF',
                duration=8.0,  # 较长时间，慢速执行
                wait_for_ack=True,
                timeout=15
            )
            
            success, details = self.wait_for_completion(result, timeout=15)
            self.log_test_result("smooth_circle", success, details)
            
            if success:
                time.sleep(1)
                
        except Exception as e:
            self.log_test_result("smooth_circle", False, f"异常: {e}")

    def test_smooth_arc_center(self):
        """测试 smooth_arc_center"""
        print("\n🌙 测试 smooth_arc_center")
        print("-" * 30)
        
        if not self.current_pose or not self.safe_workspace_center:
            self.log_test_result("smooth_arc_center", False, "缺少基准位置")
            return
        
        # 创建弧形路径
        center = self.safe_workspace_center.copy()
        end_pose = self.current_pose.copy()
        end_pose[0] += 15  # X方向偏移
        end_pose[1] += 10  # Y方向偏移
        
        if not self.is_pose_safe(end_pose):
            self.log_test_result("smooth_arc_center", False, "弧形终点不安全")
            return
        
        print(f"🎯 弧形: 中心{[round(c, 2) for c in center]} -> 终点{[round(p, 2) for p in end_pose[:3]]}")
        
        try:
            result = smooth_arc_center(
                end_pose=end_pose,
                center=center,
                frame='WRF',
                duration=6.0,
                wait_for_ack=True,
                timeout=12
            )
            
            success, details = self.wait_for_completion(result, timeout=12)
            self.log_test_result("smooth_arc_center", success, details)
            
            if success:
                time.sleep(1)
                
        except Exception as e:
            self.log_test_result("smooth_arc_center", False, f"异常: {e}")

    def test_smooth_spline(self):
        """测试 smooth_spline"""
        print("\n🌊 测试 smooth_spline")
        print("-" * 30)
        
        if not self.current_pose:
            self.log_test_result("smooth_spline", False, "无当前位姿")
            return
        
        # 创建样条路径的路径点
        waypoints = []
        
        # 起始点（当前位置小幅偏移）
        wp1 = self.create_safe_target_pose(offset_x=5, offset_y=5)
        wp2 = self.create_safe_target_pose(offset_x=10, offset_y=-5)  
        wp3 = self.create_safe_target_pose(offset_x=15, offset_y=0)
        
        if not all([wp1, wp2, wp3]):
            self.log_test_result("smooth_spline", False, "无法创建安全路径点")
            return
        
        waypoints = [wp1, wp2, wp3]
        print(f"🎯 样条路径: {len(waypoints)} 个路径点")
        
        try:
            result = smooth_spline(
                waypoints=waypoints,
                frame='WRF',
                duration=8.0,  # 较长时间
                wait_for_ack=True,
                timeout=15
            )
            
            success, details = self.wait_for_completion(result, timeout=15)
            self.log_test_result("smooth_spline", success, details)
            
            if success:
                time.sleep(1)
                
        except Exception as e:
            self.log_test_result("smooth_spline", False, f"异常: {e}")

    def test_smooth_helix(self):
        """测试 smooth_helix"""
        print("\n🌀 测试 smooth_helix")
        print("-" * 30)
        
        if not self.safe_workspace_center:
            self.log_test_result("smooth_helix", False, "无安全工作空间中心")
            return
        
        # 小螺旋参数
        center = self.safe_workspace_center.copy()
        center[2] -= 10  # 稍微降低中心高度
        radius = 6
        pitch = 5  # 螺距
        height = 15  # 总高度
        
        print(f"🎯 螺旋: 中心{[round(c, 2) for c in center]}, r={radius}mm, h={height}mm")
        
        try:
            result = smooth_helix(
                center=center,
                radius=radius,
                pitch=pitch,
                height=height,
                frame='WRF',
                duration=10.0,  # 更长时间
                wait_for_ack=True,
                timeout=18
            )
            
            success, details = self.wait_for_completion(result, timeout=18)
            self.log_test_result("smooth_helix", success, details)
            
            if success:
                time.sleep(1)
                
        except Exception as e:
            self.log_test_result("smooth_helix", False, f"异常: {e}")

    def run_all_tests(self):
        """运行所有测试"""
        print("🚀 开始修复后API完整测试")
        print("=" * 50)
        
        # 初始化
        if not self.initialize_safe_baseline():
            print("❌ 初始化失败，无法继续测试")
            return
        
        print("\n⚠️ 安全提醒:")
        print("• 测试将使用很小的移动距离和很慢的速度")
        print("• 确保机器人周围安全无障碍")
        print("• 急停按钮随时可用")
        
        if input("\n确认开始测试? (y/N): ").lower() != 'y':
            print("❌ 用户取消测试")
            return
        
        start_time = time.time()
        
        try:
            # 依次测试每个API
            test_functions = [
                ("move_robot_pose", self.test_move_robot_pose),
                ("move_robot_cartesian", self.test_move_robot_cartesian), 
                ("jog_cartesian", self.test_jog_cartesian),
                ("smooth_circle", self.test_smooth_circle),
                ("smooth_arc_center", self.test_smooth_arc_center),
                ("smooth_spline", self.test_smooth_spline),
                ("smooth_helix", self.test_smooth_helix),
            ]
            
            for api_name, test_func in test_functions:
                print(f"\n{'='*20} {api_name.upper()} {'='*20}")
                
                if input(f"测试 {api_name}? (Y/n/s=跳过全部): ").lower() in ['n', 's']:
                    if input("跳过全部剩余测试? (y/N): ").lower() == 'y':
                        break
                    continue
                
                test_func()
                
                # 测试间短暂休息
                time.sleep(1)
                
        except KeyboardInterrupt:
            print("\n🛑 测试被用户中断")
        except Exception as e:
            print(f"\n❌ 测试异常: {e}")
        finally:
            self.generate_test_report(time.time() - start_time)

    def generate_test_report(self, total_time):
        """生成测试报告"""
        print("\n" + "=" * 60)
        print("📊 修复后API测试报告")
        print("=" * 60)
        
        successful_apis = []
        failed_apis = []
        
        for api_name, result in self.test_results.items():
            if result['success']:
                successful_apis.append(api_name)
            else:
                failed_apis.append((api_name, result['details']))
        
        print(f"🕐 总测试时间: {total_time:.1f}秒")
        print(f"📈 测试统计:")
        print(f"   总测试API: {len(self.test_results)}")
        print(f"   ✅ 成功: {len(successful_apis)}")
        print(f"   ❌ 失败: {len(failed_apis)}")
        
        if self.test_results:
            success_rate = len(successful_apis) / len(self.test_results) * 100
            print(f"   📊 成功率: {success_rate:.1f}%")
        
        if successful_apis:
            print(f"\n✅ 成功的API:")
            for api in successful_apis:
                print(f"   • {api}")
        
        if failed_apis:
            print(f"\n❌ 失败的API:")
            for api, details in failed_apis:
                print(f"   • {api}: {details}")
        
        print(f"\n💡 总结:")
        if len(successful_apis) >= 5:
            print("   🎉 大部分API修复成功！")
        elif len(successful_apis) >= 2:
            print("   👍 部分API修复成功，还需进一步调整")
        else:
            print("   🔧 仍需要更多修复工作")
        
        print("=" * 60)

def main():
    """主程序"""
    print("🛡️ PAROL6 修复后API安全测试程序")
    print("专门测试ikine_LM修复后的API功能")
    
    tester = SafeAPITester()
    tester.run_all_tests()

if __name__ == "__main__":
    main()