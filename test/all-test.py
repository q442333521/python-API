#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
PAROL6 API 全功能安全测试程序
系统性测试所有可用的API功能
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
import traceback
from robot_api import *

class PAROL6APITester:
    def __init__(self):
        """初始化API测试器"""
        self.test_results = {}
        self.safe_pose = [200, 0, 200, 180, 0, 90]  # 安全的测试位姿
        self.safe_joints = [0, -45, 90, 0, 45, 90]  # 安全的关节角度
        self.test_count = 0
        self.passed_count = 0
        self.failed_count = 0
        self.skipped_count = 0
        
        print("🔬 PAROL6 API 全功能测试器已初始化")
        print("⚠️  注意：本程序将测试所有API功能，请确保:")
        print("   1. 机器人周围安全无障碍物")
        print("   2. 急停按钮随时可用")
        print("   3. 有足够的工作空间")
        print("   4. 已连接夹具（如需测试夹具功能）")

    def log_test(self, test_name, result, details=""):
        """记录测试结果"""
        self.test_count += 1
        status = "✅ 通过" if result == "PASS" else "❌ 失败" if result == "FAIL" else "⏭️ 跳过"
        
        if result == "PASS":
            self.passed_count += 1
        elif result == "FAIL":
            self.failed_count += 1
        else:
            self.skipped_count += 1
            
        self.test_results[test_name] = {"status": result, "details": details}
        print(f"{status} {test_name}: {details}")

    def safe_execute(self, func, *args, **kwargs):
        """安全执行函数"""
        try:
            result = func(*args, **kwargs)
            return result, None
        except Exception as e:
            return None, str(e)

    def user_confirm(self, message, default="y"):
        """用户确认"""
        choice = input(f"{message} ({default}/n/s=跳过): ").lower() or default
        if choice == 's':
            return "skip"
        return choice == 'y'

    # ================================
    # 1. 状态查询API测试
    # ================================
    
    def test_status_apis(self):
        """测试所有状态查询API"""
        print("\n" + "="*50)
        print("🔍 测试状态查询API")
        print("="*50)
        
        # 1. 获取关节角度
        result, error = self.safe_execute(get_robot_joint_angles)
        if result is not None:
            self.log_test("get_robot_joint_angles", "PASS", f"关节角度: {[round(a,2) for a in result]}")
        else:
            self.log_test("get_robot_joint_angles", "FAIL", f"错误: {error}")
        
        # 2. 获取机器人位姿
        result, error = self.safe_execute(get_robot_pose)
        if result is not None:
            self.log_test("get_robot_pose", "PASS", f"位姿: {[round(p,2) for p in result]}")
        else:
            self.log_test("get_robot_pose", "FAIL", f"错误: {error}")
        
        # 3. 获取关节速度
        result, error = self.safe_execute(get_robot_joint_speeds)
        if result is not None:
            self.log_test("get_robot_joint_speeds", "PASS", f"速度: {result}")
        else:
            self.log_test("get_robot_joint_speeds", "FAIL", f"错误: {error}")
        
        # 4. 获取IO状态
        # result, error = self.safe_execute(get_io_status)
        # if result is not None:
        #     self.log_test("get_io_status", "PASS", f"IO状态: {result}")
        # else:
        #     self.log_test("get_io_status", "FAIL", f"错误: {error}")
        
        # 5. 获取电动夹具状态
        result, error = self.safe_execute(get_electric_gripper_status)
        if result is not None:
            self.log_test("get_electric_gripper_status", "PASS", f"夹具状态: {result}")
        else:
            self.log_test("get_electric_gripper_status", "FAIL", f"错误: {error}")
        
        # 6. 获取变换矩阵
        result, error = self.safe_execute(get_robot_pose_matrix)
        if result is not None:
            self.log_test("get_robot_pose_matrix", "PASS", "矩阵获取成功")
        else:
            self.log_test("get_robot_pose_matrix", "FAIL", f"错误: {error}")
        
        # 7. 检查是否停止
        result, error = self.safe_execute(is_robot_stopped)
        if result is not None:
            self.log_test("is_robot_stopped", "PASS", f"停止状态: {result}")
        else:
            self.log_test("is_robot_stopped", "FAIL", f"错误: {error}")
        
        # 8. 获取综合状态
        result, error = self.safe_execute(get_robot_status)
        if result is not None:
            self.log_test("get_robot_status", "PASS", "综合状态获取成功")
        else:
            self.log_test("get_robot_status", "FAIL", f"错误: {error}")

    # ================================
    # 2. 基础移动API测试
    # ================================
    
    def test_basic_movement_apis(self):
        """测试基础移动API"""
        print("\n" + "="*50)
        print("🤖 测试基础移动API")
        print("="*50)
        
        # 1. 归零测试
        if self.user_confirm("测试机器人归零?") == "skip":
            self.log_test("home_robot", "SKIP", "用户跳过")
        else:
            result, error = self.safe_execute(home_robot, wait_for_ack=True, timeout=30)
            if result and result.get('status') == 'COMPLETED':
                self.log_test("home_robot", "PASS", "归零成功")
                time.sleep(2)
            else:
                self.log_test("home_robot", "FAIL", f"归零失败: {error or result}")
        
        # 2. 关节移动测试
        if self.user_confirm("测试关节移动到安全位置?") == "skip":
            self.log_test("move_robot_joints", "SKIP", "用户跳过")
        else:
            result, error = self.safe_execute(
                move_robot_joints, 
                self.safe_joints, 
                speed_percentage=20, 
                wait_for_ack=True
            )
            if result and result.get('status') == 'COMPLETED':
                self.log_test("move_robot_joints", "PASS", f"移动到: {self.safe_joints}")
                time.sleep(1)
            else:
                self.log_test("move_robot_joints", "FAIL", f"失败: {error or result}")
        
        # 3. 位姿移动测试
        if self.user_confirm("测试位姿移动?") == "skip":
            self.log_test("move_robot_pose", "SKIP", "用户跳过")
        else:
            result, error = self.safe_execute(
                move_robot_pose, 
                self.safe_pose, 
                speed_percentage=20, 
                wait_for_ack=True
            )
            if result and result.get('status') == 'COMPLETED':
                self.log_test("move_robot_pose", "PASS", f"移动到: {self.safe_pose}")
                time.sleep(1)
            else:
                self.log_test("move_robot_pose", "FAIL", f"失败: {error or result}")
        
        # 4. 直线移动测试
        if self.user_confirm("测试笛卡尔直线移动?") == "skip":
            self.log_test("move_robot_cartesian", "SKIP", "用户跳过")
        else:
            target_pose = self.safe_pose.copy()
            target_pose[2] += 20  # Z轴上移20mm
            result, error = self.safe_execute(
                move_robot_cartesian, 
                target_pose, 
                speed_percentage=15, 
                wait_for_ack=True
            )
            if result and result.get('status') == 'COMPLETED':
                self.log_test("move_robot_cartesian", "PASS", "直线移动成功")
                time.sleep(1)
            else:
                self.log_test("move_robot_cartesian", "FAIL", f"失败: {error or result}")

    # ================================
    # 3. 点动API测试
    # ================================
    
    def test_jog_apis(self):
        """测试点动API"""
        print("\n" + "="*50)
        print("🕹️ 测试点动API")
        print("="*50)
        
        # 1. 单关节点动
        if self.user_confirm("测试单关节点动(底座+2度)?") == "skip":
            self.log_test("jog_robot_joint", "SKIP", "用户跳过")
        else:
            result, error = self.safe_execute(
                jog_robot_joint,
                joint_index=0,
                speed_percentage=15,
                distance_deg=2.0,
                wait_for_ack=True
            )
            if result and result.get('status') == 'COMPLETED':
                self.log_test("jog_robot_joint", "PASS", "底座点动+2度成功")
                time.sleep(1)
            else:
                self.log_test("jog_robot_joint", "FAIL", f"失败: {error or result}")
        
        # 2. 多关节点动
        if self.user_confirm("测试多关节同时点动?") == "skip":
            self.log_test("jog_multiple_joints", "SKIP", "用户跳过")
        else:
            result, error = self.safe_execute(
                jog_multiple_joints,
                joints=[0, 1],  # 底座和肩部
                speeds=[10, 10],
                duration=1.0,
                wait_for_ack=True
            )
            if result and result.get('status') == 'COMPLETED':
                self.log_test("jog_multiple_joints", "PASS", "多关节点动成功")
                time.sleep(1)
            else:
                self.log_test("jog_multiple_joints", "FAIL", f"失败: {error or result}")
        
        # 3. 笛卡尔点动
        if self.user_confirm("测试笛卡尔点动(Z轴向上)?") == "skip":
            self.log_test("jog_cartesian", "SKIP", "用户跳过")
        else:
            result, error = self.safe_execute(
                jog_cartesian,
                frame='WRF',
                axis='Z+',
                speed_percentage=15,
                duration=1.0,
                wait_for_ack=True
            )
            if result and result.get('status') == 'COMPLETED':
                self.log_test("jog_cartesian", "PASS", "笛卡尔点动成功")
                time.sleep(1)
            else:
                self.log_test("jog_cartesian", "FAIL", f"失败: {error or result}")

    # ================================
    # 4. 平滑运动API测试
    # ================================
    
    def test_smooth_motion_apis(self):
        """测试平滑运动API"""
        print("\n" + "="*50)
        print("🌊 测试平滑运动API")
        print("="*50)
        
        # 1. 圆形运动
        if self.user_confirm("测试圆形运动(小半径)?") == "skip":
            self.log_test("smooth_circle", "SKIP", "用户跳过")
        else:
            result, error = self.safe_execute(
                smooth_circle,
                center=[200, 0, 200],
                radius=20,
                plane='XY',
                duration=5.0,
                wait_for_ack=True
            )
            if result and result.get('status') == 'COMPLETED':
                self.log_test("smooth_circle", "PASS", "圆形运动成功")
                time.sleep(1)
            else:
                self.log_test("smooth_circle", "FAIL", f"失败: {error or result}")
        
        # 2. 圆弧运动(中心点)
        if self.user_confirm("测试圆弧运动?") == "skip":
            self.log_test("smooth_arc_center", "SKIP", "用户跳过")
        else:
            result, error = self.safe_execute(
                smooth_arc_center,
                end_pose=[220, 20, 200, 180, 0, 90],
                center=[210, 10, 200],
                duration=3.0,
                wait_for_ack=True
            )
            if result and result.get('status') == 'COMPLETED':
                self.log_test("smooth_arc_center", "PASS", "圆弧运动成功")
                time.sleep(1)
            else:
                self.log_test("smooth_arc_center", "FAIL", f"失败: {error or result}")
        
        # 3. 参数化圆弧
        if self.user_confirm("测试参数化圆弧?") == "skip":
            self.log_test("smooth_arc_parametric", "SKIP", "用户跳过")
        else:
            result, error = self.safe_execute(
                smooth_arc_parametric,
                end_pose=[200, 20, 200, 180, 0, 90],
                radius=15,
                arc_angle=90,
                duration=3.0,
                wait_for_ack=True
            )
            if result and result.get('status') == 'COMPLETED':
                self.log_test("smooth_arc_parametric", "PASS", "参数化圆弧成功")
                time.sleep(1)
            else:
                self.log_test("smooth_arc_parametric", "FAIL", f"失败: {error or result}")
        
        # 4. 样条曲线
        if self.user_confirm("测试样条曲线运动?") == "skip":
            self.log_test("smooth_spline", "SKIP", "用户跳过")
        else:
            waypoints = [
                [200, 0, 200, 180, 0, 90],
                [210, 10, 210, 180, 0, 90],
                [220, 0, 200, 180, 0, 90]
            ]
            result, error = self.safe_execute(
                smooth_spline,
                waypoints=waypoints,
                duration=5.0,
                wait_for_ack=True
            )
            if result and result.get('status') == 'COMPLETED':
                self.log_test("smooth_spline", "PASS", "样条曲线运动成功")
                time.sleep(1)
            else:
                self.log_test("smooth_spline", "FAIL", f"失败: {error or result}")
        
        # 5. 螺旋运动
        if self.user_confirm("测试螺旋运动?") == "skip":
            self.log_test("smooth_helix", "SKIP", "用户跳过")
        else:
            result, error = self.safe_execute(
                smooth_helix,
                center=[200, 0, 180],
                radius=15,
                pitch=10,
                height=30,
                duration=6.0,
                wait_for_ack=True
            )
            if result and result.get('status') == 'COMPLETED':
                self.log_test("smooth_helix", "PASS", "螺旋运动成功")
                time.sleep(1)
            else:
                self.log_test("smooth_helix", "FAIL", f"失败: {error or result}")

    # ================================
    # 5. 夹具控制API测试
    # ================================
    
    def test_gripper_apis(self):
        """测试夹具控制API"""
        print("\n" + "="*50)
        print("🤏 测试夹具控制API")
        print("="*50)
        
        # 1. 气动夹具控制
        if self.user_confirm("测试气动夹具控制?") == "skip":
            self.log_test("control_pneumatic_gripper", "SKIP", "用户跳过")
        else:
            # 打开夹具
            result, error = self.safe_execute(
                control_pneumatic_gripper,
                action='open',
                port=1,
                wait_for_ack=True
            )
            if result and result.get('status') == 'COMPLETED':
                self.log_test("control_pneumatic_gripper(open)", "PASS", "气动夹具打开成功")
                time.sleep(1)
                
                # 关闭夹具
                result, error = self.safe_execute(
                    control_pneumatic_gripper,
                    action='close',
                    port=1,
                    wait_for_ack=True
                )
                if result and result.get('status') == 'COMPLETED':
                    self.log_test("control_pneumatic_gripper(close)", "PASS", "气动夹具关闭成功")
                else:
                    self.log_test("control_pneumatic_gripper(close)", "FAIL", f"失败: {error or result}")
            else:
                self.log_test("control_pneumatic_gripper(open)", "FAIL", f"失败: {error or result}")
        
        # 2. 电动夹具控制
        if self.user_confirm("测试电动夹具控制?") == "skip":
            self.log_test("control_electric_gripper", "SKIP", "用户跳过")
        else:
            # 校准夹具
            result, error = self.safe_execute(
                control_electric_gripper,
                action='calibrate',
                wait_for_ack=True
            )
            if result and result.get('status') == 'COMPLETED':
                self.log_test("control_electric_gripper(calibrate)", "PASS", "电动夹具校准成功")
                time.sleep(2)
                
                # 移动夹具
                result, error = self.safe_execute(
                    control_electric_gripper,
                    action='move',
                    position=200,
                    speed=100,
                    wait_for_ack=True
                )
                if result and result.get('status') == 'COMPLETED':
                    self.log_test("control_electric_gripper(move)", "PASS", "电动夹具移动成功")
                else:
                    self.log_test("control_electric_gripper(move)", "FAIL", f"失败: {error or result}")
            else:
                self.log_test("control_electric_gripper(calibrate)", "FAIL", f"失败: {error or result}")

    # ================================
    # 6. 系统控制API测试
    # ================================
    
    def test_system_apis(self):
        """测试系统控制API"""
        print("\n" + "="*50)
        print("⚙️ 测试系统控制API")
        print("="*50)
        
        # 1. 延迟命令
        print("测试延迟命令(2秒)...")
        start_time = time.time()
        result, error = self.safe_execute(delay_robot, duration=2.0, wait_for_ack=True)
        elapsed = time.time() - start_time
        
        if result and result.get('status') == 'COMPLETED' and 1.8 <= elapsed <= 2.5:
            self.log_test("delay_robot", "PASS", f"延迟{elapsed:.1f}秒成功")
        else:
            self.log_test("delay_robot", "FAIL", f"失败: {error or result}, 用时{elapsed:.1f}秒")
        
        # 2. 停止运动(需要有运动时测试)
        print("测试停止命令...")
        result, error = self.safe_execute(stop_robot_movement, wait_for_ack=True)
        if result and result.get('status') == 'COMPLETED':
            self.log_test("stop_robot_movement", "PASS", "停止命令成功")
        else:
            self.log_test("stop_robot_movement", "FAIL", f"失败: {error or result}")

    # ================================
    # 7. 高级功能API测试
    # ================================
    
    def test_advanced_apis(self):
        """测试高级功能API"""
        print("\n" + "="*50)
        print("🚀 测试高级功能API")
        print("="*50)
        
        # 1. 轨迹执行
        if self.user_confirm("测试轨迹执行?") == "skip":
            self.log_test("execute_trajectory", "SKIP", "用户跳过")
        else:
            trajectory = [
                [200, 0, 200, 180, 0, 90],
                [210, 10, 200, 180, 0, 90],
                [200, 20, 200, 180, 0, 90]
            ]
            result, error = self.safe_execute(
                execute_trajectory,
                trajectory=trajectory,
                timing_mode='duration',
                timing_value=5.0,
                motion_type='spline',
                wait_for_ack=True
            )
            if result and result.get('status') == 'COMPLETED':
                self.log_test("execute_trajectory", "PASS", "轨迹执行成功")
            else:
                self.log_test("execute_trajectory", "FAIL", f"失败: {error or result}")
        
        # 2. 等待停止
        # result, error = self.safe_execute(wait_for_robot_stop, timeout=5.0)
        # if result is True:
        #     self.log_test("wait_for_robot_stop", "PASS", "等待停止成功")
        # else:
        #     self.log_test("wait_for_robot_stop", "FAIL", f"失败: {error}")
        
        # 3. 检查跟踪状态
        result, error = self.safe_execute(is_tracking_active)
        if result is not None:
            self.log_test("is_tracking_active", "PASS", f"跟踪状态: {result}")
        else:
            self.log_test("is_tracking_active", "FAIL", f"失败: {error}")
        
        # 4. 获取跟踪统计
        result, error = self.safe_execute(get_tracking_stats)
        if result is not None:
            self.log_test("get_tracking_stats", "PASS", f"跟踪统计获取成功")
        else:
            self.log_test("get_tracking_stats", "FAIL", f"失败: {error}")

    # ================================
    # 主测试流程
    # ================================
    
    def run_all_tests(self):
        """运行所有测试"""
        print("\n🚀 开始PAROL6 API全功能测试")
        print("测试将分为7个类别进行...")
        
        start_time = time.time()
        
        try:
            # 1. 状态查询测试(安全，总是执行)
            self.test_status_apis()
            
            # 2. 基础移动测试
            if self.user_confirm("\n是否测试基础移动API?") != False:
                self.test_basic_movement_apis()
            else:
                print("⏭️ 跳过基础移动测试")
            
            # 3. 点动测试
            if self.user_confirm("\n是否测试点动API?") != False:
                self.test_jog_apis()
            else:
                print("⏭️ 跳过点动测试")
            
            # 4. 平滑运动测试
            if self.user_confirm("\n是否测试平滑运动API?") != False:
                self.test_smooth_motion_apis()
            else:
                print("⏭️ 跳过平滑运动测试")
            
            # 5. 夹具控制测试
            if self.user_confirm("\n是否测试夹具控制API?") != False:
                self.test_gripper_apis()
            else:
                print("⏭️ 跳过夹具控制测试")
            
            # 6. 系统控制测试
            if self.user_confirm("\n是否测试系统控制API?") != False:
                self.test_system_apis()
            else:
                print("⏭️ 跳过系统控制测试")
            
            # 7. 高级功能测试
            if self.user_confirm("\n是否测试高级功能API?") != False:
                self.test_advanced_apis()
            else:
                print("⏭️ 跳过高级功能测试")
                
        except KeyboardInterrupt:
            print("\n🛑 测试被用户中断")
        except Exception as e:
            print(f"\n❌ 测试过程发生错误: {e}")
            traceback.print_exc()
        
        finally:
            # 生成测试报告
            self.generate_report(time.time() - start_time)

    def generate_report(self, total_time):
        """生成测试报告"""
        print("\n" + "="*60)
        print("📊 PAROL6 API 测试报告")
        print("="*60)
        
        print(f"🕐 总测试时间: {total_time:.1f}秒")
        print(f"📈 测试统计:")
        print(f"   总测试数: {self.test_count}")
        print(f"   ✅ 通过: {self.passed_count}")
        print(f"   ❌ 失败: {self.failed_count}")
        print(f"   ⏭️ 跳过: {self.skipped_count}")
        
        if self.test_count > 0:
            success_rate = (self.passed_count / (self.test_count - self.skipped_count)) * 100 if (self.test_count - self.skipped_count) > 0 else 0
            print(f"   📊 成功率: {success_rate:.1f}%")
        
        # 详细结果
        print(f"\n📋 详细测试结果:")
        for test_name, result in self.test_results.items():
            status_icon = {"PASS": "✅", "FAIL": "❌", "SKIP": "⏭️"}[result["status"]]
            print(f"   {status_icon} {test_name}: {result['details']}")
        
        # 失败分析
        if self.failed_count > 0:
            print(f"\n⚠️ 失败的测试:")
            for test_name, result in self.test_results.items():
                if result["status"] == "FAIL":
                    print(f"   ❌ {test_name}: {result['details']}")
        
        print(f"\n🎯 测试建议:")
        if self.failed_count == 0:
            print("   🎉 所有测试通过！API功能正常")
        else:
            print("   🔧 请检查失败的API功能")
            print("   📖 参考文档确认使用方法")
            print("   🔗 检查机器人连接和状态")
        
        print("="*60)

def main():
    """主程序"""
    print("🔬 PAROL6 API 全功能安全测试程序")
    print("="*50)
    
    # 创建测试器
    tester = PAROL6APITester()
    
    # 安全确认
    print("\n⚠️ 安全检查清单:")
    print("□ 机器人周围安全无障碍")
    print("□ 急停按钮可随时按下")
    print("□ 工作空间足够大")
    print("□ 已做好应急准备")
    
    if not tester.user_confirm("\n确认以上安全条件已满足，开始测试?"):
        print("❌ 用户取消测试")
        return
    
    try:
        # 运行所有测试
        tester.run_all_tests()
        
    except KeyboardInterrupt:
        print("\n🛑 程序被用户中断")
    except Exception as e:
        print(f"\n❌ 程序错误: {e}")
        traceback.print_exc()
    finally:
        print("\n👋 测试程序结束")

if __name__ == "__main__":
    main()