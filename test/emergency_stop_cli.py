#!/usr/bin/env python3
"""
PAROL6快速急停命令行工具
"""

import sys
sys.path.append('09-PAROL6-python-API')

try:
    from robot_api import stop_robot_movement
    import rclpy
    from std_msgs.msg import String, Bool
    
    def emergency_stop():
        print("执行急停...")
        
        # 1. API直接停止
        try:
            result = stop_robot_movement(wait_for_ack=True, timeout=3)
            if result:
                print("✅ API急停成功")
            else:
                print("❌ API急停失败")
        except Exception as e:
            print(f"❌ API急停异常: {e}")
        
        # 2. ROS急停
        try:
            rclpy.init()
            node = rclpy.create_node('emergency_stop_cli')
            
            emergency_pub = node.create_publisher(Bool, '/parol6/emergency_stop', 10)
            cmd_pub = node.create_publisher(String, '/parol6/api_command', 10)
            
            # 发布急停信号
            emergency_msg = Bool()
            emergency_msg.data = True
            emergency_pub.publish(emergency_msg)
            
            cmd_msg = String()
            cmd_msg.data = 'emergency_stop'
            cmd_pub.publish(cmd_msg)
            
            print("✅ ROS急停信号已发送")
            
            node.destroy_node()
            rclpy.shutdown()
            
        except Exception as e:
            print(f"❌ ROS急停异常: {e}")
    
    if __name__ == "__main__":
        emergency_stop()
        
except ImportError as e:
    print(f"导入失败: {e}")
    print("请确保PAROL6 API和ROS2环境正确配置")