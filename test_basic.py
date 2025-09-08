#!/usr/bin/env python3
"""
PAROL6 简单功能测试
测试基本的串口通信和机械臂控制
"""
import sys
import os
import time
import serial
import socket
import threading

# 添加上级目录到Python路径
sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

def test_basic_functions():
    """测试基本功能"""
    print("=" * 60)
    print("PAROL6 基本功能测试")
    print("=" * 60)
    
    # 1. 测试串口连接
    print("\n1️⃣ 测试串口连接...")
    try:
        ser = serial.Serial(
            port='/dev/ttyACM0',
            baudrate=3000000,
            timeout=0.5
        )
        print(f"   ✅ 串口打开成功: {ser.port}")
        
        # 尝试发送一些基本命令
        test_commands = [
            b"READY\n",
            b"HOME\n",
            b"STATUS\n"
        ]
        
        for cmd in test_commands:
            print(f"   📤 发送: {cmd.decode().strip()}")
            ser.write(cmd)
            time.sleep(0.1)
            
            if ser.in_waiting > 0:
                response = ser.read(ser.in_waiting)
                print(f"   📥 响应: {response}")
            else:
                print(f"   ⚠️  无响应")
        
        ser.close()
        print("   ✅ 串口测试完成")
        
    except Exception as e:
        print(f"   ❌ 串口错误: {e}")
        return False
    
    # 2. 测试UDP端口
    print("\n2️⃣ 测试UDP端口...")
    try:
        sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        sock.settimeout(1.0)
        sock.bind(('0.0.0.0', 5001))
        print(f"   ✅ UDP端口5001绑定成功")
        
        # 尝试接收数据（1秒超时）
        print("   ⏳ 等待UDP数据（1秒）...")
        try:
            data, addr = sock.recvfrom(1024)
            print(f"   📥 收到数据: {data} from {addr}")
        except socket.timeout:
            print("   ⚠️  无UDP数据（正常）")
        
        sock.close()
        print("   ✅ UDP测试完成")
        
    except Exception as e:
        print(f"   ❌ UDP错误: {e}")
    
    # 3. 测试导入模块
    print("\n3️⃣ 测试导入模块...")
    try:
        from GUI.files import PAROL6_ROBOT
        print("   ✅ PAROL6_ROBOT模块导入成功")
        
        import robot_api
        print("   ✅ robot_api模块导入成功")
        
        import smooth_motion
        print("   ✅ smooth_motion模块导入成功")
        
    except Exception as e:
        print(f"   ❌ 模块导入错误: {e}")
    
    print("\n" + "=" * 60)
    print("测试完成")
    print("=" * 60)
    return True

if __name__ == "__main__":
    test_basic_functions()
