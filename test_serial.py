#!/usr/bin/env python3
"""
PAROL6 串口连接测试脚本
用于验证串口通信是否正常
"""
import serial
import serial.tools.list_ports
import time
import sys
import os

def test_serial_connection():
    """测试串口连接"""
    print("🔍 PAROL6 串口连接测试")
    print("-" * 40)
    
    # 列出所有可用串口
    print("\n📡 可用的串口:")
    ports = serial.tools.list_ports.comports()
    
    if not ports:
        print("  ❌ 未检测到任何串口设备")
        return False
    
    for port in ports:
        print(f"  • {port.device} - {port.description}")
    
    # 测试串口权限
    print("\n🔑 检查串口权限:")
    test_ports = ["/dev/ttyACM0", "/dev/ttyUSB0"]
    
    for port_name in test_ports:
        if os.path.exists(port_name):
            # 检查文件权限
            import stat
            st = os.stat(port_name)
            mode = st.st_mode
            
            # 检查是否可读写
            if os.access(port_name, os.R_OK | os.W_OK):
                print(f"  ✅ {port_name} - 可读写")
            else:
                print(f"  ❌ {port_name} - 权限不足")
                print(f"     运行: sudo chmod 666 {port_name}")
                print(f"     或: sudo usermod -a -G dialout $USER")
    
    # 尝试连接
    print("\n🔌 尝试连接串口:")
    
    for port in ports:
        if "ACM" in port.device or "USB" in port.device:
            print(f"\n尝试连接 {port.device}...")
            try:
                ser = serial.Serial(
                    port=port.device,
                    baudrate=3000000,
                    timeout=1,
                    write_timeout=1
                )
                
                print(f"  ✅ 成功打开 {port.device}")
                
                # 发送测试命令
                print("  📤 发送测试命令...")
                ser.write(b"READY\n")
                time.sleep(0.5)
                
                # 读取响应
                if ser.in_waiting > 0:
                    response = ser.read(ser.in_waiting)
                    print(f"  📥 收到响应: {response}")
                else:
                    print("  ⚠️  未收到响应（设备可能未准备好）")
                
                ser.close()
                print(f"  ✅ 串口测试成功!")
                return True
                
            except serial.SerialException as e:
                print(f"  ❌ 连接失败: {e}")
            except PermissionError as e:
                print(f"  ❌ 权限错误: {e}")
                print(f"     请运行: sudo chmod 666 {port.device}")
                print(f"     或使用: sudo python {sys.argv[0]}")
    
    print("\n❌ 无法连接到任何串口")
    return False

if __name__ == "__main__":
    print("=" * 50)
    print("PAROL6 串口诊断工具")
    print("=" * 50)
    
    success = test_serial_connection()
    
    print("\n" + "=" * 50)
    if success:
        print("✅ 测试通过！可以运行主程序:")
        print("   python run_parol6_fixed.py")
    else:
        print("❌ 测试失败，请检查:")
        print("   1. 机械臂是否连接并上电")
        print("   2. USB线缆是否正常")
        print("   3. 用户权限是否正确")
        print("   4. 尝试使用sudo运行")
    print("=" * 50)
