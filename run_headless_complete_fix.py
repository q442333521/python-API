#!/usr/bin/env python3
"""
PAROL6 Headless Commander 完整修复版
修复所有Linux串口相关问题，包括com_port_str变量定义
"""
import os
import sys
import platform
import serial
import serial.tools.list_ports
import time

def init_serial_connection():
    """初始化串口连接并返回串口对象和端口字符串"""
    print("🔧 Initializing serial connection for Linux...")
    
    ser = None
    com_port_str = None
    
    # 尝试从文件读取串口配置
    try:
        with open("com_port.txt", "r") as f:
            com_port_str = f.read().strip()
            ser = serial.Serial(port=com_port_str, baudrate=3000000, timeout=0)
            print(f"✅ Connected to saved serial port: {com_port_str}")
            return ser, com_port_str
    except (FileNotFoundError, serial.SerialException) as e:
        print(f"⚠️ Could not connect to saved port: {e}")
    
    # 尝试默认端口
    default_ports = ["/dev/ttyACM0", "/dev/ttyUSB0", "/dev/ttyACM1", "/dev/ttyUSB1"]
    print("🔍 Trying default ports...")
    
    for port in default_ports:
        if os.path.exists(port):
            try:
                ser = serial.Serial(port=port, baudrate=3000000, timeout=0)
                com_port_str = port
                print(f"✅ Successfully connected to {port}")
                # 保存成功的端口到文件
                with open("com_port.txt", "w") as f:
                    f.write(port)
                return ser, com_port_str
            except serial.SerialException as e:
                print(f"❌ Failed to connect to {port}: {e}")
                continue
        else:
            print(f"⚠️ Port {port} does not exist")
    
    # 如果都失败了，显示错误信息
    print("\n❌ Could not connect to any serial port.")
    print("\nAvailable serial ports:")
    ports = serial.tools.list_ports.comports()
    if ports:
        for port in ports:
            print(f"  📡 {port.device} - {port.description}")
    else:
        print("  No serial ports detected!")
    
    print("\n🔧 Troubleshooting:")
    print("1. Make sure the robot is connected and powered on")
    print("2. Check if user has permission to access serial ports:")
    print("   sudo usermod -a -G dialout $USER")
    print("   (then logout and login again)")
    print("3. Try running with sudo (temporary fix):")
    print("   sudo -E python run_headless_complete_fix.py")
    
    return None, None

def patch_and_run_original():
    """读取、修复并运行原始脚本"""
    # 初始化串口连接
    ser, com_port_str = init_serial_connection()
    
    if ser is None or com_port_str is None:
        print("❌ Failed to initialize serial connection. Exiting.")
        sys.exit(1)
    
    print(f"✅ Serial connection established: {ser.port}")
    
    # 读取原始脚本
    with open('headless_commander.py', 'r') as f:
        script_content = f.read()
    
    # 创建修复后的全局命名空间
    script_globals = globals().copy()
    script_globals.update({
        'ser': ser,
        'com_port_str': com_port_str,
        'my_os': platform.system()
    })
    
    print("🚀 Starting PAROL6 headless commander...")
    print(f"📡 Using serial port: {com_port_str}")
    print("🔄 Press Ctrl+C to stop\n")
    
    try:
        # 执行原始脚本
        exec(script_content, script_globals)
    except KeyboardInterrupt:
        print("\n⏹️ Program interrupted by user")
    except Exception as e:
        print(f"\n❌ Error running program: {e}")
        import traceback
        traceback.print_exc()
    finally:
        # 清理串口连接
        if ser and ser.is_open:
            print("🔌 Closing serial connection...")
            ser.close()

if __name__ == "__main__":
    print("🤖 PAROL6 Headless Commander - Linux Complete Fix")
    print("=" * 50)
    
    # 检查是否在Linux系统上
    if platform.system() == "Windows":
        print("ℹ️ This is a Linux-specific fix. On Windows, run headless_commander.py directly.")
        sys.exit(1)
    
    # 检查文件是否存在
    if not os.path.exists('headless_commander.py'):
        print("❌ headless_commander.py not found in current directory!")
        sys.exit(1)
    
    patch_and_run_original()

