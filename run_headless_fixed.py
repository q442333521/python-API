#!/usr/bin/env python3
"""
PAROL6 Headless Commander 启动器 - Linux修复版
修复原始脚本中缺少Linux串口初始化的问题
"""
import os
import sys
import platform
import serial
import serial.tools.list_ports

def init_serial():
    """初始化串口连接"""
    print("Initializing serial connection...")
    
    my_os = platform.system()
    ser = None
    
    if my_os != "Windows":
        # Linux/Mac系统
        try:
            # 尝试从文件读取串口配置
            with open("com_port.txt", "r") as f:
                com_port_str = f.read().strip()
                ser = serial.Serial(port=com_port_str, baudrate=3000000, timeout=0)
                print(f"Connected to saved serial port: {com_port_str}")
        except (FileNotFoundError, serial.SerialException) as e:
            print(f"Could not connect to saved port: {e}")
            # 尝试默认端口
            default_ports = ["/dev/ttyACM0", "/dev/ttyUSB0", "/dev/ttyACM1", "/dev/ttyUSB1"]
            for port in default_ports:
                try:
                    ser = serial.Serial(port=port, baudrate=3000000, timeout=0)
                    print(f"Successfully connected to {port}")
                    # 保存成功的端口到文件
                    with open("com_port.txt", "w") as f:
                        f.write(port)
                    break
                except serial.SerialException as e:
                    print(f"Failed to connect to {port}: {e}")
                    continue
            
            if ser is None:
                print("\nError: Could not connect to any serial port.")
                print("\nAvailable serial ports:")
                ports = serial.tools.list_ports.comports()
                if ports:
                    for port in ports:
                        print(f"  {port.device} - {port.description}")
                else:
                    print("  No serial ports found!")
                
                print("\nTroubleshooting:")
                print("1. Make sure the robot is connected and powered on")
                print("2. Check if user has permission to access serial ports:")
                print("   sudo usermod -a -G dialout $USER")
                print("   (then logout and login again)")
                print("3. Check device permissions:")
                print("   ls -l /dev/ttyACM* /dev/ttyUSB*")
                print("4. Try running with sudo (temporary fix):")
                print("   sudo -E python run_headless_fixed.py")
                return None
    
    return ser

def run_original_with_ser(ser):
    """运行原始脚本，预先定义ser变量"""
    # 将ser变量注入全局命名空间
    globals()['ser'] = ser
    
    # 读取原始脚本内容
    with open('headless_commander.py', 'r') as f:
        script_content = f.read()
    
    # 在全局命名空间中执行脚本
    print("Starting PAROL6 headless commander...")
    exec(script_content, globals())

if __name__ == "__main__":
    # 初始化串口
    ser = init_serial()
    
    if ser is not None:
        print(f"Serial port initialized successfully: {ser.port}")
        try:
            run_original_with_ser(ser)
        except KeyboardInterrupt:
            print("\nProgram interrupted by user")
            if ser and ser.is_open:
                ser.close()
        except Exception as e:
            print(f"Error running program: {e}")
            if ser and ser.is_open:
                ser.close()
    else:
        print("Failed to initialize serial connection. Exiting.")
        sys.exit(1)

