#!/usr/bin/env python3
"""
启动器脚本来修复headless_commander.py中的Linux串口问题
"""
import platform
import serial
import serial.tools.list_ports

def initialize_serial_for_linux():
    """为Linux系统初始化串口连接"""
    ser = None
    
    # 尝试从文件读取串口配置
    try:
        with open("com_port.txt", "r") as f:
            com_port_str = f.read().strip()
            ser = serial.Serial(port=com_port_str, baudrate=3000000, timeout=0)
            print(f"Connected to saved serial port: {com_port_str}")
            return ser
    except (FileNotFoundError, serial.SerialException) as e:
        print(f"Failed to connect to saved port: {e}")
    
    # 尝试默认端口
    default_ports = ["/dev/ttyACM0", "/dev/ttyUSB0", "/dev/ttyACM1", "/dev/ttyUSB1"]
    for port in default_ports:
        try:
            ser = serial.Serial(port=port, baudrate=3000000, timeout=0)
            print(f"Successfully connected to {port}")
            # 保存成功的端口到文件
            with open("com_port.txt", "w") as f:
                f.write(port)
            return ser
        except serial.SerialException:
            print(f"Failed to connect to {port}")
            continue
    
    # 如果都失败了，显示可用端口并要求用户输入
    print("Could not find any available serial ports.")
    ports = serial.tools.list_ports.comports()
    if ports:
        print("Available ports:")
        for port in ports:
            print(f"  {port.device}")
    else:
        print("No serial ports detected!")
    
    while True:
        try:
            com_port = input("Enter the serial port (e.g., /dev/ttyACM0): ").strip()
            if not com_port:
                continue
            ser = serial.Serial(port=com_port, baudrate=3000000, timeout=0)
            print(f"Successfully connected to {com_port}")
            # 保存成功的端口到文件
            with open("com_port.txt", "w") as f:
                f.write(com_port)
            return ser
        except serial.SerialException as e:
            print(f"Could not open port {com_port}: {e}. Please try again.")

if __name__ == "__main__":
    print("Initializing serial connection for Linux...")
    
    my_os = platform.system()
    if my_os != "Windows":
        ser = initialize_serial_for_linux()
        if ser:
            # 将ser变量注入到全局命名空间，然后导入原始脚本
            import sys
            import os
            
            # 临时将ser添加到builtins中，这样原始脚本就能看到它
            import builtins
            builtins.ser = ser
            
            print("Serial connection established. Starting main program...")
            
            # 现在执行原始脚本
            exec(compile(open("headless_commander.py").read(), "headless_commander.py", 'exec'))
        else:
            print("Failed to establish serial connection!")
            sys.exit(1)
    else:
        print("This is a Linux-specific fix. On Windows, run headless_commander.py directly.")
        sys.exit(1)

