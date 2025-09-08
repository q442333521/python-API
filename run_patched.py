#!/usr/bin/env python3
"""
PAROL6 Headless Commander - 完整Linux修补版
修复原始文件缺少Linux串口初始化的问题
"""
import os
import sys
import serial
import platform
import serial.tools.list_ports

# 添加上级目录到Python路径
sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

def patch_headless_commander():
    """修补headless_commander.py以支持Linux"""
    
    # 读取原始文件
    with open('headless_commander.py', 'r') as f:
        content = f.read()
    
    # 找到Windows串口初始化的位置
    windows_init_start = content.find('my_os = platform.system()')
    if windows_init_start == -1:
        print("❌ 找不到操作系统检测代码")
        return None
    
    # 找到结束位置（第一个int_to_3_bytes定义）
    init_end = content.find('# in big endian machines')
    if init_end == -1:
        print("❌ 找不到初始化代码结束位置")
        return None
    
    # 创建新的初始化代码（支持Windows和Linux）
    new_init_code = '''
my_os = platform.system()

# Initialize serial port based on OS
ser = None
com_port_str = None

if my_os == "Windows":
    # Windows serial port initialization
    try:
        with open("com_port.txt", "r") as f:
            com_port_str = f.read().strip()
            ser = serial.Serial(port=com_port_str, baudrate=3000000, timeout=0)
            print(f"Connected to saved COM port: {com_port_str}")
    except (FileNotFoundError, serial.SerialException):
        while True:
            try:
                com_port = input("Enter the COM port (e.g., COM9): ")
                ser = serial.Serial(port=com_port, baudrate=3000000, timeout=0)
                com_port_str = com_port
                print(f"Successfully connected to {com_port}")
                with open("com_port.txt", "w") as f:
                    f.write(com_port)
                break
            except serial.SerialException:
                print(f"Could not open port {com_port}. Please try again.")

else:  # Linux/Mac
    # Linux serial port initialization
    print("🔧 Initializing serial port for Linux/Mac...")
    
    # Try to read saved port
    try:
        with open("com_port.txt", "r") as f:
            com_port_str = f.read().strip()
            if os.path.exists(com_port_str):
                ser = serial.Serial(port=com_port_str, baudrate=3000000, timeout=0)
                print(f"✅ Connected to saved port: {com_port_str}")
    except (FileNotFoundError, serial.SerialException) as e:
        print(f"⚠️ Could not connect to saved port: {e}")
        ser = None
    
    # If not connected, try default ports
    if ser is None:
        default_ports = ["/dev/ttyACM0", "/dev/ttyUSB0", "/dev/ttyACM1", "/dev/ttyUSB1"]
        for port in default_ports:
            if os.path.exists(port):
                try:
                    ser = serial.Serial(port=port, baudrate=3000000, timeout=0)
                    com_port_str = port
                    print(f"✅ Connected to {port}")
                    with open("com_port.txt", "w") as f:
                        f.write(port)
                    break
                except serial.SerialException as e:
                    print(f"❌ Failed to connect to {port}: {e}")
    
    # If still not connected, list available ports
    if ser is None:
        print("\\n📡 Available serial ports:")
        ports = serial.tools.list_ports.comports()
        for port in ports:
            print(f"  • {port.device} - {port.description}")
        
        # Try to connect to the first available ACM/USB port
        for port in ports:
            if "ACM" in port.device or "USB" in port.device:
                try:
                    ser = serial.Serial(port=port.device, baudrate=3000000, timeout=0)
                    com_port_str = port.device
                    print(f"✅ Connected to {port.device}")
                    with open("com_port.txt", "w") as f:
                        f.write(port.device)
                    break
                except Exception:
                    pass
        
        if ser is None:
            print("\\n❌ Could not connect to any serial port!")
            print("Please check:")
            print("1. Robot is connected and powered on")
            print("2. User has permission (sudo usermod -a -G dialout $USER)")
            print("3. No other program is using the serial port")
            sys.exit(1)

print(f"📡 Using serial port: {com_port_str}")
print(f"⚡ Baudrate: 3000000")
print("🤖 PAROL6 Headless Commander Ready\\n")

'''
    
    # 替换初始化代码
    new_content = (
        content[:windows_init_start] + 
        new_init_code + 
        content[init_end:]
    )
    
    return new_content

def main():
    print("=" * 60)
    print("🔧 PAROL6 Headless Commander - Linux修补器")
    print("=" * 60)
    
    # 修补文件
    patched_content = patch_headless_commander()
    if patched_content is None:
        print("❌ 修补失败")
        return 1
    
    # 保存修补后的文件
    with open('headless_commander_patched.py', 'w') as f:
        f.write(patched_content)
    
    print("✅ 已创建修补文件: headless_commander_patched.py")
    print("\n🚀 正在运行修补后的程序...")
    print("-" * 60)
    
    # 运行修补后的程序
    try:
        exec(patched_content, {'__name__': '__main__'})
    except KeyboardInterrupt:
        print("\n⏹️ 程序被用户中断")
    except Exception as e:
        print(f"\n❌ 运行错误: {e}")
        import traceback
        traceback.print_exc()
    
    return 0

if __name__ == "__main__":
    sys.exit(main())
