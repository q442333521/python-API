#!/usr/bin/env python3
"""
PAROL6 Headless Commander 最小化启动器
直接初始化串口并运行原始程序
"""
import os
import sys
import serial
import platform

# 设置环境
sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
os.environ['PYTHONPATH'] = os.pathsep.join([
    os.path.dirname(os.path.dirname(os.path.abspath(__file__))),
    os.environ.get('PYTHONPATH', '')
])

# 初始化串口
print("🔧 初始化串口...")
try:
    ser = serial.Serial(
        port='/dev/ttyACM0',
        baudrate=3000000,
        timeout=0
    )
    com_port_str = '/dev/ttyACM0'
    print(f"✅ 串口连接成功: {com_port_str}")
except Exception as e:
    print(f"❌ 串口初始化失败: {e}")
    sys.exit(1)

# 设置全局变量
import builtins
builtins.ser = ser
builtins.com_port_str = com_port_str
builtins.my_os = platform.system()

# 运行原始程序
print("🚀 启动PAROL6 Headless Commander...")
print("-" * 50)

try:
    exec(open('headless_commander.py').read(), {
        '__name__': '__main__',
        'ser': ser,
        'com_port_str': com_port_str,
        'my_os': platform.system()
    })
except KeyboardInterrupt:
    print("\n⏹️ 程序被用户中断")
except Exception as e:
    print(f"\n❌ 运行错误: {e}")
    import traceback
    traceback.print_exc()
finally:
    if ser and ser.is_open:
        ser.close()
        print("🔌 串口已关闭")
