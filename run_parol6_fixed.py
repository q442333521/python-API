#!/usr/bin/env python3
"""
PAROL6 Headless Commander - 完整Linux修复版
修复所有串口连接问题并提供稳定的运行环境
"""
import os
import sys
import platform
import serial
import serial.tools.list_ports
import time
import threading

# 添加上级目录到Python路径（用于导入GUI模块）
sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

def check_permissions():
    """检查串口权限"""
    import grp
    import pwd
    
    username = pwd.getpwuid(os.getuid()).pw_name
    groups = [g.gr_name for g in grp.getgrall() if username in g.gr_mem]
    
    if 'dialout' not in groups:
        print("⚠️  警告: 用户不在dialout组中")
        print("   请运行: sudo usermod -a -G dialout $USER")
        print("   然后重新登录")
        return False
    return True

def init_serial_connection():
    """初始化串口连接"""
    print("🔧 初始化Linux串口连接...")
    
    ser = None
    com_port_str = None
    
    # 1. 检查权限
    if not check_permissions():
        print("⚠️  继续尝试连接（可能需要sudo）...")
    
    # 2. 尝试从文件读取上次使用的端口
    try:
        with open("com_port.txt", "r") as f:
            saved_port = f.read().strip()
            if os.path.exists(saved_port):
                try:
                    ser = serial.Serial(
                        port=saved_port,
                        baudrate=3000000,
                        timeout=0.1,
                        write_timeout=0.1
                    )
                    com_port_str = saved_port
                    print(f"✅ 连接到保存的端口: {saved_port}")
                    return ser, com_port_str
                except serial.SerialException as e:
                    print(f"⚠️  无法连接到保存的端口 {saved_port}: {e}")
    except FileNotFoundError:
        pass
    
    # 3. 自动检测可用端口
    print("🔍 扫描可用串口...")
    
    # 默认端口列表
    default_ports = ["/dev/ttyACM0", "/dev/ttyUSB0", "/dev/ttyACM1", "/dev/ttyUSB1"]
    
    for port in default_ports:
        if os.path.exists(port):
            print(f"  尝试 {port}...")
            try:
                ser = serial.Serial(
                    port=port,
                    baudrate=3000000,
                    timeout=0.1,
                    write_timeout=0.1
                )
                com_port_str = port
                print(f"✅ 成功连接到 {port}")
                
                # 保存成功的端口
                with open("com_port.txt", "w") as f:
                    f.write(port)
                
                return ser, com_port_str
            except serial.SerialException as e:
                print(f"  ❌ 无法连接到 {port}: {e}")
                continue
    
    # 4. 列出所有可用串口供选择
    print("\n📡 可用的串口设备:")
    ports = serial.tools.list_ports.comports()
    
    if not ports:
        print("  未检测到任何串口设备!")
        print("\n排查步骤:")
        print("1. 确认机械臂已连接并上电")
        print("2. 检查USB线缆连接")
        print("3. 运行 'lsusb' 查看USB设备")
        return None, None
    
    for i, port in enumerate(ports):
        print(f"  [{i}] {port.device} - {port.description}")
    
    # 5. 让用户选择（或自动尝试所有）
    for port in ports:
        if "ACM" in port.device or "USB" in port.device:
            try:
                ser = serial.Serial(
                    port=port.device,
                    baudrate=3000000,
                    timeout=0.1,
                    write_timeout=0.1
                )
                com_port_str = port.device
                print(f"✅ 成功连接到 {port.device}")
                
                with open("com_port.txt", "w") as f:
                    f.write(port.device)
                
                return ser, com_port_str
            except Exception as e:
                continue
    
    print("\n❌ 无法连接到任何串口")
    return None, None


def run_headless_commander():
    """运行headless commander主程序"""
    print("\n🚀 启动PAROL6 Headless Commander...")
    
    # 初始化串口
    ser, com_port_str = init_serial_connection()
    
    if ser is None or com_port_str is None:
        print("❌ 无法初始化串口连接")
        print("\n可能的解决方案:")
        print("1. 使用sudo运行: sudo -E python run_parol6_fixed.py")
        print("2. 添加用户到dialout组: sudo usermod -a -G dialout $USER")
        print("3. 检查设备连接和电源")
        return False
    
    print(f"📡 使用串口: {com_port_str}")
    print(f"⚡ 波特率: 3000000")
    
    # 准备运行原始脚本
    script_path = os.path.join(os.path.dirname(__file__), 'headless_commander.py')
    
    if not os.path.exists(script_path):
        print(f"❌ 找不到 {script_path}")
        return False
    
    # 读取原始脚本
    with open(script_path, 'r') as f:
        script_content = f.read()
    
    # 设置全局变量
    script_globals = {
        '__name__': '__main__',
        '__file__': script_path,
        'ser': ser,
        'com_port_str': com_port_str,
        'my_os': platform.system()
    }
    
    # 添加所需的导入
    import_code = """
import serial
import serial.tools.list_ports
import platform
import sys
import os
import time
sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
"""
    
    print("\n✅ 串口连接就绪")
    print("🤖 按Ctrl+C停止程序\n")
    print("-" * 50)
    
    try:
        # 先执行导入
        exec(import_code, script_globals)
        # 然后执行主脚本
        exec(script_content, script_globals)
        return True
    except KeyboardInterrupt:
        print("\n\n⏹️  用户中断程序")
        return True
    except Exception as e:
        print(f"\n❌ 运行错误: {e}")
        import traceback
        traceback.print_exc()
        return False
    finally:
        # 清理
        if ser and ser.is_open:
            print("🔌 关闭串口连接...")
            ser.close()


def main():
    """主函数"""
    print("=" * 60)
    print("🤖 PAROL6 Headless Commander - Linux完整修复版")
    print("=" * 60)
    
    # 检查操作系统
    if platform.system() == "Windows":
        print("ℹ️  这是Linux修复版。Windows请直接运行headless_commander.py")
        return 1
    
    # 检查Python版本
    if sys.version_info < (3, 6):
        print(f"❌ 需要Python 3.6或更高版本，当前版本: {sys.version}")
        return 1
    
    # 检查必要的依赖
    try:
        import serial
        import numpy
        import roboticstoolbox
    except ImportError as e:
        print(f"❌ 缺少依赖: {e}")
        print("请运行: pip install pyserial numpy roboticstoolbox-python")
        return 1
    
    # 运行主程序
    if run_headless_commander():
        print("\n✅ 程序正常退出")
        return 0
    else:
        print("\n❌ 程序异常退出")
        return 1

if __name__ == "__main__":
    sys.exit(main())
