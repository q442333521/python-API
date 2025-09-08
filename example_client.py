# safe_home_client.py
import time
from robot_api import (
    connect, disconnect, is_connected,
    stop_robot, clear_queue, home_robot, get_state
)

def main():
    # 1) 连接（如果 robot_api 需要 IP/端口，替换 connect(host, port) 版本）
    connect()
    assert is_connected(), "连接失败，请检查 headless_commander 是否在运行"

    # 2) 安全：停止并清队列
    try:
        stop_robot()    # 某些实现是软停；若无该函数可忽略
    except Exception:
        pass
    try:
        clear_queue()
    except Exception:
        pass

    # 3) 读取当前状态并打印（确认未在运动）
    try:
        s = get_state()
        print("当前状态：", s)
    except Exception as e:
        print("读取状态失败，不影响 Home：", e)

    # 4) 低速 Home（关键！）
    print("开始低速 Home ...")
    home_robot(speed_percentage=10, wait_for_ack=True, timeout=120)

    # 5) 等待稳定
    time.sleep(1.0)
    print("Home 指令完成。再次读取状态：")
    try:
        s2 = get_state()
        print("状态：", s2)
    except Exception:
        pass

    # 6) 断开
    disconnect()
    print("完成")

if __name__ == "__main__":
    main()
