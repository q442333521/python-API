# safe_base_test.py
import time
from robot_api import (
    connect, disconnect, is_connected,
    clear_queue, stop_robot, get_state,
    move_robot_joints, home_robot  # 若有 home_joint 可改用它
)

DEG2RAD = 3.141592653589793 / 180.0

def main():
    # 1) 连接
    connect()  # 若你的实现需要IP/端口，改成 connect("192.168.x.x", 5001)
    assert is_connected(), "连接失败，请检查 headless_commander 是否已运行"

    # 2) 安全：软停 + 清队列（如函数不存在可注释）
    try: stop_robot()
    except: pass
    try: clear_queue()
    except: pass

    # 3) 读取当前关节角（弧度）
    st = get_state()  # 期待返回有 joints 列表，单位通常为度或弧度，视实现而定
    joints = list(st["joints"] if isinstance(st, dict) else st.joints)

    # 4) 只改 J1（base）一点点：+5°
    # 如果 get_state 返回的是度，请去掉 *DEG2RAD；如果是弧度，保留如下：
    target = joints[:]
    target[0] = target[0] + 5.0 * DEG2RAD  # 基座 +5°

    print("准备以 5% 速度缓慢转动基座 +5° ...")
    move_robot_joints(
        target,
        speed_percentage=5,       # 很慢
        wait_for_ack=True,        # 等确认
        non_blocking=False        # 阻塞到完成（更安全）
    )

    time.sleep(0.5)
    print("基座小角度动作完成。开始回 Home（低速）...")
    # 若有按关节回零API，可替换为 home_joint(joint_index=0, speed_percentage=5, wait_for_ack=True)
    home_robot(speed_percentage=5, wait_for_ack=True, timeout=120)

    disconnect()
    print("完成。")

if __name__ == "__main__":
    main()
