# d:/Onedrive/p/09-PAROL6-python-API/test/testik.py
import numpy as np
import spatialmath as sm
from roboticstoolbox.models.DH import Puma560

r = Puma560()  # DHRobot

# 目标位姿：0.5m, 0.1m, 0.2m，姿态用RPY(度)
T = sm.SE3(0.5, 0.1, 0.2) * sm.SE3.RPY([0, 90, 0], unit='deg')

# 初始猜测（给个合理的初值更容易收敛）
q0 = r.qz  # 零姿；也可以用当前关节角

# 用 ikine_LM（1.0.3 版本正确方法名）
sol = r.ikine_LM(T, q0=q0)  # 可加 ilimit=100, slimit=100 调参

if sol.success:
    print("IK 成功！关节角(度)：", np.rad2deg(sol.q))
else:
    print("IK 失败：", sol.reason)
