# PAROL6 Python API - Linux修复版

## 📋 项目说明
这是PAROL6机械臂的Python API，已修复Linux环境下的串口连接问题。

## ✅ 已修复的问题

### 1. **串口连接问题**
- 修复了Linux环境下串口初始化失败的问题
- 添加了自动串口检测功能
- 实现了串口权限检查

### 2. **模块导入问题**
- 修复了GUI模块导入路径问题
- 正确设置了PYTHONPATH

### 3. **依赖管理**
- 使用conda虚拟环境管理依赖
- 确保所有必要的包都已安装

## 🚀 快速开始

### 1. 激活环境
```bash
source ~/anaconda3/bin/activate
conda activate parol
```

### 2. 测试串口连接
```bash
cd ~/PAROL-commander-software/PAROL6-python-API
python test_serial.py
```

### 3. 运行主程序
```bash
python run_parol6_fixed.py
```

## 📁 文件说明

- `run_parol6_fixed.py` - 主启动脚本（修复版）
- `test_serial.py` - 串口连接测试工具
- `parol6_controller.py` - PAROL6控制器类（准备ROS2集成）
- `headless_commander.py` - 原始控制程序
- `robot_api.py` - 机器人API
- `PAROL6_ROBOT.py` - 机器人模型定义
- `smooth_motion.py` - 平滑运动控制

## 🔧 故障排除

### 串口权限问题
如果遇到权限错误，运行：
```bash
sudo usermod -a -G dialout $USER
# 然后重新登录
```

### 临时解决方案
```bash
sudo -E python run_parol6_fixed.py
```

### 端口冲突
如果遇到 "Address already in use" 错误：
```bash
# 查找占用端口的进程
lsof -i :5001
# 结束进程
kill -9 <PID>
```

## 📡 串口配置

- 默认波特率: 3000000
- 默认端口: /dev/ttyACM0 或 /dev/ttyUSB0
- 端口配置文件: com_port.txt

## 🤖 ROS2集成（下一步）

准备工作已完成，下一步将集成到ROS2和MoveIt：
1. 创建ROS2包
2. 实现MoveIt控制器接口
3. 配置URDF模型
4. 设置MoveIt配置

## 📝 更新日志

### 2024-09-01
- 修复Linux串口连接问题
- 添加串口自动检测
- 创建测试工具
- 准备ROS2集成接口

## 👥 维护者
- wzy

## 📄 许可证
MIT License
