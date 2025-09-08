#!/bin/bash
# PAROL6 启动脚本

echo "=================================================="
echo "🤖 PAROL6 机械臂控制系统启动脚本"
echo "=================================================="

# 激活conda环境
echo "🔧 激活Python环境..."
source ~/anaconda3/bin/activate
conda activate parol

# 切换到工作目录
cd ~/PAROL-commander-software/PAROL6-python-API

# 检查依赖
echo "📦 检查依赖..."
python -c "import serial; import roboticstoolbox; import numpy" 2>/dev/null
if [ $? -ne 0 ]; then
    echo "❌ 缺少依赖，正在安装..."
    pip install pyserial roboticstoolbox-python numpy
fi

# 测试串口连接
echo "🔍 测试串口连接..."
python test_serial.py

if [ $? -eq 0 ]; then
    echo ""
    echo "✅ 串口测试通过"
    echo ""
    echo "🚀 启动PAROL6控制程序..."
    echo "=================================================="
    python run_parol6_fixed.py
else
    echo ""
    echo "❌ 串口连接失败"
    echo ""
    echo "故障排除："
    echo "1. 检查机械臂是否连接并上电"
    echo "2. 检查USB线缆"
    echo "3. 运行: sudo usermod -a -G dialout \$USER"
    echo "4. 或使用: sudo bash start_parol6.sh"
    exit 1
fi
