#!/usr/bin/env python3
"""
PAROL6 机械臂控制器 - 完整版本
包含串口通信修复和ROS2接口准备
"""
import os
import sys
import platform
import serial
import serial.tools.list_ports
import time
import threading
import numpy as np
from typing import Optional, List, Tuple
import logging

# 设置日志
logging.basicConfig(level=logging.INFO, 
                   format='%(asctime)s - %(levelname)s - %(message)s')
logger = logging.getLogger(__name__)

class PAROL6Controller:
    """PAROL6机械臂控制器类"""
    
    def __init__(self, port: Optional[str] = None, baudrate: int = 3000000):
        """
        初始化PAROL6控制器
        
        Args:
            port: 串口端口号，如果为None则自动检测
            baudrate: 波特率，默认3000000
        """
        self.ser = None
        self.com_port_str = port
        self.baudrate = baudrate
        self.is_connected = False
        self.lock = threading.Lock()
        
        # 机械臂参数
        self.num_joints = 6
        self.joint_limits = [
            (-170, 170),  # Joint 1
            (-120, 120),  # Joint 2
            (-145, 145),  # Joint 3
            (-180, 180),  # Joint 4
            (-120, 120),  # Joint 5
            (-360, 360),  # Joint 6
        ]
        
        # 初始化连接
        self.connect()

    
    def connect(self) -> bool:
        """连接到PAROL6机械臂"""
        logger.info("🔧 正在连接PAROL6机械臂...")
        
        # 如果已指定端口，直接尝试连接
        if self.com_port_str:
            if self._try_connect(self.com_port_str):
                return True
        
        # 尝试从配置文件读取
        try:
            with open("com_port.txt", "r") as f:
                saved_port = f.read().strip()
                if self._try_connect(saved_port):
                    self.com_port_str = saved_port
                    return True
        except FileNotFoundError:
            pass
        
        # 自动检测端口
        return self._auto_detect_port()
    
    def _try_connect(self, port: str) -> bool:
        """尝试连接指定端口"""
        if not os.path.exists(port):
            logger.warning(f"端口 {port} 不存在")
            return False
            
        try:
            self.ser = serial.Serial(
                port=port,
                baudrate=self.baudrate,
                timeout=0.1,
                write_timeout=0.1
            )
            self.is_connected = True
            logger.info(f"✅ 成功连接到 {port}")
            
            # 保存成功的端口
            with open("com_port.txt", "w") as f:
                f.write(port)
            
            # 发送初始化命令测试连接
            time.sleep(2)  # 等待设备准备
            if self._test_connection():
                return True
            else:
                logger.warning(f"设备在 {port} 无响应")
                self.disconnect()
                return False
                
        except serial.SerialException as e:
            logger.error(f"连接 {port} 失败: {e}")
            return False

    
    def _auto_detect_port(self) -> bool:
        """自动检测串口"""
        logger.info("🔍 自动检测串口...")
        
        # 默认端口列表
        default_ports = ["/dev/ttyACM0", "/dev/ttyUSB0", "/dev/ttyACM1", "/dev/ttyUSB1"]
        
        # 尝试默认端口
        for port in default_ports:
            if self._try_connect(port):
                self.com_port_str = port
                return True
        
        # 列出所有可用串口
        ports = serial.tools.list_ports.comports()
        for port in ports:
            logger.info(f"📡 发现端口: {port.device} - {port.description}")
            if "ACM" in port.device or "USB" in port.device:
                if self._try_connect(port.device):
                    self.com_port_str = port.device
                    return True
        
        logger.error("❌ 未找到可用的串口")
        return False
    
    def _test_connection(self) -> bool:
        """测试与机械臂的连接"""
        try:
            # 发送测试命令（根据PAROL6协议调整）
            self.ser.write(b"READY\n")
            time.sleep(0.1)
            if self.ser.in_waiting > 0:
                response = self.ser.read(self.ser.in_waiting)
                logger.debug(f"收到响应: {response}")
                return True
            return False
        except Exception as e:
            logger.error(f"测试连接失败: {e}")
            return False
    
    def disconnect(self):
        """断开与机械臂的连接"""
        if self.ser and self.ser.is_open:
            self.ser.close()
            self.is_connected = False
            logger.info("🔌 已断开连接")

