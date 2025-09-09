#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
快速修复脚本：ikine_LMS -> ikine_LM
基于测试通过的方法
"""

import os
import shutil
import time

def quick_fix():
    """快速修复所有文件"""
    print("🚀 PAROL6 快速修复：ikine_LMS -> ikine_LM")
    print("="*50)
    
    files = ["headless_commander.py", "PAROL6_ROBOT.py", "smooth_motion.py"]
    
    total_fixes = 0
    
    for filename in files:
        if not os.path.exists(filename):
            print(f"⚠️ {filename}: 文件不存在")
            continue
            
        try:
            # 读取文件
            with open(filename, 'r', encoding='utf-8') as f:
                content = f.read()
            
            # 检查是否需要修复
            count = content.count('ikine_LMS')
            if count == 0:
                print(f"📄 {filename}: 无需修复")
                continue
            
            # 备份
            backup = f"{filename}.backup_{int(time.time())}"
            shutil.copy2(filename, backup)
            
            # 修复
            fixed_content = content.replace('ikine_LMS', 'ikine_LM')
            
            # 写入
            with open(filename, 'w', encoding='utf-8') as f:
                f.write(fixed_content)
            
            print(f"✅ {filename}: 修复 {count} 处，备份到 {backup}")
            total_fixes += count
            
        except Exception as e:
            print(f"❌ {filename}: 修复失败 - {e}")
    
    print("="*50)
    print(f"🎉 修复完成！总共修复 {total_fixes} 处")
    print("\n📋 下一步:")
    print("1. 重启 headless_commander.py")
    print("2. 运行API测试验证修复效果")

if __name__ == "__main__":
    if input("确认执行快速修复? (y/N): ").lower() == 'y':
        quick_fix()
    else:
        print("❌ 取消修复")