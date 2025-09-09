
#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
快速修复 ikine_LM wN 参数问题
"""

import os
import re
import shutil
import time

def quick_fix_wN_parameter():
    """快速修复 wN 参数问题"""
    print("🚀 快速修复 ikine_LM wN 参数问题")
    print("="*40)
    
    files = ["headless_commander.py", "PAROL6_ROBOT.py", "smooth_motion.py"]
    
    for filename in files:
        if not os.path.exists(filename):
            print(f"⚠️ {filename}: 文件不存在")
            continue
        
        try:
            # 读取文件
            with open(filename, 'r', encoding='utf-8') as f:
                content = f.read()
            
            original_content = content
            
            # 检查是否有问题
            if 'wN' not in content:
                print(f"✅ {filename}: 无需修复")
                continue
            
            # 备份
            backup = f"{filename}.backup_wN_{int(time.time())}"
            shutil.copy2(filename, backup)
            
            # 修复所有 wN 参数相关问题
            fixes = [
                # 删除 wN=xxx 参数
                (r',\s*wN\s*=[^,)]*', ''),
                (r'wN\s*=[^,)]*,\s*', ''),
                # 删除单独的 wN 参数
                (r',\s*wN\s*,', ','),
                (r'\(\s*wN\s*,', '('),
                (r',\s*wN\s*\)', ')'),
            ]
            
            fixes_applied = 0
            for pattern, replacement in fixes:
                if re.search(pattern, content):
                    content = re.sub(pattern, replacement, content)
                    fixes_applied += 1
            
            # 清理多余的逗号
            content = re.sub(r',\s*,', ',', content)
            content = re.sub(r'\(\s*,', '(', content)
            content = re.sub(r',\s*\)', ')', content)
            
            # 写入修复后的内容
            if content != original_content:
                with open(filename, 'w', encoding='utf-8') as f:
                    f.write(content)
                
                print(f"✅ {filename}: 修复完成，备份到 {backup}")
                
                # 显示修复的内容
                lines = content.split('\n')
                for i, line in enumerate(lines, 1):
                    if 'ikine_LM' in line:
                        print(f"   行 {i}: {line.strip()}")
            else:
                print(f"📄 {filename}: 未发现具体问题")
                os.remove(backup)  # 删除不必要的备份
                
        except Exception as e:
            print(f"❌ {filename}: 修复失败 - {e}")
    
    print("\n🎉 快速修复完成！")
    print("📋 下一步:")
    print("1. 重启 headless_commander.py")  
    print("2. 运行 API 测试")

if __name__ == "__main__":
    if input("确认执行 wN 参数快速修复? (y/N): ").lower() == 'y':
        quick_fix_wN_parameter()
    else:
        print("❌ 取消修复")