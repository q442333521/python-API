#!/usr/bin/env python3
# PAROL6 ikine_LMS 替换脚本

import os
import re

def replace_in_file(filename, old_pattern, new_pattern):
    """在文件中替换内容"""
    try:
        with open(filename, 'r', encoding='utf-8') as f:
            content = f.read()
        
        # 备份原文件
        backup_filename = filename + '.backup'
        with open(backup_filename, 'w', encoding='utf-8') as f:
            f.write(content)
        print(f"✅ 已备份: {backup_filename}")
        
        # 执行替换
        new_content = re.sub(old_pattern, new_pattern, content)
        changes = content.count(old_pattern.replace(r'\b', ''))
        
        if changes > 0:
            with open(filename, 'w', encoding='utf-8') as f:
                f.write(new_content)
            print(f"✅ {filename}: 替换了 {changes} 处")
        else:
            print(f"📄 {filename}: 无需替换")
            
    except Exception as e:
        print(f"❌ {filename}: 替换失败 - {e}")

# 要修改的文件列表
files_to_modify = [
    "headless_commander.py",
    "PAROL6_ROBOT.py", 
    "smooth_motion.py"
]

print("🔧 开始替换 ikine_LMS -> ikine_LM")
for filename in files_to_modify:
    if os.path.exists(filename):
        replace_in_file(filename, r'\bikine_LMS\b', 'ikine_LM')
    else:
        print(f"⚠️ 文件不存在: {filename}")

print("\n🎉 替换完成！")
print("💡 如果 ikine_LM 不工作，请尝试替换为 ik_LM")
