#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
pypinyin库安装脚本
pypinyin是一个优秀的汉字转拼音Python库，支持几乎所有汉字
"""

import subprocess
import sys

def install_pypinyin():
    """安装pypinyin库"""
    try:
        print("正在安装pypinyin库...")
        result = subprocess.run([sys.executable, '-m', 'pip', 'install', 'pypinyin'], 
                              capture_output=True, text=True)
        
        if result.returncode == 0:
            print("✓ pypinyin库安装成功！")
            print("现在可以使用label_to_pinyin.py脚本处理所有汉字了")
            
            # 测试安装
            try:
                from pypinyin import pinyin, Style
                test_result = pinyin('测试', style=Style.TONE3)
                print(f"测试结果: '测试' -> {test_result}")
                return True
            except ImportError:
                print("❌ 安装后仍无法导入pypinyin")
                return False
        else:
            print("❌ 安装失败:")
            print(result.stderr)
            return False
            
    except Exception as e:
        print(f"❌ 安装过程中出错: {e}")
        return False

def check_pypinyin():
    """检查pypinyin是否已安装"""
    try:
        import pypinyin
        print("✓ pypinyin库已安装")
        print(f"版本: {pypinyin.__version__ if hasattr(pypinyin, '__version__') else '未知'}")
        return True
    except ImportError:
        print("❌ pypinyin库未安装")
        return False

if __name__ == "__main__":
    print("pypinyin库安装和检查工具")
    print("=" * 40)
    
    if not check_pypinyin():
        user_input = input("是否要安装pypinyin库？(y/n): ")
        if user_input.lower() in ['y', 'yes', '是']:
            install_pypinyin()
        else:
            print("跳过安装，将使用内置拼音字典")
    else:
        print("pypinyin库已可用，无需安装")
