#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import os
import re
import argparse
from typing import Dict, List, Tuple

try:
    from pypinyin import pinyin, Style
    HAS_PYPINYIN = True
    print("✓ 使用pypinyin库进行拼音转换，支持所有汉字")
except ImportError:
    HAS_PYPINYIN = False
    print("⚠️ 未安装pypinyin库，使用内置拼音字典")
    print("安装命令: pip install pypinyin")

class ChineseToPinyinConverter:
    """汉字转拼音转换器"""
    
    def __init__(self):
        # 从keywords.txt中读取已有的拼音映射
        self.existing_keywords = {}
    
    def load_keywords_file(self, keywords_path: str) -> bool:
        """读取keywords.txt文件，获取已有的拼音映射"""
        try:
            with open(keywords_path, 'r', encoding='utf-8') as f:
                lines = f.readlines()
            
            for line in lines:
                line = line.strip()
                if '@' in line:
                    parts = line.split('@')
                    if len(parts) == 2:
                        pinyin = parts[0].strip()
                        chinese = parts[1].strip()
                        self.existing_keywords[chinese] = pinyin
            
            print(f"从keywords.txt读取了 {len(self.existing_keywords)} 个关键词映射")
            return True
            
        except FileNotFoundError:
            print(f"警告: 找不到keywords.txt文件: {keywords_path}")
            return False
        except Exception as e:
            print(f"读取keywords.txt文件时出错: {e}")
            return False
    
    def chinese_to_pinyin(self, chinese_text: str) -> str:
        """将汉字转换为拼音格式"""
        # 首先检查是否在已有关键词中
        if chinese_text in self.existing_keywords:
            return self.existing_keywords[chinese_text]
        
        # 如果安装了pypinyin库，优先使用
        if HAS_PYPINYIN:
            try:
                # 使用pypinyin进行转换，获取带声调的拼音
                result = pinyin(chinese_text, style=Style.TONE3, errors='ignore')
                pinyin_parts = []
                
                for char_pinyin in result:
                    if char_pinyin:
                        # 将数字声调转换为符号声调格式
                        py = char_pinyin[0]
                        py_formatted = self.format_pinyin_tone(py)
                        # 分离声母和韵母
                        py_separated = self.separate_pinyin(py_formatted)
                        pinyin_parts.append(py_separated)
                
                if pinyin_parts:
                    return ' '.join(pinyin_parts)
            except Exception as e:
                print(f"pypinyin转换出错: {e}，回退到内置字典")
        
        # 回退到pypinyin库的默认转换
        if HAS_PYPINYIN:
            try:
                # 使用pypinyin进行转换，获取带声调的拼音
                result = pinyin(chinese_text, style=Style.TONE3, errors='ignore')
                pinyin_parts = []
                
                for char_pinyin in result:
                    if char_pinyin:
                        # 将数字声调转换为符号声调格式
                        py = char_pinyin[0]
                        py_formatted = self.format_pinyin_tone(py)
                        # 分离声母和韵母
                        py_separated = self.separate_pinyin(py_formatted)
                        pinyin_parts.append(py_separated)
                
                if pinyin_parts:
                    return ' '.join(pinyin_parts)
            except Exception as e:
                print(f"pypinyin转换出错: {e}")
        
        # 如果pypinyin不可用，返回错误标记
        print(f"警告: 无法转换汉字 '{chinese_text}'，请安装pypinyin库")
        return f"[{chinese_text}]"
    
    def format_pinyin_tone(self, pinyin_with_number: str) -> str:
        """将数字声调转换为符号声调"""
        # 声调映射表
        tone_marks = {
            'a': ['a', 'ā', 'á', 'ǎ', 'à'],
            'o': ['o', 'ō', 'ó', 'ǒ', 'ò'],
            'e': ['e', 'ē', 'é', 'ě', 'è'],
            'i': ['i', 'ī', 'í', 'ǐ', 'ì'],
            'u': ['u', 'ū', 'ú', 'ǔ', 'ù'],
            'v': ['ü', 'ǖ', 'ǘ', 'ǚ', 'ǜ'],
            'ü': ['ü', 'ǖ', 'ǘ', 'ǚ', 'ǜ']
        }
        
        # 查找数字声调
        import re
        tone_match = re.search(r'(\d)$', pinyin_with_number)
        if not tone_match:
            return pinyin_with_number
        
        tone = int(tone_match.group(1))
        if tone < 1 or tone > 4:
            return pinyin_with_number.rstrip('0123456789')
        
        # 移除数字
        base_pinyin = pinyin_with_number.rstrip('0123456789')
        
        # 声调标注规则：
        # 1. 有a、o、e时，声调标在a、o、e上
        # 2. 有iu时，声调标在u上
        # 3. 其他情况标在第一个元音上
        
        for vowel in ['a', 'o', 'e']:
            if vowel in base_pinyin:
                return base_pinyin.replace(vowel, tone_marks[vowel][tone])
        
        # 处理iu的情况
        if 'iu' in base_pinyin:
            return base_pinyin.replace('u', tone_marks['u'][tone])
        
        # 处理其他元音
        for vowel in ['i', 'u', 'v', 'ü']:
            if vowel in base_pinyin:
                return base_pinyin.replace(vowel, tone_marks[vowel][tone])
        
        return base_pinyin
    
    def separate_pinyin(self, pinyin_str: str) -> str:
        """将拼音分离为声母和韵母"""
        # 声母列表（按长度排序，先匹配长的声母）
        initials = ['zh', 'ch', 'sh', 'b', 'p', 'm', 'f', 'd', 't', 'n', 'l', 
                   'g', 'k', 'h', 'j', 'q', 'x', 'r', 'z', 'c', 's', 'y', 'w']
        
        for initial in initials:
            if pinyin_str.startswith(initial):
                final = pinyin_str[len(initial):]
                if final:  # 有韵母
                    return f"{initial} {final}"
                else:  # 只有声母（这种情况很少见）
                    return initial
        
        # 没有声母，直接返回（零声母：a, e, o, ai, ei, ao, ou, an, en, ang, eng, er等）
        return pinyin_str
    
    def has_chinese_characters(self, text: str) -> bool:
        """检查文本是否包含汉字"""
        chinese_pattern = re.compile(r'[\u4e00-\u9fff]+')
        return bool(chinese_pattern.search(text))
    
    def extract_chinese_text(self, text: str) -> str:
        """提取文本中的汉字部分"""
        chinese_pattern = re.compile(r'[\u4e00-\u9fff]+')
        matches = chinese_pattern.findall(text)
        return ''.join(matches)

class TopologyLabelConverter:
    """拓扑地图标签转换器"""
    
    def __init__(self, keywords_path: str = None):
        self.converter = ChineseToPinyinConverter()
        self.nodes_with_chinese = []
        self.converted_labels = []
        
        if keywords_path:
            self.converter.load_keywords_file(keywords_path)
    
    def read_topology_map(self, file_path: str) -> bool:
        """读取拓扑地图文件"""
        try:
            with open(file_path, 'r', encoding='utf-8') as f:
                lines = f.readlines()
            
            self.nodes_with_chinese.clear()
            current_section = "header"
            
            for line_num, line in enumerate(lines):
                stripped_line = line.strip()
                
                # 检查当前处理的部分
                if "# Topology Nodes" in line or "# Key Navigation Nodes" in line:
                    current_section = "nodes"
                    continue
                elif "# Topology Edges" in line:
                    break  # 只读取节点部分
                
                # 跳过注释行和空行
                if stripped_line.startswith('#') or not stripped_line:
                    continue
                
                if current_section == "nodes":
                    parts = stripped_line.split()
                    if len(parts) >= 5:
                        try:
                            node_id = int(parts[0])
                            x = float(parts[1])
                            y = float(parts[2])
                            z = float(parts[3])
                            label = parts[4]
                            
                            # 检查标签是否包含汉字
                            if self.converter.has_chinese_characters(label):
                                chinese_text = self.converter.extract_chinese_text(label)
                                pinyin = self.converter.chinese_to_pinyin(chinese_text)
                                
                                self.nodes_with_chinese.append({
                                    'id': node_id,
                                    'x': x, 'y': y, 'z': z,
                                    'original_label': label,
                                    'chinese_text': chinese_text,
                                    'pinyin': pinyin
                                })
                            
                        except ValueError as e:
                            print(f"警告: 解析节点第{line_num+1}行时出错: {stripped_line} - {e}")
                            continue
            
            print(f"找到 {len(self.nodes_with_chinese)} 个包含汉字的节点")
            return True
            
        except FileNotFoundError:
            print(f"错误: 找不到文件 {file_path}")
            return False
        except Exception as e:
            print(f"错误: 读取文件时出错 - {e}")
            return False
    
    def generate_keywords_format(self, output_path: str) -> bool:
        """生成keywords.txt格式的文件"""
        try:
            with open(output_path, 'w', encoding='utf-8') as f:
                # 首先写入固定的语音命令
                fixed_commands = [
                    "n ǐ h ǎo x iǎo y ì @你好小益",
                    "d ǎo h áng k āi sh ǐ @导航开始",
                    "d ǎo h áng zh ōng zh ǐ @导航终止",
                    "t íng x ià @停下",
                    "q ián j ìn @前进",
                    "h òu t uì @后退",
                    "z uǒ zh uǎn @左转",
                    "y òu zh uǎn @右转",
                    "h uí j iā @回家",
                    "d ūn x ià @蹲下",
                    "q ǐ l ì @起立",
                    "j ì x ù @继续"
                ]
                
                for command in fixed_commands:
                    f.write(f"{command}\n")
                
                # 然后写入从拓扑地图提取的标签
                for node in self.nodes_with_chinese:
                    f.write(f"{node['pinyin']} @{node['chinese_text']}\n")
            
            print(f"成功生成keywords格式文件到: {output_path}")
            print(f"包含 {len(fixed_commands)} 个固定命令和 {len(self.nodes_with_chinese)} 个节点标签")
            return True
            
        except Exception as e:
            print(f"错误: 写入文件时出错 - {e}")
            return False
    
    def print_statistics(self):
        """打印统计信息"""
        print("\n=== 统计信息 ===")
        print(f"包含汉字的节点数量: {len(self.nodes_with_chinese)}")
        
        if self.nodes_with_chinese:
            print("\n=== 转换结果 ===")
            for node in self.nodes_with_chinese:
                print(f"节点 {node['id']}: {node['chinese_text']} -> {node['pinyin']}")
                if '[' in node['pinyin']:
                    print(f"  ⚠️  包含未知汉字，需要手动添加拼音")

def main():
    """主函数"""
    parser = argparse.ArgumentParser(description='拓扑地图汉字标签转拼音工具')
    parser.add_argument('--input', '-i', 
                       default='../map/key_nodes_test.txt',
                       help='输入拓扑地图文件路径 (默认: ../map/key_nodes_test.txt)')
    parser.add_argument('--keywords', '-k',
                       default='../../go2_drive/sherpa_onnx_ros/models/sherpa-onnx-kws-zipformer-wenetspeech-3.3M-2024-01-01/keywords.txt',
                       help='keywords.txt文件路径')
    parser.add_argument('--output', '-o', 
                       default='../map/topology_keywords.txt',
                       help='输出keywords格式文件路径 (默认: ../map/topology_keywords.txt)')
    parser.add_argument('--stats', '-s', 
                       action='store_true',
                       help='显示详细统计信息')
    
    args = parser.parse_args()
    
    # 获取脚本所在目录
    script_dir = os.path.dirname(os.path.abspath(__file__))
    
    # 处理相对路径
    for path_attr in ['input', 'keywords', 'output']:
        attr_name = path_attr.replace('_', '')
        path_value = getattr(args, attr_name, None)
        if path_value and not os.path.isabs(path_value):
            setattr(args, attr_name, os.path.join(script_dir, path_value))
    
    print(f"输入文件: {args.input}")
    print(f"Keywords文件: {args.keywords}")
    print(f"输出文件: {args.output}")
    
    # 创建转换器
    converter = TopologyLabelConverter(args.keywords)
    
    # 读取拓扑地图
    if not converter.read_topology_map(args.input):
        return 1
    
    # 检查是否找到包含汉字的节点
    if not converter.nodes_with_chinese:
        print("没有找到包含汉字的节点标签")
        return 1
    
    # 显示统计信息
    if args.stats:
        converter.print_statistics()
    
    # 生成keywords格式文件
    if not converter.generate_keywords_format(args.output):
        return 1
    
    print("转换完成！")
    return 0

if __name__ == "__main__":
    exit(main())
