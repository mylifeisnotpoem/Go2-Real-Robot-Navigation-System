#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import os
import re
import argparse
from typing import List, Dict

class TopologyNode:
    """拓扑节点类"""
    def __init__(self, node_id: int, x: float, y: float, z: float, label: str):
        self.id = node_id
        self.x = x
        self.y = y
        self.z = z
        self.label = label
    
    def __str__(self):
        return f"Node {self.id}: ({self.x:.6f}, {self.y:.6f}, {self.z:.6f}) - {self.label}"

class KeyNodeExtractor:
    """关键节点提取器"""
    
    def __init__(self):
        self.all_nodes: Dict[int, TopologyNode] = {}
        self.key_nodes: List[TopologyNode] = []
    
    def has_chinese_characters(self, text: str) -> bool:
        """检查文本是否包含汉字"""
        chinese_pattern = re.compile(r'[\u4e00-\u9fff]+')
        return bool(chinese_pattern.search(text))
    
    def read_topology_map(self, file_path: str) -> bool:
        """读取拓扑地图文件"""
        try:
            with open(file_path, 'r', encoding='utf-8') as f:
                lines = f.readlines()
            
            self.all_nodes.clear()
            current_section = "header"
            
            for line_num, line in enumerate(lines):
                stripped_line = line.strip()
                
                # 检查当前处理的部分
                if "# Topology Nodes" in line:
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
                            
                            node = TopologyNode(node_id, x, y, z, label)
                            self.all_nodes[node_id] = node
                            
                        except ValueError as e:
                            print(f"警告: 解析节点第{line_num+1}行时出错: {stripped_line} - {e}")
                            continue
            
            print(f"成功读取 {len(self.all_nodes)} 个节点")
            return True
            
        except FileNotFoundError:
            print(f"错误: 找不到文件 {file_path}")
            return False
        except Exception as e:
            print(f"错误: 读取文件时出错 - {e}")
            return False
    
    def extract_key_nodes(self):
        """提取带有汉字标签的关键节点"""
        self.key_nodes.clear()
        
        for node in self.all_nodes.values():
            if self.has_chinese_characters(node.label):
                self.key_nodes.append(node)
        
        # 按原始ID排序
        self.key_nodes.sort(key=lambda x: x.id)
        
        print(f"发现 {len(self.key_nodes)} 个带有汉字标签的关键节点")
        
        # 显示找到的关键节点
        if self.key_nodes:
            print("关键节点列表:")
            for i, node in enumerate(self.key_nodes):
                print(f"  {i}: 原ID={node.id}, 标签='{node.label}', 坐标=({node.x:.3f}, {node.y:.3f}, {node.z:.3f})")
    
    def write_key_nodes_map(self, output_path: str) -> bool:
        """写入关键节点地图文件"""
        try:
            with open(output_path, 'w', encoding='utf-8') as f:
                # 写入文件头
                f.write("# 关键导航节点地图文件 (UTF-8编码)\n")
                f.write("# 格式: 节点ID X Y Z 标签\n")
                f.write(f"# 提取自原始拓扑地图的 {len(self.key_nodes)} 个汉字标签节点\n")
                f.write("# Key Navigation Nodes\n")
                
                # 重新分配节点ID并写入
                for new_id, node in enumerate(self.key_nodes):
                    f.write(f"{new_id} {node.x:.6f} {node.y:.6f} {node.z:.6f} {node.label}\n")
                
                f.write("\n# 原始节点ID映射关系\n")
                for new_id, node in enumerate(self.key_nodes):
                    f.write(f"# 新ID {new_id} <- 原ID {node.id} ({node.label})\n")
            
            print(f"成功写入关键节点地图到: {output_path}")
            return True
            
        except Exception as e:
            print(f"错误: 写入文件时出错 - {e}")
            return False
    
    def print_statistics(self):
        """打印统计信息"""
        print("\n=== 统计信息 ===")
        print(f"原始节点总数: {len(self.all_nodes)}")
        print(f"关键节点数量: {len(self.key_nodes)}")
        print(f"提取比例: {len(self.key_nodes)/len(self.all_nodes)*100:.1f}%")
        
        if self.key_nodes:
            print("\n=== 关键节点详情 ===")
            for new_id, node in enumerate(self.key_nodes):
                print(f"节点 {new_id}: {node.label} (原ID: {node.id})")
                print(f"  坐标: ({node.x:.6f}, {node.y:.6f}, {node.z:.6f})")
    
    def save_mapping_file(self, mapping_path: str) -> bool:
        """保存ID映射文件"""
        try:
            with open(mapping_path, 'w', encoding='utf-8') as f:
                f.write("# 节点ID映射文件\n")
                f.write("# 格式: 新ID 原ID 标签 X Y Z\n")
                
                for new_id, node in enumerate(self.key_nodes):
                    f.write(f"{new_id} {node.id} {node.label} {node.x:.6f} {node.y:.6f} {node.z:.6f}\n")
            
            print(f"成功保存ID映射文件到: {mapping_path}")
            return True
            
        except Exception as e:
            print(f"错误: 保存映射文件时出错 - {e}")
            return False

def main():
    """主函数"""
    parser = argparse.ArgumentParser(description='关键导航节点提取工具')
    parser.add_argument('--input', '-i', 
                       default='../map/connected_topology_map.txt',
                       help='输入拓扑地图文件路径 (默认: ../map/connected_topology_map.txt)')
    parser.add_argument('--output', '-o', 
                       default='../map/key_nodes_map.txt',
                       help='输出关键节点地图文件路径 (默认: ../map/key_nodes_map.txt)')
    parser.add_argument('--mapping', '-m',
                       default='../map/node_id_mapping.txt',
                       help='输出ID映射文件路径 (默认: ../map/node_id_mapping.txt)')
    parser.add_argument('--stats', '-s', 
                       action='store_true',
                       help='显示详细统计信息')
    parser.add_argument('--save-mapping', 
                       action='store_true',
                       help='保存ID映射文件')
    
    args = parser.parse_args()
    
    # 获取脚本所在目录
    script_dir = os.path.dirname(os.path.abspath(__file__))
    
    # 处理相对路径
    if not os.path.isabs(args.input):
        input_path = os.path.join(script_dir, args.input)
    else:
        input_path = args.input
        
    if not os.path.isabs(args.output):
        output_path = os.path.join(script_dir, args.output)
    else:
        output_path = args.output
    
    if not os.path.isabs(args.mapping):
        mapping_path = os.path.join(script_dir, args.mapping)
    else:
        mapping_path = args.mapping
    
    print(f"输入文件: {input_path}")
    print(f"输出文件: {output_path}")
    
    # 创建提取器
    extractor = KeyNodeExtractor()
    
    # 读取拓扑地图
    if not extractor.read_topology_map(input_path):
        return 1
    
    # 提取关键节点
    extractor.extract_key_nodes()
    
    # 检查是否找到关键节点
    if not extractor.key_nodes:
        print("警告: 没有找到带有汉字标签的节点")
        return 1
    
    # 显示统计信息
    if args.stats:
        extractor.print_statistics()
    
    # 写入关键节点地图
    if not extractor.write_key_nodes_map(output_path):
        return 1
    
    # 保存ID映射文件
    if args.save_mapping:
        if not extractor.save_mapping_file(mapping_path):
            return 1
    
    print("提取完成！")
    return 0

if __name__ == "__main__":
    exit(main())