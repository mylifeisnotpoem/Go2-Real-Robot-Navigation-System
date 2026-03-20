#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import os
import math
import argparse
from typing import List, Tuple, Dict, Set

class TopologyNode:
    """拓扑节点类"""
    def __init__(self, node_id: int, x: float, y: float, z: float, label: str):
        self.id = node_id
        self.x = x
        self.y = y
        self.z = z
        self.label = label
    
    def distance_to(self, other_node):
        """计算到另一个节点的欧几里得距离"""
        return math.sqrt(
            (self.x - other_node.x) ** 2 + 
            (self.y - other_node.y) ** 2 + 
            (self.z - other_node.z) ** 2
        )
    
    def __str__(self):
        return f"Node {self.id}: ({self.x:.6f}, {self.y:.6f}, {self.z:.6f}) - {self.label}"

class TopologyEdge:
    """拓扑边类"""
    def __init__(self, edge_id: int, start_node: int, end_node: int, weight: float, edge_type: str, bidirectional: int):
        self.id = edge_id
        self.start_node = start_node
        self.end_node = end_node
        self.weight = weight
        self.type = edge_type
        self.bidirectional = bidirectional
    
    def get_node_pair(self):
        """获取节点对（较小ID在前）"""
        return tuple(sorted([self.start_node, self.end_node]))
    
    def __str__(self):
        return f"Edge {self.id}: {self.start_node} -> {self.end_node} ({self.weight:.6f}, {self.type})"

class TopologyMapProcessor:
    """拓扑地图处理器"""
    
    def __init__(self, connection_threshold: float = 3.0):
        self.nodes: Dict[int, TopologyNode] = {}
        self.edges: Dict[int, TopologyEdge] = {}
        self.existing_connections: Set[Tuple[int, int]] = set()  # 已存在的连接
        self.new_connections: List[Tuple[int, int, float]] = []  # 新发现的连接
        self.connection_threshold = connection_threshold
        self.node_lines: List[str] = []  # 存储节点行
        self.edge_lines: List[str] = []  # 存储边行
        self.header_lines: List[str] = []  # 存储文件头
    
    def read_map_file(self, file_path: str) -> bool:
        """读取拓扑地图文件"""
        try:
            with open(file_path, 'r', encoding='utf-8') as f:
                lines = f.readlines()
            
            self.nodes.clear()
            self.edges.clear()
            self.existing_connections.clear()
            self.node_lines.clear()
            self.edge_lines.clear()
            self.header_lines.clear()
            
            current_section = "header"
            
            for line_num, line in enumerate(lines):
                stripped_line = line.strip()
                
                # 检查当前处理的部分
                if "# Topology Nodes" in line:
                    current_section = "nodes"
                    self.header_lines.append(line)
                    continue
                elif "# Topology Edges" in line:
                    current_section = "edges"
                    continue
                
                # 跳过注释行和空行
                if stripped_line.startswith('#') or not stripped_line:
                    if current_section == "header":
                        self.header_lines.append(line)
                    continue
                
                parts = stripped_line.split()
                
                if current_section == "nodes" and len(parts) >= 5:
                    try:
                        node_id = int(parts[0])
                        x = float(parts[1])
                        y = float(parts[2])
                        z = float(parts[3])
                        label = parts[4]
                        
                        node = TopologyNode(node_id, x, y, z, label)
                        self.nodes[node_id] = node
                        self.node_lines.append(line)
                        
                    except ValueError as e:
                        print(f"警告: 解析节点第{line_num+1}行时出错: {stripped_line} - {e}")
                        continue
                
                elif current_section == "edges" and len(parts) >= 6:
                    try:
                        edge_id = int(parts[0])
                        start_node = int(parts[1])
                        end_node = int(parts[2])
                        weight = float(parts[3])
                        edge_type = parts[4]
                        bidirectional = int(parts[5])
                        
                        edge = TopologyEdge(edge_id, start_node, end_node, weight, edge_type, bidirectional)
                        self.edges[edge_id] = edge
                        self.edge_lines.append(line)
                        
                        # 记录现有连接
                        connection_pair = tuple(sorted([start_node, end_node]))
                        self.existing_connections.add(connection_pair)
                        
                    except ValueError as e:
                        print(f"警告: 解析边第{line_num+1}行时出错: {stripped_line} - {e}")
                        continue
            
            print(f"成功读取 {len(self.nodes)} 个节点和 {len(self.edges)} 条边")
            return True
            
        except FileNotFoundError:
            print(f"错误: 找不到文件 {file_path}")
            return False
        except Exception as e:
            print(f"错误: 读取文件时出错 - {e}")
            return False
    
    def find_new_connections(self):
        """找到距离小于阈值但尚未连接的节点对"""
        self.new_connections.clear()
        new_connection_count = 0
        
        node_list = list(self.nodes.values())
        
        print(f"正在检查 {len(node_list)} 个节点之间的距离...")
        print(f"现有连接数: {len(self.existing_connections)}")
        
        for i in range(len(node_list)):
            for j in range(i + 1, len(node_list)):
                node1 = node_list[i]
                node2 = node_list[j]
                
                # 检查这对节点是否已经连接
                connection_pair = tuple(sorted([node1.id, node2.id]))
                if connection_pair in self.existing_connections:
                    continue
                
                distance = node1.distance_to(node2)
                
                if distance <= self.connection_threshold:
                    self.new_connections.append((node1.id, node2.id, distance))
                    new_connection_count += 1
        
        print(f"根据距离阈值 {self.connection_threshold:.2f} 发现了 {new_connection_count} 个新连接")
        return new_connection_count
    
    def write_connected_map(self, output_path: str) -> bool:
        """写入带新连接的地图文件"""
        try:
            with open(output_path, 'w', encoding='utf-8') as f:
                # 写入文件头
                for line in self.header_lines:
                    if "连接阈值" not in line:  # 避免重复添加阈值信息
                        f.write(line)
                
                # 添加处理信息
                f.write(f"# 连接阈值: {self.connection_threshold:.2f}\n")
                f.write(f"# 新增连接数: {len(self.new_connections)}\n")
                f.write("# Topology Nodes\n")
                
                # 写入节点信息
                for line in self.node_lines:
                    f.write(line)
                
                # 写入边信息
                f.write("# 格式: 边ID 起始节点ID 结束节点ID 权重 类型 双向\n")
                f.write("# Topology Edges\n")
                
                # 写入现有边
                for line in self.edge_lines:
                    f.write(line)
                
                # 写入新发现的边
                if self.new_connections:
                    f.write("# 新增连接 (基于距离阈值自动生成)\n")
                    
                    # 获取下一个边ID
                    next_edge_id = max(self.edges.keys()) + 1 if self.edges else 0
                    
                    for node1_id, node2_id, distance in self.new_connections:
                        f.write(f"{next_edge_id} {node1_id} {node2_id} {distance:.6f} auto 1\n")
                        next_edge_id += 1
            
            print(f"成功写入连接地图到: {output_path}")
            return True
            
        except Exception as e:
            print(f"错误: 写入文件时出错 - {e}")
            return False
    
    def print_statistics(self):
        """打印统计信息"""
        if not self.nodes:
            print("没有节点数据")
            return
        
        # 统计每个节点的连接数（包括新连接）
        node_connections = {node_id: 0 for node_id in self.nodes.keys()}
        
        # 统计现有连接
        for connection in self.existing_connections:
            node1_id, node2_id = connection
            if node1_id in node_connections:
                node_connections[node1_id] += 1
            if node2_id in node_connections:
                node_connections[node2_id] += 1
        
        # 统计新连接
        for node1_id, node2_id, _ in self.new_connections:
            if node1_id in node_connections:
                node_connections[node1_id] += 1
            if node2_id in node_connections:
                node_connections[node2_id] += 1
        
        nodes_with_connections = sum(1 for count in node_connections.values() if count > 0)
        nodes_without_connections = len(self.nodes) - nodes_with_connections
        
        print("\n=== 统计信息 ===")
        print(f"总节点数: {len(self.nodes)}")
        print(f"原有连接数: {len(self.existing_connections)}")
        print(f"新增连接数: {len(self.new_connections)}")
        print(f"总连接数: {len(self.existing_connections) + len(self.new_connections)}")
        print(f"有连接的节点数: {nodes_with_connections}")
        print(f"孤立节点数: {nodes_without_connections}")
        
        if self.nodes:
            total_connections = len(self.existing_connections) + len(self.new_connections)
            avg_connections = (total_connections * 2) / len(self.nodes)
            print(f"平均每节点连接数: {avg_connections:.2f}")
            
            # 显示连接数分布
            if node_connections.values():
                max_connections = max(node_connections.values())
                min_connections = min(node_connections.values())
                print(f"最大连接数: {max_connections}")
                print(f"最小连接数: {min_connections}")
        
        # 显示新连接的详细信息
        if self.new_connections and len(self.new_connections) <= 20:
            print("\n=== 新增连接详情 ===")
            for i, (node1_id, node2_id, distance) in enumerate(self.new_connections):
                print(f"{i+1}. 节点 {node1_id} <-> 节点 {node2_id}, 距离: {distance:.3f}")

def main():
    """主函数"""
    parser = argparse.ArgumentParser(description='拓扑地图节点连接处理工具')
    parser.add_argument('--input', '-i', 
                       default='../map/generated_map.txt',
                       help='输入地图文件路径 (默认: ../map/generated_map.txt)')
    parser.add_argument('--output', '-o', 
                       default='../map/connected_topology_map.txt',
                       help='输出地图文件路径 (默认: ../map/connected_topology_map.txt)')
    parser.add_argument('--threshold', '-t', 
                       type=float, default=3.0,
                       help='连接距离阈值 (默认: 3.0)')
    parser.add_argument('--stats', '-s', 
                       action='store_true',
                       help='显示详细统计信息')
    
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
    
    print(f"输入文件: {input_path}")
    print(f"输出文件: {output_path}")
    print(f"连接阈值: {args.threshold}")
    
    # 创建处理器
    processor = TopologyMapProcessor(args.threshold)
    
    # 读取地图文件
    if not processor.read_map_file(input_path):
        return 1
    
    # 查找新的连接
    processor.find_new_connections()
    
    # 显示统计信息
    if args.stats:
        processor.print_statistics()
    
    # 写入连接后的地图
    if processor.write_connected_map(output_path):
        print("处理完成！")
        return 0
    else:
        return 1

if __name__ == "__main__":
    exit(main())
