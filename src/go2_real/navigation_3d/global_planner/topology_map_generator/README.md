# Topology Map Generator

这个功能包用于从ROS路径消息自动生成拓扑地图。

## 功能特点

- 订阅 `nav_msgs/Path` 类型的路径消息
- 每隔可配置的路径点数量生成一个拓扑节点
- 自动创建节点间的顺序连接
- 支持C++和Python两种实现
- 程序关闭时自动保存为 `map.txt` 格式

## 使用方法

### 基本启动
```bash
roslaunch topology_map_generator topology_map_generator.launch
```

### 自定义参数启动
```bash
roslaunch topology_map_generator topology_map_generator.launch \
    node_interval:=30 \
    output_file:="/path/to/your/map.txt" \
    path_topic:="/your_path_topic"
```

### 使用C++版本
```bash
roslaunch topology_map_generator topology_map_generator.launch use_cpp:=true
```

## 参数说明

| 参数名 | 类型 | 默认值 | 说明 |
|--------|------|--------|------|
| `node_interval` | int | 50 | 每隔多少个路径点生成一个拓扑节点 |
| `output_file` | string | `generated_map.txt` | 输出地图文件路径 |
| `path_topic` | string | `/path` | 输入路径话题名称 |
| `edge_type` | string | `auto` | 边的类型标签 |
| `auto_bidirectional` | bool | true | 是否自动设置边为双向 |
| `use_cpp` | bool | false | 是否使用C++版本 |

## 输出格式

生成的 `map.txt` 文件格式与您提供的格式完全兼容：

```
# 拓扑地图文件 (UTF-8编码)
# 格式: 节点ID X Y Z 标签
# Topology Nodes
0 -4.447878 0.011323 -0.076628 node_
1 8.463875 0.029127 -0.077397 node_
...
# 格式: 边ID 起始节点ID 结束节点ID 权重 类型 双向
# Topology Edges
0 0 1 12.911765 auto 1
1 1 2 23.415349 auto 1
...
```

## 工作原理

1. **节点生成**: 每当接收到的路径点数量达到 `node_interval` 的倍数时，从最新的路径点位置生成一个拓扑节点
2. **边连接**: 新生成的节点会自动与前一个节点创建连接边
3. **权重计算**: 边的权重为两个节点间的欧几里得距离
4. **自动保存**: 程序关闭时（Ctrl+C）会自动添加最终节点并保存地图文件

## 示例使用场景

### 场景1: 机器人巡航建图
```bash
# 启动建图
roslaunch your_mapping_package mapping.launch

# 启动拓扑地图生成器（每20个点生成一个节点）
roslaunch topology_map_generator topology_map_generator.launch \
    node_interval:=20 \
    path_topic:="/optimized_path" \
    output_file:="/home/user/robot_topology_map.txt"
    
# 机器人巡航完成后按 Ctrl+C 保存地图
```

### 场景2: 从录制的路径生成拓扑图
```bash
# 播放录制的bag文件
rosbag play your_recorded_path.bag

# 生成拓扑地图
roslaunch topology_map_generator topology_map_generator.launch \
    node_interval:=50 \
    path_topic:="/recorded_path"
```

## 编译

在您的工作空间中编译：
```bash
cd /home/server/WS_ROS1/ws_go2_slam_3d
catkin_make
source devel/setup.bash
```

## 注意事项

- 确保路径话题正在发布
- 程序运行期间会实时显示生成的节点和边信息
- 按 Ctrl+C 优雅关闭程序以确保地图文件正确保存
- 生成的节点ID从0开始递增
- 边ID也从0开始递增
