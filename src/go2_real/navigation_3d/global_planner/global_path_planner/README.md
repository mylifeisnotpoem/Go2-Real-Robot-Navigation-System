# Global Path Planner

基于拓扑地图的全局路径规划功能包，支持起点和终点不在拓扑图上的情况，具备智能可行性检查和多连接策略。

## 功能特性

- **拓扑地图读取**: 读取拓扑地图文件，解析节点和边信息
- **智能路径规划**: 使用Dijkstra算法在拓扑图中寻找最短路径
- **灵活起终点**: 支持起点和终点不在拓扑节点上，自动连接到最近的节点
- **可行性检查**: 智能评估连接的可行性，避免不可行的路径
- **多连接策略**: 为临时节点连接多个附近节点，避免折返路线
- **直接连接**: 当起点和终点距离很近时，优先使用直接连接
- **路径平滑**: 在节点间进行插值，生成平滑的路径
- **实时响应**: 接收里程计和目标点，实时规划全局路径

## 输入输出

### 输入
- **里程计话题** (`/odom`): `nav_msgs/Odometry` - 机器人当前位置
- **目标点话题** (`/clicked_point`): `geometry_msgs/PointStamped` - RViz点击的目标点

### 输出
- **全局路径** (`/global_path`): `nav_msgs/Path` - 规划的全局路径

## 参数配置

```yaml
# 基础参数
map_file_path: "/path/to/topology_map.txt"  # 拓扑地图文件路径
global_frame: "map"                         # 全局坐标系
robot_frame: "base_link"                    # 机器人坐标系
connection_distance_threshold: 5.0         # 连接阈值距离
interpolation_points: 20                   # 插值点数量

# 可行性检查参数
enable_feasibility_check: true             # 启用可行性检查
free_space_radius: 1.2                     # 自由空间半径
edge_proximity_threshold: 0.8              # 边接近阈值
line_of_sight_resolution: 0.3              # 视线检查分辨率

# 直接连接参数
enable_direct_connection: true             # 启用直接连接
direct_connection_threshold: 4.0           # 直接连接距离阈值
```

## 使用方法

### 1. 编译
```bash
cd /path/to/ws_pcl_map
catkin_make
source devel/setup.bash
```

### 2. 启动节点
```bash
# 基本启动
roslaunch global_path_planner global_path_planner.launch

# 带可行性检查的高级启动
roslaunch global_path_planner test_feasibility_check.launch

# 带路径平滑器的完整启动
roslaunch path_smoother test_with_smoother.launch
```

### 3. 使用RViz设置目标点
1. 在RViz中点击 "Publish Point" 工具
2. 在地图上点击设置目标点
3. 系统会自动规划从当前位置到目标点的路径

### 4. 功能验证
- **多连接测试**: 观察起点和终点是否连接到多个附近节点
- **可行性测试**: 设置不可行的目标点，验证是否正确拒绝
- **直接连接测试**: 设置距离较近的目标点，验证是否使用直接连接
- **折返避免测试**: 验证路径是否避免了不必要的折返

## 拓扑地图格式

节点格式：
```
节点ID X坐标 Y坐标 Z坐标 标签
```

边格式：
```
边ID 起始节点ID 结束节点ID 权重 类型 双向标志
```

示例：
```
# Topology Nodes
0 -45.502869 37.861420 -4.255585 node_
1 -58.564003 37.241459 -4.346906 node_

# Topology Edges  
0 0 1 13.076158 manual 1
```

## 算法原理

1. **起点处理**: 
   - 创建临时起点节点
   - 智能连接到多个附近的可行节点（2-5个）
   - 进行可行性检查，确保连接路径安全

2. **终点处理**: 
   - 创建临时终点节点
   - 同样进行多连接和可行性检查

3. **直接连接优化**:
   - 当起点和终点距离很近且可行时，直接连接
   - 避免不必要的绕行

4. **拓扑路径规划**: 使用Dijkstra算法在临时拓扑图中寻找最短路径

5. **完整路径构建**: 
   - 基于拓扑路径构建完整路径
   - 在各节点间进行插值
   - 生成平滑连续的路径

## 可行性检查机制

- **自由空间检查**: 验证点是否在已知的自由空间内
- **视线检查**: 沿连接线采样多个点，确保路径无障碍
- **距离限制**: 避免过长的不合理连接
- **代价计算**: 综合考虑距离和可行性因素

## 节点架构

```
global_path_planner_node
├── 订阅话题
│   ├── /odom (nav_msgs/Odometry)
│   └── /move_base_simple/goal (geometry_msgs/PoseStamped)
├── 发布话题
│   ├── /global_path_planner/path (nav_msgs/Path)
│   └── /global_path_planner/visualization (visualization_msgs/MarkerArray)
└── 参数服务器
    ├── map_file_path
    ├── global_frame
    ├── robot_frame
    ├── connection_distance_threshold
    └── interpolation_points
```

## 依赖关系

- ROS Noetic
- geometry_msgs
- nav_msgs
- tf
- visualization_msgs
- roscpp

## 注意事项

1. 确保拓扑地图文件格式正确
2. 确保坐标系配置正确
3. 建议在使用前通过RViz检查拓扑图是否正确加载
4. 路径规划依赖于TF变换，确保坐标变换链正常
