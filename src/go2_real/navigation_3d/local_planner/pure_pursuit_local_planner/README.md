# Pure Pursuit Local Planner

## 概述

Pure Pursuit Local Planner是一个基于Pure Pursuit算法的局部路径规划器，结合了障碍物避障功能。该规划器能够跟踪全局路径，同时根据局部传感器信息进行实时避障。

## 功能特性

- **Pure Pursuit算法**: 基于前瞻点的路径跟踪算法
- **动态避障**: 基于激光雷达和代价地图的实时障碍物检测与避障
- **速度控制**: 根据路径曲率和障碍物距离动态调整速度
- **可视化支持**: 完整的RViz可视化，包括路径、前瞻点、障碍物等
- **参数可配置**: 丰富的参数配置选项，适应不同应用场景

## 系统架构

```
pure_pursuit_local_planner/
├── include/pure_pursuit_local_planner/
│   ├── pure_pursuit_local_planner.h    # 主规划器类
│   ├── obstacle_avoidance.h            # 障碍物避障模块
│   └── path_tracker.h                  # 路径跟踪模块
├── src/
│   ├── pure_pursuit_node.cpp           # 主节点
│   ├── pure_pursuit_local_planner.cpp  # 主规划器实现
│   ├── obstacle_avoidance.cpp          # 避障模块实现
│   └── path_tracker.cpp                # 路径跟踪实现
├── config/
│   └── pure_pursuit_params.yaml        # 参数配置文件
├── launch/
│   ├── pure_pursuit_local_planner.launch   # 基本启动文件
│   └── test_with_costmap.launch             # 集成测试启动文件
└── rviz/
    └── pure_pursuit.rviz                # RViz配置文件
```

## 主要组件

### 1. PurePursuitLocalPlanner (主规划器)
- 负责整体的路径跟踪和控制指令生成
- 协调路径跟踪器和避障模块
- 处理TF变换和时间同步

### 2. PathTracker (路径跟踪器)
- 管理全局路径数据
- 计算前瞻点和路径曲率
- 提供路径相关的计算功能

### 3. ObstacleAvoidance (避障模块)
- 基于人工势场法的实时避障
- 直接处理代价地图数据
- 计算吸引力和排斥力生成避障指令
- 提供势场可视化功能

## 输入输出

### 输入话题
- `/global_path` (nav_msgs/Path): 全局路径
- `/local_costmap` (nav_msgs/OccupancyGrid): 局部代价地图（用于人工势场计算）

### 输出话题
- `/cmd_vel` (geometry_msgs/Twist): 控制指令
- `/pure_pursuit_local_planner/lookahead_point` (visualization_msgs/Marker): 前瞻点可视化
- `/pure_pursuit_local_planner/path_markers` (visualization_msgs/Marker): 路径可视化
- `/potential_field/force_markers` (visualization_msgs/MarkerArray): 势场力向量可视化
- `/potential_field/field_markers` (visualization_msgs/MarkerArray): 势场区域可视化

## 参数配置

### Pure Pursuit参数
- `lookahead_distance_min`: 最小前瞻距离 (默认: 1.0m)
- `lookahead_distance_max`: 最大前瞻距离 (默认: 3.0m)
- `lookahead_ratio`: 前瞻距离与速度的比例系数 (默认: 2.0s)
- `goal_tolerance`: 目标点容忍距离 (默认: 0.2m)

### 速度参数
- `max_linear_velocity`: 最大线速度 (默认: 1.0m/s)
- `max_angular_velocity`: 最大角速度 (默认: 1.0rad/s)
- `min_linear_velocity`: 最小线速度 (默认: 0.1m/s)
- `acceleration_limit`: 加速度限制 (默认: 0.5m/s²)
- `deceleration_limit`: 减速度限制 (默认: 0.8m/s²)

### 避障参数（人工势场法）
- `costmap_threshold`: 代价地图阈值 (默认: 50)
- `potential_field/attractive_gain`: 吸引力增益 (默认: 1.5)
- `potential_field/repulsive_gain`: 排斥力增益 (默认: 20.0)
- `potential_field/influence_radius`: 障碍物影响半径 (默认: 2.5m)
- `potential_field/safety_radius`: 安全半径 (默认: 0.8m)
- `potential_field/force_limit`: 最大力限制 (默认: 10.0)
- `potential_field/angular_gain`: 角速度调整增益 (默认: 2.0)
- `potential_field/speed_factor_gain`: 速度因子增益 (默认: 0.5)

### 路径跟踪权重
- `avoidance_weight`: 避障权重 (默认: 0.7)
- `path_following_weight`: 路径跟踪权重 (默认: 0.3)

## 使用方法

### 1. 编译
```bash
cd /home/server/WS_ROS1/ws_go2_real
catkin_make
source devel/setup.bash
```

### 2. 基本运行
```bash
# 启动Pure Pursuit规划器
roslaunch pure_pursuit_local_planner pure_pursuit_local_planner.launch

# 或者启动完整的导航测试
roslaunch pure_pursuit_local_planner test_with_costmap.launch
```

### 3. 参数调整
修改 `config/pure_pursuit_params.yaml` 文件中的参数，然后重新启动节点。

### 4. GO2机器人集成
```bash
# 启动完整的GO2导航系统（包括定位、全局规划、局部规划）
roslaunch pure_pursuit_local_planner go2_navigation_complete.launch

# 仅启动Pure Pursuit规划器（假设定位和全局规划已运行）
roslaunch pure_pursuit_local_planner go2_pure_pursuit.launch

# 启动带RViz可视化的完整系统
roslaunch pure_pursuit_local_planner go2_navigation_complete.launch launch_rviz:=true
```

### 5. 障碍物检测配置选项
支持多种障碍物检测配置，可根据实际需求选择：

```bash
# 使用激光雷达和代价地图（默认，最佳性能）
roslaunch pure_pursuit_local_planner pure_pursuit_configurations.launch config:=both

# 仅使用激光雷达（快速响应，适合简单环境）
roslaunch pure_pursuit_local_planner pure_pursuit_configurations.launch config:=laser_only

# 仅使用代价地图（适合无激光雷达或激光雷达数据质量差的情况）
roslaunch pure_pursuit_local_planner pure_pursuit_configurations.launch config:=costmap_only

# 纯路径跟踪（无避障，最高性能）
roslaunch pure_pursuit_local_planner pure_pursuit_configurations.launch config:=none
```

**配置选择建议：**
- `both`: 推荐用于大多数应用，综合考虑静态和动态障碍物
- `laser_only`: 适合开阔环境，需要快速响应动态障碍物
- `costmap_only`: 适合静态环境或激光雷达不可用时
- `none`: 适合已知安全环境或仅作为路径跟踪器使用

## 算法原理

### Pure Pursuit算法
1. **前瞻点计算**: 根据当前速度和配置参数计算前瞻距离
2. **路径点搜索**: 在全局路径上找到距离机器人前瞻距离的点
3. **控制指令计算**: 基于前瞻点计算转向半径和角速度

### 避障策略
1. **障碍物检测**: 融合激光雷达和代价地图数据
2. **危险评估**: 根据障碍物距离和方向评估危险程度
3. **速度调整**: 
   - 角速度调整：避开侧方障碍物
   - 线速度调整：根据前方障碍物距离减速或停止

### 速度规划
- 根据路径曲率动态调整目标速度
- 考虑加速度限制的平滑加减速
- 基于障碍物距离的安全减速

## 调优建议

### 1. 路径跟踪精度调优
- 减小 `lookahead_distance_min` 提高跟踪精度
- 增大 `lookahead_ratio` 提高高速时的稳定性

### 2. 避障敏感度调优
- 调整 `safety_distance` 改变避障安全裕度
- 调整 `angular_avoidance_gain` 改变避障反应强度

### 3. 速度性能调优
- 根据机器人动力学调整 `acceleration_limit`
- 根据环境复杂度调整 `max_linear_velocity`

## 故障排除

### 常见问题

1. **机器人不动**
   - 检查是否接收到全局路径
   - 检查TF变换是否正常
   - 检查控制指令话题是否正确

2. **路径跟踪不准确**
   - 调整前瞻距离参数
   - 检查路径质量
   - 调整控制频率

3. **避障过于敏感**
   - 增大 `safety_distance`
   - 减小 `angular_avoidance_gain`
   - 检查传感器数据质量

### 调试工具
- 使用RViz查看可视化信息
- 监控话题数据：`rostopic echo /cmd_vel`
- 查看日志输出：`rosnode logs`

## 依赖项

- ROS Melodic/Noetic
- Eigen3
- tf2
- costmap_2d (可选，用于代价地图集成)

## 许可证

MIT License

## 作者

开发者: AI Assistant
维护者: [Your Name]
邮箱: [your_email@example.com]

## 版本历史

- v1.0.0: 初始版本，包含基本的Pure Pursuit和避障功能
