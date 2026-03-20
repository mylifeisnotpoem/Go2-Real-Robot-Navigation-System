# 双雷达可行区域提取器

基于PGO包的地面分割算法，融合激光雷达和毫米波雷达数据进行可行区域提取的ROS包。

## 功能特性

- **单雷达/双雷达模式**: 支持仅使用激光雷达或融合激光雷达+毫米波雷达数据
- **智能模式切换**: 通过参数`use_dual_radar`控制是否使用双雷达融合
- **时间同步**: 在双雷达模式下自动同步两种传感器的数据
- **地面分割**: 基于PGO的简化高度阈值地面分割算法
- **可行区域提取**: 考虑机器人半径的安全区域计算
- **多格式输出**: 提供点云和多边形两种可行区域表示

## 模式说明

### 单雷达模式（默认）
- 仅使用激光雷达数据 `/cloud_registered_body_1`
- 该点云已在body frame下，无需额外坐标变换
- 适合激光雷达数据质量较好的场景

### 双雷达模式
- 融合激光雷达和毫米波雷达数据
- 自动进行时间同步和坐标变换
- 适合需要增强环境感知的复杂场景

## 订阅话题

- `/cloud_registered_body_1` (sensor_msgs/PointCloud2): 激光雷达点云
- `/radar_points` (sensor_msgs/PointCloud2): 毫米波雷达点云

## 发布话题

- `traversable_area_cloud` (sensor_msgs/PointCloud2): 可行区域点云
- `ground_cloud` (sensor_msgs/PointCloud2): 地面点云
- `obstacle_cloud` (sensor_msgs/PointCloud2): 障碍物点云
- `combined_cloud` (sensor_msgs/PointCloud2): 融合后的原始点云
- `traversable_polygon` (geometry_msgs/PolygonStamped): 可行区域多边形

## 参数配置

主要参数在 `config/dual_radar_config.yaml` 中配置：

### 核心参数
- `use_dual_radar`: 是否启用双雷达模式（默认false）
- `height_threshold`: 地面高度阈值（默认0.1米，参考PGO设置）
- `robot_radius`: 机器人半径，用于安全距离计算
- `time_sync_threshold`: 雷达数据时间同步阈值（仅双雷达模式）
- `voxel_leaf_size`: 体素滤波器大小

### 话题配置
- `lidar_topic`: 激光雷达点云话题（默认`/cloud_registered_body_1`）
- `radar_topic`: 毫米波雷达点云话题（默认`/radar_points`，仅双雷达模式使用）

## 使用方法

### 编译
```bash
cd /home/server/WS_ROS1/ws_go2_sim
catkin_make --pkg dual_radar_traversable_extractor
source devel/setup.bash
```

### 运行
```bash
# 使用默认配置（单雷达模式）
roslaunch dual_radar_traversable_extractor dual_radar_extractor.launch

# 启用双雷达模式
roslaunch dual_radar_traversable_extractor dual_radar_extractor.launch use_dual_radar:=true

# 查看发布的话题
rostopic list | grep dual_radar

# 查看可行区域点云
rostopic echo /traversable_area_cloud

# 检查模式状态
rosnode info /dual_radar_traversable_extractor
```

## 算法原理

1. **数据同步**: 根据时间戳匹配激光雷达和毫米波雷达数据
2. **坐标变换**: 将两种传感器数据统一到base_link坐标系
3. **点云融合**: 合并两种传感器的点云数据
4. **预处理**: 距离滤波、体素滤波、统计滤波
5. **地面分割**: 使用PGO的高度阈值算法进行地面分割
6. **可行区域提取**: 基于机器人半径判断安全可行区域
7. **结果发布**: 发布多种格式的结果数据

## 与PGO的关系

本包从PGO项目中提取并简化了以下核心功能：
- `performGroundSegmentation()`: 地面分割算法
- 高度阈值过滤逻辑
- 点云变换和融合机制
- 参数配置方案

移除了PGO中的：
- GTSAM图优化
- 回环检测
- 关键帧管理
- 复杂的SLAM功能

## 调试和可视化

在RViz中添加以下显示项进行可视化：
- PointCloud2: `combined_cloud` (白色)
- PointCloud2: `ground_cloud` (绿色)  
- PointCloud2: `obstacle_cloud` (红色)
- PointCloud2: `traversable_area_cloud` (蓝色)
- Polygon: `traversable_polygon`
