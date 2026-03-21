# Path Smoother

基于B样条曲线的ROS路径平滑节点。

## 功能特性

- **B样条曲线平滑**: 使用可配置阶数的B样条曲线对路径进行平滑处理
- **控制点优化**: 自动减少控制点数量以提高计算效率
- **路径验证**: 确保平滑后的路径不会偏离原始路径过远
- **质量评估**: 计算路径长度、曲率和偏差等质量指标
- **可视化支持**: 显示原始路径、平滑路径和控制点

## 参数配置

### 基本参数
- `global_frame`: 全局坐标系名称 (默认: "map")
- `enable_visualization`: 是否启用可视化 (默认: true)

### B样条参数
- `spline_degree`: B样条阶数 (默认: 3, 建议: 2-4)
- `num_output_points`: 输出路径点数量 (默认: 100)
- `smoothing_tolerance`: 平滑容差 (默认: 0.1)

### 控制点参数
- `control_point_reduction_distance`: 控制点减少的最小距离 (默认: 1.0 m)

### 验证参数
- `max_deviation`: 允许的最大偏差 (默认: 2.0 m)

## 话题接口

### 订阅话题
- `global_path` (nav_msgs/Path): 原始全局路径

### 发布话题
- `smoothed_path` (nav_msgs/Path): 平滑后的路径
- `path_smoother_visualization` (visualization_msgs/MarkerArray): 可视化标记

## 使用方法

### 1. 单独启动路径平滑器
```bash
roslaunch path_smoother path_smoother.launch
```

### 2. 与全局路径规划器一起启动
```bash
roslaunch path_smoother test_with_smoother.launch
```

### 3. 参数调节示例
```bash
# 启动时设置参数
roslaunch path_smoother path_smoother.launch spline_degree:=2 num_output_points:=150 max_deviation:=1.5
```

## 算法说明

### B样条曲线
使用开放均匀B样条曲线进行路径平滑：
1. 从原始路径提取控制点
2. 可选地减少控制点数量以提高效率
3. 生成B样条节点向量
4. 计算B样条曲线上的采样点
5. 为每个点计算合适的方向

### 路径验证
- **偏差检查**: 确保平滑路径不会偏离原始路径超过设定阈值
- **长度检查**: 确保路径长度变化在合理范围内
- **连续性检查**: 确保路径的连续性和平滑性

### 质量指标
- **路径长度**: 计算总路径长度
- **平均曲率**: 评估路径的平滑程度
- **最大偏差**: 与原始路径的最大偏离距离

## 可视化说明

在RViz中可以看到：
- **红色线条**: 原始路径
- **绿色线条**: 平滑后的路径  
- **蓝色球体**: B样条控制点

## 性能调优

### 提高平滑度
- 增加`spline_degree`（2-4之间）
- 减少`control_point_reduction_distance`

### 提高计算效率
- 减少`num_output_points`
- 增加`control_point_reduction_distance`

### 保持路径准确性
- 减少`max_deviation`
- 减少`control_point_reduction_distance`

## 故障排除

### 常见问题

1. **路径偏差过大**
   - 调整`max_deviation`参数
   - 检查原始路径质量

2. **平滑效果不明显**
   - 增加`spline_degree`
   - 减少`control_point_reduction_distance`

3. **计算性能问题**
   - 减少`num_output_points`
   - 增加`control_point_reduction_distance`
   - 降低`spline_degree`

### 日志信息
- INFO: 路径平滑完成情况和质量指标
- WARN: 验证失败或参数问题
- ERROR: 严重错误，将发布原始路径

## 依赖项

- ROS Noetic
- nav_msgs
- geometry_msgs
- visualization_msgs
- tf
- Eigen3 (用于数学计算)

## 编译和安装

```bash
cd your_catkin_workspace
catkin_make --only-pkg-with-deps path_smoother
source devel/setup.bash
```
