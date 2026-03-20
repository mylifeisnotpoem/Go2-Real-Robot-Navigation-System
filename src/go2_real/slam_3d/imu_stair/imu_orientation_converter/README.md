# IMU Orientation Converter

这个功能包用于将IMU数据中的四元数姿态转换为欧拉角。

## 功能

- 订阅 `/imu/data` 话题（`sensor_msgs/Imu` 类型）
- 将四元数 `geometry_msgs/Quaternion` 转换为欧拉角
- 应用低通滤波器平滑角度数据
- 发布到 `/imu/data_orientation` 话题（`geometry_msgs/Vector3Stamped` 类型，**角度制**）
- **Pitch偏差计算**: 计算当前pitch角相对于可配置基准值的偏差并发布
- **楼梯检测**: 当pitch偏差超过阈值时发布stairs_flag，状态改变时才发布

## 数据格式

### 输入
- 话题: `/imu/data`
- 类型: `sensor_msgs/Imu`
- 使用字段: `orientation` (geometry_msgs/Quaternion)

### 输出
1. **欧拉角输出**:
   - 话题: `/imu/data_orientation`
   - 类型: `geometry_msgs/Vector3Stamped`
   - 数据含义:
     - `vector.x`: roll角 (绕x轴旋转) [**度数**]
     - `vector.y`: pitch角 (绕y轴旋转) [**度数**]
     - `vector.z`: yaw角 (绕z轴旋转) [**度数**]

2. **Pitch偏差输出**:
   - 话题: `/imu/pitch_deviation`
   - 类型: `std_msgs/Float32`
   - 数据含义: `abs(当前filtered_pitch - pitch_baseline)` [**度数**]
   - 基准值: 可配置参数

3. **楼梯检测输出**:
   - 话题: `/stairs_flag`
   - 类型: `std_msgs/Bool`
   - 数据含义: 当pitch偏差 > stairs_threshold时为true，否则为false
   - 发布策略: 仅在状态改变时发布，避免重复消息

## 使用方法

### 1. 编译
```bash
cd /path/to/your/workspace
catkin_make
source devel/setup.bash
```

### 2. 运行节点
```bash
# 单独运行节点
rosrun imu_orientation_converter imu_orientation_converter_node

# 或使用launch文件
roslaunch imu_orientation_converter imu_orientation_converter.launch
```

### 3. 查看输出
```bash
# 查看原始IMU数据
rostopic echo /imu/data

# 查看转换后的欧拉角
rostopic echo /imu/data_orientation
```

## 转换说明

- 使用Eigen库进行四元数到欧拉角的转换
- 采用ZYX欧拉角约定（yaw-pitch-roll）
- **输出角度范围**: 
  - Roll: [-180°, 180°]
  - Pitch: [-90°, 90°] 
  - Yaw: [-180°, 180°]
- **低通滤波**: 应用一阶低通滤波器平滑角度数据，减少噪声
- **角度制输出**: 直接输出度数，无需额外转换
- **Pitch偏差计算**: 
  - 使用可配置基准值（默认20°）
  - 计算 `abs(当前pitch - baseline)` 并发布
  - 立即开始工作，无初始化等待时间
- **楼梯检测**: 
  - 基于pitch偏差的阈值检测（默认14°）
  - 仅在状态改变时发布，避免重复消息
- 保持与输入IMU消息相同的时间戳和坐标系

## 参数配置

### 滤波参数
- `alpha`: 低通滤波器系数 (0 < alpha < 1)
  - 默认值: 0.1
  - 较小的alpha值 = 更强的滤波效果
  - 较大的alpha值 = 更弱的滤波效果，响应更快

### 检测参数
- `pitch_baseline`: pitch角基准值（度）
  - 默认值: 20.0
  - 用于计算pitch偏差的参考值

- `stairs_threshold`: 楼梯检测阈值（度）
  - 默认值: 14.0
  - 当pitch偏差超过此阈值时检测为楼梯

### 通信参数
- `queue_size`: 订阅和发布的队列大小
  - 默认值: 10
  - 较小值 = 更低延迟，较大值 = 更好的数据完整性

### 调试参数
- `verbose_logging`: 是否启用详细日志输出
  - 默认值: true
  - false = 只输出关键信息，减少日志量

### 话题重映射参数
- `imu_topic`: 输入IMU话题名称
  - 默认值: "/imu/data"
- `euler_topic`: 输出欧拉角话题名称
  - 默认值: "/imu/data_orientation"
- `pitch_deviation_topic`: 输出pitch偏差话题名称
  - 默认值: "/imu/pitch_deviation"
- `stairs_flag_topic`: 输出楼梯检测标志话题名称
  - 默认值: "/stairs_flag"
- `node_name`: 节点名称
  - 默认值: "imu_orientation_converter"

### 使用示例
```bash
# 使用默认参数
roslaunch imu_orientation_converter imu_orientation_converter.launch

# 自定义检测参数
roslaunch imu_orientation_converter imu_orientation_converter.launch \
  pitch_baseline:=15.0 \
  stairs_threshold:=10.0

# 自定义滤波参数（强滤波）
roslaunch imu_orientation_converter imu_orientation_converter.launch alpha:=0.05

# 自定义话题名称
roslaunch imu_orientation_converter imu_orientation_converter.launch \
  imu_topic:=/my_imu/data \
  euler_topic:=/my_imu/orientation \
  pitch_deviation_topic:=/my_imu/pitch_dev \
  stairs_flag_topic:=/my_imu/stairs

# 快速响应配置
roslaunch imu_orientation_converter imu_orientation_converter.launch \
  alpha:=0.3 \
  verbose_logging:=false

# 查看各种数据
rostopic echo /imu/pitch_deviation        # pitch偏差
rostopic echo /imu/data_orientation       # 欧拉角
rostopic echo /stairs_flag                # 楼梯检测标志
```

## 依赖项

- ROS Noetic
- Eigen3
- sensor_msgs
- geometry_msgs
- tf2_ros
- tf2_geometry_msgs
