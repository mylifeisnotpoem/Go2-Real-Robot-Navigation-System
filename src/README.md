# Go2 Real Robot Navigation System

这是一个基于ROS的Go2机器人实时导航系统，集成了SLAM建图、定位、路径规划和语音交互功能。

## 🏗️ 项目结构

```
ws_go2_real/src/
├── CMakeLists.txt
├── README.md
├── .gitignore
├── 1_run_bag_record.sh          # 数据录制脚本
├── 2_run_slam.sh                # SLAM建图脚本
├── 3_edit_map.sh                # 地图编辑脚本
├── 4_map_preproces.sh           # 地图预处理脚本
├── 5_nav_start.sh               # 导航启动脚本
├── run_localization.sh          # 定位脚本
├── run_localization_bag.sh      # 离线定位脚本
├── run_slam_rosbag.sh           # 离线SLAM脚本
├──
├── common/                      # 通用组件
│   ├── go2_drive/              # Go2机器人驱动
│   │   ├── go2_ros_control/    # ROS控制接口
│   │   ├── livox_ros_driver2/  # Livox激光雷达驱动
│   │   ├── livox2pointcloud/   # 激光雷达点云转换
│   │   └── sherpa_onnx_ros/    # 语音识别与交互
│   └── msg/                    # 自定义消息类型
│       ├── state_machine_msg/  # 状态机消息
│       └── unitree_go/         # Unitree Go消息
├──
├── slam_3d/                     # 3D SLAM模块
│   ├── FAST_LIO_SLAM/          # Fast-LIO SLAM算法
│   ├── imu_stair/              # IMU楼梯检测
│   └── PGO/                    # 后端图优化
├──
├── navigation_3d/               # 3D导航模块
│   ├── fast_lio_nav/           # Fast-LIO导航
│   │   ├── FAST_LIO_NAV/       # Fast-LIO导航算法
│   │   └── open3d_loc/         # Open3D定位
│   ├── global_planner/         # 全局路径规划
│   │   ├── global_path_planner/    # 全局路径规划器
│   │   ├── path_smoother/          # 路径平滑
│   │   ├── pcd_map/               # PCD地图处理
│   │   ├── topology_map_generator/ # 拓扑地图生成
│   │   └── topology_rviz_plugin/  # RViz拓扑插件
│   └── local_planner/          # 局部路径规划
│       ├── costmap_2d/         # 2D代价地图
│       ├── costmap_2d_my/      # 自定义代价地图
│       ├── dual_radar_traversable_extractor/ # 双雷达可通行性提取
│       ├── pointcloud_to_laserscan/         # 点云转激光
│       └── pure_pursuit_local_planner/      # 纯跟踪局部规划器
```

## 🚀 快速开始

按照以下顺序执行脚本，完成从数据收集到导航的完整流程：

### 步骤 1: 数据录制
```bash
./1_run_bag_record.sh
```
**作用：** 启动机器人传感器数据录制功能
- 启动Go2机器人的各种传感器（激光雷达、IMU、相机等）
- 开始录制rosbag数据包，用于后续的离线SLAM建图
- 建议在目标环境中移动机器人，收集完整的环境数据

### 步骤 2: SLAM建图
```bash
./2_run_slam.sh
```
**作用：** 运行3D SLAM算法进行环境建图
- 启动Fast-LIO SLAM算法
- 使用激光雷达和IMU数据进行实时建图
- 生成点云地图和轨迹信息
- 可以在RViz中实时查看建图效果

### 步骤 3: 地图编辑
```bash
./3_edit_map.sh
```
**作用：** 对生成的地图进行编辑和优化
- 启动地图编辑工具
- 可以删除地图中的噪声点
- 添加关键导航点和标签
- 优化地图质量，提高导航精度

### 步骤 4: 地图预处理
```bash
./4_map_preproces.sh
```
**作用：** 对编辑后的地图进行预处理
- 生成拓扑地图和导航图
- 计算可通行区域和障碍物信息
- 创建用于路径规划的数据结构
- 生成关键节点地图文件

### 步骤 5: 启动导航
```bash
./5_nav_start.sh
```
**作用：** 启动完整的导航系统
- 加载预处理后的地图
- 启动定位、路径规划和控制模块
- 开启语音交互功能
- 机器人进入可导航状态，可接受目标点指令

## 🎯 导航功能

### 语音交互命令
- **"导航开始"** - 开始循环导航所有关键点
- **"导航终止"** - 停止当前导航任务
- **"回家"** - 返回原点位置(0,0,0)
- **标签导航** - 说出地图中的标签名称，直接导航到对应位置

### 手动导航
- 在RViz中使用"2D Nav Goal"工具点击目标位置
- 通过发布`/clicked_point`话题设置导航目标

## 📋 系统要求

- **操作系统：** Ubuntu 18.04/20.04
- **ROS版本：** ROS Melodic/Noetic
- **硬件要求：**
  - Unitree Go2机器人
  - Livox激光雷达
  - 足够的计算资源用于实时SLAM和导航

## 🛠️ 编译与安装

```bash
# 进入工作空间
cd ws_go2_real

# 安装依赖
rosdep install --from-paths src --ignore-src -r -y

# 编译
catkin_make

# 设置环境变量
source devel/setup.bash
```

## 📝 注意事项

1. **执行顺序：** 请严格按照1→2→3→4→5的顺序执行脚本
2. **数据录制：** 步骤1中要确保在目标环境中充分移动机器人
3. **地图质量：** 步骤3的地图编辑对最终导航效果很重要
4. **安全第一：** 导航过程中请确保环境安全，随时准备紧急停止
5. **配置文件：** 各模块的参数可在对应的config目录中调整

## 🐛 故障排除

### 常见问题
- **传感器连接失败：** 检查硬件连接和驱动程序
- **建图效果差：** 调整SLAM参数或重新录制数据
- **导航精度低：** 优化地图质量和定位参数
- **语音识别失败：** 检查麦克风和语音模型配置

### 日志查看
```bash
# 查看ROS日志
roscd
cd ../log

# 实时查看节点日志
rostopic echo /rosout
```

## 🤝 贡献

欢迎提交Issue和Pull Request来改进这个项目。

**项目开发者：** 李申奥  
**所属机构：** 杭州电子科技大学  
**联系方式：** 1692931357@qq.com

## 📄 许可证

MIT License

---

**最后更新：** 2025年8月16日  
**维护者：** 李申奥 (杭州电子科技大学)  
**联系邮箱：** 1692931357@qq.com
