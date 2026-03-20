# IMU约束增强功能测试指南

## 功能概述

新实现的IMU约束增强功能旨在解决特征点稀疏环境（如走廊、空旷区域）下的滑动漂移问题。当特征点数量不足时，系统会自动启用IMU约束来增强定位鲁棒性。

## 主要特性

1. **自动触发机制**：根据特征点数量自动启用不同强度的IMU约束
2. **可配置参数**：通过YAML配置文件灵活调整约束行为
3. **多级约束策略**：强约束、中等约束、无约束三种模式
4. **选择性约束**：可分别控制位置、速度、旋转约束的启用

## 配置参数说明

在 `config/mapping_mid360.yaml` 文件中新增的参数：

```yaml
imu_constraint:
    enable_auto_constraint: true    # 是否启用自动IMU约束
    feature_threshold_strong: 100   # 触发强约束的特征点阈值
    feature_threshold_medium: 200   # 触发中等约束的特征点阈值
    strong_constraint_weight: 3.0   # 强约束时的IMU权重
    medium_constraint_weight: 2.0   # 中等约束时的IMU权重
    max_rotation_correction: 0.1    # 最大旋转修正量（弧度）
    enable_velocity_constraint: true # 是否约束速度
    enable_rotation_constraint: true # 是否约束旋转
```

## 约束逻辑

1. **特征点 < 100**: 启用强约束（权重3.0）
2. **特征点 100-200**: 启用中等约束（权重2.0）
3. **特征点 > 200**: 正常模式，不启用额外约束

## 测试方法

### 1. 基础功能测试

```bash
# 1. 启动主节点（使用默认配置，已启用IMU约束）
roslaunch fast_lio_localization mapping_mid360.launch

# 2. 播放数据包（走廊或特征稀疏环境）
rosbag play your_corridor_data.bag

# 3. 观察日志输出，查看约束触发情况
```

### 2. 性能对比测试

```bash
# 测试1：启用IMU约束
roslaunch fast_lio_localization mapping_mid360.launch

# 测试2：禁用IMU约束
# 修改 config/mapping_mid360.yaml 中的 enable_auto_constraint: false
roslaunch fast_lio_localization mapping_mid360.launch
```

### 3. 参数调优测试

根据实际环境调整以下参数：

- 降低阈值（更早触发约束）: `feature_threshold_strong: 50, feature_threshold_medium: 150`
- 增强约束强度: `strong_constraint_weight: 4.0, medium_constraint_weight: 3.0`
- 仅约束位置: `enable_velocity_constraint: false, enable_rotation_constraint: false`

## 日志监控

系统会输出以下关键日志信息：

```
[INFO] IMU约束配置: 自动启用=是, 强约束阈值=100, 中等约束阈值=200
[WARN] 特征点过少(45<100)，启用强IMU约束 (权重: 3.0)
[INFO] IMU约束融合: 特征点=45, 权重=0.25/0.75, 位置差=0.156m
```

## 预期效果

1. **走廊环境**: 显著减少前进方向的漂移
2. **空旷区域**: 提高定位稳定性，减少震荡
3. **特征变化**: 在特征丰富区域自动退出约束，保持精度

## 故障排除

### 如果约束过强（运动滞后）
- 降低权重: `strong_constraint_weight: 2.0, medium_constraint_weight: 1.5`
- 提高阈值: `feature_threshold_strong: 150`

### 如果约束不足（仍有漂移）
- 增强权重: `strong_constraint_weight: 4.0`
- 降低阈值: `feature_threshold_strong: 80`
- 启用速度约束: `enable_velocity_constraint: true`

### 如果旋转异常
- 禁用旋转约束: `enable_rotation_constraint: false`
- 降低最大修正量: `max_rotation_correction: 0.05`

## 高级配置

### 针对不同场景的推荐配置

**狭窄走廊**:
```yaml
feature_threshold_strong: 80
strong_constraint_weight: 4.0
enable_rotation_constraint: false
```

**开放空间**:
```yaml
feature_threshold_strong: 120
medium_constraint_weight: 1.8
max_rotation_correction: 0.15
```

**高动态环境**:
```yaml
enable_velocity_constraint: true
strong_constraint_weight: 2.5
```
