# PCD → PGM 离线转换工具（C++版，ROS1/ROS2 map_server 兼容）

参考 ROS2 组件 `/Volumes/ssd/back/zs/zs-slam_v1/src/pcd2pgm`，用 PCL 重新实现离线转换，输出 map.pgm + map.yaml（occupied=0，free=254，unknown=205），默认做障碍膨胀，边缘更清晰。

## 依赖
- PCL（随 ROS 安装通常自带）
- CMake >= 3.10，g++ 支持 C++17

## 编译
```bash
cd ws_nav/src/tools/pcd_to_pgm
mkdir -p build && cd build
cmake ..
make -j4
# 生成 ./pcd_to_pgm_cpp
```

## 用法
### 单个 PCD
```bash
./pcd_to_pgm_cpp --pcd /path/to/map.pcd --out ./output \
  --resolution 0.05 --z-min -0.5 --z-max 0.5 --inflate 0 \
  --radius 0.1 --min-neighbors 10 --min-points-per-cell 1 --open-radius 0 \
  --transform 0 0 0 0 0 0
# 结果：output/map.pgm + output/map.yaml
```

### 目录批处理（多楼层）
```bash
./pcd_to_pgm_cpp --pcd ./pcd_dir --out ./maps_out
# pcd_dir 下所有 *.pcd 按文件序生成 floor_*/map.{pgm,yaml}，并输出 index.yaml
```

### 可选参数（已对齐 pcd2pgm.yaml 默认）
- `--resolution` 栅格分辨率（默认 0.05 m）
- `--z-min / --z-max` Z 轴截取范围（默认 -0.5 ~ 0.5）；`--invert-z` 可保留范围外的点（与 ROS2 节点的 `flag_pass_through` 等价）
- `--radius / --min-neighbors` 半径离群滤波（默认 0.1 / 10，等效 pcd2pgm.yaml）
- `--min-points-per-cell` 栅格中判为占用所需的点数（默认 1，建议 2~3 可滤掉运动轨迹）
- `--open-radius` 形态学 opening 半径，去除细线条噪声（默认 0，可设 1~2）
- `--inflate` 占用膨胀半径，单位 cell（默认 0，保持 ROS2 结果无额外外框）
- `--transform x y z roll pitch yaw` 先旋转后平移，保持与 ROS2 参数 `odom_to_lidar_odom` 一致（内部同样取逆）
- `--origin-z`、`--occupied-thresh`、`--free-thresh` 写入 YAML 的配置

## 输出格式
- PGM：黑=障碍(0)，白=可通行(254)，灰=未知(205)；写入时翻转 Y 以保证原点在左下角。
- YAML：`image, resolution, origin[x,y,z], negate=0, occupied_thresh, free_thresh`
- 多楼层：`index.yaml` 记录 floor id、源 PCD 文件名、对应 map YAML 相对路径。

## 旧版 Python 脚本
如需对比或快速测试，`pcd_to_pgm.py` 依然保留。
