# 导航相关ROS Shell命令速查

根据 `navigation_control_cli.py` 与 `navigation_control_gui.py` 提取的常用话题与命令行示例，便于直接在终端操作。默认使用 `rostopic`，需要 ROS 环境已启动。

## 发送导航任务 / 点位
- **CLI整数信号（std_msgs/Int32 -> /nav_task_signal）**
  - 开始循环导航: `rostopic pub /nav_task_signal std_msgs/Int32 "data: 1" -1`
  - 停止任务: `rostopic pub /nav_task_signal std_msgs/Int32 "data: 2" -1`
  - 继续执行: `rostopic pub /nav_task_signal std_msgs/Int32 "data: 3" -1`
  - 回家: `rostopic pub /nav_task_signal std_msgs/Int32 "data: 4" -1`
- **GUI字符串信号（std_msgs/String -> /nav_task_signal）**
  - 发送任务文件名（不含 `.txt` 扩展名）启动导航: `rostopic pub /nav_task_signal std_msgs/String "data: '二教学楼巡检任务'" -1`
  - 回家: `rostopic pub /nav_task_signal std_msgs/String "data: 'GO_HOME'" -1`
  - 停止任务: `rostopic pub /nav_task_signal std_msgs/String "data: 'STOP'" -1`
  - 继续任务: `rostopic pub /nav_task_signal std_msgs/String "data: 'CONTINUE'" -1`

## 紧急停止
- 激活紧急停止: `rostopic pub /emergency_stop std_msgs/Bool "data: true" -1`
- 解除紧急停止: `rostopic pub /emergency_stop std_msgs/Bool "data: false" -1`

## 查询当前位置 / 状态
- 导航状态（`std_msgs/Bool`，true表示已到达目标点）: `rostopic echo -n 1 /nav_state`
- 当前导航目标点（`geometry_msgs/Pose`，脚本订阅的目标点位）: `rostopic echo -n 1 /target_points`
  - 输出包含 `position {x, y, z}` 与 `orientation {x, y, z, w}`；这是当前导航的目标点，而非实时位姿。

## 接收X/Y点位坐标的导航接口
- `/clicked_point`（geometry_msgs/PointStamped）  
  - 订阅方：全局路径规划器 `global_path_planner`（见 `navigation_3d/global_planner/global_path_planner/src/global_path_planner.cpp: goalCallback`），以及 `common/sherpa_onnx_ros/src/goal_loop_node.cpp` 等会向该话题发布循环目标。  
  - 作用：接受 `x,y,z` 点位作为导航目标，规划全局路径。  
  - 发布示例（坐标系默认 `map`）：  
    - `rostopic pub /clicked_point geometry_msgs/PointStamped "{header: {frame_id: 'map'}, point: {x: 2.0, y: -0.70, z: 0.0}}" -1`
- `/target_points`（geometry_msgs/Pose）  
  - 发布方：导航任务状态机 `navigation_task_state_machine` 会根据任务发布目标点；GUI/CLI 仅展示。  
  - 订阅方：`global_path_planner`（`targetPointsCallback`）和 `pure_pursuit_local_planner`（`targetPointsCallback`）直接将其中的 `position.x/y/z` 作为导航目标。  
  - 手动发布示例（需要四元数朝向；若只关心位置可用单位四元数）：  
    - `rostopic pub /target_points geometry_msgs/Pose "{position: {x: 1.0, y: 2.0, z: 0.0}, orientation: {x: 0.0, y: 0.0, z: 0.0, w: 1.0}}" -1`
