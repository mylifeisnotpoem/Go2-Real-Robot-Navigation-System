cmake_minimum_required(VERSION 3.0.2)
project(go2_ros_control)

# 设置C++标准
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_BUILD_TYPE Release)
list(APPEND CMAKE_PREFIX_PATH "/opt/ros/noetic" "/opt/unitree_robotics")

# 查找Unitree SDK包（关键修改）
find_package(unitree_sdk2 REQUIRED)

# 查找ROS和Boost依赖
find_package(catkin REQUIRED COMPONENTS 
  roscpp 
  geometry_msgs
  std_msgs
  std_srvs
  state_machine_msg
  
)
find_package(Boost REQUIRED COMPONENTS system thread)

# Catkin配置
catkin_package(
  CATKIN_DEPENDS roscpp geometry_msgs std_msgs std_srvs state_machine_msg 
)

# 安装Python脚本到正确的位置，使其可通过rosrun执行
catkin_install_python(PROGRAMS
  scripts/go2_driver_node.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

# 头文件路径（通过target_include_directories更规范地管理）
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${Boost_INCLUDE_DIRS}
)

# 添加所有可执行文件（参考官网示例）
add_executable(go2_vel_controller_node src/go2_vel_controller_node.cpp)
add_executable(keyboard_control_node src/keyboard_control_node.cpp)
# add_executable(go2_mode_switch_node src/go2_mode_switch_node.cpp)

# 统一配置所有目标
foreach(target
  go2_vel_controller_node
  keyboard_control_node
  # go2_mode_switch_node
)
  # 依赖项
  add_dependencies(${target} ${catkin_EXPORTED_TARGETS})

  # 包含目录
  target_include_directories(${target} PRIVATE
    ${UNITREE_SDK2_INCLUDE_DIRS}  # 假设find_package导出该变量
    ${catkin_INCLUDE_DIRS}
    ${Boost_INCLUDE_DIRS}
  )

  # 链接库
  target_link_libraries(${target}
    ${catkin_LIBRARIES}
    ${Boost_LIBRARIES}
    unitree_sdk2
  )

  # 编译选项
  target_compile_options(${target} PRIVATE
    -O3 -Wall -Wno-deprecated
  )
endforeach()
