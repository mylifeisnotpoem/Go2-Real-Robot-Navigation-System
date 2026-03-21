cmake_minimum_required(VERSION 3.0.2)
project(topology_rviz_plugin)

## 添加C++14支持
add_compile_options(-std=c++14)

## 查找catkin宏和库
find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  interactive_markers
  pluginlib
  rviz
  std_msgs
  visualization_msgs
  sensor_msgs
  pcl_conversions
  pcl_ros
)

## 系统依赖 
find_package(Qt5 REQUIRED COMPONENTS Core Widgets)
find_package(PkgConfig REQUIRED)
find_package(PCL 1.10 REQUIRED)

## 设置Qt5 MOC
set(CMAKE_AUTOMOC ON)
set(CMAKE_INCLUDE_CURRENT_DIR ON)

## catkin特定配置
catkin_package(
  INCLUDE_DIRS include
  LIBRARIES ${PROJECT_NAME}
  CATKIN_DEPENDS geometry_msgs interactive_markers pluginlib rviz std_msgs visualization_msgs sensor_msgs pcl_conversions pcl_ros
  DEPENDS Qt5 PCL
)

## 指定头文件的额外位置
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${Qt5Core_INCLUDE_DIRS}
  ${Qt5Widgets_INCLUDE_DIRS}
  ${PCL_INCLUDE_DIRS}
)

## 生成头文件的MOC
qt5_wrap_cpp(MOC_FILES
  include/topology_rviz_plugin/topology_tool.h
)

## 声明C++库
add_library(${PROJECT_NAME}
  src/topology_tool.cpp
  ${MOC_FILES}
)

## 声明C++可执行文件
add_executable(target_points_visualizer_node
  src/target_points_visualizer_node.cpp
)

## 添加cmake目标依赖
add_dependencies(${PROJECT_NAME} ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
add_dependencies(target_points_visualizer_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

## 指定链接库
target_link_libraries(${PROJECT_NAME}
  ${catkin_LIBRARIES}
  Qt5::Core
  Qt5::Widgets
  ${PCL_LIBRARIES}
)

target_link_libraries(target_points_visualizer_node
  ${catkin_LIBRARIES}
)

## 安装
install(TARGETS ${PROJECT_NAME} target_points_visualizer_node
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION}
)

install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  FILES_MATCHING PATTERN "*.h"
  PATTERN ".svn" EXCLUDE
)

install(FILES plugin_description.xml
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
