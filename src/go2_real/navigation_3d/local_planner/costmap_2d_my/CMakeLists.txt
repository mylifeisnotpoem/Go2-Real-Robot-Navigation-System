cmake_minimum_required(VERSION 3.0.2)
project(local_planner)

## 编译标准
add_compile_options(-std=c++14)

## 查找catkin宏和库
find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  sensor_msgs
  geometry_msgs
  nav_msgs
  tf2
  tf2_ros
  tf2_geometry_msgs
  pcl_ros
  pcl_conversions
)

## 查找系统依赖
find_package(PCL REQUIRED)
find_package(Eigen3 REQUIRED)

## catkin特定配置
catkin_package(
  INCLUDE_DIRS include
  LIBRARIES ${PROJECT_NAME}
  CATKIN_DEPENDS 
    roscpp 
    rospy 
    std_msgs 
    sensor_msgs 
    geometry_msgs 
    nav_msgs 
    tf2 
    tf2_ros 
    tf2_geometry_msgs
    pcl_ros
    pcl_conversions
  DEPENDS PCL EIGEN3
)

## 包含目录
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${PCL_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIRS}
)

## 添加PCL定义
add_definitions(${PCL_DEFINITIONS})

## 链接目录
link_directories(${PCL_LIBRARY_DIRS})

## 创建costmap_2d库
add_library(${PROJECT_NAME}
  src/costmap_2d.cpp
)

## 链接库
target_link_libraries(${PROJECT_NAME}
  ${catkin_LIBRARIES}
  ${PCL_LIBRARIES}
)

## 创建costmap_2d节点
add_executable(costmap_2d_node_my src/costmap_2d_node.cpp)
target_link_libraries(costmap_2d_node_my
  ${PROJECT_NAME}
  ${catkin_LIBRARIES}
  ${PCL_LIBRARIES}
)

## 添加依赖
add_dependencies(${PROJECT_NAME} ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
add_dependencies(costmap_2d_node_my ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

## 安装
install(TARGETS ${PROJECT_NAME} costmap_2d_node_my
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  FILES_MATCHING PATTERN "*.h"
)

install(DIRECTORY launch/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
)

install(DIRECTORY config/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/config
)
