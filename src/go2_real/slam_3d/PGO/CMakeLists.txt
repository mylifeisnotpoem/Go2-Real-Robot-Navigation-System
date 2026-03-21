cmake_minimum_required(VERSION 3.0.2)
project(pgo)

# 添加编译选项
SET(CMAKE_BUILD_TYPE "Debug")
SET(CMAKE_CXX_FLAGS_DEBUG "$ENV{CXXFLAGS} -O0 -Wall -g -ggdb")
SET(CMAKE_CXX_FLAGS_RELEASE "$ENV{CXXFLAGS} -O3 -Wall")
add_definitions(-DROOT_DIR="${CMAKE_CURRENT_SOURCE_DIR}/")
# 添加Boost组件
find_package(Boost REQUIRED COMPONENTS 
  system 
  thread
  serialization
  timer 
  chrono
)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  nav_msgs
  sensor_msgs
  geometry_msgs
  pcl_ros
)

find_package(PCL REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(GTSAM REQUIRED)

# 设置版本号为4.0以上的GTSAM
if(NOT ${GTSAM_VERSION} VERSION_GREATER "4.0")
  message(FATAL_ERROR "Required GTSAM version >= 4.0")
endif()

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES ${PROJECT_NAME}
  jsk_recognition_msgs
  CATKIN_DEPENDS roscpp nav_msgs sensor_msgs geometry_msgs pcl_ros
  DEPENDS PCL EIGEN3 GTSAM Boost
)

# 包含目录
include_directories(
  ${PROJECT_SOURCE_DIR}/include
  ${PROJECT_SOURCE_DIR}/include/ground_seg
  ${catkin_INCLUDE_DIRS}
  ${PCL_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIRS}
  ${GTSAM_INCLUDE_DIR}
  ${Boost_INCLUDE_DIRS}
)


# 添加可执行文件
add_executable(${PROJECT_NAME}_node
  src/pgo_node.cpp 
  src/pgo.cpp
  src/loop_closure/scan_matching/scan_matching.cpp
)
# 添加链接选项允许多重定义
target_link_options(${PROJECT_NAME}_node PRIVATE 
  "LINKER:--allow-multiple-definition"
)
# 设置目标属性
set_target_properties(${PROJECT_NAME}_node PROPERTIES OUTPUT_NAME pgo_node PREFIX "")

# 链接库
target_link_libraries(${PROJECT_NAME}_node
  ${catkin_LIBRARIES}
  ${PCL_LIBRARIES}
  ${EIGEN3_LIBRARIES}
  gtsam
  ${Boost_LIBRARIES}
)

# 安装规则
install(TARGETS ${PROJECT_NAME}_node
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  FILES_MATCHING PATTERN "*.hpp"
)

install(DIRECTORY launch/ config/ rviz/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
  FILES_MATCHING PATTERN "*.launch" PATTERN "*.yaml" PATTERN "*.rviz"
)