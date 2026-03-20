cmake_minimum_required(VERSION 3.0.2)
project(pcd_map)

## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  nav_msgs
  pcl_ros
  roscpp
  rospy
  std_msgs
  geometry_msgs
  sensor_msgs
  visualization_msgs
)

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES pcd2pgm
#  CATKIN_DEPENDS nav_msgs pcl roscpp rospy std_msgs
#  DEPENDS system_lib
)


include_directories(
# include
  ${catkin_INCLUDE_DIRS}
)


find_package(PCL REQUIRED)

include_directories(
  include
		${PCL_INCLUDE_DIRS})


add_executable(pcd_filter src/pcd_filter.cpp)
target_link_libraries(pcd_filter ${catkin_LIBRARIES} ${PCL_LIBRARIES})

add_executable(cloud_publisher src/cloud_publisher.cpp)
target_link_libraries(cloud_publisher ${catkin_LIBRARIES} ${PCL_LIBRARIES})
