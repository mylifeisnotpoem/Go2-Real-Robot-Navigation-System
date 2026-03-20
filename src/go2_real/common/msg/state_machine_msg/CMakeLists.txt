cmake_minimum_required(VERSION 3.0.2)
project(state_machine_msg)

find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  message_generation
  std_msgs
)

## Generate messages in the 'msg' folder
add_message_files(
  FILES
  keywords.msg
)

## Generate added messages and services with any dependencies listed here
generate_messages(
  DEPENDENCIES
  geometry_msgs
  std_msgs
)

catkin_package(
  CATKIN_DEPENDS geometry_msgs message_runtime std_msgs
)
