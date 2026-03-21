cmake_minimum_required(VERSION 3.0.2)
project(unitree_go)

find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  message_generation
  std_msgs
)

## Generate messages in the 'msg' folder
add_message_files(
  FILES
  AudioData.msg
  BmsCmd.msg
  BmsState.msg
  Error.msg
  Go2FrontVideoData.msg
  HeightMap.msg
  IMUState.msg
  InterfaceConfig.msg
  LowState_time.msg
  LidarState.msg
  LowCmd.msg
  LowState.msg
  MotorCmd.msg
  MotorCmds.msg
  MotorState.msg
  MotorStates.msg
  PathPoint.msg
  Req.msg
  Res.msg
  SportModeCmd.msg
  SportModeState.msg
  TimeSpec.msg
  UwbState.msg
  UwbSwitch.msg
  VoxelMapCompressed.msg
  VoxelHeightMapState.msg
  WebRtcReq.msg
  WirelessController.msg
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
