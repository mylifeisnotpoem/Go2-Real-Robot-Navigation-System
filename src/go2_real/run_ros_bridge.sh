# 1. 关键修改：在 XML 里加上 AllowMulticast=false 和 Peer=127.0.0.1
#export CYCLONEDDS_URI='<CycloneDDS><Domain><General><NetworkInterfaceAddress>enP8p1s0</NetworkInterfaceAddress><AllowMulticast>true</AllowMulticast></General><Discovery><ParticipantIndex>auto</ParticipantIndex></Discovery></Domain></CycloneDDS>'

# 2. 指定实现
#export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
#export ROS_MASTER_URI=http://localhost:11311

# 3. source 环境
source /opt/ros/noetic/setup.bash
source /opt/ros/foxy/setup.bash

# 4. 启动！
#ros2 run ros1_bridge dynamic_bridge --bridge-all-topics

ros2 run ros1_bridge dynamic_bridge
