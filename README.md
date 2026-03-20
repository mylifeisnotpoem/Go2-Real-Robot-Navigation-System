# Go2-Real-Robot-Navigation-System
ROS1 建图导航流程

首先配置相关网络（对于9045）

1. 连接狗子的wifi，这个ip是通过网线连接狗内路由的
ssh wheeltec@192.168.168.50   密码：dongguan
给狗子连接wifi，如已经连接可跳过：sudo nmcli dev wifi connect "WL-OFFICE" password "wlwl1102"

ifconfig   找到公司路由分配的IP 记住这个IP

2.连接公司wifi，，
ssh wheeltec@192.168.10.xxx    密码：dongguan


3. 若是网络ip变动，解决方案：
nmcli connection show #列出所有的网络配置，找到不对的地方

wheeltec@wheeltec:~$ nmcli connection show 
NAME UUID TYPE DEVICE 
Wi-Fi connection 1 
435bc051-93a1-4a8f-b22b-8920240e9be7 
wifi 
wlP1p1s0 

sudo nmcli connection down "Wi-Fi connection 1"
sudo nmcli connection up "WL-OFFICE"



启动（这一步导航时再做，会抢占遥控器的控制权）

1. 启动底层运控 ws_ros2
cd ./ws_ros2/ws_robot_control
source ./install/setup.bash
cd ..
bash ./ws_robot_control/start_new.sh start
2.急停
bash ./ws_robot_control/start_new.sh stop


3.启动容器
docker ps -a
docker start robot_runner
docker exec -it robot_runner bash



SLAM建图

1. 在docker中：bash /root/ws_nav/src/go2_real/1_run_online_slam.sh  启动建图脚本

#这个是优化过建图的建图脚本
bash /root/ws_nav/src/go2_real/1_run_online_slam_voxel.sh

2. 手持遥控器建图

3.处理地图
docker exec -it robot_runner bash
#原项目
bash /root/ws_nav/src/go2_real/2_edit_map.sh（这个要保持1脚本开启）


#优化后项目
bash /root/ws_nav/src/go2_real/2_edit_map_voxel.sh   (这个需要把建图关闭后，再进行处理)

4.退出容器
exit

5.转换地图
## 在宿主机上执行（开发板）

cd ~/wzy/src/go2_real
# 原项目   这个地图在slam3d里面
bash 2.1_pcd2pgm.sh
#优化后项目，注意里面的地图路径   这个地图保存在工作空间下
bash /home/wheeltec/wzy/src/go2_real/2.1_pcd2pgm_voxel.sh


## 以下仅对原项目需要，voxel不需要
6. 复制地图到导航目录
mkdir -p ~/ws_nav/src/go2_real/navigation_3d/global_planner/pcd_map/pgm
cp /home/wheeltec/maps/$(ls -t /home/wheeltec/maps/ | head -1)/* ~/wzy/src/go2_real/navigation_3d/global_planner/pcd_map/pgm/

7.确认地图文件
ls -lh ~/ws_nav/src/go2_real/navigation_3d/global_planner/pcd_map/pgm/


NAV导航

接受速度逻辑

cmd节点接收导航速度
ros2 topic pub  /inspecting_state std_msgs/msg/Bool '{data: true}' -r1

cmd节点屏蔽导航速度
ros2 topic pub  /inspecting_state std_msgs/msg/Bool '{data: false}' -r1


可视化

#3_nav_start_gui.sh是可视化脚本，，需要用vnc连接狗子，
# 在主机上（wheeltec@robot-9004）执行
exit  # 如果还在容器内，先退出

# 设置显示权限
export DISPLAY=:0
xhost +local:docker

# 重启导航程序
docker exec -it robot_runner bash
export DISPLAY=:0
/root/ws_nav/src/go2_real/3_nav_start.sh


启动导航

1. 启动导航前，把狗移动到建图起点附近，且保持朝向与建图时⼀致。等待⽇志输出控制⽂本后，表示启动成功
docker exec -it robot_runner bash 
# 原项目
bash /root/ws_nav/src/go2_real/3_nav_start.sh


#这里需要安装一个pcl的包
#命令：
sudo apt update
sudo apt install pcl-tools


# 加入重定位后的项目
bash /root/ws_nav/src/go2_real/3_nav_hdl.sh




2.启动狗的运动控制
# 启动底层运控 ws_ros2
cd ./ws_ros2
source ./install/setup.bash 
bash ./ws_robot_control/start_new.sh start


# 急停(停止整个底层运动控制)
bash ./ws_robot_control/start_new.sh stop


3. 正式导航
# 发布导航点位
bash /root/ws_nav/src/tools/publish_clicked_point.sh 0 0 0
# 软急停
bash /root/ws_nav/src/tools/emergency_stop.sh


由于使用voxel，地图太大，加载太慢，因此必须降采样

pcl_voxel_grid /root/ws_nav/src/go2_real/navigation_3d/global_planner/pcd_map/pcd/voxel_free_space.pcd \
    /root/ws_nav/src/go2_real/navigation_3d/global_planner/pcd_map/pcd/voxel_free_space_downsampled.pcd \
    -leaf 0.2,0.2,0.2

# 查看大小
ls -lh /root/ws_nav/src/go2_real/navigation_3d/global_planner/pcd_map/pcd/voxel_free_space_downsampled.pcd
