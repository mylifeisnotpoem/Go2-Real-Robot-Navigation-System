/*** 
 * @Author: YYJ
 * @Date: 2025-04-22 16:04:50
 * @LastEditTime: 2025-06-20 17:36:18
 * @LastEditors: YYJ
 * @Description: 
 */

 #ifndef PGO_PGO_H_
 #define PGO_PGO_H_

#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <nav_msgs/Path.h>
#include <sensor_msgs/PointCloud2.h>
#include <geometry_msgs/PoseStamped.h>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl_conversions/pcl_conversions.h>
#include <Eigen/Dense>
#include <tf/tf.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>
#include <gtsam/geometry/Rot3.h>
#include <gtsam/geometry/Pose3.h>
#include <gtsam/slam/PriorFactor.h>
#include <gtsam/slam/BetweenFactor.h>
#include <gtsam/navigation/GPSFactor.h>
#include <gtsam/navigation/ImuFactor.h>
#include <gtsam/navigation/CombinedImuFactor.h>
#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <gtsam/nonlinear/LevenbergMarquardtOptimizer.h>
#include <gtsam/nonlinear/Marginals.h>
#include <gtsam/nonlinear/Values.h>
#include <gtsam/inference/Symbol.h>
#include <gtsam/nonlinear/ISAM2.h>
#include <gtsam/geometry/OrientedPlane3.h>
#include <gtsam/slam/OrientedPlane3Factor.h>
#include <gtsam/nonlinear/ExpressionFactor.h>
#include <gtsam/nonlinear/AdaptAutoDiff.h>
#include <gtsam/3rdparty/ceres/example.h>

#include <deque>
#include <mutex>
#include <thread>
#include "utility/common.h"
#include "utility/tic_toc.h"
#include "loop_closure/loop_closure_interface.hpp"
#include "loop_closure/scan_matching/scan_matching.h"
#include "ground_seg/patchworkpp/patchworkpp.hpp"

class PGO {
public:
    PGO(ros::NodeHandle& nh, ros::NodeHandle& pnh);
    ~PGO();

    void Run();     // 主循环函数
    // 保存数据
    void saveOptimizedCloud();          // 保存优化后的点云数据

private:
    // ROS相关
    ros::NodeHandle nh_;
    ros::NodeHandle private_nh_;
    ros::Subscriber odom_sub_;          // 订阅odom
    ros::Subscriber pointcloud_sub_;    // 订阅点云
    ros::Subscriber radar_sub_;         // 订阅雷达点云（宇树L1）

    ros::Publisher optimized_path_pub_; // 发布优化后的路径
    ros::Publisher optimized_odom_pub_; // 发布优化后的里程计数据
    ros::Publisher optimized_pointcloud_pub_; // 发布优化后的点云数据

    ros::Publisher free_space_pub_;     // 发布可行域点云

    // 地面分割发布器
    ros::Publisher ground_cloud_pub_;
    ros::Publisher nonground_cloud_pub_;

    std::shared_ptr<PatchWorkpp<PointType>> patchwork_ground_seg_;
    bool enable_ground_segmentation_{false};

    // 参数
    bool publish_optimized_path_{true};
    double optimize_every_n_nodes_{10};
    std::string odom_topic_{"/odom"};
    std::string point_cloud_topic_{"/velodyne_points"};
    std::string radar_topic_{"/radar_points"};  // 雷达点云话题

    double key_frame_distance_{0.5}; // 关键帧距离
    double key_frame_angle_{0.1};    // 关键帧角度

    bool seg_radar_en_{false};  // 是否对雷达点云进行地面分割

    double all_pcd_downsample_rate_{0.1}; // 全局点云下采样率
    double free_space_downsample_rate_{0.1}; // 可行域点云下采样率

    tf2_ros::Buffer* tf_buffer_ptr = nullptr;
    tf2_ros::TransformListener* tf_listener_ptr_new = nullptr;
    Eigen::Matrix4d radar_to_base_matrix_ = Eigen::Matrix4d::Identity(); // radar雷达转到cloud坐标系的矩阵


    // GTSAM相关
    gtsam::NonlinearFactorGraph graph_;     // 因子图
    gtsam::Values initial_estimate_;        // 初始估计
    gtsam::Values optimized_estimate_;      // 优化后的估计
    gtsam::ISAM2 *isam_;                    // ISAM2对象
    gtsam::ISAM2Params isam_params_;        // ISAM2参数
    gtsam::Values isam_current_estimate_;   // 当前估计
    
    // 数据缓存
    std::deque<nav_msgs::Odometry> odom_queue_;                 // 里程计队列

    std::deque<pcl::PointCloud<PointType>> cloud_queue_;        // 点云队列
    std::deque<double> cloud_time_queue_;                       // 点云时间队列

    std::deque<pcl::PointCloud<PointType>::Ptr> radar_queue_;   // 雷达点云队列
    std::deque<double> radar_time_queue_;                       // 雷达点云时间队列

    std::mutex odom_mutex_;                                     // 里程计锁
    std::mutex pointcloud_mutex_;                               // 点云锁
    std::mutex radar_mutex_;                                    // 雷达点云锁

    std::vector<KeyFrame> key_frames_;                          // 关键帧队列
    std::mutex key_frames_mutex_;                                // 关键帧锁
    nav_msgs::Path global_path_;                                // 全局路径
    
    int vertex_count_{0};                                       // 顶点数量
    std::vector<gtsam::Pose3> pose_estimates_;                  // 估计的位姿
    
    pcl::PointCloud<PointType>::Ptr optimized_point_cloud_;     // 局部地图
    pcl::PointCloud<PointType>::Ptr free_space_cloud_;          // 可行域点云
    pcl::VoxelGrid<PointType> downsize_filter_pcd_;                 // 下采样滤波器
    pcl::VoxelGrid<PointType> downsize_filter_free_space_;                 // 可行域点云下采样滤波器
    // 回环检测
    bool loop_closure_enable_{false};
    std::thread loop_closure_thread_;                           // 回环检测线程
    std::atomic<bool> loop_closure_running_;
    std::mutex loop_closure_mutex_;
    std::shared_ptr<LoopDetectorInterface> loop_detector_;      // 回环检测器
    std::deque<LoopClosure> loop_closure_queue_;                // 回环检测结果队列
    double loop_closure_frequency_{1.0};                        // 回环检测频率 
    bool loop_closure_succeeded_{false};                        // 回环检测是否成功


    // 回调函数
    void odomCallback(const nav_msgs::OdometryConstPtr& odom_msg);  // 里程计回调函数
    void pointcloudCallback(const sensor_msgs::PointCloud2ConstPtr& pointcloud_msg); // 点云回调函数
    void radarCallback(const sensor_msgs::PointCloud2ConstPtr& radar_msg); // 雷达点云回调函数
    
    void initMemory(); // 初始化内存
    bool saveKeyFrames(const nav_msgs::Odometry& odom);
    void updatePathAndCloud(const KeyFrame& key_frame); // 更新路径 和 点云
    void updateLoopEstimates();

    bool initLoopDetector(std::shared_ptr<LoopDetectorInterface>& loop_detector_ptr, ros::NodeHandle& nh);

    // 优化相关函数
    void addPriorFactor();
    void addOdomFactor();
    void addLoopClosureFactor();
    void optimize();
    // 发布数据
    void publishOptimizedPath();        // 发布优化后的路径
    void publishOptimizedOdom();        // 发布优化后的里程计数据
    void publishOptimizedPointCloud();  // 发布优化后的点云数据


    // 回环检测相关函数
    void loopClosureThread();
    void performLoopClosure();
    bool isLoopClosured(LoopClosure &loop_closure);


    // 可行域分割相关函数
    void performGroundSegmentation(KeyFrame& key_frame);

    // 工具函数
    gtsam::Pose3 odomToPose3(const nav_msgs::Odometry& odom);
    gtsam::Pose3 keyFrameToPose3(const PointTypePose& key_frame);
    Eigen::Matrix4d getFrame1ToFrame2Matrix(std::string const& frame_id1,
                                    std::string const& frame_id2);
    
};

#endif  // PGO_PGO_HPP_