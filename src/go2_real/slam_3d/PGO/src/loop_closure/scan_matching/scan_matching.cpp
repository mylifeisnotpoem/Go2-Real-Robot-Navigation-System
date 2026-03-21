/*** 
 * @Author: YYJ
 * @Date: 2025-04-27 19:30:52
 * @LastEditTime: 2025-05-20 15:41:58
 * @LastEditors: YYJ
 * @Description: 使用LIO-SAM中的scan to map的ICP方式进行回环检测
 */
#include "loop_closure/scan_matching/scan_matching.h"
#include <sensor_msgs/PointCloud2.h>
ScanMatching::ScanMatching(ros::NodeHandle& nh) {
    // 初始化参数
    init(nh);
    pub_current_keyframe = nh.advertise<sensor_msgs::PointCloud2>("/pgo/current_keyframe", 10000);
    pub_history_keyframes = nh.advertise<sensor_msgs::PointCloud2>("/pgo/history_keyframes", 100000);
}
ScanMatching::~ScanMatching(){
    
}
bool ScanMatching::init(const ros::NodeHandle& nh) {
    // 初始化参数
    nh.param("/loop_closure/scan_matching/surrounding_keyframe_size", surrounding_keyframe_size_, 10);
    nh.param("/loop_closure/scan_matching/history_keyframe_search_radius", history_keyframe_search_radius_, 5.0);
    nh.param("/loop_closure/scan_matching/history_keyframe_search_difftime", history_keyframe_search_difftime_, 5.0);
    nh.param("/loop_closure/scan_matching/history_keyframe_search_num", history_keyframe_search_num_, 10);
    nh.param("/loop_closure/scan_matching/min_score", min_score_, 0.3);
    nh.param("/loop_closure/downsample_rate", downsample_rate_, 0.3);
    ROS_INFO("ScanMatching initialized!");
    ROS_INFO("surrounding_keyframe_size: %d", surrounding_keyframe_size_);
    ROS_INFO("history_keyframe_search_radius: %f", history_keyframe_search_radius_);
    ROS_INFO("history_keyframe_search_difftime: %f", history_keyframe_search_difftime_);
    ROS_INFO("history_keyframe_search_num: %d", history_keyframe_search_num_);
    ROS_INFO("min_score: %f", min_score_);
    ROS_INFO("downsample_rate: %f", downsample_rate_);

    // 设置ICP参数
    icp.setMaxCorrespondenceDistance(history_keyframe_search_radius_ * 2);
    icp.setMaximumIterations(max_iterations_);                      // 迭代次数 
    icp.setTransformationEpsilon(transformation_epsilon_);          // 优化精度 变换矩阵的变化量
    icp.setEuclideanFitnessEpsilon(euclidean_fitness_epsilon_);     // 优化精度 配准误差的变化量​​
    icp.setRANSACIterations(ransac_iterations_);

    // 设置下采样参数
    downsize_filter_ICP.setLeafSize(downsample_rate_, downsample_rate_, downsample_rate_);

    local_map_.reset(new pcl::PointCloud<PointType>());
    cloud_key_poses3D.reset(new pcl::PointCloud<PointType>());
    kdtree_history_key_pos_.reset(new pcl::KdTreeFLANN<PointType>());
    return true;
}

bool ScanMatching::detect(const KeyFrame& keyframe, LoopClosure& result) {
    if (keyframe.cloud == nullptr) {
        ROS_ERROR("Keyframe cloud is null!");
        return false;
    }
    if (keyframe.id == 0) {     // 第一帧不进行回环检测
        return false;
    }
    // find keys
    int loop_key_cur = keyframe.id;
    int loop_key_pre;

    if(detectLoopClosureDistance(keyframe, &loop_key_pre) == false){
        return false;
    }
    // extract cloud
    pcl::PointCloud<PointType>::Ptr cur_keyframe_cloud(new pcl::PointCloud<PointType>());
    pcl::PointCloud<PointType>::Ptr pre_keyframs_cloud(new pcl::PointCloud<PointType>());
    *cur_keyframe_cloud = *transKeyFrameCloud(keyframe);    // 当前帧点云(odom系)
    loopFindNearKeyframes(pre_keyframs_cloud, loop_key_pre, history_keyframe_search_num_); // 提取历史关键帧点云(odom系)

    icp.setInputSource(cur_keyframe_cloud);
    icp.setInputTarget(pre_keyframs_cloud);
    pcl::PointCloud<PointType>::Ptr unused_result(new pcl::PointCloud<PointType>());
    icp.align(*unused_result);
    if (icp.hasConverged() == false || icp.getFitnessScore() > min_score_)
        return false;
    
    if(pub_current_keyframe.getNumSubscribers() != 0){
        pcl::PointCloud<PointType>::Ptr closed_cloud(new pcl::PointCloud<PointType>());
        pcl::transformPointCloud(*cur_keyframe_cloud, *closed_cloud, icp.getFinalTransformation());
        publishCloud(pub_current_keyframe, closed_cloud, ros::Time(keyframe.time), "odom");
    }
    if(pub_history_keyframes.getNumSubscribers() != 0){
        pcl::PointCloud<PointType>::Ptr history_cloud(new pcl::PointCloud<PointType>());
        *history_cloud = *pre_keyframs_cloud;
        publishCloud(pub_history_keyframes, history_cloud, ros::Time(keyframe.time), "odom");
    }
        
    std::cout << "LoopClosure succeed!  icp score: " << icp.getFitnessScore() << std::endl;
    Eigen::Affine3f correctionLidarFrame;
    float x, y, z, roll, pitch, yaw;
    correctionLidarFrame = icp.getFinalTransformation(); 
    Eigen::Affine3f t_wrong = convertPose3ToAffine3f(scan_matching_key_frames_[loop_key_cur].pose);

    Eigen::Affine3f t_correct = correctionLidarFrame * t_wrong;
    pcl::getTranslationAndEulerAngles(t_correct, x, y, z, roll, pitch, yaw);
    gtsam::Pose3 pose_from = gtsam::Pose3(gtsam::Rot3::RzRyRx(roll, pitch, yaw), gtsam::Point3(x, y, z));
    gtsam::Pose3 pose_to = scan_matching_key_frames_[loop_key_pre].pose;

    gtsam::Vector noise_confidence6(6);
    float noise_score = icp.getFitnessScore();
    noise_confidence6 << noise_score, noise_score, noise_score, noise_score, noise_score, noise_score;

    result.detected = true;
    result.query_id = loop_key_cur;
    result.candidate_id = loop_key_pre;
    result.relative_pose = pose_from.between(pose_to);
    result.confidence = noise_confidence6;

    return true;
}

void ScanMatching::addKeyFrame(const KeyFrame& frame) {   
    // 存储关键帧数据
    scan_matching_key_frames_.push_back(frame); 
    // 存储关键帧位置信息
    PointType point;
    point.x = frame.pose.translation().x();
    point.y = frame.pose.translation().y();
    point.z = frame.pose.translation().z();
    point.intensity = frame.id;
    cloud_key_poses3D->points.push_back(point);
    return;
}
void ScanMatching::updateKeyFrame(const std::vector<KeyFrame> frames) { 
    for (size_t i = 0; i < scan_matching_key_frames_.size(); i++) {
        scan_matching_key_frames_[i].pose = frames[i].pose;
    }    
}

bool ScanMatching::detectLoopClosureDistance(const KeyFrame& keyframe, int *closestID){
    int key_pre = -1;

    // 构建kd-tree搜索距离当前关键帧最近的关键帧
    std::vector<int> point_search_idx;
    std::vector<float> point_search_squared_distance;
    kdtree_history_key_pos_->setInputCloud(cloud_key_poses3D);
    PointType last_pos;
    last_pos.x = keyframe.pose.translation().x();
    last_pos.y = keyframe.pose.translation().y();
    last_pos.z = keyframe.pose.translation().z();
    kdtree_history_key_pos_->radiusSearch(last_pos, history_keyframe_search_radius_, point_search_idx, point_search_squared_distance);
    
    for(size_t i = 0; i < point_search_idx.size(); i++){
        int id = point_search_idx[i];
        if(abs(scan_matching_key_frames_[id].time - keyframe.time) > history_keyframe_search_difftime_){
            key_pre = id;
            break;
        }
    }
    if(key_pre == -1)
        return false;
    *closestID = key_pre;
    return true;
}
void ScanMatching::loopFindNearKeyframes(pcl::PointCloud<PointType>::Ptr& nearKeyframes, 
                                            const int& key, const int& searchNum){
    nearKeyframes->clear();
    int all_frame_num = scan_matching_key_frames_.size();
    // std::cout << "combined keyframe: " ;
    for(int i = -searchNum; i <= searchNum; ++i){
        int key_near = key + i;
        if(key_near < 0 || key_near >= all_frame_num){
            continue;
        }
        *nearKeyframes += *(transKeyFrameCloud(scan_matching_key_frames_[key_near]));
        // std::cout << "[" << key_near << "]" ;
    }
    std::cout << std::endl;
    if(nearKeyframes->size() == 0){
        return;
    }
    // 下采样
    pcl::PointCloud<PointType>::Ptr downsampled_cloud(new pcl::PointCloud<PointType>());
    downsize_filter_ICP.setInputCloud(nearKeyframes);
    downsize_filter_ICP.filter(*downsampled_cloud);
    // 更新
    nearKeyframes->clear();
    *nearKeyframes = *downsampled_cloud;
}

