#include "pgo/pgo.h"
PGO::PGO(ros::NodeHandle& nh, ros::NodeHandle& pnh) 
    : nh_(nh), private_nh_(pnh) {
    
    // 从参数服务器读取参数
    private_nh_.param<bool>("/seg_cloud_en/radar_en", seg_radar_en_ ,false);
    private_nh_.param<bool>("/seg_cloud_en/ground_segmentation_en", enable_ground_segmentation_ ,false);
    private_nh_.param<bool>("publish_optimized_path", publish_optimized_path_, true);
    private_nh_.param<double>("optimize_every_n_nodes", optimize_every_n_nodes_, 10);
    private_nh_.param<std::string>("/topics/odom_topic", odom_topic_, "/odom");
    private_nh_.param<std::string>("/topics/point_cloud_topic", point_cloud_topic_, "/velodyne_points");
    private_nh_.param<std::string>("/topics/radar_topic", radar_topic_, "/radar_points");

    private_nh_.param<double>("/parameters/key_frame_distance_thr", key_frame_distance_, 0.5);
    private_nh_.param<double>("/parameters/key_frame_angle_thr", key_frame_angle_, 0.1);

    private_nh_.param<bool>("/loop_closure/enable", loop_closure_enable_, false);
    private_nh_.param<double>("/loop_closure/frequency", loop_closure_frequency_, 1.0);
    
    private_nh_.param<double>("/pgo_pcd_save/all_pcd_downsample_rate", all_pcd_downsample_rate_, 0.5);
    private_nh_.param<double>("/pgo_pcd_save/free_space_downsample_rate", free_space_downsample_rate_, 0.1);

    // 初始化订阅器和发布器
    odom_sub_ = nh_.subscribe(odom_topic_, 1, &PGO::odomCallback, this);
    pointcloud_sub_ = nh_.subscribe(point_cloud_topic_, 1, &PGO::pointcloudCallback, this);
    radar_sub_ = nh_.subscribe(radar_topic_, 1, &PGO::radarCallback, this);
    
    optimized_path_pub_ = nh_.advertise<nav_msgs::Path>("/pgo/optimized_path", 1);
    optimized_odom_pub_ = nh_.advertise<nav_msgs::Odometry>("/pgo/optimized_odom", 1);
    optimized_pointcloud_pub_ = nh_.advertise<sensor_msgs::PointCloud2>("/pgo/optimized_pointcloud", 1);
    free_space_pub_ = nh_.advertise<sensor_msgs::PointCloud2>("/pgo/free_space", 1);
    
    downsize_filter_pcd_.setLeafSize(all_pcd_downsample_rate_, all_pcd_downsample_rate_, all_pcd_downsample_rate_);
    downsize_filter_free_space_.setLeafSize(free_space_downsample_rate_, free_space_downsample_rate_, free_space_downsample_rate_);
    initMemory();
    ROS_INFO("PGO node initialized!");
    ROS_INFO("key_frame_distance_thr: %f", key_frame_distance_);
    ROS_INFO("key_frame_angle_thr: %f", key_frame_angle_);
    ROS_INFO("seg_cloud_en/radar_en: %s", seg_radar_en_ ? "true" : "false");
    ROS_INFO("all_pcd_downsample_rate: %f", all_pcd_downsample_rate_);
    ROS_INFO("free_space_downsample_rate: %f", free_space_downsample_rate_);

    if(loop_closure_enable_ && initLoopDetector(loop_detector_, nh)){         // 初始化回环检测器
        loop_closure_thread_ = std::thread(&PGO::loopClosureThread, this);
        loop_closure_running_ = true;
    }
    
}

PGO::~PGO() {
    ROS_INFO("PGO node shutting down...");
    
    loop_closure_running_ = false;
    if(loop_closure_thread_.joinable()){
        loop_closure_thread_.join();
    }
    ROS_INFO("PGO node shut down.");
}

void PGO::Run() {
    while (!odom_queue_.empty() && !cloud_queue_.empty())
    {
        // 获取里程计数据 和 点云数据
        nav_msgs::Odometry cur_odom = odom_queue_.front();
        pcl::PointCloud<PointType> cur_cloud = cloud_queue_.front();
        double cur_cloud_time = cloud_time_queue_.front();
        double delta_time = cur_odom.header.stamp.toSec() - cur_cloud_time;

        if (delta_time > 0.05) {
            cloud_queue_.pop_front();
            cloud_time_queue_.pop_front();
            continue;
        } 
        else if (delta_time < -0.05) {
            odom_queue_.pop_front();
            continue;
        }
        if(!saveKeyFrames(cur_odom)){
            odom_queue_.pop_front();
            cloud_queue_.pop_front();
            cloud_time_queue_.pop_front();            
            continue;
        }
        
        // 执行图优化
        optimize();

        updateLoopEstimates();

        // 发布优化后的路径
        publishOptimizedPath();
        // 发布优化后的里程计数据
        publishOptimizedOdom();
        // 发布优化后的点云数据
        publishOptimizedPointCloud();

        odom_queue_.pop_front();
        cloud_queue_.pop_front();
        cloud_time_queue_.pop_front();
    }
}
void PGO::initMemory() {

    // 初始化ISAM2
    isam_params_.relinearizeSkip = 1;
    isam_params_.relinearizeThreshold = 0.1;
    isam_ = new gtsam::ISAM2(isam_params_);

    optimized_point_cloud_ = pcl::PointCloud<PointType>::Ptr(new pcl::PointCloud<PointType>());  
    free_space_cloud_ = pcl::PointCloud<PointType>::Ptr(new pcl::PointCloud<PointType>());
    
    tf_buffer_ptr = new tf2_ros::Buffer(ros::Duration(10.0));
    tf_listener_ptr_new = new tf2_ros::TransformListener(*tf_buffer_ptr); 
    
    patchwork_ground_seg_.reset(new PatchWorkpp<PointType>(&nh_));
}
bool PGO::initLoopDetector(std::shared_ptr<LoopDetectorInterface>& loop_detector_ptr, ros::NodeHandle& nh) {
    std::string loop_closure_method = 
        nh.param<std::string>("/loop_closure/method", " ");
    if(loop_closure_method == "scan_matching") {
        loop_detector_ptr = std::make_shared<ScanMatching>(nh);
        ROS_INFO("Using scan-matching for loop closure detection.");
        return true;
    } else if(loop_closure_method == "scan_context") {
        ROS_INFO("Using scan-context for loop closure detection.");
        return true;
    } else {
        ROS_ERROR("Invalid loop closure method: %s", loop_closure_method.c_str());
        return false;
    }
}
bool PGO::saveKeyFrames(const nav_msgs::Odometry& odom) {
    // 获取当前里程计数据
    PointType point;
    point.x = odom.pose.pose.position.x;
    point.y = odom.pose.pose.position.y;
    point.z = odom.pose.pose.position.z;

    std::lock_guard<std::mutex> lock(key_frames_mutex_);
    if(key_frames_.empty()){
        return true;
    }
    
    gtsam::Pose3 prev_pose = key_frames_.back().pose;
    gtsam::Pose3 cur_pose = odomToPose3(odom);
    // 计算当前位姿与上一个关键帧的距离和角度
    gtsam::Pose3 transBetween = prev_pose.between(cur_pose);
    float delta_x, delta_y, delta_z, delta_roll, delta_pitch, delta_yaw;
    // 提取平移分量
    delta_x = static_cast<float>(transBetween.translation().x());
    delta_y = static_cast<float>(transBetween.translation().y());
    delta_z = static_cast<float>(transBetween.translation().z());

    // 提取欧拉角（RPY顺序）
    gtsam::Vector3 rpy = transBetween.rotation().rpy();
    delta_roll  = static_cast<float>(rpy[0]);  // X轴旋转
    delta_pitch = static_cast<float>(rpy[1]);  // Y轴旋转 
    delta_yaw   = static_cast<float>(rpy[2]);  // Z轴旋转
   // 判断是否满足关键帧条件
    if (abs(delta_roll)  < key_frame_angle_ &&
        abs(delta_pitch) < key_frame_angle_ && 
        abs(delta_yaw)   < key_frame_angle_ &&
        sqrt(delta_x*delta_x + delta_y*delta_y + delta_z*delta_z) < key_frame_distance_)
        return false;
    return true;  
}



void PGO::odomCallback(const nav_msgs::OdometryConstPtr& odom_msg) {
    // 加锁
    std::lock_guard<std::mutex> lock(odom_mutex_);
    // 获取里程计数据
    nav_msgs::Odometry odom = *odom_msg;
    odom_queue_.push_back(odom);
    // 解锁
    odom_mutex_.unlock();
}

void PGO::pointcloudCallback(const sensor_msgs::PointCloud2ConstPtr &pointcloud_msg)
{
    // 加锁
    std::lock_guard<std::mutex> lock(pointcloud_mutex_);
    // 获取点云数据
    pcl::PointCloud<PointType> pointcloud;
    pcl::fromROSMsg(*pointcloud_msg, pointcloud);
    cloud_queue_.push_back(pointcloud);
    cloud_time_queue_.push_back(pointcloud_msg->header.stamp.toSec());
    // 解锁
    pointcloud_mutex_.unlock();
}
void PGO::radarCallback(const sensor_msgs::PointCloud2ConstPtr &radar_msg) {
    // 首先监听radar到base_link的转换矩阵
    if(radar_to_base_matrix_ == Eigen::Matrix4d::Identity())
        radar_to_base_matrix_ = getFrame1ToFrame2Matrix("radar", "base_link");
    std::lock_guard<std::mutex> lock(radar_mutex_);
    pcl::PointCloud<PointType>::Ptr radar_cloud(new pcl::PointCloud<PointType>);
    pcl::fromROSMsg(*radar_msg, *radar_cloud);
    // 将雷达点云从雷达坐标系转换到base_link坐标系
    for (auto& point : radar_cloud->points) {
        Eigen::Vector4d p_radar(point.x, point.y, point.z, 1.0);
        Eigen::Vector4d p_base = radar_to_base_matrix_ * p_radar;
        point.x = p_base.x();
        point.y = p_base.y();
        point.z = p_base.z();
    }
    radar_queue_.push_back(radar_cloud);
    radar_time_queue_.push_back(radar_msg->header.stamp.toSec());
    // 解锁
    radar_mutex_.unlock();
}


void PGO::addPriorFactor() {
    gtsam::noiseModel::Diagonal::shared_ptr prior_noise = 
        gtsam::noiseModel::Diagonal::Variances((gtsam::Vector(6) << 1e-2, 1e-2, M_PI*M_PI, 1e8, 1e8, 1e8).finished());;
    gtsam::PriorFactor<gtsam::Pose3> prior_factor(
        gtsam::Symbol('x', 0), 
        odomToPose3(odom_queue_.front()), 
        prior_noise);
    graph_.add(prior_factor);
    initial_estimate_.insert(gtsam::Symbol('x', 0), odomToPose3(odom_queue_.front()));
    
    std::cout << "add prior factor" << std::endl;
}

void PGO::addOdomFactor() {
    static nav_msgs::Odometry last_odom;
    
    if(key_frames_.empty()){
        // 添加先验因子
        addPriorFactor();
    } else {
        // 添加里程计因子
        gtsam::noiseModel::Diagonal::shared_ptr odom_noise = 
            gtsam::noiseModel::Diagonal::Variances((gtsam::Vector(6) << 1e-6, 1e-6, 1e-6, 1e-4, 1e-4, 1e-4).finished());
        // gtsam::Pose3 prev_pose = key_frames_.back().pose;
        gtsam::Pose3 prev_pose = odomToPose3(last_odom);
        gtsam::Pose3 cur_pose = odomToPose3(odom_queue_.front());
        vertex_count_++;
        gtsam::BetweenFactor<gtsam::Pose3> odom_factor(
            gtsam::Symbol('x', vertex_count_ - 1), 
            gtsam::Symbol('x', vertex_count_), 
            prev_pose.between(cur_pose), 
            odom_noise);
        graph_.add(odom_factor);
        // 添加初始估计
        initial_estimate_.insert(gtsam::Symbol('x', vertex_count_), cur_pose);

    }
    last_odom = odom_queue_.front();
}

void PGO::addLoopClosureFactor() {
    std::lock_guard<std::mutex> lock(loop_closure_mutex_);
    if(loop_closure_queue_.empty())
        return;
    while(!loop_closure_queue_.empty()) {
        const auto& loop_closure = loop_closure_queue_.front();
        // 添加调试信息
        std::cout << "Adding loop closure factor between frame " 
                  << loop_closure.query_id << " and " 
                  << loop_closure.candidate_id << std::endl;
        // 添加闭环因子
        gtsam::noiseModel::Diagonal::shared_ptr loop_noise = 
            gtsam::noiseModel::Diagonal::Variances(loop_closure.confidence);
        graph_.add(gtsam::BetweenFactor<gtsam::Pose3>(
            gtsam::Symbol('x', loop_closure.query_id),
            gtsam::Symbol('x', loop_closure.candidate_id),
            loop_closure.relative_pose,
            loop_noise));
            
        loop_closure_queue_.pop_front();  // 处理完后移除
    }
    loop_closure_succeeded_ = true;
}

void PGO::optimize() {
    // 添加里程计因子
    addOdomFactor();
    // 添加闭环因子
    addLoopClosureFactor(); 
    
    // 执行优化
    isam_->update(graph_, initial_estimate_);
    isam_->update();
    if(loop_closure_succeeded_ == true){
        isam_->update();
        isam_->update();
        isam_->update();
        isam_->update();
        isam_->update();
    }

    graph_.resize(0);
    initial_estimate_.clear();
    gtsam::Pose3 latest_estimate;

    isam_current_estimate_ = isam_->calculateEstimate();
    latest_estimate = isam_current_estimate_.at<gtsam::Pose3>(gtsam::Symbol('x', vertex_count_));

    pcl::PointCloud<PointType>::Ptr cur_cloud(cloud_queue_.front().makeShared());
    if(seg_radar_en_){
        if(radar_queue_.size() > 0 && radar_time_queue_.size() > 0){
            while (radar_time_queue_.front() < odom_queue_.front().header.stamp.toSec() + 0.05 && radar_time_queue_.size() > 0) {
                // 获取点云
                pcl::PointCloud<PointType>::Ptr radar_cloud = radar_queue_.front();
                double radar_time = radar_time_queue_.front();
                // 时间戳对齐
                if (radar_time < odom_queue_.front().header.stamp.toSec() - 0.05) {
                    radar_queue_.pop_front();
                    radar_time_queue_.pop_front();
                    continue;
                } else if (radar_time > odom_queue_.front().header.stamp.toSec() + 0.05) {
                    break; // 雷达点云时间戳大于当前点云时间戳，跳出循环
                }
                *cur_cloud += *radar_cloud;
                radar_queue_.pop_front();
                radar_time_queue_.pop_front();
            }
        }

    }

    KeyFrame key_frame;
    key_frame.cloud = cur_cloud;
    key_frame.pose = latest_estimate;
    key_frame.id = vertex_count_;
    key_frame.time = odom_queue_.front().header.stamp.toSec();

    // 在这里添加可行域分割模块
    if(enable_ground_segmentation_){
        performGroundSegmentation(key_frame);
    }

    std::lock_guard<std::mutex> lock(key_frames_mutex_);
    key_frames_.push_back(key_frame);
    updatePathAndCloud(key_frame);
    if(loop_closure_enable_){
        loop_detector_->addKeyFrame(key_frame); 
    }
}

void PGO::updatePathAndCloud(const KeyFrame& key_frame) {
    // 更新全局路径
    geometry_msgs::PoseStamped pose_stamped;
    pose_stamped.header.frame_id = "odom";
    pose_stamped.header.stamp = ros::Time(key_frame.time);
    pose_stamped.pose.position.x = key_frame.pose.translation().x();
    pose_stamped.pose.position.y = key_frame.pose.translation().y();
    pose_stamped.pose.position.z = key_frame.pose.translation().z();
    pose_stamped.pose.orientation = tf::createQuaternionMsgFromRollPitchYaw(
        key_frame.pose.rotation().roll(),
        key_frame.pose.rotation().pitch(),
        key_frame.pose.rotation().yaw());
    global_path_.poses.push_back(pose_stamped);

    pcl::PointCloud<PointType>::Ptr trans_cloud = transKeyFrameCloud(key_frame);
    // 降采样
    downsize_filter_pcd_.setInputCloud(trans_cloud);
    downsize_filter_pcd_.filter(*trans_cloud);
    *optimized_point_cloud_ += *trans_cloud;

    pcl::PointCloud<PointType>::Ptr free_space_cloud = transPointCloud(key_frame.cloud_ground, key_frame.pose);
    // 降采样
    downsize_filter_free_space_.setInputCloud(free_space_cloud);
    downsize_filter_free_space_.filter(*free_space_cloud);
    *free_space_cloud_ += *free_space_cloud;
}

void PGO::publishOptimizedPath() {
    if(publish_optimized_path_ && optimized_path_pub_.getNumSubscribers() > 0) {
        global_path_.header.frame_id = "odom";
        global_path_.header.stamp = ros::Time(odom_queue_.front().header.stamp.toSec());
        optimized_path_pub_.publish(global_path_);
    }   
}
void PGO::publishOptimizedOdom() {
    if(optimized_odom_pub_.getNumSubscribers() > 0) {
        std::lock_guard<std::mutex> lock(key_frames_mutex_);
        nav_msgs::Odometry optimized_odom;
        optimized_odom.header.frame_id = "odom";
        optimized_odom.header.stamp = ros::Time(key_frames_.back().time);
        optimized_odom.pose.pose.position.x = key_frames_.back().pose.translation().x();
        optimized_odom.pose.pose.position.y = key_frames_.back().pose.translation().y();
        optimized_odom.pose.pose.position.z = key_frames_.back().pose.translation().z();
        optimized_odom.pose.pose.orientation = tf::createQuaternionMsgFromRollPitchYaw(
            key_frames_.back().pose.rotation().roll(),
            key_frames_.back().pose.rotation().pitch(),
            key_frames_.back().pose.rotation().yaw());
        optimized_odom_pub_.publish(optimized_odom);
    }
}
void PGO::publishOptimizedPointCloud() {
    // 发布优化后的点云数据
    if(optimized_pointcloud_pub_.getNumSubscribers() > 0) {
        if(optimized_point_cloud_->points.size() > 0){
            sensor_msgs::PointCloud2 optimized_pointcloud_msg;
            pcl::toROSMsg(*optimized_point_cloud_, optimized_pointcloud_msg);
            optimized_pointcloud_msg.header.frame_id = "odom";
            optimized_pointcloud_msg.header.stamp = ros::Time(odom_queue_.front().header.stamp.toSec());
            optimized_pointcloud_pub_.publish(optimized_pointcloud_msg);
        }
    }
    if(free_space_pub_.getNumSubscribers() > 0) {
        if(free_space_cloud_->points.size() > 0){
            sensor_msgs::PointCloud2 free_space_msg;
            pcl::toROSMsg(*free_space_cloud_, free_space_msg);
            free_space_msg.header.frame_id = "odom";
            free_space_msg.header.stamp = ros::Time(odom_queue_.front().header.stamp.toSec());
            free_space_pub_.publish(free_space_msg);
        }

    }

}
gtsam::Pose3 PGO::odomToPose3(const nav_msgs::Odometry& odom) {
    gtsam::Rot3 rotation = gtsam::Rot3::Quaternion(
        odom.pose.pose.orientation.w,
        odom.pose.pose.orientation.x,
        odom.pose.pose.orientation.y,
        odom.pose.pose.orientation.z);
        
    gtsam::Point3 position(
        odom.pose.pose.position.x,
        odom.pose.pose.position.y,
        odom.pose.pose.position.z);
        
    return gtsam::Pose3(rotation, position);
}

gtsam::Pose3 PGO::keyFrameToPose3(const PointTypePose& key_frame) {
    gtsam::Rot3 rotation = gtsam::Rot3::RzRyRx(key_frame.roll, key_frame.pitch, key_frame.yaw);
    gtsam::Point3 position(key_frame.x, key_frame.y, key_frame.z);   
    return gtsam::Pose3(rotation, position);
}


void PGO::loopClosureThread() {
    ros::Rate rate(loop_closure_frequency_);
    while(ros::ok()) {
        rate.sleep();
        performLoopClosure();
    }
}
void PGO::performLoopClosure() {
    // 回环检测
    static uint64_t keyframe_id = -1;
    LoopClosure loop_closure_result;
    KeyFrame last_key_frame;
    {
        std::lock_guard<std::mutex> lock(key_frames_mutex_);
        if(key_frames_.empty() || key_frames_.back().id == keyframe_id) {
            return;
        }
        last_key_frame = key_frames_.back();
        keyframe_id = last_key_frame.id;
    }
    // 添加关键帧到回环检测器
    if(loop_detector_->detect(last_key_frame, loop_closure_result)) {
        if(isLoopClosured(loop_closure_result)){
            std::lock_guard<std::mutex> lock(loop_closure_mutex_);
            loop_closure_queue_.push_back(loop_closure_result);
        }
    }
}
bool PGO::isLoopClosured(LoopClosure &loop_closure) {
    // 检测回环
    if(!loop_closure.detected) {
        return false;
    }
    for(auto& loop : loop_closure_queue_) {
        if(loop_closure.query_id == loop.query_id ||
            loop_closure.query_id == loop.candidate_id ||
            loop_closure.candidate_id == loop.candidate_id ||
            loop_closure.candidate_id == loop.query_id) {
            return false;
        }
    }
    return true;
}
void PGO::updateLoopEstimates(){
    if(key_frames_.empty()){
        return;
    }

    if(loop_closure_succeeded_ == true){
        global_path_.poses.clear();
        optimized_point_cloud_->points.clear();
        free_space_cloud_->points.clear();
        int num = isam_current_estimate_.size();
        for(int i = 0; i < num; i++){
            key_frames_[i].pose = isam_current_estimate_.at<gtsam::Pose3>(gtsam::Symbol('x', i));
            updatePathAndCloud(key_frames_[i]);
        }
        loop_detector_->updateKeyFrame(key_frames_);
        loop_closure_succeeded_ = false;
    }
}

void PGO::performGroundSegmentation(KeyFrame& key_frame){
    // 可行域分割
    if(patchwork_ground_seg_){
        double time_taken = 0.0;
        pcl::PointCloud<PointType> pc_ground;
        pcl::PointCloud<PointType> pc_non_ground;
        patchwork_ground_seg_->estimate_ground(*key_frame.cloud, 
                                                pc_ground, pc_non_ground, 
                                                time_taken);
        float height_threshold = 0.1; // 设置高度阈值（单位：米）
        pcl::PointCloud<PointType> pc_ground_filtered;
        pcl::PointCloud<PointType> elevated_points;
        // 遍历原始地面点云
        for (const auto& pt : pc_ground.points) {
            if (pt.z > height_threshold) {
                elevated_points.push_back(pt);  // 保存需要移除的高点
            } else {
                pc_ground_filtered.push_back(pt);  // 保留有效地面点
            }
        }
        // 更新地面点云（过滤后）
        pc_ground.swap(pc_ground_filtered);
        // 将高点合并到非地面点云
        pc_non_ground += elevated_points;

        // 将点云存到key_frame中
        key_frame.cloud_ground = pc_ground.makeShared();
        key_frame.cloud_noground = pc_non_ground.makeShared();

    } else {
        ROS_WARN("PatchWorkpp not initialized, skipping ground segmentation.");
    }
}

void PGO::saveOptimizedCloud() {
    // 保存优化后的点云数据
    if(key_frames_.empty()){
        return;
    }
    ROS_INFO("Saving optimized point cloud...");
    optimized_point_cloud_->points.clear();
    for(auto& key_frame : key_frames_) {
        pcl::PointCloud<PointType>::Ptr trans_cloud = transKeyFrameCloud(key_frame);
        *optimized_point_cloud_ += *trans_cloud;

        // 可行域
        pcl::PointCloud<PointType>::Ptr trans_cloud_ground = transPointCloud(key_frame.cloud_ground, key_frame.pose);
        *free_space_cloud_ += *trans_cloud_ground;
    }
    if(optimized_point_cloud_->points.empty()){
        ROS_WARN("No points in optimized point cloud.");
        return;
    }
    std::string all_cloud_file_name = "aft_pgo_cloud.pcd";
    std::string free_space_file_name = "aft_pgo_free_space.pcd";
    std::string all_points_dir(std::string(std::string(ROOT_DIR) + "PCD/") + all_cloud_file_name);
    std::string free_space_dir(std::string(std::string(ROOT_DIR) + "PCD/") + free_space_file_name);
    // 保存点云
    pcl::PCDWriter pcd_writer;
    pcd_writer.writeBinary(all_points_dir, *optimized_point_cloud_);
    pcd_writer.writeBinary(free_space_dir, *free_space_cloud_);
    std::cout << "Saved optimized point cloud to " << all_points_dir << std::endl;
    std::cout << "Saved optimized free  space  to " << free_space_dir << std::endl;

}
Eigen::Matrix4d PGO::getFrame1ToFrame2Matrix(std::string const& frame_id1,
                                    std::string const& frame_id2) {
    Eigen::Matrix4d frame1_to_frame2_matrix = Eigen::Matrix4d::Identity();
    try {
        // 获取最新的变换（从radar到base_link）
        geometry_msgs::TransformStamped transform = 
        tf_buffer_ptr->lookupTransform(frame_id2, frame_id1, ros::Time(0), ros::Duration(1.0));
        // 提取平移向量
        Eigen::Vector3d translation(
            transform.transform.translation.x,
            transform.transform.translation.y,
            transform.transform.translation.z
        );
        
        // 提取旋转四元数并归一化
        Eigen::Quaterniond rotation(
            transform.transform.rotation.w,
            transform.transform.rotation.x,
            transform.transform.rotation.y,
            transform.transform.rotation.z
        );
        rotation.normalize();
        
        // 构造等距变换矩阵（刚体变换）
        Eigen::Isometry3d tf_matrix = Eigen::Isometry3d::Identity();
        tf_matrix.linear() = rotation.toRotationMatrix(); // 设置旋转部分
        tf_matrix.translation() = translation;            // 设置平移部分
        
        // 转换为4x4矩阵
        frame1_to_frame2_matrix = tf_matrix.matrix();  
        
        // 输出变换矩阵
        std::cout << "frame_id1 to frame_id2 transform matrix:\n" << frame1_to_frame2_matrix << std::endl;
    } catch (tf2::TransformException &ex) {
        ROS_WARN("Could not get transform from radar to base_link: %s", ex.what());
    }
    return frame1_to_frame2_matrix;
}