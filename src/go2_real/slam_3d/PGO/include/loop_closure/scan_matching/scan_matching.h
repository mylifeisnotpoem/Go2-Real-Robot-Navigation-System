/*** 
 * @Author: YYJ
 * @Date: 2025-04-27 19:23:23
 * @LastEditTime: 2025-05-20 14:00:20
 * @LastEditors: YYJ
 * @Description: 
 */
#ifndef SCAN_MATCHING_H_
#define SCAN_MATCHING_H_
#include "utility/common.h"
#include "loop_closure/loop_closure_interface.hpp"
#include <pcl/registration/icp.h>
#include <pcl/registration/icp_nl.h>
#include <pcl/registration/ndt.h>
#include <ros/ros.h>

class ScanMatching : public LoopDetectorInterface {
public:
    ScanMatching() = default;
    ScanMatching(ros::NodeHandle& nh);
    ~ScanMatching();

    bool init(const ros::NodeHandle& nh) override;
    bool detect(const KeyFrame& keyframe, LoopClosure& result) override;
    void addKeyFrame(const KeyFrame& frame) override;
    void updateKeyFrame(const std::vector<KeyFrame> frames) override;

    bool detectLoopClosureDistance(const KeyFrame& keyframe, int *closestID);
    void loopFindNearKeyframes(pcl::PointCloud<PointType>::Ptr& nearKeyframes, 
                                const int& key, const int& searchNum);

private:
    ros::Publisher pub_current_keyframe;  // 发布当前点云
    ros::Publisher pub_history_keyframes; // 发布历史关键帧

    int surrounding_keyframe_size_{0};              // 周围关键帧数量
    double history_keyframe_search_radius_{0.0};    // 历史关键帧搜索半径
    double history_keyframe_search_difftime_{0.0};  // 历史关键帧搜索时间差
    int history_keyframe_search_num_{10};         // 历史关键帧搜索数量
    double min_score_{0.3};                         // 最小得分

    double downsample_rate_{0.3};                   // 下采样率

    // icp参数
    pcl::IterativeClosestPoint<PointType, PointType> icp;
    int max_iterations_{100};
    double max_correspondence_distance_{0.0};
    double transformation_epsilon_{1e-5};
    double euclidean_fitness_epsilon_{1e-6};
    int ransac_iterations_{10};

    pcl::PointCloud<PointType>::Ptr local_map_;         // 候选关键帧组成的局部地图
    pcl::PointCloud<PointType>::Ptr cloud_key_poses3D;  // 存储关键帧位置信息  为了快速搜索
    std::vector<KeyFrame> scan_matching_key_frames_;    // 关键帧队列

    pcl::KdTreeFLANN<PointType>::Ptr kdtree_history_key_pos_; 
    pcl::VoxelGrid<PointType> downsize_filter_ICP;
};

#endif // SCAN_MATCHING_H_