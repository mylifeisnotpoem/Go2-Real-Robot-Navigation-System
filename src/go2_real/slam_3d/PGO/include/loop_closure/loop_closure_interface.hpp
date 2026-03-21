/*** 
 * @Author: YYJ
 * @Date: 2025-04-26 16:20:52
 * @LastEditTime: 2025-05-19 15:28:24
 * @LastEditors: YYJ
 * @Description: 
 */
#pragma once
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <ros/ros.h>
#include "utility/common.h"

// 回环检测结果结构体
struct LoopClosure {
    bool detected;                      // 是否检测到回环
    uint64_t query_id;                  // 当前关键帧ID
    uint64_t candidate_id;              // 候选关键帧ID
    gtsam::Pose3 relative_pose;         // 相对位姿(T_query^candidate)
    gtsam::Vector confidence;                   // 置信度得分
};

class LoopDetectorInterface {
public:
    virtual ~LoopDetectorInterface() = default;
    
    // 初始化检测器
    virtual bool init(const ros::NodeHandle& nh) = 0;
    
    // 检测回环
    virtual bool detect(const KeyFrame& keyframe, LoopClosure& result) = 0;
        
    // 添加关键帧到数据库
    virtual void addKeyFrame(const KeyFrame& frame) = 0;

    // 更新关键帧
    virtual void updateKeyFrame(const std::vector<KeyFrame> frames) = 0;
};