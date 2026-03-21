/*** 
 * @Author: YYJ
 * @Date: 2025-04-25 11:21:59
 * @LastEditTime: 2025-06-20 16:38:18
 * @LastEditors: YYJ
 * @Description: 
 */
#ifndef COMMON_H_
#define COMMON_H_
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/common/transforms.h>
#include <gtsam/geometry/Pose3.h>
#include <sensor_msgs/PointCloud2.h>
#include <ros/ros.h>
#include <pcl_conversions/pcl_conversions.h>
struct PointXYZIRPYT
{
    PCL_ADD_POINT4D
    PCL_ADD_INTENSITY;                  // preferred way of adding a XYZ+padding
    float roll;
    float pitch;
    float yaw;
    double time;
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW   // make sure our new allocators are aligned
} EIGEN_ALIGN16;                    // enforce SSE padding for correct memory alignment

POINT_CLOUD_REGISTER_POINT_STRUCT (PointXYZIRPYT,
                                   (float, x, x) (float, y, y)
                                   (float, z, z) (float, intensity, intensity)
                                   (float, roll, roll) (float, pitch, pitch) (float, yaw, yaw)
                                   (double, time, time))

typedef PointXYZIRPYT  PointTypePose;
using PointType = pcl::PointXYZI;


struct KeyFrame {
    pcl::PointCloud<PointType>::Ptr cloud;  // cloud
    pcl::PointCloud<PointType>::Ptr cloud_ground; // filtered cloud
    pcl::PointCloud<PointType>::Ptr cloud_noground; // non-ground cloud
    gtsam::Pose3 pose;                      // pose
    uint64_t id;                            // key id
    double time;                            // time
};

template<typename T>
sensor_msgs::PointCloud2 publishCloud(const ros::Publisher& thisPub, const T& thisCloud, ros::Time thisStamp, std::string thisFrame)
{
    sensor_msgs::PointCloud2 tempCloud;
    pcl::toROSMsg(*thisCloud, tempCloud);
    tempCloud.header.stamp = thisStamp;
    tempCloud.header.frame_id = thisFrame;
    if (thisPub.getNumSubscribers() != 0)
        thisPub.publish(tempCloud);
    return tempCloud;
}

inline pcl::PointCloud<PointType>::Ptr transKeyFrameCloud(const KeyFrame &thisKeyFrame){
    pcl::PointCloud<PointType>::Ptr cloud_trans(new pcl::PointCloud<PointType>());
    Eigen::Matrix4d transform_matrix = thisKeyFrame.pose.matrix();
    
    // 将double类型的矩阵转换为float以适配PCL
    Eigen::Matrix4f transform_matrix_float = transform_matrix.cast<float>();

    pcl::transformPointCloud(*(thisKeyFrame.cloud), *cloud_trans, transform_matrix_float);
    return cloud_trans;
}
// 新的重载函数：接受点云和位姿作为独立参数
inline pcl::PointCloud<PointType>::Ptr transPointCloud(
    const pcl::PointCloud<PointType>::Ptr& cloud, 
    const gtsam::Pose3& pose) {
    
    pcl::PointCloud<PointType>::Ptr cloud_trans(new pcl::PointCloud<PointType>());
    Eigen::Matrix4d transform_matrix = pose.matrix();
    
    // 将double类型的矩阵转换为float以适配PCL
    Eigen::Matrix4f transform_matrix_float = transform_matrix.cast<float>();

    pcl::transformPointCloud(*cloud, *cloud_trans, transform_matrix_float);
    return cloud_trans;
}
inline Eigen::Isometry3d convertPose3ToIsometry3d(const gtsam::Pose3& pose) {
    // 1. 提取旋转和平移
    const gtsam::Rot3& R = pose.rotation();
    const gtsam::Point3& t = pose.translation();

    // 2. 转换为 Eigen::Matrix3d 和 Eigen::Vector3d
    Eigen::Matrix3d R_eigen = R.matrix();
    Eigen::Vector3d t_eigen(t.x(), t.y(), t.z());

    // 3. 构建 Isometry3d
    Eigen::Isometry3d iso = Eigen::Isometry3d::Identity();
    iso.linear() = R_eigen;
    iso.translation() = t_eigen;

    return iso;
}
inline Eigen::Affine3f convertPose3ToAffine3f(const gtsam::Pose3& pose) {
    // 1. 提取旋转矩阵（gtsam::Rot3 转 Eigen::Matrix3d）
    const gtsam::Rot3& R = pose.rotation();
    Eigen::Matrix3d R_double = R.matrix(); // GTSAM 默认使用 double

    // 2. 提取平移向量（gtsam::Point3 转 Eigen::Vector3d）
    const gtsam::Point3& t = pose.translation();
    Eigen::Vector3d t_double(t.x(), t.y(), t.z());

    // 3. 转换为单精度（Eigen::Matrix3f 和 Eigen::Vector3f）
    Eigen::Matrix3f R_float = R_double.cast<float>();
    Eigen::Vector3f t_float = t_double.cast<float>();

    // 4. 构建 Eigen::Affine3f
    Eigen::Affine3f affine;
    affine.linear() = R_float;      // 设置旋转部分
    affine.translation() = t_float; // 设置平移部分

    return affine;
}
inline Eigen::Affine3f pclPointToAffine3f(PointTypePose thisPoint)
{ 
    return pcl::getTransformation(thisPoint.x, thisPoint.y, thisPoint.z, thisPoint.roll, thisPoint.pitch, thisPoint.yaw);
}



#endif