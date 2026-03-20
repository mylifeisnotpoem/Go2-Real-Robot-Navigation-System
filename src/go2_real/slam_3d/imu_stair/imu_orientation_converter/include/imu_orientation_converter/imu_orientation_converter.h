#ifndef IMU_ORIENTATION_CONVERTER_H
#define IMU_ORIENTATION_CONVERTER_H

#include <ros/ros.h>
#include <sensor_msgs/Imu.h>
#include <geometry_msgs/Vector3Stamped.h>
#include <geometry_msgs/Quaternion.h>
#include <std_msgs/Float32.h>
#include <std_msgs/Bool.h>
#include <std_msgs/Int32.h>
#include <Eigen/Dense>
#include <Eigen/Geometry>
#include <vector>

class ImuOrientationConverter
{
public:
    ImuOrientationConverter();
    ~ImuOrientationConverter();

private:
    ros::NodeHandle nh_;
    ros::NodeHandle private_nh_;
    
    ros::Subscriber imu_sub_;
    ros::Publisher euler_pub_;
    ros::Publisher pitch_deviation_pub_;  // Publisher for pitch deviation
    ros::Publisher stairs_flag_pub_;  // Publisher for stairs detection flag
    ros::Publisher stairs_scene_pub_;  // Publisher for stair scene: 1 up, -1 down, 0 flat
    
    // Low-pass filter parameters
    double alpha_;  // Filter coefficient (0 < alpha < 1)
    bool first_data_;
    geometry_msgs::Vector3 filtered_euler_deg_;  // Store filtered angles in degrees
    
    // Configurable baseline and threshold parameters
    double pitch_baseline_;  // Configurable pitch baseline
    double stairs_threshold_;  // Configurable stairs detection threshold
    
    // Stairs detection state
    bool last_stairs_state_;  // Track last published stairs state
    int last_stairs_scene_;   // Track last published stairs scene
    int pending_stairs_scene_;          // Candidate scene waiting for confirmation
    int pending_stairs_scene_count_;    // Consecutive frames for candidate scene
    double stairs_enter_threshold_;     // Enter stairs threshold (degrees)
    double stairs_exit_threshold_;      // Exit stairs threshold (degrees)
    int stairs_confirm_frames_;         // Consecutive frames required to switch scene
    
    // Communication parameters
    int queue_size_;
    bool verbose_logging_;
    bool stairs_debug_mode_;  // Detailed stair-related debug logs
    
    void imuCallback(const sensor_msgs::Imu::ConstPtr& msg);
    geometry_msgs::Vector3 quaternionToEuler(const geometry_msgs::Quaternion& quat);
    geometry_msgs::Vector3 applyLowPassFilter(const geometry_msgs::Vector3& new_euler_deg);
};

#endif // IMU_ORIENTATION_CONVERTER_H
