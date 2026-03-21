#include "imu_orientation_converter/imu_orientation_converter.h"
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

ImuOrientationConverter::ImuOrientationConverter()
    : nh_(), private_nh_("~"), first_data_(true),
      last_stairs_state_(false), last_stairs_scene_(0),
      pending_stairs_scene_(0), pending_stairs_scene_count_(0)
{
    // Get parameters from parameter server
    private_nh_.param("alpha", alpha_, 0.1);  // Default alpha = 0.1 (stronger filtering)
    private_nh_.param("queue_size", queue_size_, 10);  // Default queue size = 10
    private_nh_.param("verbose_logging", verbose_logging_, true);  // Default verbose logging = true
    private_nh_.param("stairs_debug_mode", stairs_debug_mode_, true);  // Default stair debug = false
    private_nh_.param("pitch_baseline", pitch_baseline_, 20.0);  // Default pitch baseline = 20.0 degrees
    private_nh_.param("stairs_threshold", stairs_threshold_, 14.0);  // Default stairs threshold = 14.0 degrees
    private_nh_.param("stairs_enter_threshold", stairs_enter_threshold_, stairs_threshold_);
    private_nh_.param("stairs_exit_threshold", stairs_exit_threshold_, stairs_threshold_ * 0.7);
    private_nh_.param("stairs_confirm_frames", stairs_confirm_frames_, 3);

    if (stairs_confirm_frames_ < 1) stairs_confirm_frames_ = 1;
    if (stairs_exit_threshold_ > stairs_enter_threshold_) stairs_exit_threshold_ = stairs_enter_threshold_;
    
    // Subscribe to IMU data
    imu_sub_ = nh_.subscribe("/imu/data", queue_size_, &ImuOrientationConverter::imuCallback, this);
    
    // Publish Euler angles as Vector3Stamped (x=roll, y=pitch, z=yaw) in degrees
    euler_pub_ = nh_.advertise<geometry_msgs::Vector3Stamped>("/imu/data_orientation", queue_size_);
    
    // Publish pitch deviation as Float32
    pitch_deviation_pub_ = nh_.advertise<std_msgs::Float32>("/imu/pitch_deviation", queue_size_);
    
    // Publish stairs detection flag as Bool
    stairs_flag_pub_ = nh_.advertise<std_msgs::Bool>("/stairs_flag", queue_size_);
    // Publish stair scene as Int32: 1 up, -1 down, 0 flat
    stairs_scene_pub_ = nh_.advertise<std_msgs::Int32>("/stairs_scene", queue_size_);
    
    // Initialize filtered angles
    filtered_euler_deg_.x = 0.0;
    filtered_euler_deg_.y = 0.0;
    filtered_euler_deg_.z = 0.0;
    
    ROS_INFO("IMU Orientation Converter started. Subscribing to /imu/data and publishing to /imu/data_orientation");
    ROS_INFO("Parameters: alpha=%.3f, queue_size=%d, verbose=%s", 
             alpha_, queue_size_, verbose_logging_ ? "true" : "false");
    ROS_INFO("Stair debug mode: %s", stairs_debug_mode_ ? "true" : "false");
    ROS_INFO("Pitch baseline: %.1f degrees, Stairs threshold: %.1f degrees",
             pitch_baseline_, stairs_threshold_);
    ROS_INFO("Stairs hysteresis: enter=%.1f°, exit=%.1f°, confirm_frames=%d",
             stairs_enter_threshold_, stairs_exit_threshold_, stairs_confirm_frames_);
}

ImuOrientationConverter::~ImuOrientationConverter()
{
}

void ImuOrientationConverter::imuCallback(const sensor_msgs::Imu::ConstPtr& msg)
{
    // Convert quaternion to Euler angles (in radians)
    geometry_msgs::Vector3 euler_angles_rad = quaternionToEuler(msg->orientation);
    
    // Convert to degrees
    geometry_msgs::Vector3 euler_angles_deg;
    euler_angles_deg.x = euler_angles_rad.x * 180.0 / M_PI;  // roll
    euler_angles_deg.y = euler_angles_rad.y * 180.0 / M_PI;  // pitch
    euler_angles_deg.z = euler_angles_rad.z * 180.0 / M_PI;  // yaw
    
    // Apply low-pass filter
    geometry_msgs::Vector3 filtered_euler_deg = applyLowPassFilter(euler_angles_deg);
    
    // Calculate and publish signed pitch deviation.
    // Sign convention after this line:
    // positive => up stairs, negative => down stairs.
    double pitch_deviation = pitch_baseline_ - filtered_euler_deg.y;
    
    std_msgs::Float32 deviation_msg;
    deviation_msg.data = static_cast<float>(pitch_deviation);
    pitch_deviation_pub_.publish(deviation_msg);
    
    // Stair scene candidate with hysteresis:
    // 1 = up stairs, -1 = down stairs, 0 = flat.
    double signed_pitch_delta = pitch_deviation;
    int candidate_stairs_scene = 0;
    if (last_stairs_scene_ == 1) {
        candidate_stairs_scene = (signed_pitch_delta > stairs_exit_threshold_) ? 1 : 0;
    } else if (last_stairs_scene_ == -1) {
        candidate_stairs_scene = (signed_pitch_delta < -stairs_exit_threshold_) ? -1 : 0;
    } else {
        if (signed_pitch_delta > stairs_enter_threshold_) {
            candidate_stairs_scene = 1;
        } else if (signed_pitch_delta < -stairs_enter_threshold_) {
            candidate_stairs_scene = -1;
        }
    }

    // Confirm scene switch with consecutive frames.
    if (candidate_stairs_scene != last_stairs_scene_) {
        if (pending_stairs_scene_ != candidate_stairs_scene) {
            pending_stairs_scene_ = candidate_stairs_scene;
            pending_stairs_scene_count_ = 1;
        } else {
            pending_stairs_scene_count_++;
        }

        if (pending_stairs_scene_count_ >= stairs_confirm_frames_) {
            std_msgs::Int32 scene_msg;
            scene_msg.data = candidate_stairs_scene;
            stairs_scene_pub_.publish(scene_msg);
            last_stairs_scene_ = candidate_stairs_scene;
            pending_stairs_scene_count_ = 0;

            if (verbose_logging_) {
                ROS_INFO("Stairs scene changed: %d (1=UP, -1=DOWN, 0=FLAT), signed_delta: %.2f°, enter: %.1f°, exit: %.1f°",
                         last_stairs_scene_, signed_pitch_delta, stairs_enter_threshold_, stairs_exit_threshold_);
            }
        }
    } else {
        pending_stairs_scene_ = candidate_stairs_scene;
        pending_stairs_scene_count_ = 0;
    }

    // Stairs flag follows stable scene state.
    bool current_stairs_state = (last_stairs_scene_ != 0);

    // Only publish stairs flag when state changes.
    if (current_stairs_state != last_stairs_state_) {
        std_msgs::Bool stairs_msg;
        stairs_msg.data = current_stairs_state;
        stairs_flag_pub_.publish(stairs_msg);
        
        last_stairs_state_ = current_stairs_state;
        
        if (verbose_logging_) {
            ROS_INFO("Stairs state changed: %s (signed_deviation: %.2f°, enter_threshold: %.1f°)", 
                     current_stairs_state ? "DETECTED" : "CLEAR", 
                     pitch_deviation, stairs_threshold_);
        }
    }
    
    if (verbose_logging_) {
        ROS_INFO("Pitch deviation: %.2f°", pitch_deviation);
    }

    if (stairs_debug_mode_) {
        ROS_INFO_THROTTLE(1.0, "[StairDebug][IMU] baseline=%.2f enter=%.2f exit=%.2f raw_pitch=%.2f filtered_pitch=%.2f "
                 "signed_delta=%.2f abs_delta=%.2f candidate_scene=%d stable_scene=%d pending=%d/%d stairs_flag=%d",
                 pitch_baseline_, stairs_enter_threshold_, stairs_exit_threshold_,
                 euler_angles_deg.y, filtered_euler_deg.y,
                 signed_pitch_delta, pitch_deviation,
                 candidate_stairs_scene, last_stairs_scene_,
                 pending_stairs_scene_count_, stairs_confirm_frames_,
                 current_stairs_state ? 1 : 0);
    }
    
    // Create and publish Vector3Stamped message with filtered degrees
    geometry_msgs::Vector3Stamped euler_msg;
    euler_msg.header = msg->header;  // Use same header as input IMU message
    euler_msg.vector = filtered_euler_deg;
    
    euler_pub_.publish(euler_msg);
    
    // Print both raw and filtered Euler angles in degrees (if verbose logging enabled)
    if (verbose_logging_) {
        ROS_INFO("Quaternion: [x=%.3f, y=%.3f, z=%.3f, w=%.3f]", 
                 msg->orientation.x, msg->orientation.y, msg->orientation.z, msg->orientation.w);
        ROS_INFO("Raw Euler: [roll=%.2f°, pitch=%.2f°, yaw=%.2f°]",
                 euler_angles_deg.x, euler_angles_deg.y, euler_angles_deg.z);
        ROS_INFO("Filtered Euler: [roll=%.2f°, pitch=%.2f°, yaw=%.2f°]",
                 filtered_euler_deg.x, filtered_euler_deg.y, filtered_euler_deg.z);
        ROS_INFO("Pitch baseline: %.1f°, Stairs flag: %s", 
                 pitch_baseline_, current_stairs_state ? "TRUE" : "FALSE");
        ROS_INFO("---");
    }
}

geometry_msgs::Vector3 ImuOrientationConverter::quaternionToEuler(const geometry_msgs::Quaternion& quat)
{
    // Convert ROS quaternion to Eigen quaternion
    Eigen::Quaterniond eigen_quat(quat.w, quat.x, quat.y, quat.z);
    
    // Convert to rotation matrix first, then to Euler angles (ZYX convention)
    Eigen::Matrix3d rotation_matrix = eigen_quat.toRotationMatrix();
    
    // Extract Euler angles (roll, pitch, yaw) using ZYX convention
    Eigen::Vector3d euler_angles = rotation_matrix.eulerAngles(2, 1, 0); // ZYX order
    
    // Convert to ROS message format
    geometry_msgs::Vector3 result;
    result.x = euler_angles(2);  // roll  (rotation around x-axis)
    result.y = euler_angles(1);  // pitch (rotation around y-axis)  
    result.z = euler_angles(0);  // yaw   (rotation around z-axis)
    
    // Normalize angles to proper ranges
    // Roll: [-pi, pi]
    while (result.x > M_PI) result.x -= 2.0 * M_PI;
    while (result.x < -M_PI) result.x += 2.0 * M_PI;
    
    // Pitch: [-pi/2, pi/2], handle gimbal lock
    if (result.y > M_PI/2.0) {
        result.y = M_PI - result.y;
        result.x += M_PI;
        result.z += M_PI;
    }
    if (result.y < -M_PI/2.0) {
        result.y = -M_PI - result.y;
        result.x += M_PI;
        result.z += M_PI;
    }
    
    // Yaw: [-pi, pi]
    while (result.z > M_PI) result.z -= 2.0 * M_PI;
    while (result.z < -M_PI) result.z += 2.0 * M_PI;
    
    // Re-normalize roll after potential adjustment
    while (result.x > M_PI) result.x -= 2.0 * M_PI;
    while (result.x < -M_PI) result.x += 2.0 * M_PI;
    
    return result;
}

geometry_msgs::Vector3 ImuOrientationConverter::applyLowPassFilter(const geometry_msgs::Vector3& new_euler_deg)
{
    if (first_data_) {
        // Initialize filter with first data point
        filtered_euler_deg_ = new_euler_deg;
        first_data_ = false;
        return filtered_euler_deg_;
    }
    
    // Handle angle wraparound for proper filtering
    geometry_msgs::Vector3 diff;
    diff.x = new_euler_deg.x - filtered_euler_deg_.x;
    diff.y = new_euler_deg.y - filtered_euler_deg_.y;
    diff.z = new_euler_deg.z - filtered_euler_deg_.z;
    
    // Normalize angle differences to [-180, 180]
    while (diff.x > 180.0) diff.x -= 360.0;
    while (diff.x < -180.0) diff.x += 360.0;
    while (diff.y > 180.0) diff.y -= 360.0;
    while (diff.y < -180.0) diff.y += 360.0;
    while (diff.z > 180.0) diff.z -= 360.0;
    while (diff.z < -180.0) diff.z += 360.0;
    
    // Apply low-pass filter: filtered = (1-alpha) * filtered + alpha * new
    filtered_euler_deg_.x += alpha_ * diff.x;
    filtered_euler_deg_.y += alpha_ * diff.y;
    filtered_euler_deg_.z += alpha_ * diff.z;
    
    // Normalize filtered angles to [-180, 180]
    while (filtered_euler_deg_.x > 180.0) filtered_euler_deg_.x -= 360.0;
    while (filtered_euler_deg_.x < -180.0) filtered_euler_deg_.x += 360.0;
    while (filtered_euler_deg_.y > 90.0) filtered_euler_deg_.y = 180.0 - filtered_euler_deg_.y;
    while (filtered_euler_deg_.y < -90.0) filtered_euler_deg_.y = -180.0 - filtered_euler_deg_.y;
    while (filtered_euler_deg_.z > 180.0) filtered_euler_deg_.z -= 360.0;
    while (filtered_euler_deg_.z < -180.0) filtered_euler_deg_.z += 360.0;
    
    return filtered_euler_deg_;
}

int main(int argc, char** argv)
{
    setlocale(LC_ALL, "");

    ros::init(argc, argv, "imu_orientation_converter_node");
    
    ImuOrientationConverter converter;
    
    ros::spin();
    
    return 0;
}
