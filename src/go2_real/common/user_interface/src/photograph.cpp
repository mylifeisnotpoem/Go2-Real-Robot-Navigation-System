#include <unitree/robot/go2/video/video_client.hpp>
#include <iostream>
#include <fstream>
#include <ctime>
#include <unistd.h>
#include <ros/ros.h>
#include <std_msgs/Int32.h>

// Global variables
unitree::robot::go2::VideoClient* video_client_ptr = nullptr;
bool take_photo_flag = false;
std::string image_save_path = "/home/unitree/ws_go2edu_real/photos/";
ros::Time photo_trigger_time;

void taskActionCallback(const std_msgs::Int32::ConstPtr& msg)
{
    if (msg->data == 1) {
        take_photo_flag = true;
        photo_trigger_time = ros::Time::now();
        ROS_INFO("Received take_photo command (data: %d), will take photo in 2 seconds", msg->data);
    }
}

void takePhoto()
{
    if (!video_client_ptr) return;
    
    std::vector<uint8_t> image_sample;
    int ret = video_client_ptr->GetImageSample(image_sample);

    if (ret == 0) {
        time_t rawtime;
        struct tm *timeinfo;
        char buffer[80];

        time(&rawtime);
        timeinfo = localtime(&rawtime);

        strftime(buffer, sizeof(buffer), "%Y%m%d%H%M%S.jpg", timeinfo);
        std::string image_name = image_save_path + std::string(buffer);

        std::ofstream image_file(image_name, std::ios::binary);
        if (image_file.is_open()) {
            image_file.write(reinterpret_cast<const char*>(image_sample.data()), image_sample.size());
            image_file.close();
            std::cout << "Image saved successfully as " << image_name << std::endl;
            ROS_INFO("Photo taken: %s", image_name.c_str());
        } else {
            std::cerr << "Error: Failed to save image." << std::endl;
            ROS_ERROR("Failed to save image: %s", image_name.c_str());
        }
    } else {
        ROS_WARN("Failed to get image sample, ret: %d", ret);
    }
}

int main(int argc, char **argv)
{
    // Initialize ROS
    ros::init(argc, argv, "photograph_node");
    ros::NodeHandle nh;
    
    // Get image save path from parameter
    nh.param<std::string>("image_save_path", image_save_path, "/home/unitree/ws_go2edu_real/photos/");
    
    // Create directory if it doesn't exist
    std::string mkdir_cmd = "mkdir -p " + image_save_path;
    system(mkdir_cmd.c_str());
    
    ROS_INFO("Image save path: %s", image_save_path.c_str());
    
    // Subscribe to task action topic
    ros::Subscriber task_sub = nh.subscribe("/task_action", 10, taskActionCallback);
    
    /*
     * Initilaize ChannelFactory
     */
    unitree::robot::ChannelFactory::Instance()->Init(0);
    unitree::robot::go2::VideoClient video_client;
    video_client_ptr = &video_client;

    /*
     * Set request timeout 1.0s
     */
    video_client.SetTimeout(1.0f);
    video_client.Init();

    ROS_INFO("Photograph node started, waiting for commands...");

    // Main loop with ROS
    ros::Rate rate(10); // 10Hz
    while (ros::ok())
    {
        ros::spinOnce();
        
        if (take_photo_flag) {
            // Check if 2 seconds have passed since trigger
            ros::Duration elapsed = ros::Time::now() - photo_trigger_time;
            if (elapsed.toSec() >= 2.0) {
                takePhoto();
                take_photo_flag = false;
            }
        }
        
        rate.sleep();
    }

    return 0;
}