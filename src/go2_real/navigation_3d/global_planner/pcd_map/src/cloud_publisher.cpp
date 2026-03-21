#include <ros/ros.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/filters/voxel_grid.h>
#include <sensor_msgs/PointCloud2.h>

/**
 * 点云发布节点 - 将PCD文件加载并发布为ROS话题
 */
class CloudPublisher
{
public:
    CloudPublisher(ros::NodeHandle &nh, ros::NodeHandle &private_nh)
        : nh_(nh), private_nh_(private_nh)
    {
        setlocale(LC_ALL, "");
        // 获取ROS参数
        private_nh_.param("file_directory", file_directory_, std::string("/home/server/WS_ROS1/ws_pcl_map/src/pcd_map/pcd/"));
        private_nh_.param("file_name", file_name_, std::string("free_space_filtered"));
        private_nh_.param("frame_id", frame_id_, std::string("map"));
        private_nh_.param("publish_rate", publish_rate_, 1.0); // Hz
        private_nh_.param("latch_publisher", latch_publisher_, true);
        private_nh_.param("downsample_leaf_size", downsample_leaf_size_, 0.1); // 体素滤波器叶子大小 (米)
        private_nh_.param("enable_downsampling", enable_downsampling_, false); // 是否启用下采样

        // 创建发布者
        cloud_pub_ = nh_.advertise<sensor_msgs::PointCloud2>("free_space_cloud", 1, latch_publisher_);

        // 构建完整的文件路径
        pcd_file_ = file_directory_ + file_name_ + ".pcd";

        ROS_INFO("点云发布器已初始化, 文件路径: %s", pcd_file_.c_str());
    }

    // 加载点云文件
    bool loadPointCloud()
    {
        cloud_.reset(new pcl::PointCloud<pcl::PointXYZ>);

        ROS_INFO("正在加载点云文件: %s", pcd_file_.c_str());

        if (pcl::io::loadPCDFile<pcl::PointXYZ>(pcd_file_, *cloud_) == -1)
        {
            ROS_ERROR("无法读取文件: %s", pcd_file_.c_str());
            return false;
        }

        ROS_INFO("成功加载点云文件, 点数: %lu", cloud_->points.size());
        
        // 如果启用下采样，对点云进行体素滤波
        if (enable_downsampling_)
        {
            downsampleCloud();
        }
        
        return true;
    }

    // 对点云进行下采样以减少数据量
    void downsampleCloud()
    {
        if (!cloud_ || cloud_->empty())
        {
            ROS_WARN("点云为空，无法进行下采样");
            return;
        }

        size_t original_size = cloud_->points.size();
        
        pcl::PointCloud<pcl::PointXYZ>::Ptr filtered_cloud(new pcl::PointCloud<pcl::PointXYZ>);
        pcl::VoxelGrid<pcl::PointXYZ> voxel_grid;
        voxel_grid.setInputCloud(cloud_);
        voxel_grid.setLeafSize(downsample_leaf_size_, downsample_leaf_size_, downsample_leaf_size_);
        voxel_grid.filter(*filtered_cloud);
        
        cloud_ = filtered_cloud;
        
        ROS_INFO("点云下采样完成: %lu -> %lu 点 (叶子大小: %.2f m)", 
                 original_size, cloud_->points.size(), downsample_leaf_size_);
    }

    // 发布点云
    void publishCloud()
    {
        if (!cloud_ || cloud_->empty())
        {
            ROS_WARN("点云为空, 无法发布");
            return;
        }

        // 转换为ROS消息
        sensor_msgs::PointCloud2 output_msg;
        pcl::toROSMsg(*cloud_, output_msg);
        output_msg.header.frame_id = frame_id_;
        output_msg.header.stamp = ros::Time::now();

        // 发布点云
        cloud_pub_.publish(output_msg);
        ROS_INFO("已发布点云数据, 点数: %lu", cloud_->points.size());
    }

    // 开始连续发布
    void startPublishing()
    {
        if (!loadPointCloud())
        {
            return;
        }

        // 创建定时器, 定期发布点云
        // timer_ = nh_.createTimer(ros::Duration(1.0 / publish_rate_),
        //                          &CloudPublisher::timerCallback, this);
        publishCloud();
        // ROS_INFO("开始发布点云, 频率: %.1f Hz", publish_rate_);
    }

private:
    // 定时器回调函数
    // void timerCallback(const ros::TimerEvent &event)
    // {
    //     // 只在有订阅者时才发布消息，避免不必要的计算
    //     if (cloud_pub_.getNumSubscribers() > 0) {
    //         publishCloud();
    //     }
    // }

    ros::NodeHandle nh_;
    ros::NodeHandle private_nh_;
    ros::Publisher cloud_pub_;
    ros::Timer timer_;

    std::string file_directory_;
    std::string file_name_;
    std::string pcd_file_;
    std::string frame_id_;
    double publish_rate_;
    bool latch_publisher_;
    double downsample_leaf_size_;
    bool enable_downsampling_;

    // 点云数据
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_;
};

int main(int argc, char **argv)
{
    ros::init(argc, argv, "cloud_publisher");
    ros::NodeHandle nh;
    ros::NodeHandle private_nh("~");

    CloudPublisher publisher(nh, private_nh);
    publisher.startPublishing();

    ros::spin();

    return 0;
}