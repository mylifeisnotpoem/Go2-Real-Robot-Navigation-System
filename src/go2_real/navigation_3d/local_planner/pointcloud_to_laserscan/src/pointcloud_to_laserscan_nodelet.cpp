/*
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2010-2012, Willow Garage, Inc.
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of Willow Garage, Inc. nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *
 *
 */

/*
 * Author: Paul Bovbel
 */

#include <limits>
#include <pluginlib/class_list_macros.h>
#include <pointcloud_to_laserscan/pointcloud_to_laserscan_nodelet.h>
#include <sensor_msgs/LaserScan.h>
#include <sensor_msgs/point_cloud2_iterator.h>
#include <string>
#include <tf2_sensor_msgs/tf2_sensor_msgs.h>
#include <cstring>  // for memcpy
#include <map>      // for density filtering
#include <utility>  // for std::pair

namespace pointcloud_to_laserscan
{
PointCloudToLaserScanNodelet::PointCloudToLaserScanNodelet()
{
}

void PointCloudToLaserScanNodelet::onInit()
{
  boost::mutex::scoped_lock lock(connect_mutex_);
  private_nh_ = getPrivateNodeHandle();

  private_nh_.param<std::string>("target_frame", target_frame_, "");
  private_nh_.param<double>("transform_tolerance", tolerance_, 0.01);
  private_nh_.param<double>("min_height", min_height_, std::numeric_limits<double>::min());
  private_nh_.param<double>("max_height", max_height_, std::numeric_limits<double>::max());

  private_nh_.param<double>("angle_min", angle_min_, -M_PI);
  private_nh_.param<double>("angle_max", angle_max_, M_PI);
  private_nh_.param<double>("angle_increment", angle_increment_, M_PI / 180.0);
  private_nh_.param<double>("scan_time", scan_time_, 1.0 / 30.0);
  private_nh_.param<double>("range_min", range_min_, 0.0);
  private_nh_.param<double>("range_max", range_max_, std::numeric_limits<double>::max());
  private_nh_.param<double>("inf_epsilon", inf_epsilon_, 1.0);
  
  // 新增的密度过滤参数
  private_nh_.param<bool>("enable_density_filter", enable_density_filter_, false);
  private_nh_.param<int>("min_point_count", min_point_count_, 3);
  private_nh_.param<double>("density_grid_size", density_grid_size_, 0.1);

  int concurrency_level;
  private_nh_.param<int>("concurrency_level", concurrency_level, 1);
  private_nh_.param<bool>("use_inf", use_inf_, true);

  // Check if explicitly single threaded, otherwise, let nodelet manager dictate thread pool size
  if (concurrency_level == 1)
  {
    nh_ = getNodeHandle();
  }
  else
  {
    nh_ = getMTNodeHandle();
  }

  // Only queue one pointcloud per running thread
  if (concurrency_level > 0)
  {
    input_queue_size_ = concurrency_level;
  }
  else
  {
    input_queue_size_ = boost::thread::hardware_concurrency();
  }

  // if pointcloud target frame specified, we need to filter by transform availability
  if (!target_frame_.empty())
  {
    tf2_.reset(new tf2_ros::Buffer());
    tf2_listener_.reset(new tf2_ros::TransformListener(*tf2_));
    message_filter_.reset(new MessageFilter(sub_, *tf2_, target_frame_, input_queue_size_, nh_));
    message_filter_->registerCallback(boost::bind(&PointCloudToLaserScanNodelet::cloudCb, this, _1));
    message_filter_->registerFailureCallback(boost::bind(&PointCloudToLaserScanNodelet::failureCb, this, _1, _2));
  }
  else  // otherwise setup direct subscription
  {
    sub_.registerCallback(boost::bind(&PointCloudToLaserScanNodelet::cloudCb, this, _1));
  }

  pub_ = nh_.advertise<sensor_msgs::LaserScan>("scan", 10, boost::bind(&PointCloudToLaserScanNodelet::connectCb, this),
                                               boost::bind(&PointCloudToLaserScanNodelet::disconnectCb, this));
  
  // 添加新的发布器用于观察点云处理过程
  pub_raw_cloud_ = nh_.advertise<sensor_msgs::PointCloud2>("cloud_raw", 10);
  pub_filtered_cloud_ = nh_.advertise<sensor_msgs::PointCloud2>("cloud_filtered", 10);
}

void PointCloudToLaserScanNodelet::connectCb()
{
  boost::mutex::scoped_lock lock(connect_mutex_);
  if (pub_.getNumSubscribers() > 0 && sub_.getSubscriber().getNumPublishers() == 0)
  {
    NODELET_INFO("Got a subscriber to scan, starting subscriber to pointcloud");
    sub_.subscribe(nh_, "cloud_in", input_queue_size_);
  }
}

void PointCloudToLaserScanNodelet::disconnectCb()
{
  boost::mutex::scoped_lock lock(connect_mutex_);
  if (pub_.getNumSubscribers() == 0)
  {
    NODELET_INFO("No subscibers to scan, shutting down subscriber to pointcloud");
    sub_.unsubscribe();
  }
}

void PointCloudToLaserScanNodelet::failureCb(const sensor_msgs::PointCloud2ConstPtr& cloud_msg,
                                             tf2_ros::filter_failure_reasons::FilterFailureReason reason)
{
  NODELET_WARN_STREAM_THROTTLE(1.0, "Can't transform pointcloud from frame " << cloud_msg->header.frame_id << " to "
                                                                             << message_filter_->getTargetFramesString()
                                                                             << " at time " << cloud_msg->header.stamp
                                                                             << ", reason: " << reason);
}

void PointCloudToLaserScanNodelet::cloudCb(const sensor_msgs::PointCloud2ConstPtr& cloud_msg)
{
  // build laserscan output
  sensor_msgs::LaserScan output;
  output.header = cloud_msg->header;
  if (!target_frame_.empty())
  {
    output.header.frame_id = target_frame_;
  }

  output.angle_min = angle_min_;
  output.angle_max = angle_max_;
  output.angle_increment = angle_increment_;
  output.time_increment = 0.0;
  output.scan_time = scan_time_;
  output.range_min = range_min_;
  output.range_max = range_max_;

  // determine amount of rays to create
  uint32_t ranges_size = std::ceil((output.angle_max - output.angle_min) / output.angle_increment);

  // determine if laserscan rays with no obstacle data will evaluate to infinity or max_range
  if (use_inf_)
  {
    output.ranges.assign(ranges_size, std::numeric_limits<double>::infinity());
  }
  else
  {
    output.ranges.assign(ranges_size, output.range_max + inf_epsilon_);
  }

  sensor_msgs::PointCloud2ConstPtr cloud_out;
  sensor_msgs::PointCloud2Ptr cloud;

  // Transform cloud if necessary
  if (!(output.header.frame_id == cloud_msg->header.frame_id))
  {
    try
    {
      cloud.reset(new sensor_msgs::PointCloud2);
      tf2_->transform(*cloud_msg, *cloud, target_frame_, ros::Duration(tolerance_));
      cloud_out = cloud;
    }
    catch (tf2::TransformException& ex)
    {
      NODELET_ERROR_STREAM("Transform failure: " << ex.what());
      return;
    }
  }
  else
  {
    cloud_out = cloud_msg;
  }

  // 发布原始点云（坐标变换后）
  if (pub_raw_cloud_.getNumSubscribers() > 0)
  {
    pub_raw_cloud_.publish(cloud_out);
  }

  // 创建过滤后的点云用于发布
  sensor_msgs::PointCloud2 filtered_cloud;
  std::vector<float> filtered_points;
  
  if (pub_filtered_cloud_.getNumSubscribers() > 0)
  {
    filtered_cloud.header = cloud_out->header;
    filtered_cloud.height = 1;  // 无序点云
    filtered_cloud.width = 0;   // 将在循环中计算
    filtered_cloud.is_dense = false;
    filtered_cloud.is_bigendian = cloud_out->is_bigendian;
    
    // 创建简化的点云结构（只包含x,y,z）
    filtered_cloud.fields.resize(3);
    filtered_cloud.fields[0].name = "x";
    filtered_cloud.fields[0].offset = 0;
    filtered_cloud.fields[0].datatype = sensor_msgs::PointField::FLOAT32;
    filtered_cloud.fields[0].count = 1;
    filtered_cloud.fields[1].name = "y";
    filtered_cloud.fields[1].offset = 4;
    filtered_cloud.fields[1].datatype = sensor_msgs::PointField::FLOAT32;
    filtered_cloud.fields[1].count = 1;
    filtered_cloud.fields[2].name = "z";
    filtered_cloud.fields[2].offset = 8;
    filtered_cloud.fields[2].datatype = sensor_msgs::PointField::FLOAT32;
    filtered_cloud.fields[2].count = 1;
    filtered_cloud.point_step = 12; // 3 * sizeof(float)
  }

  // 密度过滤数据结构
  std::map<std::pair<int, int>, std::vector<std::pair<double, double>>> density_grid;  // 网格坐标 -> (range, z)向量

  // Iterate through pointcloud
  for (sensor_msgs::PointCloud2ConstIterator<float> iter_x(*cloud_out, "x"), iter_y(*cloud_out, "y"),
       iter_z(*cloud_out, "z");
       iter_x != iter_x.end(); ++iter_x, ++iter_y, ++iter_z)
  {
    if (std::isnan(*iter_x) || std::isnan(*iter_y) || std::isnan(*iter_z))
    {
      NODELET_DEBUG("rejected for nan in point(%f, %f, %f)\n", *iter_x, *iter_y, *iter_z);
      continue;
    }

    // 检查高度过滤条件
    bool height_filtered = (*iter_z > max_height_ || *iter_z < min_height_);
    
    if (!height_filtered)
    {
      // 如果需要发布过滤后的点云，保存通过高度过滤的点
      if (pub_filtered_cloud_.getNumSubscribers() > 0)
      {
        filtered_points.push_back(*iter_x);
        filtered_points.push_back(*iter_y);
        filtered_points.push_back(*iter_z);
      }
    }
    else
    {
      NODELET_DEBUG("rejected for height %f not in range (%f, %f)\n", *iter_z, min_height_, max_height_);
      continue;
    }

    double range = hypot(*iter_x, *iter_y);
    if (range < range_min_)
    {
      NODELET_DEBUG("rejected for range %f below minimum value %f. Point: (%f, %f, %f)", range, range_min_, *iter_x,
                    *iter_y, *iter_z);
      continue;
    }
    if (range > range_max_)
    {
      NODELET_DEBUG("rejected for range %f above maximum value %f. Point: (%f, %f, %f)", range, range_max_, *iter_x,
                    *iter_y, *iter_z);
      continue;
    }

    double angle = atan2(*iter_y, *iter_x);
    if (angle < output.angle_min || angle > output.angle_max)
    {
      NODELET_DEBUG("rejected for angle %f not in range (%f, %f)\n", angle, output.angle_min, output.angle_max);
      continue;
    }

    if (enable_density_filter_)
    {
      // 密度过滤：将点分配到网格中
      int grid_x = static_cast<int>(std::floor(*iter_x / density_grid_size_));
      int grid_y = static_cast<int>(std::floor(*iter_y / density_grid_size_));
      std::pair<int, int> grid_key = std::make_pair(grid_x, grid_y);
      
      density_grid[grid_key].push_back(std::make_pair(range, *iter_z));
    }
    else
    {
      // 不使用密度过滤，直接更新激光扫描数据
      int index = (angle - output.angle_min) / output.angle_increment;
      if (range < output.ranges[index])
      {
        output.ranges[index] = range;
      }
    }
  }

  // 如果启用了密度过滤，处理网格数据
  if (enable_density_filter_)
  {
    for (const auto& grid_cell : density_grid)
    {
      const std::vector<std::pair<double, double>>& points = grid_cell.second;
      
      // 检查这个网格单元中的点数是否达到阈值
      if (static_cast<int>(points.size()) >= min_point_count_)
      {
        // 找到最近的点
        double min_range = std::numeric_limits<double>::max();
        double best_x = 0, best_y = 0;
        
        for (const auto& point : points)
        {
          if (point.first < min_range)
          {
            min_range = point.first;
            // 从range和网格坐标重建x,y坐标(用网格中心)
            best_x = (grid_cell.first.first + 0.5) * density_grid_size_;
            best_y = (grid_cell.first.second + 0.5) * density_grid_size_;
          }
        }
        
        // 计算角度并更新激光扫描数据
        double angle = atan2(best_y, best_x);
        if (angle >= output.angle_min && angle <= output.angle_max)
        {
          int index = (angle - output.angle_min) / output.angle_increment;
          if (index >= 0 && index < static_cast<int>(output.ranges.size()))
          {
            if (min_range < output.ranges[index])
            {
              output.ranges[index] = min_range;
            }
          }
        }
      }
    }
  }
  
  // 发布过滤后的点云
  if (pub_filtered_cloud_.getNumSubscribers() > 0 && !filtered_points.empty())
  {
    filtered_cloud.width = filtered_points.size() / 3;  // 每个点有3个坐标
    filtered_cloud.row_step = filtered_cloud.width * filtered_cloud.point_step;
    
    // 将float数据转换为uint8数据
    filtered_cloud.data.resize(filtered_points.size() * sizeof(float));
    memcpy(&filtered_cloud.data[0], &filtered_points[0], filtered_points.size() * sizeof(float));
    
    pub_filtered_cloud_.publish(filtered_cloud);
  }
  
  pub_.publish(output);
}
}  // namespace pointcloud_to_laserscan

PLUGINLIB_EXPORT_CLASS(pointcloud_to_laserscan::PointCloudToLaserScanNodelet, nodelet::Nodelet)
