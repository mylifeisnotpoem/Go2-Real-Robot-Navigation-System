#include <ros/ros.h>

#include <pcl/io/pcd_io.h>
#include <pcl/filters/conditional_removal.h>         //条件滤波器头文件
#include <pcl/filters/passthrough.h>                 //直通滤波器头文件
#include <pcl/filters/radius_outlier_removal.h>      //半径滤波器头文件
#include <pcl/filters/statistical_outlier_removal.h> //统计滤波器头文件
#include <pcl/filters/voxel_grid.h>                  //体素滤波器头文件
#include <pcl/point_types.h>

std::string file_directory;  // PCD文件所在目录
std::string file_name;       // PCD文件名称
std::string input_pcd_file;  // 完整输入文件路径
std::string output_pcd_file; // 完整输出文件路径

const std::string pcd_format = ".pcd";

// 半径滤波参数
bool use_radius_filter = true; // 是否使用半径滤波
double thre_radius = 0.1;
int thres_point_count = 10;

// 是否使用统计滤波
bool use_statistical_filter = false;
// 统计滤波参数
double stat_mean_k = 50;
double stat_std_dev = 1.0;

// 体素滤波参数
bool use_voxel_filter = false;
double voxel_leaf_size = 0.01;

// 点云数据指针
pcl::PointCloud<pcl::PointXYZ>::Ptr
    cloud_input(new pcl::PointCloud<pcl::PointXYZ>); // 输入点云
pcl::PointCloud<pcl::PointXYZ>::Ptr
    cloud_after_Radius(new pcl::PointCloud<pcl::PointXYZ>); // 半径滤波后
pcl::PointCloud<pcl::PointXYZ>::Ptr
    cloud_after_Statistical(new pcl::PointCloud<pcl::PointXYZ>); // 统计滤波后
pcl::PointCloud<pcl::PointXYZ>::Ptr
    cloud_after_Voxel(new pcl::PointCloud<pcl::PointXYZ>); // 体素滤波后

// 半径滤波
void RadiusOutlierFilter(const pcl::PointCloud<pcl::PointXYZ>::Ptr &input_cloud,
                         const double &radius, const int &thre_count);
// 统计滤波
void StatisticalOutlierFilter(const pcl::PointCloud<pcl::PointXYZ>::Ptr &input_cloud,
                              const double &mean_k, const double &std_dev);
// 体素滤波
void VoxelGridFilter(const pcl::PointCloud<pcl::PointXYZ>::Ptr &input_cloud,
                     const double &leaf_size);
// 保存结果
bool SaveFilteredCloud(const pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud,
                       const std::string &file_path);

int main(int argc, char **argv)
{
  setlocale(LC_ALL, "");

  ros::init(argc, argv, "pcl_filter");
  ros::NodeHandle nh;
  ros::NodeHandle private_nh("~");

  // 获取ROS参数
  private_nh.param("file_directory", file_directory, std::string("/home/"));
  private_nh.param("file_name", file_name, std::string("free_space"));
  private_nh.param("use_radius_filter", use_radius_filter, true);
  private_nh.param("thre_radius", thre_radius, 0.1);
  private_nh.param("thres_point_count", thres_point_count, 10);
  private_nh.param("use_statistical_filter", use_statistical_filter, false);
  private_nh.param("stat_mean_k", stat_mean_k, 50.0);
  private_nh.param("stat_std_dev", stat_std_dev, 1.0);
  private_nh.param("use_voxel_filter", use_voxel_filter, false);
  private_nh.param("voxel_leaf_size", voxel_leaf_size, 0.01);

  // 构建完整的输入文件路径
  input_pcd_file = file_directory + file_name + pcd_format;
  output_pcd_file = file_directory + file_name + "_filtered" + pcd_format;

  ROS_INFO("加载点云文件: %s", input_pcd_file.c_str());

  // 加载PCD文件
  if (pcl::io::loadPCDFile<pcl::PointXYZ>(input_pcd_file, *cloud_input) == -1)
  {
    PCL_ERROR("无法读取文件: %s \n", input_pcd_file.c_str());
    return (-1);
  }

  ROS_INFO("初始点云数据点数: %lu", cloud_input->points.size());

  // 当前处理的点云
  pcl::PointCloud<pcl::PointXYZ>::Ptr current_cloud = cloud_input;

  // 执行半径滤波 (可选)
  if (use_radius_filter)
  {
    RadiusOutlierFilter(current_cloud, thre_radius, thres_point_count);
    current_cloud = cloud_after_Radius;

    // 保存半径滤波后的结果
    ROS_INFO("保存半径滤波结果到: %s", (file_directory + file_name + "_radius.pcd").c_str());
    SaveFilteredCloud(cloud_after_Radius, file_directory + file_name + "_radius.pcd");
  }

  // 执行统计滤波 (可选)
  if (use_statistical_filter)
  {
    StatisticalOutlierFilter(current_cloud, stat_mean_k, stat_std_dev);
    current_cloud = cloud_after_Statistical;

    // 保存统计滤波后的结果
    ROS_INFO("保存统计滤波结果到: %s", (file_directory + file_name + "_statistical.pcd").c_str());
    SaveFilteredCloud(cloud_after_Statistical, file_directory + file_name + "_statistical.pcd");
  }

  // 执行体素滤波 (可选)
  if (use_voxel_filter)
  {
    VoxelGridFilter(current_cloud, voxel_leaf_size);
    current_cloud = cloud_after_Voxel;

    // 保存体素滤波后的结果
    ROS_INFO("保存体素滤波结果到: %s", (file_directory + file_name + "_voxel.pcd").c_str());
    SaveFilteredCloud(cloud_after_Voxel, file_directory + file_name + "_voxel.pcd");
  }

  // 保存最终处理后的点云
  ROS_INFO("保存最终处理后的点云到: %s", output_pcd_file.c_str());
  SaveFilteredCloud(current_cloud, output_pcd_file);

  ROS_INFO("点云处理完成!");

  return 0;
}

// 半径滤波
void RadiusOutlierFilter(const pcl::PointCloud<pcl::PointXYZ>::Ptr &input_cloud,
                         const double &radius, const int &thre_count)
{
  // 创建滤波器
  pcl::RadiusOutlierRemoval<pcl::PointXYZ> radiusoutlier;
  // 设置输入点云
  radiusoutlier.setInputCloud(input_cloud);
  // 设置半径,在该范围内找临近点
  radiusoutlier.setRadiusSearch(radius);
  // 设置查询点的邻域点集数，小于该阈值的删除
  radiusoutlier.setMinNeighborsInRadius(thre_count);
  radiusoutlier.filter(*cloud_after_Radius);
  ROS_INFO("半径滤波后点云数据点数: %lu", cloud_after_Radius->points.size());
}

// 统计滤波
void StatisticalOutlierFilter(const pcl::PointCloud<pcl::PointXYZ>::Ptr &input_cloud,
                              const double &mean_k, const double &std_dev)
{
  // 创建滤波器
  pcl::StatisticalOutlierRemoval<pcl::PointXYZ> statistical;
  // 设置输入点云
  statistical.setInputCloud(input_cloud);
  // 设置平均距离估计的最近邻数量
  statistical.setMeanK(mean_k);
  // 设置标准差阈值系数
  statistical.setStddevMulThresh(std_dev);
  // 执行滤波
  statistical.filter(*cloud_after_Statistical);
  ROS_INFO("统计滤波后点云数据点数: %lu", cloud_after_Statistical->points.size());
}

// 体素滤波
void VoxelGridFilter(const pcl::PointCloud<pcl::PointXYZ>::Ptr &input_cloud,
                     const double &leaf_size)
{
  // 创建滤波器
  pcl::VoxelGrid<pcl::PointXYZ> voxel;
  // 设置输入点云
  voxel.setInputCloud(input_cloud);
  // 设置体素大小
  voxel.setLeafSize(leaf_size, leaf_size, leaf_size);
  // 执行滤波
  voxel.filter(*cloud_after_Voxel);
  ROS_INFO("体素滤波后点云数据点数: %lu", cloud_after_Voxel->points.size());
}

// 保存结果
bool SaveFilteredCloud(const pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud,
                       const std::string &file_path)
{
  if (cloud->points.empty())
  {
    ROS_WARN("点云为空，无法保存!");
    return false;
  }

  if (pcl::io::savePCDFile(file_path, *cloud) == -1)
  {
    PCL_ERROR("无法保存点云到 %s\n", file_path.c_str());
    return false;
  }

  return true;
}
