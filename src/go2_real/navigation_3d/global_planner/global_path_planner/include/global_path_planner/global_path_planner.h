#ifndef GLOBAL_PATH_PLANNER_H
#define GLOBAL_PATH_PLANNER_H

#include <ros/ros.h>
#include <nav_msgs/Path.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/PointStamped.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/Pose.h>
#include <visualization_msgs/MarkerArray.h>
#include <sensor_msgs/PointCloud2.h>
#include <tf/transform_listener.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/search/kdtree.h>
#include <vector>
#include <string>
#include <map>
#include <cmath>

namespace global_path_planner
{

struct TopologyNode
{
    int id;
    double x, y, z;
    std::string label;
    
    TopologyNode() : id(0), x(0.0), y(0.0), z(0.0), label("") {}
    TopologyNode(int _id, double _x, double _y, double _z, const std::string& _label)
        : id(_id), x(_x), y(_y), z(_z), label(_label) {}
};

struct TopologyEdge
{
    int id;
    int from_id;
    int to_id;
    double weight;
    bool bidirectional;
    
    TopologyEdge() : id(0), from_id(0), to_id(0), weight(0.0), bidirectional(false) {}
    TopologyEdge(int _id, int _from, int _to, double _weight, bool _bi)
        : id(_id), from_id(_from), to_id(_to), weight(_weight), bidirectional(_bi) {}
};

struct PathSegment
{
    geometry_msgs::Point start;
    geometry_msgs::Point end;
    double length;
    
    PathSegment() : length(0.0) {}
    PathSegment(const geometry_msgs::Point& s, const geometry_msgs::Point& e)
        : start(s), end(e) 
    {
        double dx = e.x - s.x;
        double dy = e.y - s.y;
        double dz = e.z - s.z;
        length = std::sqrt(dx*dx + dy*dy + dz*dz);
    }
};

class GlobalPathPlanner
{
public:
    GlobalPathPlanner();
    ~GlobalPathPlanner();
    
    bool initialize();
    void spin();

private:
    // ROS组件
    ros::NodeHandle nh_;
    ros::Publisher global_path_pub_;
    // ros::Publisher visualization_pub_;  // 已禁用可视化以避免RViz崩溃
    ros::Subscriber odom_sub_;
    ros::Subscriber goal_sub_;
    ros::Subscriber target_points_sub_;       // 新增：目标点订阅器
    ros::Subscriber free_space_cloud_sub_;  // 可行域点云订阅器
    tf::TransformListener tf_listener_;
    
    // 参数
    std::string map_file_path_;
    std::string global_frame_;
    std::string robot_frame_;
    std::string odom_topic_;           // 里程计话题名
    double connection_distance_threshold_;
    int interpolation_points_;

    // === 新增：A* 算法距离阈值 ===
    double astar_distance_threshold_;

    // 可行性检查参数
    double free_space_radius_;
    double edge_proximity_threshold_;
    double line_of_sight_resolution_;
    bool enable_feasibility_check_;
    bool enable_direct_connection_;
    double direct_connection_threshold_;
    
    // 边合法性检查参数
    double robot_height_;              // 机器狗高度
    int edge_sample_count_;            // 边采样点数量
    double safety_sphere_radius_;      // 安全球半径
    double valid_point_threshold_;     // 有效点阈值百分比
    std::string free_space_topic_;     // 可行域点云话题
    bool enable_temp_connection_feasibility_check_;  // 是否对临时连接启用可行域检查
    
    // 可行域点云数据
    pcl::PointCloud<pcl::PointXYZ>::Ptr free_space_cloud_;
    pcl::search::KdTree<pcl::PointXYZ>::Ptr kdtree_;
    bool has_free_space_cloud_;
    
    // 数据
    std::vector<TopologyNode> nodes_;
    std::vector<TopologyEdge> edges_;
    std::map<int, TopologyNode> node_map_;
    geometry_msgs::Point current_position_;
    geometry_msgs::Point goal_position_;
    bool has_current_pose_;
    bool has_goal_;
    
    // 可视化状态 (已禁用)
    // size_t last_published_marker_count_;
    
    // 方法
    bool loadTopologyMap();
    void odomCallback(const nav_msgs::Odometry::ConstPtr& msg);
    void goalCallback(const geometry_msgs::PointStamped::ConstPtr& msg);
    void targetPointsCallback(const geometry_msgs::Pose::ConstPtr& msg);  // 新增：目标点回调函数
    
    // 路径规划核心方法
    nav_msgs::Path planGlobalPath();
    std::vector<int> findTopologyPath(int start_node_id, int goal_node_id);
    
    // 几何计算方法
    geometry_msgs::Point findNearestPointOnEdge(const geometry_msgs::Point& point, 
                                                const TopologyEdge& edge);
    double pointToSegmentDistance(const geometry_msgs::Point& point,
                                 const geometry_msgs::Point& seg_start,
                                 const geometry_msgs::Point& seg_end);
    int findNearestNode(const geometry_msgs::Point& point);
    TopologyEdge findNearestEdge(const geometry_msgs::Point& point);
    
    // 路径构建方法
    nav_msgs::Path buildCompletePath(const geometry_msgs::Point& start,
                                    const geometry_msgs::Point& goal,
                                    const std::vector<int>& topology_path);
    std::vector<geometry_msgs::Point> interpolatePath(const geometry_msgs::Point& start,
                                                     const geometry_msgs::Point& end,
                                                     int num_points);
    
    // 工具方法
    double calculateDistance(const geometry_msgs::Point& p1, const geometry_msgs::Point& p2);
    geometry_msgs::Quaternion calculateOrientation(const geometry_msgs::Point& from,
                                                   const geometry_msgs::Point& to);
    void publishVisualization(const nav_msgs::Path& path);
    
    // Dijkstra算法
    std::vector<int> dijkstraSearch(int start_id, int goal_id);
    
    // === 新增：A* 算法核心方法 ===
    std::vector<int> aStarSearchInTempGraph(const std::vector<TopologyNode>& temp_nodes,
                                            const std::vector<TopologyEdge>& temp_edges,
                                            int start_id, int goal_id);
    // 启发式代价计算函数 (计算 H 值)
    double calculateHeuristic(const TopologyNode& node, const TopologyNode& goal);

    // 临时拓扑图相关方法
    void addTemporaryConnections(std::vector<TopologyNode>& temp_nodes, 
                                std::vector<TopologyEdge>& temp_edges,
                                int temp_node_id, 
                                const geometry_msgs::Point& position);
    std::vector<int> dijkstraSearchInTempGraph(const std::vector<TopologyNode>& temp_nodes,
                                              const std::vector<TopologyEdge>& temp_edges,
                                              int start_id, int goal_id);
    nav_msgs::Path buildCompletePathFromTempGraph(const std::vector<TopologyNode>& temp_nodes,
                                                 const std::vector<int>& topology_path);
    
    // 可行性检查方法
    bool isConnectionFeasible(const geometry_msgs::Point& from, 
                             const geometry_msgs::Point& to);
    bool checkLineOfSight(const geometry_msgs::Point& from, 
                         const geometry_msgs::Point& to);
    bool isPointInFreeSpace(const geometry_msgs::Point& point);
    double calculateConnectionCost(const geometry_msgs::Point& from, 
                                  const geometry_msgs::Point& to);
    
    // 新的边合法性检查方法
    bool isEdgeLegal(const TopologyNode& node1, const TopologyNode& node2);
    void freeSpaceCloudCallback(const sensor_msgs::PointCloud2::ConstPtr& msg);
    bool isPointInFreeSpaceWithRadius(const geometry_msgs::Point& point, double radius);
    std::vector<geometry_msgs::Point> sampleEdgePoints(const geometry_msgs::Point& start, 
                                                       const geometry_msgs::Point& end, 
                                                       int sample_count);
};

} // namespace global_path_planner

#endif // GLOBAL_PATH_PLANNER_H
