#ifndef TOPOLOGY_TOOL_H
#define TOPOLOGY_TOOL_H

#include <rviz/tool.h>
#include <rviz/properties/string_property.h>
#include <rviz/properties/bool_property.h>
#include <rviz/properties/float_property.h>
#include <rviz/properties/enum_property.h>
#include <rviz/viewport_mouse_event.h>

#include <OGRE/OgreVector3.h>
#include <OGRE/OgreRay.h>

#include <ros/ros.h>
#include <geometry_msgs/PointStamped.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>
#include <interactive_markers/interactive_marker_server.h>
#include <interactive_markers/menu_handler.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/search/kdtree.h>

#include <QObject>
#include <QInputDialog>
#include <QMessageBox>

namespace topology_rviz_plugin
{

struct TopologyNode
{
    int id;
    double x, y, z;
    std::string label;
    
    TopologyNode(int _id, double _x, double _y, double _z, const std::string& _label = "")
        : id(_id), x(_x), y(_y), z(_z), label(_label) {}
};

struct TopologyEdge
{
    int id;
    int from_id;
    int to_id;
    double weight;
    std::string type;
    bool bidirectional;
    
    TopologyEdge(int _id, int _from, int _to, double _weight, const std::string& _type = "normal", bool _bidir = true)
        : id(_id), from_id(_from), to_id(_to), weight(_weight), type(_type), bidirectional(_bidir) {}
};

class TopologyTool : public rviz::Tool
{
    Q_OBJECT

public:
    TopologyTool();
    virtual ~TopologyTool();

    virtual void onInitialize();
    virtual void activate();
    virtual void deactivate();

    virtual int processMouseEvent(rviz::ViewportMouseEvent& event);

private Q_SLOTS:
    void updateProperties();
    void clearAll();
    void saveTopology();
    void loadTopology();

private:
    // 鼠标事件处理
    int processMousePress(rviz::ViewportMouseEvent& event);
    int processMouseRelease(rviz::ViewportMouseEvent& event);
    int processMouseMove(rviz::ViewportMouseEvent& event);

    // 节点和边管理
    void addNode(double x, double y, double z);
    void addNodeAutoConnect(double x, double y, double z);  // 自动连接模式下添加节点
    void addEdge(int from_id, int to_id);
    void removeNode(int node_id);
    void removeEdge(int edge_id);
    
    // 可视化
    void updateVisualization();
    void publishNodes();
    void publishEdges();
    
    // 交互标记相关
    void createInteractiveMarker(const TopologyNode& node);
    void processMarkerFeedback(const visualization_msgs::InteractiveMarkerFeedbackConstPtr& feedback);
    void createContextMenu(int node_id);
    
    // 工具函数
    double calculateDistance(const TopologyNode& n1, const TopologyNode& n2);
    int findNearestNode(double x, double y, double z, double max_distance = 1.0);
    bool getPoint(rviz::ViewportMouseEvent& event, Ogre::Vector3& point_out);
    bool findNearestCloudPoint(const Ogre::Vector3& ray_point, Ogre::Vector3& cloud_point);
    void pointcloudCallback(const sensor_msgs::PointCloud2ConstPtr& cloud_msg);
    
    // ROS相关
    ros::NodeHandle nh_;
    ros::Publisher marker_pub_;
    ros::Publisher point_pub_;
    ros::Subscriber pointcloud_sub_;
    
    // 点云数据
    pcl::PointCloud<pcl::PointXYZ>::Ptr current_cloud_;
    pcl::search::KdTree<pcl::PointXYZ>::Ptr kdtree_;
    bool cloud_received_;
    
    // 交互标记服务器
    std::shared_ptr<interactive_markers::InteractiveMarkerServer> server_;
    interactive_markers::MenuHandler menu_handler_;
    
    // 数据存储
    std::vector<TopologyNode> nodes_;
    std::vector<TopologyEdge> edges_;
    
    // 状态变量
    enum ToolMode {
        MODE_ADD_NODE,
        MODE_CONNECT_NODES,
        MODE_EDIT_NODE,
        MODE_DELETE,
        MODE_AUTO_CONNECT
    };
    ToolMode current_mode_;
    int selected_node_id_;
    int connecting_from_id_;
    int last_added_node_id_;  // 用于自动连接模式，记录上一个添加的节点ID
    
    // 属性
    rviz::EnumProperty* mode_property_;
    rviz::StringProperty* node_label_property_;
    rviz::FloatProperty* connection_distance_property_;
    rviz::FloatProperty* selection_threshold_property_;
    rviz::BoolProperty* bidirectional_property_;
    rviz::StringProperty* save_file_property_;
};

} // namespace topology_rviz_plugin

#endif // TOPOLOGY_TOOL_H
