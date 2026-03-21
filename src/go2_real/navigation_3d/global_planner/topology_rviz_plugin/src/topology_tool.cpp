#include "topology_rviz_plugin/topology_tool.h"

#include <rviz/viewport_mouse_event.h>
#include <rviz/visualization_manager.h>
#include <rviz/mesh_loader.h>
#include <rviz/geometry.h>
#include <rviz/properties/vector_property.h>

#include <OgreSceneNode.h>
#include <OgreSceneManager.h>
#include <OgreEntity.h>

#include <ros/console.h>
#include <tf/transform_listener.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/search/kdtree.h>

#include <fstream>
#include <sstream>
#include <iomanip>
#include <limits>
#include <locale>
#include <codecvt>

namespace topology_rviz_plugin
{

TopologyTool::TopologyTool()
    : current_mode_(MODE_ADD_NODE)
    , selected_node_id_(-1)
    , connecting_from_id_(-1)
    , last_added_node_id_(-1)
    , cloud_received_(false)
{
    shortcut_key_ = 't';
    current_cloud_.reset(new pcl::PointCloud<pcl::PointXYZ>);
    kdtree_.reset(new pcl::search::KdTree<pcl::PointXYZ>);
}

TopologyTool::~TopologyTool()
{
    if (server_)
    {
        server_->clear();
        server_->applyChanges();
    }
}

void TopologyTool::onInitialize()
{
    // 初始化ROS节点句柄
    nh_ = ros::NodeHandle();
    
    // 创建发布者
    marker_pub_ = nh_.advertise<visualization_msgs::MarkerArray>("topology_markers", 1, true);
    point_pub_ = nh_.advertise<geometry_msgs::PointStamped>("topology_points", 1);
    
    // 订阅点云数据
    pointcloud_sub_ = nh_.subscribe("/free_space_cloud", 1, &TopologyTool::pointcloudCallback, this);
    
    // 创建交互标记服务器
    server_.reset(new interactive_markers::InteractiveMarkerServer("topology_interactive_markers"));
    
    // 创建属性
    mode_property_ = new rviz::EnumProperty("Mode", "Add Node",
                                           "选择工具模式",
                                           getPropertyContainer(), SLOT(updateProperties()), this);
    mode_property_->addOption("Add Node", MODE_ADD_NODE);
    mode_property_->addOption("Connect Nodes", MODE_CONNECT_NODES);
    mode_property_->addOption("Edit Node", MODE_EDIT_NODE);
    mode_property_->addOption("Delete", MODE_DELETE);
    mode_property_->addOption("Auto Connect", MODE_AUTO_CONNECT);
    
    node_label_property_ = new rviz::StringProperty("Node Label", "节点",
                                                   "新节点的标签",
                                                   getPropertyContainer(), SLOT(updateProperties()), this);
    
    connection_distance_property_ = new rviz::FloatProperty("Max Connection Distance", 2.0,
                                                           "最大连接距离",
                                                           getPropertyContainer(), SLOT(updateProperties()), this);
    connection_distance_property_->setMin(0.1);
    connection_distance_property_->setMax(10.0);
    
    selection_threshold_property_ = new rviz::FloatProperty("Selection Threshold", 0.3,
                                                           "点选择精确度阈值（米）",
                                                           getPropertyContainer(), SLOT(updateProperties()), this);
    selection_threshold_property_->setMin(0.05);
    selection_threshold_property_->setMax(1.0);
    
    bidirectional_property_ = new rviz::BoolProperty("Bidirectional", true,
                                                    "连接是否双向",
                                                    getPropertyContainer(), SLOT(updateProperties()), this);
    
    save_file_property_ = new rviz::StringProperty("Save File", "topology_map.txt",
                                                  "保存文件路径",
                                                  getPropertyContainer(), SLOT(updateProperties()), this);
    
    // 添加控制按钮（通过属性实现）
    rviz::Property* control_group = new rviz::Property("Controls", QVariant(),
                                                       "控制操作",
                                                       getPropertyContainer());
    
    rviz::BoolProperty* clear_button = new rviz::BoolProperty("Clear All", false,
                                                             "清除所有节点和边",
                                                             control_group, SLOT(clearAll()), this);
    
    rviz::BoolProperty* save_button = new rviz::BoolProperty("Save", false,
                                                            "保存拓扑地图",
                                                            control_group, SLOT(saveTopology()), this);
    
    rviz::BoolProperty* load_button = new rviz::BoolProperty("Load", false,
                                                            "加载拓扑地图",
                                                            control_group, SLOT(loadTopology()), this);
    
    ROS_INFO("拓扑地图工具初始化完成");
}

void TopologyTool::activate()
{
    ROS_INFO("拓扑地图工具已激活 - 模式: %d", current_mode_);
}

void TopologyTool::deactivate()
{
    ROS_INFO("拓扑地图工具已停用");
}

int TopologyTool::processMouseEvent(rviz::ViewportMouseEvent& event)
{
    if (event.type == QEvent::MouseButtonPress)
    {
        return processMousePress(event);
    }
    else if (event.type == QEvent::MouseButtonRelease)
    {
        return processMouseRelease(event);
    }
    else if (event.type == QEvent::MouseMove)
    {
        return processMouseMove(event);
    }
    
    return Render;
}

int TopologyTool::processMousePress(rviz::ViewportMouseEvent& event)
{
    if (event.leftDown())
    {
        Ogre::Vector3 point;
        bool point_found = getPoint(event, point);
        
        switch (current_mode_)
        {
            case MODE_ADD_NODE:
            {
                if (point_found)
                {
                    addNode(point.x, point.y, point.z);
                }
                else
                {
                    ROS_WARN("无法在当前位置添加节点。请确保：");
                    ROS_WARN("1. 点云数据已正确加载");
                    ROS_WARN("2. 点击位置接近可行区域点云");
                    ROS_WARN("3. 点击的是点云可视化区域内的点");
                }
                break;
            }
                
            case MODE_CONNECT_NODES:
            {
                if (point_found)
                {
                    int nearest_id = findNearestNode(point.x, point.y, point.z, 
                                                   connection_distance_property_->getFloat());
                    if (nearest_id >= 0)
                    {
                        if (connecting_from_id_ == -1)
                        {
                            connecting_from_id_ = nearest_id;
                            ROS_INFO("选择起始节点: %d (%s)", nearest_id, nodes_[nearest_id].label.c_str());
                            updateVisualization(); // 更新可视化以高亮选中节点
                        }
                        else if (nearest_id != connecting_from_id_)
                        {
                            addEdge(connecting_from_id_, nearest_id);
                            connecting_from_id_ = -1;
                            ROS_INFO("连接完成");
                            updateVisualization(); // 更新可视化
                        }
                        else
                        {
                            ROS_INFO("不能连接节点到自身");
                        }
                    }
                    else
                    {
                        ROS_WARN("在指定范围内未找到节点，请点击更接近现有节点的位置");
                    }
                }
                break;
            }
                
            case MODE_EDIT_NODE:
            {
                if (point_found)
                {
                    int nearest_id = findNearestNode(point.x, point.y, point.z, 
                                                   connection_distance_property_->getFloat());
                    if (nearest_id >= 0)
                    {
                        selected_node_id_ = nearest_id;
                        createContextMenu(nearest_id);
                    }
                    else
                    {
                        ROS_WARN("在指定范围内未找到可编辑的节点");
                    }
                }
                break;
            }
                
            case MODE_DELETE:
            {
                if (point_found)
                {
                    int nearest_id = findNearestNode(point.x, point.y, point.z, 
                                                   connection_distance_property_->getFloat());
                    if (nearest_id >= 0)
                    {
                        removeNode(nearest_id);
                    }
                    else
                    {
                        ROS_WARN("在指定范围内未找到可删除的节点");
                    }
                }
                break;
            }
            
            case MODE_AUTO_CONNECT:
            {
                if (point_found)
                {
                    addNodeAutoConnect(point.x, point.y, point.z);
                }
                else
                {
                    ROS_WARN("无法在当前位置添加节点。请确保：");
                    ROS_WARN("1. 点云数据已正确加载");
                    ROS_WARN("2. 点击位置接近可行区域点云");
                    ROS_WARN("3. 点击的是点云可视化区域内的点");
                }
                break;
            }
        }
    }
    else if (event.rightDown())
    {
        // 右键取消当前操作
        if (connecting_from_id_ != -1)
        {
            ROS_INFO("取消连接操作");
            connecting_from_id_ = -1;
            updateVisualization(); // 更新可视化以移除高亮
        }
        
        // 在自动连接模式下，右键重置自动连接链
        if (current_mode_ == MODE_AUTO_CONNECT)
        {
            ROS_INFO("重置自动连接链，下一个节点将开始新的连接序列");
            last_added_node_id_ = -1;
        }
        
        selected_node_id_ = -1;
    }
    
    return Render;
}

int TopologyTool::processMouseRelease(rviz::ViewportMouseEvent& event)
{
    return Render;
}

int TopologyTool::processMouseMove(rviz::ViewportMouseEvent& event)
{
    return Render;
}

void TopologyTool::addNode(double x, double y, double z)
{
    int new_id = nodes_.size();
    std::string label = node_label_property_->getStdString();
    if (label.empty())
    {
        std::stringstream ss;
        ss << "节点_" << new_id;  // 使用中文默认标签
        label = ss.str();
    }
    
    // 将节点z轴抬高0.3m，适应狗的高度
    double adjusted_z = z + 0.3;
    TopologyNode new_node(new_id, x, y, adjusted_z, label);
    nodes_.push_back(new_node);
    
    // 创建交互标记
    createInteractiveMarker(new_node);
    
    // 发布点
    geometry_msgs::PointStamped point_msg;
    point_msg.header.frame_id = "map";
    point_msg.header.stamp = ros::Time::now();
    point_msg.point.x = x;
    point_msg.point.y = y;
    point_msg.point.z = adjusted_z;  // 使用抬高后的z坐标
    point_pub_.publish(point_msg);
    
    updateVisualization();
    
    ROS_INFO("添加节点 #%d: %s at (%.2f, %.2f, %.2f) [原始z: %.2f, 抬高0.3m后: %.2f]", 
             new_id, label.c_str(), x, y, adjusted_z, z, adjusted_z);
}

void TopologyTool::addNodeAutoConnect(double x, double y, double z)
{
    int new_id = nodes_.size();
    std::string label = node_label_property_->getStdString();
    if (label.empty())
    {
        std::stringstream ss;
        ss << "节点_" << new_id;  // 使用和普通节点相同的默认标签
        label = ss.str();
    }
    
    // 将节点z轴抬高0.3m，适应狗的高度
    double adjusted_z = z + 0.3;
    TopologyNode new_node(new_id, x, y, adjusted_z, label);
    nodes_.push_back(new_node);
    
    // 自动连接到上一个节点
    if (last_added_node_id_ >= 0 && last_added_node_id_ < static_cast<int>(nodes_.size()) - 1)
    {
        // 确保上一个节点存在
        bool bidirectional = bidirectional_property_->getBool();
        int new_edge_id = edges_.size();
        double weight = calculateDistance(nodes_[last_added_node_id_], new_node);
        
        TopologyEdge new_edge(new_edge_id, last_added_node_id_, new_id, weight, "auto", bidirectional);
        edges_.push_back(new_edge);
        
        ROS_INFO("自动连接: 节点 %d -> 节点 %d (距离: %.2f m, 双向: %s)", 
                 last_added_node_id_, new_id, weight, bidirectional ? "是" : "否");
    }
    else
    {
        ROS_INFO("第一个节点，无需连接");
    }
    
    // 更新最后添加的节点ID
    last_added_node_id_ = new_id;
    
    // 创建交互标记
    createInteractiveMarker(new_node);
    
    // 发布点
    geometry_msgs::PointStamped point_msg;
    point_msg.header.frame_id = "map";
    point_msg.header.stamp = ros::Time::now();
    point_msg.point.x = x;
    point_msg.point.y = y;
    point_msg.point.z = adjusted_z;  // 使用抬高后的z坐标
    point_pub_.publish(point_msg);
    
    updateVisualization();
    
    ROS_INFO("添加节点 #%d: %s at (%.2f, %.2f, %.2f) [原始z: %.2f, 抬高0.3m后: %.2f]", 
             new_id, label.c_str(), x, y, adjusted_z, z, adjusted_z);
}

void TopologyTool::addEdge(int from_id, int to_id)
{
    if (from_id < 0 || from_id >= static_cast<int>(nodes_.size()) ||
        to_id < 0 || to_id >= static_cast<int>(nodes_.size()))
    {
        ROS_WARN("无效的节点ID: %d -> %d", from_id, to_id);
        return;
    }
    
    // 检查是否已存在连接
    for (const auto& edge : edges_)
    {
        if ((edge.from_id == from_id && edge.to_id == to_id) ||
            (edge.bidirectional && edge.from_id == to_id && edge.to_id == from_id))
        {
            ROS_WARN("节点 %d 和 %d 之间已存在连接", from_id, to_id);
            return;
        }
    }
    
    int new_edge_id = edges_.size();
    double weight = calculateDistance(nodes_[from_id], nodes_[to_id]);
    bool bidirectional = bidirectional_property_->getBool();
    
    TopologyEdge new_edge(new_edge_id, from_id, to_id, weight, "manual", bidirectional);
    edges_.push_back(new_edge);
    
    updateVisualization();
    
    ROS_INFO("添加连接 #%d: 节点 %d -> 节点 %d (距离: %.2f m, 双向: %s)", 
             new_edge_id, from_id, to_id, weight, bidirectional ? "是" : "否");
}

void TopologyTool::removeNode(int node_id)
{
    if (node_id < 0 || node_id >= static_cast<int>(nodes_.size()))
    {
        return;
    }
    
    // 移除相关的边
    edges_.erase(std::remove_if(edges_.begin(), edges_.end(),
        [node_id](const TopologyEdge& edge) {
            return edge.from_id == node_id || edge.to_id == node_id;
        }), edges_.end());
    
    // 移除节点（注意：这会改变后续节点的ID）
    nodes_.erase(nodes_.begin() + node_id);
    
    // 更新边中的节点ID
    for (auto& edge : edges_)
    {
        if (edge.from_id > node_id) edge.from_id--;
        if (edge.to_id > node_id) edge.to_id--;
    }
    
    // 重新分配节点ID
    for (size_t i = 0; i < nodes_.size(); ++i)
    {
        nodes_[i].id = i;
    }
    
    // 重新分配边ID
    for (size_t i = 0; i < edges_.size(); ++i)
    {
        edges_[i].id = i;
    }
    
    // 更新自动连接状态
    if (last_added_node_id_ == node_id)
    {
        // 如果删除的是最后添加的节点，重置为前一个节点
        last_added_node_id_ = nodes_.empty() ? -1 : static_cast<int>(nodes_.size()) - 1;
    }
    else if (last_added_node_id_ > node_id)
    {
        // 如果删除的节点在最后添加节点之前，调整ID
        last_added_node_id_--;
    }
    
    updateVisualization();
    
    ROS_INFO("删除节点 #%d", node_id);
}

void TopologyTool::removeEdge(int edge_id)
{
    if (edge_id >= 0 && edge_id < static_cast<int>(edges_.size()))
    {
        edges_.erase(edges_.begin() + edge_id);
        
        // 重新分配边ID
        for (size_t i = 0; i < edges_.size(); ++i)
        {
            edges_[i].id = i;
        }
        
        updateVisualization();
        ROS_INFO("删除边 #%d", edge_id);
    }
}

void TopologyTool::createInteractiveMarker(const TopologyNode& node)
{
    visualization_msgs::InteractiveMarker int_marker;
    int_marker.header.frame_id = "map";
    int_marker.header.stamp = ros::Time::now();
    
    std::stringstream ss;
    ss << "node_" << node.id;
    int_marker.name = ss.str();
    int_marker.description = node.label;
    
    int_marker.pose.position.x = node.x;
    int_marker.pose.position.y = node.y;
    int_marker.pose.position.z = node.z;
    int_marker.pose.orientation.w = 1.0;
    
    // 创建控制器
    visualization_msgs::InteractiveMarkerControl control;
    control.always_visible = true;
    control.interaction_mode = visualization_msgs::InteractiveMarkerControl::BUTTON;
    
    // 添加标记
    visualization_msgs::Marker marker;
    marker.type = visualization_msgs::Marker::SPHERE;
    marker.scale.x = 0.3;
    marker.scale.y = 0.3;
    marker.scale.z = 0.3;
    marker.color.r = 0.0;
    marker.color.g = 1.0;
    marker.color.b = 0.0;
    marker.color.a = 0.8;
    
    control.markers.push_back(marker);
    int_marker.controls.push_back(control);
    
    server_->insert(int_marker);
    server_->setCallback(int_marker.name,
        boost::bind(&TopologyTool::processMarkerFeedback, this, _1));
    
    server_->applyChanges();
}

void TopologyTool::processMarkerFeedback(const visualization_msgs::InteractiveMarkerFeedbackConstPtr& feedback)
{
    ROS_INFO("交互标记反馈: %s", feedback->marker_name.c_str());
    
    if (feedback->event_type == visualization_msgs::InteractiveMarkerFeedback::BUTTON_CLICK)
    {
        // 从标记名称中提取节点ID
        std::string marker_name = feedback->marker_name;
        if (marker_name.find("node_") == 0)
        {
            int node_id = std::stoi(marker_name.substr(5));
            createContextMenu(node_id);
        }
    }
}

void TopologyTool::createContextMenu(int node_id)
{
    // 这里可以创建上下文菜单，暂时使用简单的日志输出
    ROS_INFO("节点 #%d 的上下文菜单", node_id);
    
    // 可以在这里添加QT对话框来编辑节点属性
    bool ok;
    QString current_label = QString::fromUtf8(nodes_[node_id].label.c_str());
    QString new_label = QInputDialog::getText(nullptr, 
                                             QString::fromUtf8("编辑节点"), 
                                             QString::fromUtf8("节点标签:"), 
                                             QLineEdit::Normal,
                                             current_label, &ok);
    if (ok && !new_label.isEmpty())
    {
        nodes_[node_id].label = new_label.toUtf8().toStdString();
        updateVisualization();
        ROS_INFO("节点 #%d 标签更新为: %s", node_id, nodes_[node_id].label.c_str());
    }
}

void TopologyTool::updateVisualization()
{
    publishNodes();
    publishEdges();
}

void TopologyTool::publishNodes()
{
    visualization_msgs::MarkerArray markers;
    
    for (size_t i = 0; i < nodes_.size(); ++i)
    {
        visualization_msgs::Marker marker;
        marker.header.frame_id = "map";
        marker.header.stamp = ros::Time::now();
        marker.ns = "topology_nodes";
        marker.id = i;
        marker.type = visualization_msgs::Marker::SPHERE;
        marker.action = visualization_msgs::Marker::ADD;
        
        marker.pose.position.x = nodes_[i].x;
        marker.pose.position.y = nodes_[i].y;
        marker.pose.position.z = nodes_[i].z;
        marker.pose.orientation.w = 1.0;
        
        marker.scale.x = 0.4;
        marker.scale.y = 0.4;
        marker.scale.z = 0.4;
        
        // 根据节点状态设置颜色
        if (static_cast<int>(i) == connecting_from_id_)
        {
            // 选中用于连接的节点 - 黄色
            marker.color.r = 1.0;
            marker.color.g = 1.0;
            marker.color.b = 0.0;
            marker.color.a = 1.0;
            marker.scale.x = 0.5; // 稍大一些
            marker.scale.y = 0.5;
            marker.scale.z = 0.5;
        }
        else if (static_cast<int>(i) == selected_node_id_)
        {
            // 选中用于编辑的节点 - 橙色
            marker.color.r = 1.0;
            marker.color.g = 0.5;
            marker.color.b = 0.0;
            marker.color.a = 1.0;
        }
        else
        {
            // 普通节点 - 蓝色
            marker.color.r = 0.0;
            marker.color.g = 0.8;
            marker.color.b = 1.0;
            marker.color.a = 0.9;
        }
        
        markers.markers.push_back(marker);
        
        // 添加文本标签
        visualization_msgs::Marker text_marker;
        text_marker.header.frame_id = "map";
        text_marker.header.stamp = ros::Time::now();
        text_marker.ns = "topology_labels";
        text_marker.id = i;
        text_marker.type = visualization_msgs::Marker::TEXT_VIEW_FACING;
        text_marker.action = visualization_msgs::Marker::ADD;
        
        text_marker.pose.position.x = nodes_[i].x;
        text_marker.pose.position.y = nodes_[i].y;
        text_marker.pose.position.z = nodes_[i].z + 0.5;
        text_marker.pose.orientation.w = 1.0;
        
        text_marker.scale.z = 0.3;
        text_marker.color.r = 1.0;
        text_marker.color.g = 1.0;
        text_marker.color.b = 1.0;
        text_marker.color.a = 1.0;
        
        // 显示节点ID和标签
        text_marker.text = std::to_string(nodes_[i].id) + ": " + nodes_[i].label;
        
        markers.markers.push_back(text_marker);
    }
    
    marker_pub_.publish(markers);
}

void TopologyTool::publishEdges()
{
    visualization_msgs::MarkerArray edge_markers;
    
    for (size_t i = 0; i < edges_.size(); ++i)
    {
        const auto& edge = edges_[i];
        
        if (edge.from_id >= 0 && edge.from_id < static_cast<int>(nodes_.size()) &&
            edge.to_id >= 0 && edge.to_id < static_cast<int>(nodes_.size()))
        {
            // 绘制连接线
            visualization_msgs::Marker edge_marker;
            edge_marker.header.frame_id = "map";
            edge_marker.header.stamp = ros::Time::now();
            edge_marker.ns = "topology_edges";
            edge_marker.id = i * 2; // 使用偶数ID for lines
            edge_marker.type = visualization_msgs::Marker::LINE_STRIP;
            edge_marker.action = visualization_msgs::Marker::ADD;
            
            geometry_msgs::Point p1, p2;
            p1.x = nodes_[edge.from_id].x;
            p1.y = nodes_[edge.from_id].y;
            p1.z = nodes_[edge.from_id].z + 0.1; // 稍微抬高避免与地面重叠
            
            p2.x = nodes_[edge.to_id].x;
            p2.y = nodes_[edge.to_id].y;
            p2.z = nodes_[edge.to_id].z + 0.1;
            
            edge_marker.points.push_back(p1);
            edge_marker.points.push_back(p2);
            
            edge_marker.scale.x = 0.08; // 线条宽度
            
            // 所有边都使用红色，不区分类型
            edge_marker.color.r = 1.0;
            edge_marker.color.g = 0.0;
            edge_marker.color.b = 0.0;
            edge_marker.color.a = 0.8;
            
            edge_markers.markers.push_back(edge_marker);
            
            // 如果是单向连接，添加箭头表示方向
            if (!edge.bidirectional)
            {
                visualization_msgs::Marker arrow_marker;
                arrow_marker.header.frame_id = "map";
                arrow_marker.header.stamp = ros::Time::now();
                arrow_marker.ns = "topology_arrows";
                arrow_marker.id = i * 2 + 1; // 使用奇数ID for arrows
                arrow_marker.type = visualization_msgs::Marker::ARROW;
                arrow_marker.action = visualization_msgs::Marker::ADD;
                
                // 计算箭头的位置（在连接线的中点）
                double mid_x = (p1.x + p2.x) / 2.0;
                double mid_y = (p1.y + p2.y) / 2.0;
                double mid_z = (p1.z + p2.z) / 2.0;
                
                arrow_marker.pose.position.x = mid_x;
                arrow_marker.pose.position.y = mid_y;
                arrow_marker.pose.position.z = mid_z;
                
                // 计算箭头的方向
                double dx = p2.x - p1.x;
                double dy = p2.y - p1.y;
                double dz = p2.z - p1.z;
                double norm = std::sqrt(dx*dx + dy*dy + dz*dz);
                
                if (norm > 0.001)
                {
                    dx /= norm;
                    dy /= norm;
                    dz /= norm;
                    
                    // 计算四元数方向
                    double yaw = std::atan2(dy, dx);
                    double pitch = std::asin(-dz);
                    
                    arrow_marker.pose.orientation.x = 0;
                    arrow_marker.pose.orientation.y = std::sin(pitch/2);
                    arrow_marker.pose.orientation.z = std::sin(yaw/2) * std::cos(pitch/2);
                    arrow_marker.pose.orientation.w = std::cos(yaw/2) * std::cos(pitch/2);
                }
                else
                {
                    arrow_marker.pose.orientation.w = 1.0;
                }
                
                arrow_marker.scale.x = 0.3; // 箭头长度
                arrow_marker.scale.y = 0.1; // 箭头宽度
                arrow_marker.scale.z = 0.1; // 箭头高度
                
                arrow_marker.color = edge_marker.color; // 使用相同的颜色
                
                edge_markers.markers.push_back(arrow_marker);
            }
            
            // 添加权重标签
            visualization_msgs::Marker weight_marker;
            weight_marker.header.frame_id = "map";
            weight_marker.header.stamp = ros::Time::now();
            weight_marker.ns = "edge_weights";
            weight_marker.id = i;
            weight_marker.type = visualization_msgs::Marker::TEXT_VIEW_FACING;
            weight_marker.action = visualization_msgs::Marker::ADD;
            
            // 在连接线中点显示权重
            weight_marker.pose.position.x = (p1.x + p2.x) / 2.0;
            weight_marker.pose.position.y = (p1.y + p2.y) / 2.0;
            weight_marker.pose.position.z = (p1.z + p2.z) / 2.0 + 0.2;
            weight_marker.pose.orientation.w = 1.0;
            
            weight_marker.scale.z = 0.2;
            weight_marker.color.r = 1.0;
            weight_marker.color.g = 1.0;
            weight_marker.color.b = 0.0;
            weight_marker.color.a = 0.8;
            
            // 显示权重（距离）
            std::ostringstream oss;
            oss << std::fixed << std::setprecision(2) << edge.weight << "m";
            weight_marker.text = oss.str();
            
            edge_markers.markers.push_back(weight_marker);
        }
    }
    
    marker_pub_.publish(edge_markers);
}

double TopologyTool::calculateDistance(const TopologyNode& n1, const TopologyNode& n2)
{
    double dx = n1.x - n2.x;
    double dy = n1.y - n2.y;
    double dz = n1.z - n2.z;
    return std::sqrt(dx * dx + dy * dy + dz * dz);
}

int TopologyTool::findNearestNode(double x, double y, double z, double max_distance)
{
    int nearest_id = -1;
    double min_distance = max_distance;
    
    for (size_t i = 0; i < nodes_.size(); ++i)
    {
        double distance = calculateDistance(TopologyNode(-1, x, y, z), nodes_[i]);
        if (distance < min_distance)
        {
            min_distance = distance;
            nearest_id = i;
        }
    }
    
    return nearest_id;
}

bool TopologyTool::getPoint(rviz::ViewportMouseEvent& event, Ogre::Vector3& point_out)
{
    // 只有在接收到可行区域点云数据时才允许选择点
    if (!cloud_received_ || current_cloud_->size() == 0)
    {
        ROS_WARN("未接收到可行区域点云数据，无法添加节点。请确保点云话题'/free_space_cloud'正在发布数据。");
        return false;
    }
    
    Ogre::Ray mouse_ray = event.viewport->getCamera()->getCameraToViewportRay(
        event.x / static_cast<float>(event.viewport->getActualWidth()),
        event.y / static_cast<float>(event.viewport->getActualHeight()));
    
    // 在点云中查找与鼠标射线最接近的精确点
    float min_distance_to_ray = std::numeric_limits<float>::max();
    pcl::PointXYZ best_point;
    bool found_valid_point = false;
    
    // 遍历所有点云点，找到与射线最接近的点
    for (size_t i = 0; i < current_cloud_->points.size(); ++i)
    {
        const auto& cloud_point = current_cloud_->points[i];
        
        // 跳过无效点
        if (!std::isfinite(cloud_point.x) || !std::isfinite(cloud_point.y) || !std::isfinite(cloud_point.z))
            continue;
            
        Ogre::Vector3 ogre_point(cloud_point.x, cloud_point.y, cloud_point.z);
        
        // 计算点到射线的距离
        Ogre::Vector3 ray_to_point = ogre_point - mouse_ray.getOrigin();
        float projection_length = ray_to_point.dotProduct(mouse_ray.getDirection());
        
        // 确保投影在射线的正方向上
        if (projection_length < 0.1f) // 至少10cm距离
            continue;
            
        Ogre::Vector3 closest_on_ray = mouse_ray.getOrigin() + projection_length * mouse_ray.getDirection();
        float distance_to_ray = ogre_point.distance(closest_on_ray);
        
        // 设置选择阈值，从属性中获取
        float selection_threshold = selection_threshold_property_->getFloat();
        
        if (distance_to_ray < selection_threshold && distance_to_ray < min_distance_to_ray)
        {
            min_distance_to_ray = distance_to_ray;
            best_point = cloud_point;
            found_valid_point = true;
        }
    }
    
    if (found_valid_point)
    {
        point_out.x = best_point.x;
        point_out.y = best_point.y;
        point_out.z = best_point.z;
        
        ROS_INFO("选择了可行区域点云中的点: (%.3f, %.3f, %.3f), 与射线距离: %.3f m", 
                 point_out.x, point_out.y, point_out.z, min_distance_to_ray);
        return true;
    }
    else
    {
        ROS_WARN("未在可行区域点云中找到合适的点。请点击更接近点云的位置。");
        return false;
    }
}

bool TopologyTool::findNearestCloudPoint(const Ogre::Vector3& ray_point, Ogre::Vector3& cloud_point)
{
    if (!cloud_received_ || current_cloud_->size() == 0)
    {
        return false;
    }
    
    pcl::PointXYZ search_point;
    search_point.x = ray_point.x;
    search_point.y = ray_point.y;
    search_point.z = ray_point.z;
    
    std::vector<int> point_indices(1);
    std::vector<float> point_distances(1);
    
    if (kdtree_->nearestKSearch(search_point, 1, point_indices, point_distances) > 0)
    {
        const auto& nearest_point = current_cloud_->points[point_indices[0]];
        cloud_point.x = nearest_point.x;
        cloud_point.y = nearest_point.y;
        cloud_point.z = nearest_point.z;
        return true;
    }
    
    return false;
}

void TopologyTool::pointcloudCallback(const sensor_msgs::PointCloud2ConstPtr& cloud_msg)
{
    // 转换点云格式
    pcl::fromROSMsg(*cloud_msg, *current_cloud_);
    
    // 更新KD树
    if (current_cloud_->size() > 0)
    {
        kdtree_->setInputCloud(current_cloud_);
        cloud_received_ = true;
        ROS_INFO_ONCE("接收到点云数据，共 %lu 个点", current_cloud_->size());
    }
}

void TopologyTool::updateProperties()
{
    current_mode_ = static_cast<ToolMode>(mode_property_->getOptionInt());
    ROS_INFO("工具模式更改为: %d", current_mode_);
}

void TopologyTool::clearAll()
{
    nodes_.clear();
    edges_.clear();
    server_->clear();
    server_->applyChanges();
    updateVisualization();
    
    // 重置状态
    connecting_from_id_ = -1;
    selected_node_id_ = -1;
    last_added_node_id_ = -1;
    
    ROS_INFO("清除所有节点和边，重置自动连接状态");
}

void TopologyTool::saveTopology()
{
    std::string filename = save_file_property_->getStdString();
    
    try
    {
        // 使用UTF-8编码打开文件
        std::ofstream file(filename, std::ios::out | std::ios::binary);
        if (!file.is_open())
        {
            ROS_ERROR("无法创建文件: %s", filename.c_str());
            return;
        }
        
        // 写入UTF-8 BOM（可选，但有助于确保正确识别编码）
        file << "\xEF\xBB\xBF";
        
        file << "# 拓扑地图文件 (UTF-8编码)\n";
        file << "# 格式: 节点ID X Y Z 标签\n";
        file << "# Topology Nodes\n";
        
        for (const auto& node : nodes_)
        {
            file << node.id << " " << std::fixed << std::setprecision(6) 
                 << node.x << " " << node.y << " " << node.z << " " 
                 << node.label << "\n";
        }
        
        file << "# 格式: 边ID 起始节点ID 结束节点ID 权重 类型 双向\n";
        file << "# Topology Edges\n";
        
        for (const auto& edge : edges_)
        {
            file << edge.id << " " << edge.from_id << " " << edge.to_id << " "
                 << std::fixed << std::setprecision(6) << edge.weight << " " 
                 << edge.type << " " << edge.bidirectional << "\n";
        }
        
        file.close();
        ROS_INFO("拓扑地图已保存到: %s", filename.c_str());
        ROS_INFO("保存了 %lu 个节点和 %lu 条边", nodes_.size(), edges_.size());
    }
    catch (const std::exception& e)
    {
        ROS_ERROR("保存拓扑地图时出错: %s", e.what());
    }
}

void TopologyTool::loadTopology()
{
    std::string filename = save_file_property_->getStdString();
    
    try
    {
        // 使用UTF-8编码打开文件
        std::ifstream file(filename, std::ios::in | std::ios::binary);
        if (!file.is_open())
        {
            ROS_ERROR("无法打开文件: %s", filename.c_str());
            return;
        }
        
        clearAll();
        
        std::string line;
        bool reading_nodes = false;
        bool reading_edges = false;
        
        // 跳过可能的UTF-8 BOM
        char bom[3];
        file.read(bom, 3);
        if (!(bom[0] == '\xEF' && bom[1] == '\xBB' && bom[2] == '\xBF'))
        {
            // 没有BOM，回到文件开头
            file.seekg(0);
        }
        
        while (std::getline(file, line))
        {
            if (line.empty() || line[0] == '#')
            {
                if (line.find("Topology Nodes") != std::string::npos)
                {
                    reading_nodes = true;
                    reading_edges = false;
                    continue;
                }
                else if (line.find("Topology Edges") != std::string::npos)
                {
                    reading_nodes = false;
                    reading_edges = true;
                    continue;
                }
                continue;
            }
            
            std::istringstream iss(line);
            
            if (reading_nodes)
            {
                int id;
                double x, y, z;
                std::string label;
                
                // 读取前四个数值字段
                if (iss >> id >> x >> y >> z)
                {
                    // 读取剩余部分作为标签（可能包含空格和中文）
                    std::string remaining;
                    std::getline(iss, remaining);
                    if (!remaining.empty() && remaining[0] == ' ')
                    {
                        label = remaining.substr(1); // 去掉前导空格
                    }
                    else
                    {
                        label = remaining;
                    }
                    
                    if (label.empty())
                    {
                        label = "节点_" + std::to_string(id);
                    }
                    
                    TopologyNode node(id, x, y, z, label);
                    nodes_.push_back(node);
                    createInteractiveMarker(node);
                }
            }
            else if (reading_edges)
            {
                int id, from_id, to_id;
                double weight;
                std::string type;
                bool bidirectional;
                
                if (iss >> id >> from_id >> to_id >> weight >> type >> bidirectional)
                {
                    TopologyEdge edge(id, from_id, to_id, weight, type, bidirectional);
                    edges_.push_back(edge);
                }
            }
        }
        
        file.close();
        updateVisualization();
        ROS_INFO("成功加载拓扑地图: %lu 个节点, %lu 条边", nodes_.size(), edges_.size());
    }
    catch (const std::exception& e)
    {
        ROS_ERROR("加载拓扑地图时出错: %s", e.what());
    }
}

} // namespace topology_rviz_plugin

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(topology_rviz_plugin::TopologyTool, rviz::Tool)
