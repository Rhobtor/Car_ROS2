#include "iostream" 
#include "visualization_msgs/msg/marker.h" 
#include "visualization_msgs/msg/marker_array.hpp" 
#include "octomap_msgs/msg/octomap.h" 
#include "octomap_msgs/conversions.h" 
#include "octomap/OcTree.h" 
#include "rclcpp/rclcpp.hpp"
 
class visualization 
{ 
private: 
    visualization_msgs::msg::MarkerArray vector_visualizacion; 
    visualization_msgs::msg::Marker punto_visualizacion; 
 
public: 
    visualization(); 
    ~visualization(); 
 
    visualization_msgs::msg::Marker PointColor(octomap::point3d point, int iter, float r, 
float g, float b); 
    visualization_msgs::msg::Marker createLineMarker(octomap::point3d start, 
octomap::point3d end,int iteracion); 
    visualization_msgs::msg::Marker deleteAllPoints(); 
}; 
