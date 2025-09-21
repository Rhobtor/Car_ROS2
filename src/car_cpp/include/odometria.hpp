#include "vector" 
#include "nav_msgs/msg/odometry.hpp" 
#include "octomap_msgs/msg/octomap.h" 
#include "octomap_msgs/conversions.h" 
 
class odometria 
{ 
private: 
 
    double x_odom; 
    double y_odom; 
    double z_odom; 
    double qx_odom; 
    double qy_odom; 
    double qz_odom; 
    double qw_odom; 
 
    double linear; 
    double angular; 
 
public: 
    odometria(); 
    ~ odometria(); 
 
    bool set_puntoOdom(const std::shared_ptr<const nav_msgs::msg::Odometry>& odometry,
                   std::vector<double>& posicion,
                   std::vector<double>& orientacion);
    void set_velOdom(const std::shared_ptr<nav_msgs::msg::Odometry>& odometry,
                     std::vector<double>& velocidad);
}; 