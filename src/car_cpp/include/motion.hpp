#ifndef MOTION_HPP
#define MOTION_HPP

#include "geometry_msgs/msg/twist.hpp"

class motion {
public:
    motion();
    ~motion();

    geometry_msgs::msg::Twist motionControlAngle(double distance, double angle);
    geometry_msgs::msg::Twist motionControlDistance(double distance, double angle);
    geometry_msgs::msg::Twist motionControlSTOP();
};

#endif // MOTION_HPP
