#ifndef LASER_DATA_PROCESSING
#define LASER_DATA_PROCESSING

#include <geometry_msgs/Twist.h>
#include <sensor_msgs/LaserScan.h>
#include <nav_msgs/Odometry.h>
#include <cmath>

struct LaserData {
    float angle_min;
    float angle_max;
    float angle_increment;
    float range_min;
    float range_max;
    int nLasers;
    std::vector<float> ranges;  // Stores all laser distance readings
};

LaserData extractLaserData(const sensor_msgs::LaserScan::ConstPtr& msg)
void processLaserData(const sensor_msgs::LaserScan::ConstPtr& msg)
void scanningBehaviour();

void 
