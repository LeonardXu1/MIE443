#ifndef LASER_DATA_PROCESSING
#define LASER_DATA_PROCESSING

#include <geometry_msgs/Twist.h>
#include <sensor_msgs/LaserScan.h>
#include <nav_msgs/Odometry.h>
#include <cmath>
#include "rConfig.h"
#include "movement.h"
#include "stateMachine.h"


struct LaserData {
    float angle_min;
    float angle_max;
    float angle_increment;
    float range_min;
    float range_max;
    int nLasers;
    std::vector<float> ranges; // Stores all laser distance readings
};

void laserCallback(const sensor_msgs::LaserScan::ConstPtr& msg);
void processLaserData();
void scanningBehaviour();

#endif
