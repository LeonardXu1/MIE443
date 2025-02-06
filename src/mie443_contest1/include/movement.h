#ifndef MOVEMENT
#define MOVEMENT

#include <geometry_msgs/Twist.h>
#include <nav_msgs/Odometry.h>
#include <tf/transform_datatypes.h>
#include <cmath>

#include "rConfig.h"

void odomCallback(const nav_msgs::Odometry::ConstPtr& msg);

posi getAbsPos();

void savePos();

velo getVelocity();

bool moveDistance(float distance, float speed, bool forward);

bool moveAngle(float angle, float speed, bool CW);

void moveLinearSpeed(float speed);

void moveAngularSpeed(float speed);

#endif
