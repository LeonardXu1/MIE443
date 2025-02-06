#ifndef MOVEMENT
#define MOVEMENT

#include <geometry_msgs/Twist.h>
#include <nav_msgs/Odometry.h>
#include <tf/transform_datatypes.h>
#include <cmath>

#include "rConfig.h"

void odomCallback(const nav_msgs::Odometry::ConstPtr& msg);

posS getAbsPos();

void savePos();

velS getVelocity();

bool moveDistance(float distance, float speed, int direction);

bool moveAngle(float angle, float speed, int direction);

void moveLinearSpeed(float speed, int direction);

void moveAngularSpeed(float speed, int direction);

#endif
