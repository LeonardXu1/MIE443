
#ifndef ROBOT_CONFIG
#define ROBOT_CONFIG

#include <ros/console.h>
#include "ros/ros.h"

#define RAD2DEG(rad) ((rad) * 180. / M_PI)
#define DEG2RAD(deg) ((deg) * M_PI / 180.)

// Robot Physical attributes
const uint8_t N_BUMPER = 3;

// defines the possible robot states and priority
enum state {STUCK_STATE=0,BUMPER_STATE = 1, EXPLORE_STATE = 2, CYCLE_STATE = 3};
const std::string stateName[] = {"STUCK_STATE","BUMPER_STATE", "EXPLORE_STATE", "CYCLE_STATE"};

const int FORWARD = 1;
const int BACKWARD = -1;
const int CW = -1;
const int CCW = 1;

// Robot Set Speeds
const float SLOW_LINEAR = 0.1;
const float FAST_LINEAR = 0.25; // must not exceed 0.25m/s
const float SLOW_ANGULAR = M_PI/3;
const float FAST_ANGULAR = M_PI/6; // must not exceed PI/6rad/s

// Robot Set Distances
const float WALL_CAUTION_DISTANCE = 0.1;

#endif


