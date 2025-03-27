

#ifndef ROBOT_CONFIG
#define ROBOT_CONFIG

#include <ros/console.h>
#include "ros/ros.h"
#include <string>

using namespace std;

#define RAD2DEG(rad) ((rad) * 180. / M_PI)
#define DEG2RAD(deg) ((deg) * M_PI / 180.)

// Robot Physical attributes
const uint8_t N_BUMPER = 3;

const int FORWARD = 1;
const int BACKWARD = -1;
const int CW = -1;
const int CCW = 1;

// Robot Set Speeds
const float SLOW_LINEAR = 0.15;
const float FAST_LINEAR = 0.18; // must not exceed 0.25m/s
const float SLOW_ANGULAR = M_PI/5;
const float FAST_ANGULAR = M_PI/4; // must not exceed PI/3rad/s



// Threshold for Distance from last 360 Rotation
const float NEW_SPACE_THRESHOLD = 2;

// creates a structure for angular and linear velociti
struct velS {
    float angular = 0.0;
    float linear = 0.0;
};
typedef struct velS velS;

struct posS {
    float x = 0.0;
    float y = 0.0;
    float yaw = 0.0;
};
typedef struct posS posS;

#endif



