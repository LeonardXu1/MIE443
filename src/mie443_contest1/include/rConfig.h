#ifndef ROBOT_CONFIG
#define ROBOT_CONFIG

#include <ros/console.h>
#include "ros/ros.h"
#include <string>

#define RAD2DEG(rad) ((rad) * 180. / M_PI)
#define DEG2RAD(deg) ((deg) * M_PI / 180.)

// Robot Physical attributes
const uint8_t N_BUMPER = 3;

// defines the possible robot states and priority
enum state {BUMPER_STATE = 0, EXPLORE_STATE = 1, CYCLE_STATE = 5};
const std::string stateName[] = {"BUMPER_STATE", "EXPLORE_STATE", "CYCLE_STATE"};


// Robot Set Speeds
const float SLOW_LINEAR = 0.1;
const float FAST_LINEAR = 0.15; // must not exceed 0.25m/s
const float SLOW_ANGULAR = M_PI/6;
const float FAST_ANGULAR = M_PI/3; // must not exceed PI/3rad/s

// Robot Set Distances
const float WALL_CAUTION_DISTANCE = 0.1;

// creates a structure for angular and linear velociti
struct velo {
    float angular = 0.0;
    float linear = 0.0;
};
typedef struct velo velo;

struct posi {
    float x = 0.0;
    float y = 0.0;
    float yaw = 0.0;
};
typedef struct posi posi;

#endif

