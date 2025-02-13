#ifndef STUCK_H
#include <ros/console.h>
#include "ros/ros.h"
#include <geometry_msgs/Twist.h>
#include <nav_msgs/Odometry.h>
#include <tf/transform_datatypes.h>
#include <stdio.h>
#include <cmath>
#include <chrono>
#include <iostream>
#include <random>
#include "stateMachine.h"
#include "movement.h"
#include "bumper.h"
#include "rConfig.h"
#include "rotation.h"
bool checkIfStuck(posS currPos, double timeElapsed);
void stuckBehaviour();
bool isStuckBehaviourComplete();
void resetStuckCompletion();
 
#endif
