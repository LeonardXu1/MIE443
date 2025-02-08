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
class stuckDetect{
    public:
    stuckDetect():
    isStuck(false),
    notMovingTime(0),
    startCounting(false){
        prevPos.x=0;
        prevPos.y=0;
    }
    bool checkIfStuck(posS currPos, double timeElapsed);
    void stuckBehaviour();

    private:
    const double stuckTime=8;
    const double stuckDist=0.005;
    bool isStuck;
    posS prevPos;
    double notMovingTime;
    bool startCounting;
    bool sus=false;
    

};
#endif
