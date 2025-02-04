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

class stuckDetect{
    public:
    const double stuckTime=2;
    const double stuckDist=0.05;
    const double recoverVel=-0.2;
    const double recoverAngu=M_PI/2;
    enum State{
        EXPLORE=0,
        BUMPER=1,
        STUCK=2
    };
    stuckDetect();
    bool checkIfStuck(double currentX, double currentY);
    void solveStuck(u_int8_t stuckState);
    bool ifStuck(){
        return isStuck;
    }
    u_int8_t getState(){
        return state;
    }
    geometry_msgs::Twist getOdoRespond()const{
        return odo_response_cmd;
    }


    private:
    uint8_t state;
    bool isStuck;
    double prevX;
    double prevY;
    ros::Time prevTime;
    int recoveryDir;
    ros::Time recoveryStart;
    ros::Time notMoving;
    bool startCounting;
    geometry_msgs::Twist odo_response_cmd;
 


};
#endif
