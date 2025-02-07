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
    const double stuckTime=5;
    const double stuckDist=0.05;
    const double recoverVel=-0.2;
    const double recoverAngu=M_PI/2;

    stuckDetect();
    bool checkIfStuck(posS currPos, double timeElapsed);
    void stuckBehaviour();
    bool ifStuck(){
        return isStuck;
    }
  
    geometry_msgs::Twist getOdoRespond()const{
        return odo_response_cmd;
    }


    private:
   
    bool isStuck;
  
    ros::Time prevTime;
   
    ros::Time recoveryStart;
    ros::Time notMoving;
    bool startCounting;
    geometry_msgs::Twist odo_response_cmd;
 


};
#endif
