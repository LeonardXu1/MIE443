#include "stuck.h"

stuckDetect::stuckDetect() : state(EXPLORE),
                             isStuck(false),
                             prevX(0),
                             prevY(0),
                             startCounting(false),
                             recoveryDir(1)
{
    prevTime = ros::Time(0);//initialize timer 
}

bool stuckDetect::checkIfStuck(double currentX, double currentY)
{
    int iter=0;
    bool sus=false;
    ros::Time currentTime = ros::Time::now();

    if (prevTime == ros::Time(0))
    {
        prevTime = currentTime;
        prevX = currentX;
        prevY = currentY;
        return false;//timer initialization
    }
    while(iter<=100){
    double distanceMoved = std::sqrt(std::pow(currentX - prevX, 2) + std::pow(currentY - prevY, 2));//Calculate disctance moved 
    if (distanceMoved <= stuckDist)
    {
        if (!startCounting)
        {
            startCounting = true;
            notMoving = currentTime;//start counting time when robot is moving in a small step 
            ROS_INFO("start not moving big");
            sus=true;
        if(isStuck==false&&sus==true)
        {
            double timeNoMoving = (currentTime - notMoving).toSec();
            if (timeNoMoving >= stuckTime && !isStuck)//if robot continuously moving small step for 2s(can be tuned), identify it might got stuck 
            {
                ROS_WARN("robot prob got stuck");
                isStuck = true;
                state = STUCK;
            }
        }
        }
        
        
    }
    else
        {
            startCounting = false;
            isStuck = false;
            state = EXPLORE;
        }

    //    double timeDiff=(currentTime-prevTime).toSec();

    //    if(timeDiff>stuckTime){
    //         double distanceMoved=std::sqrt(std::pow(currentX-prevX,2)+std::pow(currentY-prevY,2));
    //         if(distanceMoved>=stuckDist&&!isStuck){
    //             ROS_WARN("Robot is prob stuck");
    //             isStuck=true;
    //             state=STUCK;//set to stuck stage

    //         }

    prevX = currentX;
    prevY = currentY;
    prevTime = currentTime;
    iter++;
    ROS_INFO("%i",iter);
    // }

    return isStuck;
}
}
void stuckDetect::solveStuck(uint8_t stuckState)
{
    if (stuckState == STUCK)//adjustment if it is identified as stuck 
    {
        odo_response_cmd.linear.x = recoverVel;
        std::random_device rd;
        std::mt19937 gen(rd());
        std::uniform_int_distribution<int> dis(0, 1);
        int rand = dis(gen);
        odo_response_cmd.angular.z = (rand == 0) ? M_PI / 2 : -M_PI / 2;
        ROS_INFO("done adjusting");
        state = EXPLORE;
    }
    else
    {
        ROS_INFO("exit stuck state, to explore state");
        isStuck = false;
        state=EXPLORE;
    }
}
