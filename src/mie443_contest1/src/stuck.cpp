#include "../include/stuck.h"


bool stuckDetect::checkIfStuck(posS currPos, double timeElapsed)
{

   
    // ros::Time currentTime = ros::Time::now();

    // if (prevTime == ros::Time(0))
    // {
    //     prevTime = currentTime;
    //     prevPos.x=currPos.x;
    //     prevPos.y=currPos.y;
    //     return false;//timer initialization
    // }
    isStuck=false;
    double distanceMoved = std::sqrt(std::pow(currPos.x - prevPos.x, 2) + std::pow(currPos.y - prevPos.y, 2));//Calculate disctance moved 
    ROS_INFO("moving distance: %lf",distanceMoved);
   
    if(std::abs(prevPos.x-currPos.x)<=stuckDist&&std::abs(prevPos.y-currPos.y)<=stuckDist)
    {
        if (startCounting==false)
        {
           
            notMovingTime = timeElapsed;//start counting time when robot is moving in a small step 
            ROS_INFO("start not moving big at %lf",notMovingTime);
            sus=true;
            startCounting=true;
        }
        if(isStuck==false&&sus==true&&startCounting==true)
        { 
           
            double timeNoMoving = (timeElapsed - notMovingTime);
            ROS_INFO("time not moving %lf",timeNoMoving);
            if (timeNoMoving >= stuckTime)//if robot continuously moving small step for 2s(can be tuned), identify it might got stuck 
            {
                ROS_WARN("robot prob got stuck");
                isStuck = true;
                startCounting=false;//stop counting once identified stuck 
                //return true;
            }
        }
        
        
        
    }
    else
        {
           startCounting = false;
            isStuck = false;
            //return false;
            
        }

    //    double timeDiff=(currentTime-prevTime).toSec();

    //    if(timeDiff>stuckTime){
    //         double distanceMoved=std::sqrt(std::pow(currentX-prevX,2)+std::pow(currentY-prevY,2));
    //         if(distanceMoved>=stuckDist&&!isStuck){
    //             ROS_WARN("Robot is prob stuck");
    //             isStuck=true;
    //             state=STUCK;//set to stuck stage

    //         }

    prevPos=currPos;
    //prevTime = currentTime;
    
   
    // }

    return isStuck;

}
void stuckDetect::stuckBehaviour()
{
    bool taskComplete;
    int step = getStep();
    if(step == 0){
        savePos();
        takeStep();
    }
    else if(step == 1) {
        taskComplete = moveDistance(0.1, SLOW_LINEAR, BACKWARD);
        if(taskComplete){
            takeStep();
        }
    }
    else if(step == 2){
        savePos();
        takeStep();
    }
    else if(step == 3) {
         
            std::random_device rd;
            std::mt19937 gen(rd());
            std::uniform_int_distribution<int>dis(-1,1);
            int randomNum=dis(gen);
           while(randomNum==0){
            //randomNum=dis(gen);
            if(randomNum!=0){
                break;
            }
            else{
                randomNum=dis(gen);
            }
            }
            ROS_INFO("%i",randomNum);
            taskComplete = moveAngle(M_PI/2, SLOW_ANGULAR, randomNum);
            ROS_INFO("Recovered from stuck state");
            takeStep();
            if(taskComplete){
            ROS_INFO("TAKING NEXT STEP");
           
        }
    }
    else {
        resetState();
        
    }
}
