#include "../include/bumper.h"
uint8_t bumper[3] = {kobuki_msgs::BumperEvent::RELEASED, kobuki_msgs::BumperEvent::RELEASED, kobuki_msgs::BumperEvent::RELEASED};
string bumpedPosition;
int turnDirectionLaser;

void bumperCallback(const kobuki_msgs::BumperEvent::ConstPtr &msg)
{
    bumper[msg->bumper] = msg->state;

    string position = bumperPressedPosition();

    if(position != "NONE"){
        bumpedPosition = position;
    }
}

string bumperPressedPosition()
{
    string position;
    if (bumper[kobuki_msgs::BumperEvent::LEFT] == kobuki_msgs::BumperEvent::PRESSED)
    {
        position = "LEFT";
        return position;
    }
    if (bumper[kobuki_msgs::BumperEvent::RIGHT] == kobuki_msgs::BumperEvent::PRESSED)
    {
        position = "RIGHT";
        return position;
    }
    if (bumper[kobuki_msgs::BumperEvent::CENTER] == kobuki_msgs::BumperEvent::PRESSED)
    {
        position = "CENTER";
        return position;
    }
    return "NONE";
}

bool isBumperPressed()
{
    for (uint32_t b_idx = 0; b_idx < N_BUMPER; ++b_idx)
    {
        if (bumper[b_idx] == kobuki_msgs::BumperEvent::PRESSED)
        {
            return true;
        }
    }
    return false;
}

void bumperBehaviour()
{
    bool taskComplete;
    int step = getStep();

    if (step == 0){
        savePos();
        takeStep();
        ROS_INFO("%s Bumper Pressed", bumpedPosition.c_str());
    }
    else if (step == 1){
        taskComplete = moveDistance(0.1, SLOW_LINEAR, BACKWARD);

        if (taskComplete){
            takeStep();
        }
    }
    else if (step == 2){
        savePos();
        takeStep();
        turnDirectionLaser = minLaserDirection();

        if(turnDirectionLaser == CW){
            ROS_INFO("Turning CW");
        }
        else{
            ROS_INFO("Turning CCW");
        }
        
    }
    else if (step == 3){
        if (bumpedPosition == "LEFT"){
            taskComplete = moveAngle(DEG2RAD(50), SLOW_ANGULAR, CW);
        }
        else if (bumpedPosition == "RIGHT"){
            taskComplete = moveAngle(DEG2RAD(50), SLOW_ANGULAR, CCW);
        }
        else {
            taskComplete = moveAngle(DEG2RAD(90), SLOW_ANGULAR, turnDirectionLaser);
        }

        if (taskComplete){
            takeStep();
        }
    }
    else {
        resetState();
    }
}