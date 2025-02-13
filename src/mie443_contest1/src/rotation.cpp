#include "../include/rotation.h"

posS prevRotationPos;

bool isNewSpace(){
    posS curPos = getAbsPos();
    float travel = calcDistance(prevRotationPos, curPos);
    if(travel >= NEW_SPACE_THRESHOLD){
        ROS_INFO("new space travel: %f", travel);
        return true;
    }

    return false;
}

void rotateBehaviour(){
    bool taskComplete;
    int step = getStep();

    if(step == 0){
        savePos();
        takeStep();
    }
    else if(step == 1){
        taskComplete = moveAngle(M_PI, FAST_ANGULAR, CW);

        if(taskComplete){
            takeStep();
        }
    }
    else if(step == 2){
        savePos();
        takeStep();
    }
    else if(step == 3){
        taskComplete = moveAngle(M_PI, FAST_ANGULAR, CW);

        if (taskComplete){
            takeStep();
        }
    }
    else {    
        prevRotationPos = getAbsPos();
        resetState();
    }
}