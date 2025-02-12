#include "../include/rotation.h"



bool rotationComplete=false;
posS prevStatePos;
double rotThresholdDist=1.5;
double distanceMoved;
bool firstState=true;
bool isRotationComplete(){
    return rotationComplete; 
}
void resetRotation(){
    rotationComplete=false;
}
void saveStatePos(posS pos){
   if(firstState){
    firstState=false;
    prevStatePos=pos;
    return;
   }
   prevStatePos=pos;
}

bool shouldRotate(posS currentStatePos){
    if(firstState){
        return false;
    }
    
    distanceMoved = std::sqrt(std::pow(currentStatePos.x - prevStatePos.x, 2) + std::pow(currentStatePos.y - prevStatePos.y, 2));//Calculate disctance moved 
    ROS_INFO("moved distance: %lf",distanceMoved);
    if(distanceMoved>=rotThresholdDist){
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
    else if(step == 1) {
        taskComplete = moveAngle(M_PI, FAST_ANGULAR, CW);
       
        if(taskComplete){
            takeStep();
        }
    }
     else if(step == 2) {
            savePos();
            takeStep();
        
    }
    else if(step == 3){
        taskComplete = moveAngle(M_PI, FAST_ANGULAR, CW);
            if (taskComplete) {
                takeStep();
                ROS_INFO("Rotation complete");
              firstState=true;
              prevStatePos=getAbsPos();
              saveStatePos(getAbsPos());
                
            }
    }
    else {
        ROS_INFO("RESET STATE");
      
        rotationComplete=true;
        resetBumperCompletion();
        resetStuckCompletion();
        resetState();
    }
}