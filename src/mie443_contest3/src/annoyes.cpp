#include "annoyes.h"
int turnLeftCount = 0;
int turnRightCount = 0;
bool isZigZagDetected = false;
double zigZagStartTime = 0;
double lastZigZagDetectionTime = 0;
double lastDirectionChangeTime = 0;
bool wasMovingLeft = false;
bool wasMovingRight = false;

const double ZIGZAG_TIME_WINDOW = 15.0;     //should be able to detect for consecutive change direction happen in 15s   
const double ANGULAR_THRESHOLD = 0.15;         
const int DIRECTION_CHANGES_THRESHOLD = 6; //if changes 3 times each side total
bool checkMovement(double timeElapsed,const geometry_msgs::Twist& cmd){
    bool isMovingRight = (cmd.angular.z > ANGULAR_THRESHOLD);
    bool isMovingLeft = (cmd.angular.z < -ANGULAR_THRESHOLD);
    
    
    if (zigZagStartTime == 0 && (isMovingLeft || isMovingRight)) {
        zigZagStartTime = timeElapsed;
        wasMovingLeft = isMovingLeft;
        wasMovingRight = isMovingRight;
        return false;
    }//start counting
    
    
    if ((isMovingRight && wasMovingLeft) || (isMovingLeft && wasMovingRight)) {
       
        if (isMovingRight) {
            turnRightCount++;
            ROS_INFO("Right turn detected, count: %d", turnRightCount);
        }
        if (isMovingLeft) {
            turnLeftCount++;
            ROS_INFO("Left turn detected, count: %d", turnLeftCount);
        }
        
      
        lastDirectionChangeTime = timeElapsed;
        
  
        int totalDirectionChanges = turnLeftCount + turnRightCount;
        double timeWindow = timeElapsed - zigZagStartTime;
        
        if (timeWindow <= ZIGZAG_TIME_WINDOW && totalDirectionChanges >= DIRECTION_CHANGES_THRESHOLD) {//detect after time threshold and direction threshold
            ROS_INFO("prob zigzag, %d direction changes in %.1f seconds",totalDirectionChanges, timeWindow);
            isZigZagDetected = true;
            lastZigZagDetectionTime = timeElapsed;
            return true;
        }
    }
    

    wasMovingLeft = isMovingLeft;
    wasMovingRight = isMovingRight;
    
    if (timeElapsed - zigZagStartTime > ZIGZAG_TIME_WINDOW) {
        ROS_INFO("Resetting zigzag detection, time window expired");
        turnLeftCount = 0;
        turnRightCount = 0;
        zigZagStartTime = (isMovingLeft || isMovingRight) ? timeElapsed : 0;//rest if no consecutive movement detect in certain period 
    }
    
    return false;
}

void zigzagBehaviour(sound_play::SoundClient &sc, ros::Publisher &vel_pub){
    bool taskComplete;
    int step = getStep();
    geometry_msgs::Twist vel;
    if (step == 0){
        savePos();
        takeStep();
        
    }
    else if (step == 1){
        sleep(2.0);

        //if (taskComplete){
            takeStep();
        //}
    }
    else if (step == 2){
		if(soundPlayed==false){
			sc.playWave("/home/thursday2023/catkin_ws/src/mie443_contest3/sounds/sound.wav");
			soundPlayed=true;
		}
        sleep(3);
		sc.stopWave("/home/thursday2023/catkin_ws/src/mie443_contest3/sounds/sound.wav");
		takeStep();
    }
    // else if (step == 3){
    //     if (bumpedPosition == "LEFT"){
    //         taskComplete = moveAngle(DEG2RAD(50), SLOW_ANGULAR, CW);
    //     }
    //     else if (bumpedPosition == "RIGHT"){
    //         taskComplete = moveAngle(DEG2RAD(50), SLOW_ANGULAR, CCW);
    //     }
    //     else {
    //         taskComplete = moveAngle(DEG2RAD(90), SLOW_ANGULAR, turnDirectionLaser);
    //     }

    //     if (taskComplete){
    //         takeStep();
    //     }
    //}
    else if(step==3){
        taskComplete=moveAngle(DEG2RAD(5), SLOW_ANGULAR, CW);
        if (taskComplete){
                     takeStep();
                }
    }
    else if(step==4){
        taskComplete=moveAngle(DEG2RAD(5), SLOW_ANGULAR, CCW);
        if (taskComplete){
                     takeStep();
                }
    }
    else {
        isZigZagDetected = false;
        turnLeftCount = 0;
        turnRightCount = 0;
        resetState();
    }
}


