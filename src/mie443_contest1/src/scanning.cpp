
// function to extract all the important laserscan data which we'll be able to reference later easily for calculations
#include "../include/scanning.h"

LaserData dat;
float maxLaserDist;
float maxLaserAngle;
float finalLaserDist;
float finalLaserAngle;

float obsLaserAngle;
float obsLaserDist;

void laserCallback(const sensor_msgs::LaserScan::ConstPtr& msg)
{   
    dat.angle_min = msg->angle_min;
    dat.angle_max = msg->angle_max;
    dat.angle_increment = msg->angle_increment;
    // dat.range_min = msg->range_min;              don't think we'll be using these, they are the distance values corresponding to the max and min angle of field of view
    // dat.range_max = msg->range_max;
    
    // Compute number of lasers
    dat.nLasers = (msg->angle_max - msg->angle_min) / msg->angle_increment;
    
    // Store all laser readings
    dat.ranges = msg->ranges;  // Directly copying the vector
}

// Processing Laser Data 
void processLaserData()
{   
    // std::vector<float> laserDistVect;                                            //need to work on this to cycle through a sorted array of idx and distances
    // for (uint32_t laser_idx = 0; laser_idx < nLasers; ++ laser_idx){
    //     laserDistVect.push_back(msg->ranges[laser_idx])
    // }
    
    maxLaserDist = 0;
    maxLaserAngle = 0.0;
    int maxLaserIdx = -1;               //set to -1 to be a placeholder for "no valid index"
    float robotWidth = .4;               //  must change this depending on units and actual width. also currently unsure if rounding will be an error 
    
    // Variables for obstacle detection and path compensation
    int rightObsLaserIdx;
    obsLaserAngle = 0;
    obsLaserDist = 0;
    int leftObsLaserIdx;
    float obsClearance = robotWidth / 2 + 0.1;
    
    //testing can delete later
    float leftLaser, rightLaser, LLlaser;

    ROS_INFO("Laser scan size: %lu", dat.ranges.size());
    ROS_INFO("0 index distance:  %0.2f  | 640 index distance %0.2f", dat.ranges[0], dat.ranges[630]);

    //test to print all laser data values
    // ROS_INFO("Printing dat.ranges:");
    // for (size_t i = 0; i < dat.ranges.size(); i++) {
    //     ROS_INFO("dat.ranges[%lu] = %0.2f", i, dat.ranges[i]);
    // }

    for (uint32_t laser_idx = 0; laser_idx < dat.nLasers; ++ laser_idx){        //cycle through all laser distances and store the max distance and corresponding index, as well as index's angle
        if (dat.ranges[laser_idx] > maxLaserDist){
            maxLaserDist = dat.ranges[laser_idx];
            maxLaserIdx = laser_idx;
            maxLaserAngle = dat.angle_min + maxLaserIdx * dat.angle_increment;
            
            //testing
            leftLaser = dat.ranges[maxLaserIdx + 1];
            LLlaser = dat.ranges[maxLaserIdx + 4];

            rightLaser = dat.ranges[maxLaserIdx - 1];

        }
    }
    //Print the distances around maxlaser index
    for (int i = maxLaserIdx - 10; i <= maxLaserIdx + 10; ++i) {
        if (i >= 0 && i < dat.ranges.size()) {  // Check if within bounds
            ROS_INFO("dat.ranges[%d] = %0.2f", i, dat.ranges[i]);
        }
    }

    ROS_INFO("maxLaserIdx: %d | maxLaserDist: %0.2f | leftdist %0.2f | rightdist %0.2f  | more left dist %0.2f", maxLaserIdx, maxLaserDist, leftLaser, rightLaser, LLlaser);
    
    //RIGHT OBS CHECK --> First check value is not NaN
    rightObsLaserIdx = maxLaserIdx - 1;
    ROS_INFO("right wall indx %d", rightObsLaserIdx);
    while (std::isnan(dat.ranges[rightObsLaserIdx]) && rightObsLaserIdx > 0) {
        rightObsLaserIdx--; // Move to the previous index
    }
    ROS_INFO("right wall indx %d", rightObsLaserIdx);


    //LEFT OBS CHECK --> NaN Check
    leftObsLaserIdx = maxLaserIdx + 1;
    ROS_INFO("left wall indx %d", leftObsLaserIdx);
    while (std::isnan(dat.ranges[leftObsLaserIdx]) && leftObsLaserIdx < dat.nLasers) {
        leftObsLaserIdx++; // Move to the previous index
    }
    ROS_INFO("left wall indx %d", leftObsLaserIdx);

    
    //RIGHT OBS CHECK --> then complete calc with valid value
    if ((dat.ranges[maxLaserIdx] - dat.ranges[rightObsLaserIdx] > 0.2 )) {       // in other words: if the obstacle is on the right of the maxlaserdist 
    // if (dat.ranges[maxLaserIdx] + dat.ranges[maxLaserIdx - 1] > 0.2 ){
        float test_= dat.ranges[maxLaserIdx] - dat.ranges[maxLaserIdx - 1];
        ROS_INFO("obstacle on right");
        obsLaserDist = dat.ranges[rightObsLaserIdx];
        ROS_INFO("obstacle idx %0.2d", rightObsLaserIdx);            //TEST

        obsLaserAngle = atan(obsClearance / obsLaserDist);
        finalLaserAngle = maxLaserAngle + obsLaserAngle;                        //this is the case for both positive and negative maxlaserAngle cases. 
    
        ROS_INFO("max angle: %0.2f  | correction angle: %0.2f   |   final angle: %0.2f", maxLaserAngle, obsLaserAngle, finalLaserAngle);
    }


    //LEFT OBS CHECK --> Calcs
    else if ((dat.ranges[maxLaserIdx] - dat.ranges[leftObsLaserIdx] > 0.2 )){          // in other words: if the obstacle is on the left of the maxlaserdist 
        ROS_INFO("obstacle on left");                           //TEST
        
        obsLaserDist = dat.ranges[leftObsLaserIdx];
        
        ROS_INFO("obstacle idx %0.2d", leftObsLaserIdx);            //TEST

        obsLaserAngle = atan(obsClearance / obsLaserDist);
        finalLaserAngle = maxLaserAngle - obsLaserAngle;                        //this is the case for both positive and negative maxlaserAngle cases. 

        ROS_INFO("max angle: %0.2f  | correction angle: %0.2f   |   final angle: %0.2f", maxLaserAngle, obsLaserAngle, finalLaserAngle);

    }

    else{
        finalLaserAngle = maxLaserAngle;
        }
}

void scanningBehaviour(){
    bool taskComplete;
    int step = getStep();
    if(step == 0){
        if (dat.ranges.size() != 0){
            takeStep();
        }
    }

    else if(step == 1){
        processLaserData();
        ROS_INFO("max angle: %0.2f  | correction angle: %0.2f   |   final angle: %0.2f", maxLaserAngle, obsLaserAngle, finalLaserAngle);
        takeStep();
    }
    else if(step == 2){
        savePos();
        takeStep();
    }
    else if(step ==  3){
        if (maxLaserAngle < 0){
            taskComplete = moveAngle(-finalLaserAngle, SLOW_ANGULAR, CW);
        }
        else {
            taskComplete = moveAngle(finalLaserAngle, SLOW_ANGULAR, CCW);
        }
        if(taskComplete){
            takeStep();
        }
    }
    else if(step == 4){
        savePos();
        takeStep();
    }
    else if(step == 5){
        moveLinearSpeed(FAST_LINEAR, FORWARD);
        
        // if(taskComplete){
        //     takeStep();
        // }
    }
    else{
        resetState();
    }

}