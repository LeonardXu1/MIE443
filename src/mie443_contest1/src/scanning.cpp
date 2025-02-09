
// function to extract all the important laserscan data which we'll be able to reference later easily for calculations
#include "../include/scanning.h"

LaserData dat;
float maxLaserDist;
float maxLaserAngle;
float finalLaserDist;
float finalLaserAngle;

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
    float robotWidth = 10;               //  must change this depending on units and actual width. also currently unsure if rounding will be an error 
    
    // Variables for obstacle detection and path compensation
    float obsLaserAngle;
    float obsLaserDist;
    int obsLaserIdx;
    float obsClearance = robotWidth / 2 + 0.05;



    for (uint32_t laser_idx = 0; laser_idx < dat.nLasers; ++ laser_idx)
        {        //cycle through all laser distances and store the max distance and corresponding index, as well as index's angle
            if (dat.ranges[laser_idx] > maxLaserDist){
                maxLaserDist = dat.ranges[laser_idx];
                maxLaserIdx = laser_idx;
                maxLaserAngle = dat.angle_min + maxLaserIdx * dat.angle_increment;

                finalLaserAngle = maxLaserAngle;

            }
            if (dat.ranges[maxLaserIdx] - dat.ranges[maxLaserIdx - 1] > 0.1 ){          // in other words: if the obstacle is on the left of the maxlaserdist 
                obsLaserIdx = maxLaserIdx - 1;
                obsLaserDist = dat.ranges[obsLaserIdx];
                obsLaserAngle = atan(obsClearance / obsLaserDist);
                finalLaserAngle = maxLaserAngle - obsLaserAngle;                        //this is the case for both positive and negative maxlaserAngle cases. 
            }
                
            if (dat.ranges[maxLaserIdx] - dat.ranges[maxLaserIdx + 1] > 0.1 ){          // in other words: if the obstacle is on the left of the maxlaserdist 
                obsLaserIdx = maxLaserIdx + 1;
                obsLaserDist = dat.ranges[obsLaserIdx];
                obsLaserAngle = atan(obsClearance / obsLaserDist);
                finalLaserAngle = maxLaserAngle + obsLaserAngle;                        //this is the case for both positive and negative maxlaserAngle cases. 
            }
        }
}

void scanningBehaviour(){
    bool taskComplete;
    int step = getStep();

    if(step == 0){
        processLaserData();
        ROS_INFO("max angle: %0.2f  | max dist: %0.2f", maxLaserAngle, maxLaserDist);
        takeStep();
    }
    else if(step == 1){
        savePos();
        takeStep();
    }
    else if(step ==  2){
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
    else if(step == 3){
        savePos();
        takeStep();
    }
    else if(step == 4){
        moveLinearSpeed(SLOW_LINEAR, FORWARD);
        
        // if(taskComplete){
        //     takeStep();
        // }
    }
    else{
        resetState();
    }

}