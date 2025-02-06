
// function to extract all the important laserscan data which we'll be able to reference later easily for calculations
#include "../include/scanning.h"
#include "../include/movement.h"
LaserData extractLaserData(const sensor_msgs::LaserScan::ConstPtr& msg)
{
    LaserData dat;
    dat.angle_min = msg->angle_min;
    dat.angle_max = msg->angle_max;
    dat.angle_increment = msg->angle_increment;
    // dat.range_min = msg->range_min;              don't think we'll be using these, they are the distance values corresponding to the max and min angle of field of view
    // dat.range_max = msg->range_max;
    
    // Compute number of lasers
    dat.nLasers = (msg->angle_max - msg->angle_min) / msg->angle_increment;
    
    // Store all laser readings
    dat.ranges = msg->ranges;  // Directly copying the vector
    
    return dat;
}

// Processing Laser Data 
void processLaserData(const sensor_msgs::LaserScan::ConstPtr& msg)
{   
    // std::vector<float> laserDistVect;                                            //need to work on this to cycle through a sorted array of idx and distances
    // for (uint32_t laser_idx = 0; laser_idx < nLasers; ++ laser_idx){
    //     laserDistVect.push_back(msg->ranges[laser_idx])
    // }
    LaserData dat = extractLaserData(msg);          //gives me access to the extracted laser scan info function so that i don't have to reference the  

    float maxLaserDist = -std::numeric_limits<float>::infinity();
    int maxLaserIdx = -1;               //set to -1 to be a placeholder for "no valid index"
    float maxLaserAngle = 0.0f;
    bool passageblocked;
    int leftBoundIdx, rightBoundIdx;
    float leftBoundDist, rightBoundDist;
    float robotWidth = 10;               //  must change this depending on units and actual width. also currently unsure if rounding will be an error 
    
    for (uint32_t laser_idx = 0; laser_idx < dat.nLasers; ++ laser_idx)
        {        //cycle through all laser distances and store the max distance and corresponding index, as well as index's angle
            if (dat.ranges[laser_idx] > maxLaserDist){
                maxLaserDist = dat.ranges[laser_idx];
                maxLaserIdx = laser_idx;
                maxLaserAngle = dat.angle_min + maxLaserIdx * dat.angle_increment;

                float dhalfAngle = atan((robotWidth / 2) / maxLaserDist);
                leftBoundIdx = std::round((maxLaserAngle + dhalfAngle) / dat.angle_increment);          // again i don't know the directions of the max and min angles 
                rightBoundIdx = std::round((maxLaserAngle - dhalfAngle) / dat.angle_increment);         // see above comment ^ need to think about whether to round up or down depending
                
    
                leftBoundDist = dat.ranges[leftBoundIdx];                   // corresponding distances to bound indices
                rightBoundDist = dat.ranges[rightBoundIdx];

                float thresholdUpper = maxLaserDist * 1.1;
                float thresholdLower = maxLaserDist * .9;

                for (uint32_t laser_idx = leftBoundIdx; laser_idx < rightBoundIdx; ++laser_idx){
                    if (dat.ranges[laser_idx] > thresholdUpper || dat.ranges[laser_idx] < thresholdLower){
                        passageblocked = true;
                //         break;
                    }
                }

                
            }
        }
    if (maxLaserAngle < 0){
            moveAngle(maxLaserAngle, SLOW_ANGULAR, CW);
        }
        else {
            moveAngle(maxLaserAngle, SLOW_ANGULAR, CCW);
        }
        
        moveDistance(maxLaserDist, SLOW_LINEAR, BACKWARD);
}

// void scanningBehaviour(float maxLaserAngle, float maxLaserDist){
//     if (maxLaserAngle < 0){
//             moveAngle(maxLaserAngle, SLOW_ANGULAR, CW)
//         }
//         else {
//             moveAngle(maxLaserAngle, SLOW_ANGULAR, CCW)
//         }
        
//         moveDistance(maxLaserDist, SLOW_LINEAR, BACKWARD)
// }