
// function to extract all the important laserscan data which we'll be able to reference later easily for calculations
#include "../include/scanning.h"

LaserData dat;

float finalLaserDist;
float finalLaserAngle;


float robotWidth = .4;               // *****must change this depending on units and actual width. also currently unsure if rounding will be an error 


// **** STEP 0: Saving Laser Values in an Instance of time (Otherwise it's always updating)

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
    
float getRandomAngle(float minAngle, float maxAngle) {
    std::random_device rd;
    std::mt19937 gen(rd()); // Mersenne Twister RNG
    std::uniform_real_distribution<float> dis(minAngle, maxAngle);
    return dis(gen);
}


// ****STEP 1: Calc and Storing MAX_DISTANCE_ANGLE && MAX_DISTANCE
// Associated Variables
float maxLaserDist;
float maxLaserAngle;
int maxLaserIdx;

void maxLaserData()
{   
    // Assigning variable placeholder values 
    maxLaserDist = 0;
    maxLaserAngle = 0.0;
    maxLaserIdx = -1;               //set to -1 to be a placeholder for "no valid index"
    
    for (uint32_t laser_idx = 0; laser_idx < dat.nLasers; ++ laser_idx){        //cycle through all laser distances and store the max distance and corresponding index, as well as index's angle
        if (dat.ranges[laser_idx] > maxLaserDist){
            maxLaserDist = dat.ranges[laser_idx];
            maxLaserIdx = laser_idx;
            maxLaserAngle = dat.angle_min + maxLaserIdx * dat.angle_increment;
        }
    }
    // ROS_INFO("max distance %0.2f    | max laser angle %0.2f   | max laser index %0.2f", maxLaserDist, maxLaserAngle, maxLaserIdx);
}

float minLaserDist;
float minLaserAngle;
int minLaserIdx;

void minLaserData()
{   
    // Assigning variable placeholder values 
    minLaserDist = std::numeric_limits<float>::infinity();
    minLaserAngle = 0.0;
    minLaserIdx = -1;               //set to -1 to be a placeholder for "no valid index"
    
    for (uint32_t laser_idx = 0; laser_idx < dat.nLasers; ++ laser_idx){        //cycle through all laser distances and store the min distance and corresponding index, as well as index's angle
        if (dat.ranges[laser_idx] < minLaserDist){
            minLaserDist = dat.ranges[laser_idx];
            minLaserIdx = laser_idx;
            // minLaserAngle = dat.angle_min + maxLaserIdx * dat.angle_increment;
        }
    }
    // ROS_INFO("min distance %0.2f    | min laser angle %0.2f   | min laser index %0.2f", minLaserDist, minLaserAngle, minLaserIdx);
}
    
// ****STEP 2: Find Next Edge to the RIGHT [of maxLaser]
//Associated Variables
int robsLaserIdx;
float robsLaserAngle;
float robsLaserDist;
float rightCircleAngle;
float rightPathClearance;
bool right_edge_found = false;           //track if an edge was found

float edgeThresh = 0.18;


void rightEdgeFinder()
{
    int currentIdx = maxLaserIdx;
    int prevIdx = currentIdx - 1;          //keep in mind here when i say prev index, it's because were going from max laser idx to 0. so the "next value", is the previous index

    while(currentIdx > 0){
        
        while(std::isnan(dat.ranges[prevIdx]) && prevIdx > 0){
            prevIdx--;
        }

        if (dat.ranges[currentIdx] - dat.ranges[prevIdx] > edgeThresh){
            robsLaserIdx = prevIdx;
            robsLaserDist = dat.ranges[robsLaserIdx];
            robsLaserAngle = dat.angle_min + robsLaserIdx * dat.angle_increment;

            right_edge_found = true;
            break;
        }

        currentIdx = prevIdx;
        prevIdx = currentIdx - 1;
    }
    // ROS_INFO("right last current idx %d", currentIdx);
    // ROS_INFO("right last prev idx %d", prevIdx);
    // ROS_INFO("right edge found: %s", right_edge_found ? "true" : "false");

    //angle between max and right edge
    rightCircleAngle = maxLaserAngle - robsLaserAngle;
    rightPathClearance = 2 * robsLaserDist * sin(rightCircleAngle / 2);
        
    // ROS_INFO("right edge dis %0.2f    | right edge angle %0.2f   | right edge index %0.2d", robsLaserDist, robsLaserAngle, robsLaserIdx);
    // ROS_INFO("angle b/w right edge and max %0.2f    | right edge chord %0.2f", rightCircleAngle, rightPathClearance);


}


// ****STEP 3: Find Next Edge to the LEFT [of maxLaser]
//Associated Variables
int lobsLaserIdx;
float lobsLaserAngle;
float lobsLaserDist;
float leftCircleAngle;
float leftPathClearance;
bool left_edge_found = false;           //track if an edge was found


void leftEdgeFinder()
{
    int currentIdx = maxLaserIdx;
    int nextIdx = currentIdx + 1;          //keep in mind here when i say prev index, it's because were going from max laser idx to size of dat.ranges so the "next value", is the previous index

    while(currentIdx < dat.ranges.size() -1){
        
        while(std::isnan(dat.ranges[nextIdx]) && nextIdx < dat.ranges.size() - 1){
            nextIdx++;
        }

        if (dat.ranges[currentIdx] - dat.ranges[nextIdx] > edgeThresh){
            lobsLaserIdx = nextIdx;
            lobsLaserDist = dat.ranges[lobsLaserIdx];
            lobsLaserAngle = dat.angle_min + lobsLaserIdx * dat.angle_increment;

            left_edge_found = true;
            // ROS_INFO("test difference %0.2f |   current idx distance %0.2f  |   next idx dist %0.2f ",dat.ranges[currentIdx] - dat.ranges[nextIdx], dat.ranges [currentIdx], dat.ranges[nextIdx] );
            
            break;

        }

        currentIdx = nextIdx;
        nextIdx = currentIdx + 1;
    }
    // ROS_INFO("left last current idx %d", currentIdx);
    // ROS_INFO("left last next idx %d", nextIdx);
    // ROS_INFO("left edge found: %s", left_edge_found ? "true" : "false");



    //angle between max and left edge
    leftCircleAngle = lobsLaserAngle - maxLaserAngle;
    leftPathClearance = 2 * lobsLaserDist * sin(leftCircleAngle / 2);

    // ROS_INFO("left edge dis %0.3f    | left edge angle %0.2f   | left edge index %0.2d", lobsLaserDist, lobsLaserAngle, lobsLaserIdx);
    // ROS_INFO("angle b/w left edge and max %0.2f    | left edge chord %0.2f", leftCircleAngle, leftPathClearance);
}



std::vector<int> i_edgeIndicies;
std::vector<int> j_edgeIndicies;

//// SEE IF THIS WORKS
void findAllEdges()
{
    // ROS_INFO("test 0");
    int i = 0;
    int j = i + 1;
    i_edgeIndicies.clear();
    j_edgeIndicies.clear();


    while(i < dat.ranges.size() - 1){
        
        while(std::isnan(dat.ranges[j]) && j < dat.ranges.size() - 1){
            j++;
        }

        if (fabs(dat.ranges[i] - dat.ranges[j]) > edgeThresh){
            i_edgeIndicies.push_back(std::min(i,j));
            j_edgeIndicies.push_back(std::max(i,j));
            
        }

        i = j;
        j = i + 1;
    }

    // //angle between max and right edge
    // rightCircleAngle = maxLaserAngle - robsLaserAngle;
    // rightPathClearance = 2 * robsLaserDist * sin(rightCircleAngle / 2);

    // ROS_INFO to print the vector
    std::stringstream ss;
    std::stringstream aa;

    for (size_t k = 0; k < i_edgeIndicies.size(); ++k) {
        ss << i_edgeIndicies[k] << " ";
        aa << j_edgeIndicies[k] << " ";
    }
    // ROS_INFO("Closer edge indices: %s", ss.str().c_str());
    // ROS_INFO("Further edge indices: %s", aa.str().c_str());
}



//Replacing random angles
void turn_direction()
{
    minLaserData();

    int center_idx = dat.ranges.size() / 2;
    
    if (minLaserIdx < center_idx){
        finalLaserAngle = M_PI / 6;
    }
    else{
        finalLaserAngle = -M_PI /6;
    }
}

int minLaserDirection()
{
    minLaserData();
    
    int center_idx = dat.ranges.size() / 2;
    
    if (minLaserIdx < center_idx){
        return CW;
    }
    else{
        return CCW;
    }
}




float avEdgeAngle;
float avEdgeChord;
float edge_clearance = 0.18;

bool maxPathClear = leftPathClearance > robotWidth && rightPathClearance > robotWidth;
bool leftPathClear = leftPathClearance  > robotWidth && rightPathClearance < robotWidth;
bool rightPathClear = leftPathClearance  < robotWidth && rightPathClearance > robotWidth;
bool noClearPath = leftPathClearance  < robotWidth && rightPathClearance < robotWidth;

void pickBestPath ()
{
    findAllEdges();
//    bool maxPathClear = leftPathClearance > robotWidth && rightPathClearance > robotWidth;
//    bool leftPathClear = leftPathClearance  > robotWidth && rightPathClearance < robotWidth;
//    bool rightPathClear = leftPathClearance  < robotWidth && rightPathClearance > robotWidth;
//    bool noClearPath = leftPathClearance  < robotWidth && rightPathClearance < robotWidth;

    //Case: No edges detected     
    if (i_edgeIndicies.size() == 0){
        turn_direction();
        // finalLaserAngle = getRandomAngle(M_PI / 2, 3 * M_PI / 2);                //***FOR NOW IT'S GET RANDOM ANGLE UNLESS WE DO THE CHECK LEFT CHECK RIGHT
        // ROS_INFO("NO Edges --> RANDOM");
        return;

    }

    //Case: only 1 edge detected 
    if(right_edge_found ^ left_edge_found){
        if(right_edge_found){
            for (int x = 0; x < i_edgeIndicies.size()-1; x++){       // check if there are any more edges to the right of rightedge
                if(i_edgeIndicies[x] < robsLaserIdx){
                    
                    //handling the random edge if there is one
                    if (dat.ranges[i_edgeIndicies[x]] > dat.ranges[j_edgeIndicies[x]]){
                        if (i_edgeIndicies[x] > j_edgeIndicies[x])
                            finalLaserAngle = (dat.angle_min + j_edgeIndicies[x] * dat.angle_increment) + (asin((0.5 * (robotWidth + edge_clearance)) / dat.ranges[j_edgeIndicies[x]])); // only diff between this and next else is add or subtract sign
                        else{
                            finalLaserAngle = (dat.angle_min + j_edgeIndicies[x] * dat.angle_increment) - (asin((0.5 * (robotWidth + edge_clearance)) / dat.ranges[j_edgeIndicies[x]])); // only diff between this and next else is add or subtract sign
                        }    
            
                        // ROS_INFO("edge angle %0.2f  |   closer edge dist %0.2f   |   farther edge dist %0.2f", (dat.angle_min + j_edgeIndicies[x] * dat.angle_increment), dat.ranges[j_edgeIndicies[random_edge]], dat.ranges[i_edgeIndicies[random_edge]]);
                    }
                    else if (dat.ranges[i_edgeIndicies[x]] < dat.ranges[j_edgeIndicies[x]]){
                        if (i_edgeIndicies[x] > j_edgeIndicies[x]){
                            finalLaserAngle = (dat.angle_min + i_edgeIndicies[x] * dat.angle_increment) - (asin((0.5 * (robotWidth + edge_clearance)) / dat.ranges[i_edgeIndicies[x]])); // only diff between this and next else is add or subtract sign
                        }
                        else{
                        finalLaserAngle = (dat.angle_min + i_edgeIndicies[x] * dat.angle_increment) + (asin((0.5 * (robotWidth + edge_clearance)) / dat.ranges[i_edgeIndicies[x]])); // only diff between this and next else is add or subtract sign
                        
                        }       
                    }
                    return;
                } 
                
            }

            finalLaserAngle = robsLaserAngle + asin((0.5 * (robotWidth + edge_clearance)) / robsLaserDist);
        }
        else if(left_edge_found){
            for (int x = 0; x < i_edgeIndicies.size()-1; x++){
                if(i_edgeIndicies[x] > lobsLaserIdx){       //check if there are any other edges to the left of left edge
                    
                    //handling the random edge if there is one
                    if (dat.ranges[i_edgeIndicies[x]] > dat.ranges[j_edgeIndicies[x]]){
                        if (i_edgeIndicies[x] > j_edgeIndicies[x])
                            finalLaserAngle = (dat.angle_min + j_edgeIndicies[x] * dat.angle_increment) + (asin((0.5 * (robotWidth + edge_clearance)) / dat.ranges[j_edgeIndicies[x]])); // only diff between this and next else is add or subtract sign
                        else{
                            finalLaserAngle = (dat.angle_min + j_edgeIndicies[x] * dat.angle_increment) - (asin((0.5 * (robotWidth + edge_clearance)) / dat.ranges[j_edgeIndicies[x]])); // only diff between this and next else is add or subtract sign
                        }    
            
                        // ROS_INFO("edge angle %0.2f  |   closer edge dist %0.2f   |   farther edge dist %0.2f", (dat.angle_min + j_edgeIndicies[x] * dat.angle_increment), dat.ranges[j_edgeIndicies[random_edge]], dat.ranges[i_edgeIndicies[random_edge]]);
                    }
                    else if (dat.ranges[i_edgeIndicies[x]] < dat.ranges[j_edgeIndicies[x]]){
                        if (i_edgeIndicies[x] > j_edgeIndicies[x]){
                            finalLaserAngle = (dat.angle_min + i_edgeIndicies[x] * dat.angle_increment) - (asin((0.5 * (robotWidth + edge_clearance)) / dat.ranges[i_edgeIndicies[x]])); // only diff between this and next else is add or subtract sign
                        }
                        else{
                        finalLaserAngle = (dat.angle_min + i_edgeIndicies[x] * dat.angle_increment) + (asin((0.5 * (robotWidth + edge_clearance)) / dat.ranges[i_edgeIndicies[x]])); // only diff between this and next else is add or subtract sign
                        }       
                    }
                return;
                } 
                
            }
            
            finalLaserAngle = lobsLaserAngle - asin((0.5 * (robotWidth + edge_clearance)) / lobsLaserDist);
        }
    }
    
    //case: 2 edges found 
    if (right_edge_found && left_edge_found){
        if (maxPathClear){
            finalLaserAngle = maxLaserAngle;
        }
        else if (rightPathClear ^ leftPathClear){
            if (rightPathClear){
                finalLaserAngle = lobsLaserAngle - asin((0.5 * (robotWidth + edge_clearance)) / lobsLaserDist);
            }
            else{
                finalLaserAngle = robsLaserAngle + asin((0.5 * (robotWidth + edge_clearance)) / robsLaserDist);
            }
        }
        else if (noClearPath){
            avEdgeAngle = fabs(lobsLaserAngle - robsLaserAngle);
            avEdgeChord = 2 * std::min(lobsLaserDist, robsLaserDist) * sin(avEdgeAngle / 2);
            if (avEdgeChord > robotWidth){
                finalLaserAngle = (lobsLaserAngle + robsLaserAngle) / 2;
                // ROS_INFO("SQUEEZE THROUGH");
            }
            
            else{
                for (int x = 0; x < i_edgeIndicies.size()-1; x++){
                    if(i_edgeIndicies[x] > lobsLaserIdx ||  i_edgeIndicies[x] < robsLaserIdx){
                        if (dat.ranges[i_edgeIndicies[x]] > dat.ranges[j_edgeIndicies[x]]){
                            if (i_edgeIndicies[x] > j_edgeIndicies[x])
                                finalLaserAngle = (dat.angle_min + j_edgeIndicies[x] * dat.angle_increment) + (asin((0.5 * (robotWidth + edge_clearance)) / dat.ranges[j_edgeIndicies[x]])); // only diff between this and next else is add or subtract sign
                            else{
                                finalLaserAngle = (dat.angle_min + j_edgeIndicies[x] * dat.angle_increment) - (asin((0.5 * (robotWidth + edge_clearance)) / dat.ranges[j_edgeIndicies[x]])); // only diff between this and next else is add or subtract sign
                            }    
                
                        }
                        else if (dat.ranges[i_edgeIndicies[x]] < dat.ranges[j_edgeIndicies[x]]){
                            if (i_edgeIndicies[x] > j_edgeIndicies[x]){
                                finalLaserAngle = (dat.angle_min + i_edgeIndicies[x] * dat.angle_increment) - (asin((0.5 * (robotWidth + edge_clearance)) / dat.ranges[i_edgeIndicies[x]])); // only diff between this and next else is add or subtract sign
                            }
                            else{
                            finalLaserAngle = (dat.angle_min + i_edgeIndicies[x] * dat.angle_increment) + (asin((0.5 * (robotWidth + edge_clearance)) / dat.ranges[i_edgeIndicies[x]])); // only diff between this and next else is add or subtract sign
                            }       
                        }
                    return;
                    }
                }
            }


            //if no better option tha
            finalLaserAngle = getRandomAngle(M_PI /2, 3*M_PI/2);
            // ROS_INFO("NO CLEAR --> RANDOM");
            
        }
        // ROS_INFO("edge chord %0.2f  |   av edge angle %0.2f", avEdgeChord, avEdgeAngle);
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
        maxLaserData();
        leftEdgeFinder();
        rightEdgeFinder();
        findAllEdges();
        pickBestPath();
 

        // ROS_INFO("ZERO edge case: %s", !right_edge_found && !left_edge_found ? "true" : "false");
        // ROS_INFO("ONE edge case: %s", right_edge_found ^ left_edge_found ? "true" : "false");
        // ROS_INFO("TWO edge case: %s", right_edge_found && left_edge_found ? "true" : "false");

        // ROS_INFO("rightPass: %s", rightPathClear ? "true" : "false");
        // ROS_INFO("leftPass: %s", leftPathClear ? "true" : "false");
        // ROS_INFO("bothPass: %s", maxPathClear ? "true" : "false");
        // ROS_INFO("neitherPass: %s", noClearPath ? "true" : "false");
        // ROS_INFO("final angle: %0.2f", finalLaserAngle);
        takeStep();
    }
    else if(step == 2){
        savePos();
        takeStep();
    }
    else if(step ==  3){
        if (finalLaserAngle < 0){
            taskComplete = moveAngle(-finalLaserAngle, SLOW_ANGULAR, CW);  // added a negative here because then negative angles become positive
        }
        else {
            taskComplete = moveAngle(finalLaserAngle, SLOW_ANGULAR, CCW);
        }
        if(taskComplete){
            takeStep();
        }
    }
    else if(i_edgeIndicies.size() == 0){
        overrideStep(1);
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