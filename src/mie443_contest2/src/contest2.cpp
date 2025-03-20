#include <boxes.h>
#include <navigation.h>
#include <robot_pose.h>
#include <imagePipeline.h>
#include <chrono>
#include <vector>
#include <pathPlanning.h>
#include <geometry_msgs/Twist.h>
#include <iostream>
#include <cmath>

#define RAD2DEG(rad) ((rad) * 180. / M_PI)
#define DEG2RAD(deg) ((deg) * M_PI / 180.)
bool findAdaptiveOffsetAnglePrio(Navigation& nav, const std::vector<float>& boxCoords, 
    float& outX, float& outY, float& outPhi) {
float boxX = boxCoords[0];
float boxY = boxCoords[1];
float boxPhi = boxCoords[2];

// Configure search parameters
float offsetDistStart = 0.5;            
float offsetDistLimit = 1.2;            
float offsetDistStep = 0.1;             

float offsetAngleLimit = DEG2RAD(90);   
float offsetAngleStep = DEG2RAD(15);    

ROS_INFO("Starting adaptive offset approach to box at (%.2f, %.2f) with new search pattern", boxX, boxY);


for (float offDist = offsetDistStart; offDist <= offsetDistLimit; offDist += offsetDistStep) {
ROS_INFO("Trying with distance: %.2f meters", offDist);


float offAngle = 0;


{

float newX = boxX + offDist * cos(boxPhi + offAngle);
float newY = boxY + offDist * sin(boxPhi + offAngle);


float faceAngle = atan2(boxY - newY, boxX - newX);

ROS_INFO("Trying offset: dist=%.2f, angle=%.2f degrees", 
offDist, RAD2DEG(offAngle));
ROS_INFO("Position: (%.2f, %.2f), facing: %.2f", 
newX, newY, RAD2DEG(faceAngle));

bool reached = nav.moveToGoal(newX, newY, faceAngle);

if (reached) {
ROS_INFO("Successfully reached offset position!");

// Set the output position values to use in your code
outX = newX;
outY = newY;
outPhi = faceAngle;

return true;
}


ros::Duration(0.5).sleep();
}


for (float angle = offsetAngleStep; angle <= offsetAngleLimit; angle += offsetAngleStep) {

{
offAngle = angle;
float newX = boxX + offDist * cos(boxPhi + offAngle);
float newY = boxY + offDist * sin(boxPhi + offAngle);
float faceAngle = atan2(boxY - newY, boxX - newX);

ROS_INFO("Trying offset: dist=%.2f, angle=+%.2f degrees", 
   offDist, RAD2DEG(offAngle));
ROS_INFO("Position: (%.2f, %.2f), facing: %.2f", 
   newX, newY, RAD2DEG(faceAngle));

bool reached = nav.moveToGoal(newX, newY, faceAngle);

if (reached) {
ROS_INFO("Successfully reached offset position!");
outX = newX;
outY = newY;
outPhi = faceAngle;
return true;
}

ros::Duration(0.5).sleep();
}

// Try negative angle
{
offAngle = -angle;
float newX = boxX + offDist * cos(boxPhi + offAngle);
float newY = boxY + offDist * sin(boxPhi + offAngle);
float faceAngle = atan2(boxY - newY, boxX - newX);

ROS_INFO("Trying offset: dist=%.2f, angle=-%.2f degrees", 
   offDist, RAD2DEG(-offAngle));
ROS_INFO("Position: (%.2f, %.2f), facing: %.2f", 
   newX, newY, RAD2DEG(faceAngle));

bool reached = nav.moveToGoal(newX, newY, faceAngle);

if (reached) {
ROS_INFO("Successfully reached offset position!");
outX = newX;
outY = newY;
outPhi = faceAngle;
return true;
}

ros::Duration(0.5).sleep();
}
}

ROS_INFO("No valid approach angles found at distance %.2f meters, trying farther...", offDist);
}

ROS_ERROR("Failed to find valid position after trying all offsets");
return false;
}
bool findAdaptiveOffsetDistPrio(Navigation& nav, const std::vector<float>& boxCoords, 
    float& outX, float& outY, float& outPhi) {
float boxX = boxCoords[0];
float boxY = boxCoords[1];
float boxPhi = boxCoords[2];

float offsetDistStart = 0.5;            
float offsetDistLimit = 1.2;           
float offsetDistStep = 0.1;             // Distance increment, try different distance

float offsetAngleLimit = DEG2RAD(90);   
float offsetAngleStep = DEG2RAD(15);    // Angle increment (15 degrees), try different approaching angle

ROS_INFO("Starting adaptive offset approach to box at (%.2f, %.2f)", boxX, boxY);
for (float offAngle = 0; offAngle <= offsetAngleLimit; ) {
    for (float offDist = offsetDistStart; offDist <= offsetDistLimit; offDist += offsetDistStep) {
float newX = boxX + offDist * cos(boxPhi + offAngle);
float newY = boxY + offDist * sin(boxPhi + offAngle);

float faceAngle = atan2(boxY - newY, boxX - newX);

ROS_INFO("Trying offset: dist=%.2f, angle=%.2f degrees", 
offDist, RAD2DEG(offAngle));
ROS_INFO("Position: (%.2f, %.2f), facing: %.2f", 
newX, newY, RAD2DEG(faceAngle));

bool reached = nav.moveToGoal(newX, newY, faceAngle);

if (reached) {
ROS_INFO("Successfully reached offset position!");
outX = newX;
outY = newY;
outPhi = faceAngle;

return true;
}

ros::Duration(0.5).sleep();
}

if (offAngle > 0) {
offAngle = -offAngle;  // try angle from negative to positive
} else {
offAngle = -offAngle + offsetAngleStep;  
}
}

ROS_ERROR("Failed to find valid position after trying all offsets");
return false;
}
bool adaptiveReturnHome(Navigation& nav, const std::vector<float>& homePos) {
    float homeX = homePos[0];
    float homeY = homePos[1];
    float homePhi = homePos[2];
    
    ROS_INFO("Attempting to return to home position (%.2f, %.2f, %.2f)", 
             homeX, homeY, homePhi);
    
    // First try direct return
    bool reached = nav.moveToGoal(homeX, homeY, homePhi);
    if (reached) {
        ROS_INFO("Successfully returned home with direct path!");
        return true;
    }
    
    ROS_WARN("Direct path home failed. Trying adaptive return approaches...");
    
    float approachDist = 1.0;             // Distance from home to approach from
    float angleStep = DEG2RAD(45);        // 45 degree steps for wider variation
    float maxAngle = DEG2RAD(315);        // Try full 360 degrees (minus last step)
    
   
    for (float angle = 0; angle <= maxAngle; angle += angleStep) {
       
        float approachX = homeX + approachDist * cos(angle);
        float approachY = homeY + approachDist * sin(angle);
        
  
        
        ROS_INFO("Trying approach angle %.2f degrees", RAD2DEG(angle));
        ROS_INFO("Approach position: (%.2f, %.2f)", approachX, approachY);
        
        // First move to approach position
        reached = nav.moveToGoal(approachX, approachY, homePhi);
        
        if (!reached) {
            ROS_WARN("Failed to reach approach position at angle %.2f, trying next angle",
                     RAD2DEG(angle));
            continue;
        }
        
        
        ROS_INFO("Reached approach position. Now moving to final home position...");
        
      
        reached = nav.moveToGoal(homeX, homeY, homePhi);
        
        if (reached) {
            ROS_INFO("Successfully returned home via approach angle %.2f!", RAD2DEG(angle));
            return true;
        } else {
            ROS_WARN("Failed to reach home from approach position. Trying next angle.");
        }
        
        
        ros::Duration(0.5).sleep();
    }
    
   
    
    ROS_WARN("All standard approaches failed. Attempting waypoint navigation");
    
    
    float farDist = 2.0; 
    
    for (float angle = 0; angle <= maxAngle; angle += angleStep) {
     
        float waypointX = homeX + farDist * cos(angle);
        float waypointY = homeY + farDist * sin(angle);
        
        ROS_INFO("Trying waypoint at angle %.2f, distance %.2f", 
                 RAD2DEG(angle), farDist);
        
      
        reached = nav.moveToGoal(waypointX, waypointY, homePhi);
        
        if (!reached) {
            continue;  
        }
        
     
        float approachX = homeX + approachDist * cos(angle);
        float approachY = homeY + approachDist * sin(angle);
        
        reached = nav.moveToGoal(approachX, approachY, homePhi);
        
        if (!reached) {
            continue;  
        }
        
     
        reached = nav.moveToGoal(homeX, homeY, homePhi);
        
        if (reached) {
            ROS_INFO("Successfully returned home via waypoint and approach position!");
            return true;
        }
    }
    
    ROS_ERROR("All return home attempts failed.");
    return false;
}

std::vector<float> positionInput(){
    float x, y, phi;

    std::cout << "Enter x goal: ";

    std::cin >> x;

    std::cout << "Enter y goal: ";
    
    std::cin >> y;

    std::cout << "Enter phi goal: ";

    std::cin >> phi;

    std::vector<float> inputPos = {x, y, phi};

    return inputPos;
}

int boxInput(){
    float id;
    std::cout << "Enter box goal: ";

    std::cin >> id;

    id -= 1;

    return id;
}


    
  
std::vector<float> targetOffset(std::vector<float> target){
    float x = target[0];
    float y = target[1];
    float phi = target[2];

    float offset = 0.5;

    float offsetX = offset * cos(DEG2RAD(phi));
    float offsetY = offset * sin(DEG2RAD(phi));
    
    float newX = x + offsetX;
    float newY = y + offsetY;
    float newPhi = phi + 180;
    //float newPhi=atan2(y - newY, x - newX);
 

    std::vector<float> newTarget = {newX, newY, newPhi};
    return newTarget;
}

float offsetCalc(float target, float actual){
    float offset;
    if(target >= actual){
        offset = target - actual;
    }
    else{
        offset = -(actual - target);
    }
    return offset;
}


int main(int argc, char** argv) {
    // Setup ROS.
    ros::init(argc, argv, "contest2");
    ros::NodeHandle n;

    // Robot pose object + subscriber.
    RobotPose robotPose(0,0,0);
    ros::Subscriber amclSub = n.subscribe("/amcl_pose", 1, &RobotPose::poseCallback, &robotPose);
   
    // Initialize box coordinates and templates
    Boxes boxes; 
    if(!boxes.load_coords() || !boxes.load_templates()) {
        std::cout << "ERROR: could not load coords or templates" << std::endl;
        return -1;
    }
    for(int i = 0; i < boxes.coords.size(); ++i) {
        std::cout << "Box coordinates: " << std::endl;
        std::cout << i+1 << " x: " << boxes.coords[i][0] << " y: " << boxes.coords[i][1] << " z: " 
                  << boxes.coords[i][2] << std::endl;
    }
    std::cout << "Finished BOX PRINTING " << std::endl;
    // Initialize image objectand subscriber.
    ImagePipeline imagePipeline(n);
    pathPlanning path;
    if(imagePipeline.SURFIntitialize(boxes)==0){
        std::cout<<"SURF initilized"<<std::endl;
    }
    std::vector<int> templateIDS={-3,-3,-3,-3,-3};
    // Initialize navigation
    Navigation nav;

    // contest count down timer
    std::chrono::time_point<std::chrono::system_clock> start;
    start = std::chrono::system_clock::now();
    uint64_t secondsElapsed = 0;

    bool reached = false;

    std::vector<float> box;

    float x;
    float y;
    float phi;

    for(int i=0;i<3;i++){
          ros::spinOnce();
          ros::Duration(0.1).sleep();
    
    }
    bool returnHome=false;
    std::vector<float>startPos(3);
    startPos[0]=robotPose.x;
    startPos[1]=robotPose.y;
    startPos[2]=robotPose.phi;
    ROS_INFO("%f,%f,%f",startPos[0],startPos[1],startPos[2]);
    std::vector <int>route=path.pathPlan(startPos,boxes);
    // Execute strategy.
    while(ros::ok() && secondsElapsed <= 300) {
        ros::spinOnce();


        /***YOUR CODE HERE***/
        //boxes.coords;
        
        int i=0;
        while(i<route.size()&&returnHome==false){
            ros::spinOnce();

            ROS_INFO("Robot Position:");
            ROS_INFO("x: %f y: %f phi: %f", robotPose.x, robotPose.y, RAD2DEG(robotPose.phi));

            // box = boxes.coords[route[i]-1];
            // box = targetOffset(box);
        
            // x = box[0];
            // y = box[1];
            // phi = box[2];

            // ROS_INFO("Goal:");
            // ROS_INFO("x: %f y: %f phi: %f", x, y, phi);
    
            // reached = nav.moveToGoal(x, y, DEG2RAD(phi));
             
             std::vector<float> boxCoords = boxes.coords[route[i]-1];
             reached = findAdaptiveOffsetAnglePrio(nav, boxCoords, x, y, phi);



            // float phiOffset = offsetCalc(phi, RAD2DEG(robotPose.phi));
            // ROS_INFO("Phi Offset: %f", phiOffset);

           
            
            if(reached){
                
                if(templateIDS[i]==-3){
                   
                    ros::spinOnce();
                    int maxAttemps=3;
                    int attempt=1;
                    templateIDS[i]=imagePipeline.getTemplateID(boxes);
                    while(templateIDS[i]<0&&attempt<maxAttemps){
                        //ros::Duration(2).sleep();
                        ros::spinOnce();
                        templateIDS[i]=imagePipeline.getTemplateID(boxes);
                        attempt++;
                    }
                    reached=false;
                }
                
                if(i==route.size()-1){
                    returnHome=true;
                }
            }
            i += 1;
           
        }
        if(returnHome==true){
            //reached=nav.moveToGoal(startPos[0],startPos[1],startPos[2]);
             reached = adaptiveReturnHome(nav, startPos);
            if(reached){
                for(int j=0;j<templateIDS.size();j++){
                    std::cout<<"template ID: "<<templateIDS[j]<<std::endl;
                    if(templateIDS[j]==-2){
                        std::cout<<"Prob blank "<<std::endl;
                    }
                    std::cout<<"Its location: "<<std::endl;
                    
                    x = boxes.coords[route[j]-1][0];
                    y = boxes.coords[route[j]-1][1];
                    phi = boxes.coords[route[j]-1][2];
        
                    ROS_INFO("x: %f y: %f phi: %f", x, y, phi);
                    
        
                }
                break;
            }
        }
        
        // Use: boxes.coords
        // Use: robotPose.x, robotPose.y, robotPose.phi


        // else{
        //     inputPos = positionInput();

        //     x = inputPos[0];
        //     y = inputPos[1];
        //     phi = inputPos[2];

        //     reached = false;
        // }

        // imagePipeline.getTemplateID(boxes);
        ros::Duration(0.01).sleep();
        
    }
    return 0;
}
