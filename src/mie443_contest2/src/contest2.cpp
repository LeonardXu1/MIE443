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
bool navigateToBoxMultiAngle(Navigation& nav, const std::vector<float>& boxCoords) {
    float boxX = boxCoords[0];
    float boxY = boxCoords[1];
    float boxPhi = boxCoords[2];
    
    // Different angles to try (in degrees relative to box orientation)
    std::vector<float> angleOffsets = {0, 45, -45, 90, -90, 135, -135, 180};
    
    ROS_INFO("Starting multi-angle navigation to box at (%.2f, %.2f)", boxX, boxY);
    
    // Try each approach angle
    for (size_t angleIndex = 0; angleIndex < angleOffsets.size(); angleIndex++) {
        float approachAngle = boxPhi + angleOffsets[angleIndex];
        // Normalize to -180 to 180 range
        while (approachAngle > 180) approachAngle -= 360;
        while (approachAngle < -180) approachAngle += 360;
        
        ROS_INFO("Trying approach angle %.2f (offset %.2f from box)", 
                 approachAngle, angleOffsets[angleIndex]);
        
        // Step 1: Go to a position far from the box
        float farDist = 1.2;  // Farther distance to avoid obstacles
        float farX = boxX + farDist * cos(DEG2RAD(approachAngle));
        float farY = boxY + farDist * sin(DEG2RAD(approachAngle));
        float farPhi = atan2(boxY-farY,boxX-farX)*180/M_PI;  // Face toward the box
        
        // Normalize phi
        while (farPhi > 180) farPhi -= 360;
        while (farPhi < -180) farPhi += 360;
        
        ROS_INFO("Step 1: Moving to position (%.2f, %.2f) at angle %.2f", 
                 farX, farY, farPhi);
        
        bool reached = nav.moveToGoal(farX, farY, DEG2RAD(farPhi));
        
        if (!reached) {
            ROS_WARN("Failed to reach far position with angle %.2f, trying next angle", 
                     approachAngle);
            // Let the built-in recovery happen
            continue;  // Try next angle
        }
        
        ros::Duration(1.0).sleep();  // Brief pause
        
        // Step 2: Go to final viewing position directly
        // (Skip the intermediate step to simplify)
        float viewDist = 0.5;  // Viewing distance
        float viewX = boxX + viewDist * cos(DEG2RAD(boxPhi));
        float viewY = boxY + viewDist * sin(DEG2RAD(boxPhi));
        float viewAngle=boxPhi+180;
        ROS_INFO("Step 2: Moving to viewing position (%.2f, %.2f)", viewX, viewY);
        
        reached = nav.moveToGoal(viewX, viewY, DEG2RAD(viewAngle));
        
        if (reached) {
            ROS_INFO("Successfully reached viewing position!");
            return true;
        } else {
            ROS_WARN("Failed to reach viewing position with angle %.2f, trying next angle", 
                     approachAngle);
            // Let the built-in recovery happen
        }
    }
    
    ROS_ERROR("Failed to reach box with all approach angles");
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
bool navigateToBoxSimple(Navigation& nav, const std::vector<float>& boxCoords) {
    float boxX = boxCoords[0];
    float boxY = boxCoords[1];
    float boxPhi = boxCoords[2];
    
    // Try just the main approach angle
    float approachAngle = boxPhi;
    
    // First try direct approach at 0.7m (slightly farther than your original 0.5m)
    float dist = 0.5;
    float targetX = boxX + dist * cos(DEG2RAD(approachAngle));
    float targetY = boxY + dist * sin(DEG2RAD(approachAngle));
    float targetPhi = approachAngle + 180;  // Face the box
    
    // Normalize phi
    while (targetPhi > 180) targetPhi -= 360;
    while (targetPhi < -180) targetPhi += 360;
    
    ROS_INFO("Trying direct approach at distance %.2f", dist);
    bool reached = nav.moveToGoal(targetX, targetY, DEG2RAD(targetPhi));
    
    if (reached) {
        ROS_INFO("Direct approach successful!");
        return true;
    }
    
    // If direct approach failed, try from 90 degrees to the side
    ROS_INFO("Direct approach failed, trying from right side");
    
    // Try approaching from the right side (+90 degrees)
    approachAngle = boxPhi +90;
    // Normalize
    while (approachAngle > 180) approachAngle -= 360;
    
    targetX = boxX + dist * cos(DEG2RAD(approachAngle));
    targetY = boxY + dist * sin(DEG2RAD(approachAngle));
    //targetPhi = approachAngle + 180;
    targetPhi=atan2(boxY-targetY,boxX-targetX)*180/M_PI-10;
    // Normalize phi
    while (targetPhi > 180) targetPhi -= 360;
    
    reached = nav.moveToGoal(targetX, targetY, DEG2RAD(targetPhi));
    
    if (reached) {
        ROS_INFO("Side approach successful!");
        return true;
    }
    
    // If that failed too, try from the left side (-90 degrees)
    ROS_INFO("Right side approach failed, trying from left side");
    
    approachAngle = boxPhi - 90;
    // Normalize
    while (approachAngle < -180) approachAngle += 360;
    
    targetX = boxX + dist * cos(DEG2RAD(approachAngle));
    targetY = boxY + dist * sin(DEG2RAD(approachAngle));
   // targetPhi = approachAngle + 180;
   targetPhi=atan2(boxY-targetY,boxX-targetX)*180/M_PI+10;
    // Normalize phi
    while (targetPhi > 180) targetPhi -= 360;
    
    reached = nav.moveToGoal(targetX, targetY, DEG2RAD(targetPhi));
    
    if (reached) {
        ROS_INFO("Left side approach successful!");
        return true;
    }
    
    ROS_ERROR("All approach attempts failed");
    return false;
}

    
  
std::vector<float> targetOffset(std::vector<float> target){
    float x = target[0];
    float y = target[1];
    float phi = target[2];

    float offset = 0.45;

    float offsetX = offset * cos(DEG2RAD(phi));
    float offsetY = offset * sin(DEG2RAD(phi));
    
    float newX = x + offsetX;
    float newY = y + offsetY;
    float newPhi = phi + 180;
 

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

    // std::vector<float> inputPos = positionInput();

    // float x = inputPos[0];
    // float y = inputPos[1];
    // float phi = inputPos[2];
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
             reached = navigateToBoxMultiAngle(nav, boxCoords);

            // float phiOffset = offsetCalc(phi, RAD2DEG(robotPose.phi));
            // ROS_INFO("Phi Offset: %f", phiOffset);

           
            //ros::Duration(2).sleep();
            if(reached){
                //ros::Duration(5).sleep();
                if(templateIDS[i]==-3){
                   // templateIDS[i]=-99;
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
            reached=nav.moveToGoal(startPos[0],startPos[1],startPos[2]);
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
