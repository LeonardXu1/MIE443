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

            box = boxes.coords[route[i]-1];
            box = targetOffset(box);
        
            x = box[0];
            y = box[1];
            phi = box[2];

            ROS_INFO("Goal:");
            ROS_INFO("x: %f y: %f phi: %f", x, y, phi);
    
            reached = nav.moveToGoal(x, y, DEG2RAD(phi));

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
