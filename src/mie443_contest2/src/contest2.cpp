#include <boxes.h>
#include <navigation.h>
#include <robot_pose.h>
#include <imagePipeline.h>
#include <chrono>

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

    float offset = 0.5;

    float offsetX = offset * cos(phi);
    float offsetY = offset * sin(phi);
    
    float newX = x + offsetX;
    float newY = y + offsetY;
    float newPhi = phi + DEG2RAD(180);

    std::vector<float> newTarget = {newX, newY, newPhi};
    return newTarget;
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
        std::cout << i << " x: " << boxes.coords[i][0] << " y: " << boxes.coords[i][1] << " z: " 
                  << boxes.coords[i][2] << std::endl;
    }
    // Initialize image objectand subscriber.
    ImagePipeline imagePipeline(n);

    // Initialize navigation
    Navigation nav;

    // contest count down timer
    std::chrono::time_point<std::chrono::system_clock> start;
    start = std::chrono::system_clock::now();
    uint64_t secondsElapsed = 0;

    bool reached;

    float id = boxInput();
    std::vector<float> box = boxes.coords[id];
    box = targetOffset(box);

    float x = box[0];
    float y = box[1];
    float phi = box[2];

    // std::vector<float> inputPos = positionInput();

    // float x = inputPos[0];
    // float y = inputPos[1];
    // float phi = inputPos[2];
    
    // Execute strategy.
    while(ros::ok() && secondsElapsed <= 300) {
        ros::spinOnce();
        /***YOUR CODE HERE***/
        // Use: boxes.coords
        // Use: robotPose.x, robotPose.y, robotPose.phi

        ROS_INFO("Robot Position:");
        ROS_INFO("x: %f y: %f phi: %f", robotPose.x, robotPose.y, robotPose.phi);
        
        if(!reached){
            ROS_INFO("Goal:");
            ROS_INFO("x: %f y: %f phi: %f", x, y, phi);
    
            reached = nav.moveToGoal(x, y, phi);
        }
        else{
            id = boxInput();
            box = boxes.coords[id];
            box = targetOffset(box);
        
            x = box[0];
            y = box[1];
            phi = box[2];

            reached = false;
        }
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
