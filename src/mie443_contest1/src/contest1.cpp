#include <geometry_msgs/Twist.h>
#include <stdio.h>
#include <cmath>
#include <chrono>
#include <cstring>

#include "../include/rConfig.h"
#include "../include/stateMachine.h"
#include "../include/bumper.h"
#include "../include/behaviour.h"

// intiates a velocity structure
velS velocity;
double timeSinceLastRandom;
auto currentTime = std::chrono::system_clock::now();
auto contestStart = std::chrono::system_clock::now();
auto lastRandomTime = std::chrono::system_clock::now();



// calls the behaviour function related to the state
void runBehaviour(state curState){
    if(curState == BUMPER_STATE){
        bumperBehaviour();
    }
    else if(curState == RANDOM_STATE){
        randomBehaviour();
    }
    else if(curState == EXPLORE_STATE){
        exploreBehaviour();
    }
}

// Logic for changing states
void decisionMaker(){
    if(isBumperPressed() == true) {
        setState(BUMPER_STATE);
    }
    else if(timeSinceLastRandom >= 10) {  // If 3 minutes (180 seconds) have passed and we're in EXPLORE_STATE, trigger RANDOM_STATE.
        setState(RANDOM_STATE);
        lastRandomTime = currentTime;
        ROS_INFO("Switching to RANDOM state after %f seconds.", timeSinceLastRandom);
    }
    else{
        setState(EXPLORE_STATE);
    }
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "image_listener");
    ros::NodeHandle nh;

    ros::Subscriber bumper_sub = nh.subscribe("mobile_base/events/bumper", 10, &bumperCallback); //declaration of subscribers from bumper
    // ros::Subscriber laser_sub = nh.subscribe("scan", 10, &laserCallback);//from sensor

    ros::Publisher vel_pub = nh.advertise<geometry_msgs::Twist>("cmd_vel_mux/input/teleop", 1); //advertise wheel speed
    ros::Subscriber odom = nh.subscribe("odom", 1, &odomCallback);
    ros::Rate loop_rate(10);

    geometry_msgs::Twist vel; //twist class

 // Global contest timer:
    contestStart = std::chrono::system_clock::now();
    uint64_t secondsElapsed = 0;
    // Timer for RANDOM state triggering: initialize to contest start.
    lastRandomTime = contestStart;

    while(ros::ok() && secondsElapsed <= 480) {
        ros::spinOnce();

        // Update state based on bumper (or other) inputs.
        decisionMaker();

        // --- RANDOM State Trigger ---
        currentTime = std::chrono::system_clock::now();
        timeSinceLastRandom = std::chrono::duration_cast<std::chrono::seconds>(currentTime - lastRandomTime).count();

        // Run the behaviour for the current state.
        state curState = getState();
        runBehaviour(curState);

        // Get and publish velocity commands
        velocity = getVelocity();
        vel.angular.z = velocity.angular;
        vel.linear.x = velocity.linear;
        vel_pub.publish(vel);

        secondsElapsed = std::chrono::duration_cast<std::chrono::seconds>(std::chrono::system_clock::now()-contestStart).count();
        loop_rate.sleep();
    }
    
    return 0;
}
