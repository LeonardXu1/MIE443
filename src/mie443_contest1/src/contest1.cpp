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
    else {
        // Remain in current state if not preempted (note: RANDOM state will complete its maneuver)
        if(getState() != RANDOM_STATE) {
            setState(EXPLORE_STATE);
        }
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
    auto contestStart = std::chrono::system_clock::now();
    uint64_t secondsElapsed = 0;
    // Timer for RANDOM state triggering: initialize to contest start.
    auto lastRandomTime = contestStart;

    while(ros::ok() && secondsElapsed <= 480) {
        ros::spinOnce();

        // Update state based on bumper (or other) inputs.
        decisionMaker();

        // --- RANDOM State Trigger ---
        auto currentTime = std::chrono::system_clock::now();
        double timeSinceLastRandom = std::chrono::duration_cast<std::chrono::seconds>(currentTime - lastRandomTime).count();
        // If 3 minutes (180 seconds) have passed and we're in EXPLORE_STATE, trigger RANDOM_STATE.
        if(timeSinceLastRandom >= 180 && getState() == EXPLORE_STATE) {
            setState(RANDOM_STATE);
            lastRandomTime = currentTime;
            ROS_INFO("Switching to RANDOM state after %f seconds.", timeSinceLastRandom);
        }

        // Run the behaviour for the current state.
        state curState = getState();
        runBehaviour(curState);

        // Get and publish velocity commands
        velocity = getVelocity();
        ROS_INFO("State: %s", stateName[getState()].c_str());
        vel.angular.z = velocity.angular;
        vel.linear.x = velocity.linear;
        vel_pub.publish(vel);

        secondsElapsed = std::chrono::duration_cast<std::chrono::seconds>(std::chrono::system_clock::now()-contestStart).count();
        loop_rate.sleep();
    }
    
    return 0;
}
