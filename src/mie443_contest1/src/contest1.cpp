#include <geometry_msgs/Twist.h>
#include <nav_msgs/Odometry.h>
#include <tf/transform_datatypes.h>
#include <stdio.h>
#include <cmath>
#include <chrono>

#include "../include/rConfig.h"
#include "../include/stateMachine.h"
#include "../include/bumper.h"
#include "../include/behaviour.h"
#include "../include/stuck.h"
// intiates a velocity structure
velS velocity;

// calls the behaviour function related to the state
void runBehaviour(state curState){
    if(curState == BUMPER_STATE){
        bumperBehaviour();
    }
    // else if(curState==STUCK_STATE){
    //     stuckDetector.stuckBehaviour();
    // }
    else if(curState == EXPLORE_STATE){
        exploreBehaviour();
    }
}
stuckDetect stuckDetector;//can delete later once we delete the class
// Logic for changing states
void decisionMaker(double timeElapsed){
    if(isBumperPressed() == true) { // checks if any bumpers are pressed
        setState(BUMPER_STATE);
    }
    else if(stuckDetector.checkIfStuck(getAbsPos(), timeElapsed)==true){
        setState(STUCK_STATE);
    }
    else { // default state
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

    // contest count down timer
    std::chrono::time_point<std::chrono::system_clock> start;//timer
    start = std::chrono::system_clock::now();
    uint64_t secondsElapsed = 0;

    float angular = 0.0;
    float linear = 0.0;

    while(ros::ok() && secondsElapsed <= 480) {
        ros::spinOnce();

        // decides which state to be in
        decisionMaker(secondsElapsed);

        // runs the behaviour related to the state
        state curState = getState();
         ROS_INFO("State: %s", stateName[getState()].c_str());
        runBehaviour(curState);

        velocity = getVelocity();

       
        // ROS_INFO("Linear Velocity: %0.2f   | Angular Velocity: %0.2f", velocity.linear, velocity.angular);

        //sets the robot velocity
        vel.angular.z = velocity.angular;
        vel.linear.x = velocity.linear;
        vel_pub.publish(vel);

        // The last thing to do is to update the timer.
        secondsElapsed = std::chrono::duration_cast<std::chrono::seconds>(std::chrono::system_clock::now()-start).count();
        loop_rate.sleep();
    }
    
    return 0;
}