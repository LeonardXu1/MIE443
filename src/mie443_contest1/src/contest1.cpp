#include <geometry_msgs/Twist.h>
#include <stdio.h>
#include <cmath>
#include <chrono>
#include <cstring>

#include "../include/rConfig.h"
#include "../include/stateMachine.h"
#include "../include/bumper.h"

velo velocity;

int main(int argc, char **argv)
{
    ros::init(argc, argv, "image_listener");
    ros::NodeHandle nh;

    ros::Subscriber bumper_sub = nh.subscribe("mobile_base/events/bumper", 10, &bumperCallback); //declaration of subscribers from bumper
    // ros::Subscriber laser_sub = nh.subscribe("scan", 10, &laserCallback);//from sensor

    ros::Publisher vel_pub = nh.advertise<geometry_msgs::Twist>("cmd_vel_mux/input/teleop", 1); //advertise wheel speed
    ros::Subscriber odom = nh.subscribe("odom", 1, &odomCallback);
    ros::Rate loop_rate(10);

    geometry_msgs::Twist velT; //twist class

    // contest count down timer
    std::chrono::time_point<std::chrono::system_clock> start;//timer
    start = std::chrono::system_clock::now();
    uint64_t secondsElapsed = 0;

    while(ros::ok() && secondsElapsed <= 480) {
        ros::spinOnce();
        
        if(isBumperPressed() == true) {
            setState(BUMPER_STATE);
        }
        else{
            setState(EXPLORE_STATE);
        }

        updateState();

        velocity = getVelocity();

        ROS_INFO("State: %s", stateName[getState()].c_str());
        // ROS_INFO("Linear Velocity: %0.2f   | Angular Velocity: %0.2f", velocity.linear, velocity.angular);

        velT.angular.z = velocity.angular;
        velT.linear.x = velocity.linear;
        vel_pub.publish(velT);

        // The last thing to do is to update the timer.
        secondsElapsed = std::chrono::duration_cast<std::chrono::seconds>(std::chrono::system_clock::now()-start).count();
        loop_rate.sleep();
    }
    
    return 0;
}