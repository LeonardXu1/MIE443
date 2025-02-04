#include <geometry_msgs/Twist.h>
#include <nav_msgs/Odometry.h>
#include <tf/transform_datatypes.h>
#include <stdio.h>
#include <cmath>
#include <chrono>

#include "../include/rConfig.h"
#include "../include/stateMachine.h"
#include "../include/bumper.h"

state robotState = CYCLE_STATE;

//Odom variable
float ANGULAR = 0.0;
float LINEAR = 0.0;
float posX = 0.0, posY = 0.0, yaw = 0.0;

void odomCallback(const nav_msgs::Odometry::ConstPtr& msg){
    posX = msg->pose.pose.position.x;
    posY = msg->pose.pose.position.y;
    yaw = tf::getYaw(msg->pose.pose.orientation);
    ROS_INFO("Position: (%f,%f) Orientation:%frad or%fdegrees.", posX, posY, yaw, RAD2DEG(yaw));
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

        if(bumperState() == true) {
            setState(BUMPER_STATE);
        }

        vel.angular.z = ANGULAR;
        vel.linear.x = LINEAR;
        vel_pub.publish(vel);

        // The last thing to do is to update the timer.
        secondsElapsed = std::chrono::duration_cast<std::chrono::seconds>(std::chrono::system_clock::now()-start).count();
        loop_rate.sleep();
    }
    
    return 0;
}