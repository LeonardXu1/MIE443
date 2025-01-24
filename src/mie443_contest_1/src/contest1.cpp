#include <ros/console.h>
#include "ros/ros.h"
#include <geometry_msgs/Twist.h>
#include <kobuki_msgs/BumperEvent.h>
#include <sensor_msgs/LaserScan.h>
#include <nav_msgs/Odometry.h>
#include <tf/transform_datatypes.h>
#include <stdio.h>
#include <cmath>
#include <chrono>

#define N_BUMPER (3)
#define RAD2DEG(rad) ((rad)*180./M_PI)
#define DEG2RAD(deg) ((deg)*M_PI/180.)
//Odom variable
float ANGULAR=0.0;
float LINEAR=0.0;
float posX=0.0,posY=0.0,yaw=0.0;

//bumper variable 
uint8_t bumper[3]={kobuki_msgs::BumperEvent::RELEASED,kobuki_msgs::BumperEvent::RELEASED,kobuki_msgs::BumperEvent::RELEASED};
//Laser variable
float minLaserDist=std::numeric_limits<float>::infinity();
int32_t nLasers=0,desiredAngle=5,desiredNLasers=0;

void bumperCallback(const kobuki_msgs::BumperEvent::ConstPtr& msg)
{
	bumper[msg->bumper]=msg->state;

    //fill with your code
}

void laserCallback(const sensor_msgs::LaserScan::ConstPtr& msg)
{
	nLasers=(msg->angle_max-msg->angle_min)/msg->angle_increment;
    desiredNLasers=DEG2RAD(desiredAngle)/msg->angle_increment;
    ROS_INFO("Size of laser scan array: %i and size of offset: %i",nLasers,desiredAngle);
    if(desiredAngle*M_PI/180<msg->angle_max&& -desiredAngle*M_PI/180>msg->angle_min){
        for (uint32_t laser_idx=nLasers/2-desiredNLasers;laser_idx<nLasers/2+desiredNLasers;++laser_idx){
            minLaserDist=std::min(minLaserDist,msg->ranges[laser_idx]);
        }
    }
    else{
        for(uint32_t laser_idx=0;laser_idx<nLasers;++laser_idx){
            minLaserDist=std::min(minLaserDist,msg->ranges[laser_idx]);
        }
    }
    //fill with your code
}
void odomCallback(const nav_msgs::Odometry::ConstPtr& msg){
    posX=msg->pose.pose.position.x;
    posY=msg->pose.pose.position.y;
    yaw=tf::getYaw(msg->pose.pose.orientation);
    ROS_INFO("Position: (%f,%f) Orientation:%frad or%fdegrees.", posX, posY,yaw,RAD2DEG(yaw));
}
int main(int argc, char **argv)
{
    ros::init(argc, argv, "image_listener");
    ros::NodeHandle nh;

    ros::Subscriber bumper_sub = nh.subscribe("mobile_base/events/bumper", 10, &bumperCallback);//declaration of subscribers from bumper
    ros::Subscriber laser_sub = nh.subscribe("scan", 10, &laserCallback);//from sensor

    ros::Publisher vel_pub = nh.advertise<geometry_msgs::Twist>("cmd_vel_mux/input/teleop", 1);//advertise wheel speed
    ros::Subscriber odom=nh.subscribe("odom",1,&odomCallback);
    ros::Rate loop_rate(10);

    geometry_msgs::Twist vel;//twist class

    // contest count down timer
    std::chrono::time_point<std::chrono::system_clock> start;//timer
    start = std::chrono::system_clock::now();
    uint64_t secondsElapsed = 0;

    float angular = 0.0;
    float linear = 0.0;

    while(ros::ok() && secondsElapsed <= 480) {
        ros::spinOnce();
        //fill with your code

        //bumper code example
         bool any_bumper_pressed=false;
        for (uint32_t b_idx=0;b_idx<N_BUMPER;++b_idx){
            any_bumper_pressed|=(bumper[b_idx]==kobuki_msgs::BumperEvent::PRESSED);
        }
            if(posX<0.5&&yaw<M_PI/12&&!any_bumper_pressed){
                ANGULAR=0;
                LINEAR=0.2;
            }
            else if(yaw<M_PI/2&&posX>0.5&&!any_bumper_pressed){
                ANGULAR=M_PI/6;
                LINEAR=0;
            }
            else{
                ANGULAR=0;
                LINEAR=0;
                break;
            }

        //laser example code 
        // ROS_INFO("POsition: (%f,%f) orientation:%f degrees Range:%f",posX,posY, yaw*180/M_PI,minLaserDist);
        // if(posX<0.5&&yaw<M_PI/12&&!any_bumper_pressed&&minLaserDist>0.7){
        //     ANGULAR=0;
        //     LINEAR=0.2;
        // }
        // else if (yaw <M_PI/2&&posX>0.5&&!any_bumper_pressed&&minLaserDist>0.5){
        //     ANGULAR=M_PI/6;
        //     LINEAR=0;
        // }
        // else if (minLaserDist>1.&&!any_bumper_pressed){
        //     LINEAR=0.1;
        //     if(yaw<17/36*M_PI||posX>0.6){
        //         ANGULAR=M_PI/12.;
        //     }
        //     else if (yaw<19/36*M_PI||posX<0.4){
        //         ANGULAR=-M_PI/12.;
        //     }
        //     else{
        //         ANGULAR=0;
        //     }
        // }
        // else{
        //     ANGULAR=0;
        //     LINEAR=0;
        // }



        vel.angular.z = ANGULAR;
        vel.linear.x = LINEAR;
        vel_pub.publish(vel);

        // The last thing to do is to update the timer.
        secondsElapsed = std::chrono::duration_cast<std::chrono::seconds>(std::chrono::system_clock::now()-start).count();
        loop_rate.sleep();
    }

    return 0;
}
