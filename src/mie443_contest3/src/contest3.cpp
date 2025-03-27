#include <header.h>
#include <ros/package.h>
#include <imageTransporter.hpp>
#include <chrono>
#include <movement.h>
#include <stateMachine.h>
#include <blocked.h>
#include <string.h>
using namespace std;

geometry_msgs::Twist follow_cmd;
string world_state;
// void runBehaviour(string curState)
// {
//     if (curState == "BLOCKED_STATE")
//     {
//         blockBehaviour();
//     }
//     // else if (curState == "STUCK_STATE")
//     // {
//     //     stuckBehaviour();
//     // }
//     // else if (curState == "ROTATION_STATE")
//     // {
//     //     rotateBehaviour();
//     // }
//     else if (curState == "EXPLORE_STATE")
//     {
//         scanningBehaviour();
//     }
// }


//  Logic for changing states
void decisionMaker()
{
    string currentState = getState();
    if (isBumperPressed() == true){ // checks if any bumpers are pressed
        setState("BLOCKED_STATE");
		return;
    }
    // else if (currentState != STUCK_STATE && checkIfStuck(getAbsPos(), timeElapsed) == true){
    //     setState("STUCK_STATE");
    // }
    // if (isNewSpace() == true){ 
    //     setState("ROTATION_STATE");
    // }
	else{
		setState("FOLLOWING_STATE");
	}
    
}
void followerCB(const geometry_msgs::Twist msg){
    follow_cmd = msg;
}

// void bumperCB(const geometry_msgs::Twist msg){
//     //Fill with code
// 	if(msg.state==kobuki_msgs::BumperEvent::PRESSED){
// 		world_state=1;
		

// 	}
// }

//-------------------------------------------------------------

int main(int argc, char **argv)
{
	ros::init(argc, argv, "image_listener");
	ros::NodeHandle nh;
	sound_play::SoundClient sc;
	string path_to_sounds = ros::package::getPath("mie443_contest3") + "/sounds/";
	teleController eStop;

	//publishers
	ros::Publisher vel_pub = nh.advertise<geometry_msgs::Twist>("cmd_vel_mux/input/teleop",1);

	//subscribers
	ros::Subscriber follower = nh.subscribe("follower_velocity_smoother/smooth_cmd_vel", 10, &followerCB);
	ros::Subscriber bumper = nh.subscribe("mobile_base/events/bumper", 10, &bumperCB);
	ros::Subscriber odom = nh.subscribe("odom", 1, &odomCallback);

    // contest count down timer
	ros::Rate loop_rate(10);
    std::chrono::time_point<std::chrono::system_clock> start;
    start = std::chrono::system_clock::now();
    uint64_t secondsElapsed = 0;

	imageTransporter rgbTransport("camera/image/", sensor_msgs::image_encodings::BGR8); //--for Webcam
	//imageTransporter rgbTransport("camera/rgb/image_raw", sensor_msgs::image_encodings::BGR8); //--for turtlebot Camera
	imageTransporter depthTransport("camera/depth_registered/image_raw", sensor_msgs::image_encodings::TYPE_32FC1);


	double angular = 0.2;
	double linear = 0.0;

	geometry_msgs::Twist vel;
	vel.angular.z = angular;
	vel.linear.x = linear;

	sc.playWave(path_to_sounds + "sound.wav");
	ros::Duration(0.5).sleep();

	while(ros::ok() && secondsElapsed <= 480){		
		ros::spinOnce();
		decisionMaker();
		world_state = getState();
		if(world_state == "FOLLOWING_STATE"){
			//fill with your code
			//vel_pub.publish(vel);
			vel_pub.publish(follow_cmd);

		}else if(world_state == "BLOCKED_STATE"){
			blockBehaviour(sc, vel_pub);
			velS velocity = getVelocity();
			vel.angular.z = velocity.angular;
			vel.linear.x = velocity.linear;
			vel_pub.publish(vel);
			/*
			...
			...
			*/
		}
		secondsElapsed = std::chrono::duration_cast<std::chrono::seconds>(std::chrono::system_clock::now()-start).count();
		loop_rate.sleep();
	}

	return 0;
}
