#include <header.h>
#include <ros/package.h>
#include <imageTransporter.hpp>
#include <chrono>
#include <movement.h>
#include <stateMachine.h>
#include <string.h>
#include <kobuki_msgs/BumperEvent.h>
#include <kobuki_msgs/CliffEvent.h>
using namespace std;

geometry_msgs::Twist follow_cmd;
uint8_t bumpers[3] = {kobuki_msgs::BumperEvent::RELEASED, kobuki_msgs::BumperEvent::RELEASED, kobuki_msgs::BumperEvent::RELEASED};
int cliffs[3] = {0, 0, 0};
string world_state;

/*	----------	EMOTION TRIGGERS	----------	*/

float counterZigZag = 0;
float storeAngular = 0;
uint64_t storeTime = 0;

bool detectZigZag(double time){
	float angular = follow_cmd.angular.z;

	// checks if change in direction
	if((storeAngular > 0 and angular < 0) or (storeAngular < 0 and angular > 0)){
		if(angular > 0.03 || angular < -0.03){
			counterZigZag += 1;
			ROS_INFO("Zig Zag Zount: %f", counterZigZag);
		}
	}
	// checks if zigzagging is true
	if(counterZigZag >= 5){
		counterZigZag = 0;
		storeAngular = angular;
		storeTime = time;
		return true;
	}
	
	// checks how much time has past
	if(time - storeTime > 20){
		ROS_INFO("Reset Zig Zag Counter");
		counterZigZag = 0;
		storeTime = time;
	}
	storeAngular = angular;
	return false;
}

bool isBumperPressed(){
    for (uint32_t b_idx = 0; b_idx < N_BUMPER; ++b_idx)
    {
        if (bumpers[b_idx] == kobuki_msgs::BumperEvent::PRESSED)
        {
            return true;
        }
    }
    return false;
}

bool isLifted(){
    // ROS_INFO("cliffs 1 %d | cliffs 2 %d | cliffs 3 %d",cliffs[0], cliffs[1],cliffs[2]);
    if (cliffs[0] == 1 || cliffs[1] == 1 || cliffs[2] == 1){
        return true;
    }
	return false;
}

bool isLost(){
	if(follow_cmd.linear.x == 0  && follow_cmd.angular.z == 0){
		return true;
	}
	return false;
}



/*	----------	DECISION MAKER	----------	*/

void decisionMaker(double time){
    string currentState = getState();
    if(isBumperPressed()){
        setState("BUMPER_STATE");
    }
	else if(isLifted()){
		setState("LIFTED_STATE");
	}
	else if(detectZigZag(time)){ 
		setState("ZIGZAG_STATE");
	}
	else if(isLost()){
		setState("LOST_STATE");
	}
	else{
		setState("FOLLOW_STATE");
	}
}



/*	----------	ROS CALLBACKS	----------	*/

void followerCB(const geometry_msgs::Twist msg){
    follow_cmd = msg;
}

void bumperCB(const kobuki_msgs::BumperEvent::ConstPtr &msg){
	bumpers[msg->bumper] = msg->state;
}

void cliffCB(const kobuki_msgs::CliffEvent::ConstPtr& msg){
	cliffs[msg->sensor] = msg->state;
}

/*	----------	EMOTION BEHAVIOURS	----------	*/



/*	----------	MAIN CODE	----------	*/

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
	ros::Subscriber cliff = nh.subscribe("mobile_base/events/cliff", 10, &cliffCB);
	ros::Subscriber odom = nh.subscribe("odom", 1, &odomCallback);

    // contest count down timer
	ros::Rate loop_rate(10);
    std::chrono::time_point<std::chrono::system_clock> start;
    start = std::chrono::system_clock::now();
    uint64_t secondsElapsed = 0;

	imageTransporter rgbTransport("camera/image/", sensor_msgs::image_encodings::BGR8); //--for Webcam
	//imageTransporter rgbTransport("camera/rgb/image_raw", sensor_msgs::image_encodings::BGR8); //--for turtlebot Camera
	imageTransporter depthTransport("camera/depth_registered/image_raw", sensor_msgs::image_encodings::TYPE_32FC1);


	geometry_msgs::Twist vel;
	vel.angular.z = 0.0;
	vel.linear.x = 0.0;

	sc.playWave(path_to_sounds + "sound.wav");
	ros::Duration(0.5).sleep();

	while(ros::ok() && secondsElapsed <= 480){		
		ros::spinOnce();

		decisionMaker(secondsElapsed);
		world_state = getState();

		
		if(world_state == "FOLLOW_STATE"){
			vel_pub.publish(follow_cmd);
		}
		else if(world_state == "BUMPER_STATE"){
			resetState();
		}
		else if(world_state == "LIFT_STATE"){
			resetState();
		}
		else if(world_state == "LOST_STATE"){
			resetState();
		}
		else if(world_state == "ZIGZAG_STATE"){
			resetState();
		}

		secondsElapsed = std::chrono::duration_cast<std::chrono::seconds>(std::chrono::system_clock::now()-start).count();
		loop_rate.sleep();
	}

	return 0;
}
