#include "../include/blocked.h"
uint8_t bumper[3] = {kobuki_msgs::BumperEvent::RELEASED, kobuki_msgs::BumperEvent::RELEASED, kobuki_msgs::BumperEvent::RELEASED};
bool soundPlayed=false;
void bumperCB(const kobuki_msgs::BumperEvent::ConstPtr& msg){
    //Fill with code
	bumper[msg->bumper] = msg->state;
    
    
    if (msg->state == kobuki_msgs::BumperEvent::PRESSED) {
        
        setState("BLOCKED_STATE");
       
        // if (msg->bumper == 0) {
        //     bumpedPosition = "LEFT";
        // } else if (msg->bumper == 2) {
        //     bumpedPosition = "RIGHT";
        // } else {
        //     bumpedPosition = "CENTER";
        // }
    }
		
		

	}

bool isBumperPressed()
{
    for (uint32_t b_idx = 0; b_idx < N_BUMPER; ++b_idx)
    {
        if (bumper[b_idx] == kobuki_msgs::BumperEvent::PRESSED)
        {
            return true;
        }
    }
    return false;
}

void blockBehaviour(sound_play::SoundClient &sc, ros::Publisher &vel_pub){
	bool taskComplete;
    int step = getStep();
    geometry_msgs::Twist vel;
    if (step == 0){
        savePos();
        takeStep();
        bool soundplayed = false;
     
    }
    else if (step == 1){
		if(soundPlayed==false){
			sc.playWave("/home/thursday2023/catkin_ws/src/mie443_contest3/sounds/bruhh.wav");
			soundPlayed=true;
		}
        //quick pause to make the robot seem like its in shock
        ros::Duration(1).sleep();

        //basic evasive manuver
        vel.angular.z = 0.4;
        vel.linear.x = -0.25;
        vel_pub.publish(vel);
        vel.angular.z = -0.2;
        vel.linear.x = -0.1;
        vel_pub.publish(vel);


        sleep(3);
		sc.stopWave("/home/thursday2023/catkin_ws/src/mie443_contest3/sounds/sound.wav");
		takeStep();
    }
    // else if (step == 3){
    //     if (bumpedPosition == "LEFT"){
    //         taskComplete = moveAngle(DEG2RAD(50), SLOW_ANGULAR, CW);
    //     }
    //     else if (bumpedPosition == "RIGHT"){
    //         taskComplete = moveAngle(DEG2RAD(50), SLOW_ANGULAR, CCW);
    //     }
    //     else {
    //         taskComplete = moveAngle(DEG2RAD(90), SLOW_ANGULAR, turnDirectionLaser);
    //     }

    //     if (taskComplete){
    //         takeStep();
    //     }
    //}
    else {
        resetState();
    }
}