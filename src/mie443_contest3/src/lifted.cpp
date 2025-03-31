#include "../include/lifted.h"
int cliff_states[3] = {0, 0, 0};  // [LEFT, CENTER, RIGHT]
bool soundPlaying=false;


void cliffCallback(const kobuki_msgs::CliffEvent::ConstPtr& msg) {
    // Update the state of the current sensor
    cliff_states[msg->sensor] = msg->state;  // msg->sensor is 0,1, or 2

}

bool isLifted()
{
    // ROS_INFO("cliff states 1 %d | cliff states 2 %d | cliff states 3 %d",cliff_states[0], cliff_states[1],cliff_states[2]);
    if (cliff_states[0] == 1 || cliff_states[1] == 1 || cliff_states[2] == 1){
        ROS_INFO("islifted true: %s", isLifted ? "true":"false");

        return true;
    }
    else{
        ROS_INFO("islifted false: %s", isLifted ? "true":"false");

        return false;
    }
    

}

void liftedBehaviour(sound_play::SoundClient &sc, ros::Publisher &vel_pub){
    bool taskComplete;
    int step = getStep();
    geometry_msgs::Twist vel;
    if (step == 0){
        if(soundPlaying==false){
			sc.playWave("/home/thursday2023/catkin_ws/src/mie443_contest3/sounds/sound.wav");
			soundPlaying=true;
		}
        sleep(3);
		sc.stopWave("/home/thursday2023/catkin_ws/src/mie443_contest3/sounds/sound.wav");
		takeStep();
    }
    else if (step == 1){
        moveAngularSpeed(0.5, CW);
        takeStep();
    }
    else {
        resetState();
    }

}