#include "annoyes.h"

bool soundPlayed_ = false;
void zigzagBehaviour(sound_play::SoundClient &sc, ros::Publisher &vel_pub){
    bool taskComplete;
    int step = getStep();
    geometry_msgs::Twist vel;
    if (step == 0){
        savePos();
        takeStep();
        
    }
    else if (step == 1){
        sleep(2.0);

        //if (taskComplete){
            takeStep();
        //}
    }
    else if (step == 2){
		if(soundPlayed_==false){
			sc.playWave("/home/thursday2023/catkin_ws/src/mie443_contest3/sounds/sound.wav");
			soundPlayed_=true;
		}
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
    else if(step==3){
        taskComplete=moveAngle(DEG2RAD(5), SLOW_ANGULAR, CW);
        if (taskComplete){
                     takeStep();
                }
    }
    else if(step==4){
        taskComplete=moveAngle(DEG2RAD(5), SLOW_ANGULAR, CCW);
        if (taskComplete){
                     takeStep();
                }
    }
    else {

        resetState();
    }
}


