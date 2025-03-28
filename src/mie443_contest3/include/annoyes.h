#ifndef MOCEMENT_TRACKING
#define MOVEMENT_TRACKING

#include <kobuki_msgs/BumperEvent.h>
#include <movement.h>
#include <rConfig.h>
#include <stateMachine.h>
#include <header.h>

bool checkMovement(double timeElapsed,const geometry_msgs::Twist& cmd);
void zigzagBehaviour(sound_play::SoundClient &sc, ros::Publisher &vel_pub);
#endif