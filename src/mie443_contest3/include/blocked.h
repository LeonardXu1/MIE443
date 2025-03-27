#ifndef BUMPER_CONTROLLER
#define BUMPER_CONTROLLER

#include <kobuki_msgs/BumperEvent.h>
#include <movement.h>
#include <rConfig.h>
#include <stateMachine.h>
#include <header.h>

void bumperCB(const kobuki_msgs::BumperEvent::ConstPtr& msg);
bool isBumperPressed();
void blockBehaviour(sound_play::SoundClient &sc, ros::Publisher &vel_pub);
#endif
