#ifndef CLIFF_SENSOR
#define CLIFF_SENSOR

#include <kobuki_msgs/CliffEvent.h>  // For cliff sensor messages
#include <geometry_msgs/Twist.h>     // For robot motion (vel_pub)
#include <sound_play/sound_play.h>   // For playing sounds (optional)
#include <movement.h>
#include <rConfig.h>
#include <stateMachine.h>
#include <rConfig.h>
#include <header.h>

void cliffCallback(const kobuki_msgs::CliffEvent::ConstPtr& msg);
bool isLifted();
void liftedBehaviour(sound_play::SoundClient &sc, ros::Publisher &vel_pub);
#endif