#ifndef BUMPER_CONTROLLER
#define BUMPER_CONTROLLER

#include <kobuki_msgs/BumperEvent.h>
#include <random>
#include "rConfig.h"
#include "stateMachine.h"
#include "movement.h"
#include "scanning.h"

void bumperCallback(const kobuki_msgs::BumperEvent::ConstPtr& msg); // reads the kobuki messages

string bumperPressedPosition(); // checks which bumper is pressed "LEFT" "RIGHT" "CENTER" or "NONE"

bool isBumperPressed(); // returns true if any bumper is pressed

void bumperBehaviour();

#endif