#ifndef BUMPER_CONTROLLER
#define BUMPER_CONTROLLER

#include <kobuki_msgs/BumperEvent.h>
#include <string>
#include <iostream>

#include <random>
#include "rConfig.h"

void bumperCallback(const kobuki_msgs::BumperEvent::ConstPtr& msg); // reads the kobuki messages

// void bumperMovement();

std::string bumperPressedPosition(); // checks which bumper is pressed "LEFT" "RIGHT" "CENTER" or "NONE"

bool isBumperPressed(); // returns true if any bumper is pressed

void bumperBehaviour();
std::string savedBumperPosition();

#endif