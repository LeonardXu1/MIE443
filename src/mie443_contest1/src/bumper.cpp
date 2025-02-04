#include "../include/bumper.h"

uint8_t bumper[3]={kobuki_msgs::BumperEvent::RELEASED,kobuki_msgs::BumperEvent::RELEASED,kobuki_msgs::BumperEvent::RELEASED};

void bumperCallback(const kobuki_msgs::BumperEvent::ConstPtr& msg)
{
	bumper[msg->bumper]=msg->state;
}

std::string bumperPressedPosition() {
    std::string position;
    if(bumper[kobuki_msgs::BumperEvent::LEFT]==kobuki_msgs::BumperEvent::PRESSED){
        position = "LEFT";
        return position;
    }
     if(bumper[kobuki_msgs::BumperEvent::RIGHT]==kobuki_msgs::BumperEvent::PRESSED){
        position = "RIGHT";
        return position;
    }
     if(bumper[kobuki_msgs::BumperEvent::CENTER]==kobuki_msgs::BumperEvent::PRESSED){
        position = "CENTER";
        return position;
    }
    return "NONE";
}

bool isBumperPressed() {
    for (uint32_t b_idx=0; b_idx<N_BUMPER; ++b_idx){
        if(bumper[b_idx] == kobuki_msgs::BumperEvent::PRESSED){
            return true;
        }
    }
    return false;
}