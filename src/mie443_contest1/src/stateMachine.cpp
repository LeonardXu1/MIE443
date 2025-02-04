#include "../include/stateMachine.h"

state currState; // current state of robot

// returns true if state1 has a higher priority than state2
bool checkPriority(state state1, state state2){
    if(state1 <= state2){ // compares the enum value for a higher priority (lower value --> higher priority)
        return true;
    }
    return false;
}

// returns the current state
state getState(){
    return currState;
}

void bumperState(){
    ROS_INFO("Bumper State");
}

void exploreState(){
    ROS_INFO("Explore State");
}

//checks and changes the state of the robot
void setState(state newState){
    if(newState != currState && checkPriority(newState, currState)){ // checks if the new state is "new" and the priority of the new state
        if(newState == BUMPER_STATE){
            currState = BUMPER_STATE;
            bumperState();
        }
        if(newState == EXPLORE_STATE){
            currState = EXPLORE_STATE;
            exploreState();
        }
    }
}



