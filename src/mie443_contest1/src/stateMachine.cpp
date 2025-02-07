#include "../include/stateMachine.h"

state curState = CYCLE_STATE; // current state of robot
int step = 0;

// returns true if state1 has a higher priority than state2
bool checkPriority(state state1, state state2){
    if(state1 <= state2){ // compares the enum value for a higher priority (lower value --> higher priority)
        return true;
    }
    return false;
}

// returns the current state
state getState(){
    return curState;
}

void resetState(){
    curState = CYCLE_STATE;
    step = 0;
}

int getStep(){
    return step;
}

void takeStep(){
    step++;
    ROS_INFO("Step: %i", step);
}

//checks and changes the state of the robot
void setState(state newState){
    if(newState != curState && checkPriority(newState, curState)){ // checks if the new state is "new" and the priority of the new state
        if(newState == BUMPER_STATE){
            curState = BUMPER_STATE;
        }
        else if(newState == EXPLORE_STATE){
            curState = EXPLORE_STATE;
        }
        step = 0;
    }
}

