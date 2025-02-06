#include "../include/stateMachine.h"

state curState = CYCLE_STATE; // current state of robot
int subState = 0;

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
    subState = 0;
}

void bumperState(){
    bool taskComplete;
    if(subState == 0){
        savePos();
        subState++;
    }
    else if(subState == 1) {
        taskComplete = moveDistance(0.1, SLOW_LINEAR, false);
        if(taskComplete){
            subState++;
        }
    }
    else if(subState == 2){
        savePos();
        subState++;
    }
    else if(subState == 3) {
        taskComplete = moveAngle(M_PI/2, SLOW_ANGULAR, true);
        if(taskComplete){
            subState++;
        }
    }
    else {
        resetState();
    }
}

void exploreState(){
    moveLinearSpeed(FAST_LINEAR);
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
        subState = 0;
    }
}

void updateState(){
    if(curState == BUMPER_STATE){
        bumperState();
    }
    else if(curState == EXPLORE_STATE){
        exploreState();
    }
}

