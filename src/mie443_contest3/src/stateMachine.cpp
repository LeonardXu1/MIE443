#include "../include/stateMachine.h"

// defines the possible robot states and priority
map<string, int> state = {{"BUMPER_STATE", 0}, {"LIFTED_STATE", 1}, {"ZIGZAG_STATE", 2}, {"LOST_STATE", 3}, {"FOLLOW_STATE", 4}, {"CYCLE_STATE", 5}};

string curState = "FOLLOW_STATE"; // current state of robot
int step = 0;

// returns true if state1 has a higher priority than state2
bool checkPriority(string state1, string state2){
    int priority1 = state[state1];
    int priority2 = state[state2];
    if(priority1 < priority2){ // compares the enum value for a higher priority (lower value --> higher priority)
        return true;
    }
    return false;
}

// returns the current state
string getState(){
    return curState;
}

void resetState(){
    curState = "CYCLE_STATE";
    step = 0;
}

int getStep(){
    return step;
}

void takeStep(){
    step++;
    ROS_INFO("Step: %i", step);
}

void overrideStep(int newStep){
    step = newStep;
}

//checks and changes the state of the robot
void setState(string newState){
    if (!state.count(newState)){
        ROS_WARN("State Does Not Exist: %s", newState.c_str());
    }
    else if(newState != curState && checkPriority(newState, curState)){ // checks if the new state is "new" and the priority of the new state
        if(newState == "BUMPER_STATE"){
            curState = newState;
        }
        else if(newState == "LOST_STATE"){
            curState = newState;
        }
        else if(newState == "ZIGZAG_STATE"){
            curState = newState;
        }
        else if(newState == "LIFTED_STATE"){
            curState = newState;
        }
        else if(newState == "FOLLOW_STATE"){
            curState = newState;
        }
        else if(newState == "CYCLE_STATE"){
            curState = newState;
        }
        ROS_INFO("      ----------  %s  ----------", newState.c_str());

        step = 0;
    }
}

