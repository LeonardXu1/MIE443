#include "../include/stateMachine.h"

// defines the possible robot states and priority
map<string, int> state = {{"BLOCKED_STATE", 0}, {"LOSING_TRACK_STATE", 1}, {"LIFTED_STATE", 2}, {"ANNOYED_STATE", 2},{"FOLLOWING_STATE", 2}, {"CYCLE_STATE", 3}};

string curState = "FOLLOWING_STATE"; // current state of robot
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
    ROS_INFO("Step: %i", step);
    return step;
}

void takeStep(){
    step++;
    // ROS_INFO("Step: %i", step);
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
        if(newState == "BLOCKED_STATE"){
            curState = "BLOCKED_STATE";
        }
        else if(newState == "LOSING_TRACK_STATE"){
            curState = "LOSING_TRACK_STATE";
        }
        else if(newState == "LIFTED_STATE"){
            curState = "LIFTED_STATE";
        }
        else if(newState == "FOLLOWING_STATE"){
            curState = "FOLLOWING_STATE";
        }
        ROS_INFO("      ----------  %s  ----------", newState.c_str());

        step = 0;
    }
}

