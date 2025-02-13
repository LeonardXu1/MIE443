#ifndef STATE_MACHINE
#define STATE_MACHINE

#include "rConfig.h"
#include "movement.h"
#include <map>

string getState(); // gets the current state

void bumperState();

void resetState();

int getStep();

void takeStep();

void overrideStep(int newStep);

void setState(state newState); // Checks if new state should be set and sets it
void setState(string newState); // Checks if new state should be set and sets it

#endif