#ifndef STATE_MACHINE
#define STATE_MACHINE

#include "rConfig.h"

state getState(); // gets the current state

void bumperState();

void resetState();

int getStep();

void takeStep();

void setState(state newState); // Checks if new state should be set and sets it

#endif