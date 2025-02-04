#ifndef STATE_MACHINE
#define STATE_MACHINE

#include "rConfig.h"

void setState(state newState); // Checks if new state should be set and sets it

state getState(); // gets the current state

#endif