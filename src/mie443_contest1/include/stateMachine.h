#ifndef STATE_MACHINE
#define STATE_MACHINE

#include "rConfig.h"
#include "movement.h"


state getState(); // gets the current state

void setState(state newState); // Checks if new state should be set and sets it

void updateState(); // runs the set state

#endif