#ifndef STATE_MACHINE
#define STATE_MACHINE

#include "rConfig.h"
#include "movement.h"
#include <map>
#include <string>

string getState(); // gets the current state

void resetState();

int getStep();

void takeStep();

void overrideStep(int newStep);

void setState(string newState); // Checks if new state should be set and sets it

#endif