#ifndef ROTATION_STATE_H
#define ROTATION_STATE_H
#include <ros/console.h>
#include "ros/ros.h"
#include <tf/transform_datatypes.h>
#include "stateMachine.h"
#include "movement.h"
#include "rConfig.h"
#include "bumper.h"
#include "stuck.h"
bool isRotationComplete();

void resetRotation();

void rotateBehaviour();
void saveStatePos(posS pos);

bool shouldRotate(posS currentStatePos);

#endif