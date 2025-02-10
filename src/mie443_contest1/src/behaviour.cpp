#include "../include/behaviour.h"
#include "../include/stateMachine.h"
#include "../include/movement.h"
#include <cstdlib>
#include <ctime>
#include <chrono>
#include <ros/ros.h>


void exploreBehaviour(){
    moveLinearSpeed(FAST_LINEAR, FORWARD);
}



// Constants for movement (tweak as needed)
const float ROTATION_SPEED = 0.5;      // radians per second
const float FORWARD_SPEED  = 0.2;       // meters per second
const float FORWARD_DISTANCE = 0.5;     // 50 centimeters

// Internal static variables for RANDOM state control
bool randomInProgress = false;
std::chrono::time_point<std::chrono::system_clock> randomStartTime;
double randomAngle = 0.0;       // target rotation angle in radians
int rotationDirection = 1;      // +1 for CW, -1 for CCW
int randomStep = 0;             // 0 = rotation phase, 1 = forward movement

void randomBehaviour() {
    auto currentTime = std::chrono::system_clock::now();

    // Initialization when entering RANDOM state
    if (!randomInProgress) {
        randomInProgress = true;
        randomStep = 0;
        std::srand(std::time(0));  // (Consider moving this seeding to main if preferred)

        // Generate a random angle between 0 and 180 degrees (converted to radians)
        int angleDeg = std::rand() % 181; // 0 to 180 degrees
        randomAngle = angleDeg * (M_PI / 180.0);
        // Randomly choose a rotation direction: CW or CCW
        rotationDirection = (std::rand() % 2 == 0) ? 1 : -1;
        
        // Save the starting position for rotation measurement
        savePos();
        randomStartTime = currentTime;
        ROS_INFO("RANDOM state initiated: target angle = %d deg (%f rad), direction = %s", 
                 angleDeg, randomAngle, (rotationDirection == 1 ? "CW" : "CCW"));
    }

    // Abort the random maneuver if it takes longer than 10 seconds (possibly due to preemption)
    double elapsed = std::chrono::duration_cast<std::chrono::seconds>(currentTime - randomStartTime).count();
    if (elapsed > 10) {
        ROS_WARN("RANDOM state aborted after 10 seconds (preempted by higher priority state).");
        setState(EXPLORE_STATE);
        randomInProgress = false;
        lastRandomTime = currentTime; // that may stop the immediate re-triggering
        return;
    }

    if (randomStep == 0) {
        // --- Rotation Phase ---
        // moveAngle() uses savePos() as the starting point.
        bool rotationDone = moveAngle(randomAngle, ROTATION_SPEED, rotationDirection);
        if (rotationDone) {
            randomStep = 1;
            savePos();  // Reset start position for the forward movement measurement
            ROS_INFO("Rotation complete. Starting forward movement of 50 cm.");
        }
    }
    else if (randomStep == 1) {
        // --- Forward Movement Phase ---
        bool forwardDone = moveDistance(FORWARD_DISTANCE, FORWARD_SPEED, FORWARD);
        if (forwardDone) {
            ROS_INFO("Forward movement complete. Returning to EXPLORE state.");
            resetState();
            randomInProgress = false;
            lastRandomTime = currentTime; // that may stop the immediate re-triggering
        }
    }
}
