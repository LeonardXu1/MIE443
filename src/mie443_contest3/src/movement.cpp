#include "../include/movement.h"

// Odometry and Speed
posS posAbs;
posS posSave;
velS vel;
float rotationCount;

void odomCallback(const nav_msgs::Odometry::ConstPtr& msg){
    posAbs.x = msg->pose.pose.position.x;
    posAbs.y = msg->pose.pose.position.y;
    posAbs.yaw = tf::getYaw(msg->pose.pose.orientation); // radians
    // ROS_INFO("Position: (%f,%f) Orientation:%frad or%fdegrees.", posAbs.x, posAbs.y, posAbs.yaw, RAD2DEG(posAbs.yaw));
}

posS getAbsPos(){
    return posAbs;
}

void savePos(){
    posSave = posAbs;
    rotationCount = 0;
}

velS getVelocity(){
    return vel;
}

float calcDistance(posS pos1, posS pos2){
    float dx = pos2.x - pos1.x;
    float dy = pos2.y - pos1.y;
    float distance = sqrt(dx*dx + dy*dy);
    return distance;
}

float calcRotation(posS pos1, posS pos2, int direction){
    float yaw1 = pos1.yaw;
    float yaw2 = pos2.yaw;
    float rotation;

    if(direction == CCW){
        rotation = yaw2 - yaw1;
    }
    else{
        rotation = yaw1 - yaw2;
    }

    if(rotation < 0){
        rotation = rotation + M_PI*2;
    }
    
    if(fabs(rotation - rotationCount) > M_PI){
        rotation = rotationCount;
    }

    return rotation;
}

bool moveDistance(float distance, float speed, int direction) {
    float travel = calcDistance(posSave, posAbs);
    if(travel >= distance){
        vel.linear = 0.0;
        return true;
    }

    vel.linear = speed*direction;
    return false;
}

bool moveAngle(float angle, float speed, int direction){ // does not support rotation >= 360deg
    if(angle > M_PI*2) {angle = M_PI*2;}

    float travel = calcRotation(posSave, posAbs, direction);
    rotationCount = travel;

    // float angledeg = RAD2DEG(angle);
    // float traveldeg = RAD2DEG(travel);
    // float yawSave = RAD2DEG(posSave.yaw);
    // float yawCurr = RAD2DEG(posAbs.yaw);

    // ROS_INFO("Direction: %i | goal: %f  | travel: %0.2f | intial: %f | curr: %f", direction, angledeg, traveldeg, yawSave, yawCurr);
    
    if(travel >= angle){
        vel.angular = 0.0;
        return true;
    }

    vel.angular = speed*direction;
    return false;
}

void moveLinearSpeed(float speed, int direction){
    vel.linear = speed*direction;
}

void moveAngularSpeed(float speed, int direction){
    vel.angular = speed*direction;
}




