#include "../include/movement.h"

// Odometry and Speed
posi posAbs;
posi posSave;
velo vel;

void odomCallback(const nav_msgs::Odometry::ConstPtr& msg){
    posAbs.x = msg->pose.pose.position.x;
    posAbs.y = msg->pose.pose.position.y;
    posAbs.yaw = tf::getYaw(msg->pose.pose.orientation); // radians
    // ROS_INFO("Position: (%f,%f) Orientation:%frad or%fdegrees.", posAbs.x, posAbs.y, posAbs.yaw, RAD2DEG(posAbs.yaw));
}

posi getAbsPos(){
    return posAbs;
}

void savePos(){
    posSave = posAbs;
}

velo getVelocity(){
    return vel;
}

float calcDistance(posi pos1, posi pos2){
    float dx = pos2.x - pos1.x;
    float dy = pos2.y - pos1.y;
    float distance = sqrt(dx*dx + dy*dy);
    return distance;
}

float calcRotation(posi pos1, posi pos2, bool CW){
    float yaw1;
    float yaw2;
    float rotation;
    if(CW){
        yaw1 = pos1.yaw;
        yaw2 = pos2.yaw;
    }
    else{
        yaw1 = pos2.yaw;
        yaw2 = pos1.yaw;
    }

    if(yaw1 > yaw2){
        rotation = (M_PI*2) - yaw1 + yaw2;
    }
    else{
        rotation = yaw2 - yaw1;
    }
    return rotation;
}

bool moveDistance(float distance, float speed, bool forward) {
    float travel = calcDistance(posSave, posAbs);
    if(travel >= distance){
        vel.linear = 0.0;
        return true;
    }

    if(forward){
        vel.linear = speed;
    }
    else{
        vel.linear = -speed;
    }
    return false;
}

bool moveAngle(float angle, float speed, bool CW){ // does not support rotation >= 360deg
    float travel = calcRotation(posSave, posAbs, CW);

    if(travel >= angle){
        vel.angular = 0.0;
        return true;
    }

    if(CW){
        vel.angular = speed;
    }
    else{
        vel.angular = -speed;
    }
    return false;
}

void moveLinearSpeed(float speed){
    vel.linear = speed;
}

void moveAngularSpeed(float speed){
    vel.angular = speed;
}


