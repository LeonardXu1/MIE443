#ifndef PATHPLANNING
#define PATHPLANNING

#include "boxes.h"
#include <ros/console.h>
#include "ros/ros.h"
#include <vector>
#include <cmath>
#include <limits>
#include <iostream>
#include <algorithm>
class pathPlanning{
    public: 
    std::vector<int>pathPlan(std::vector<float> startPos, Boxes boxCor);

    private:
    float CalDistance(float dx,float dy);
   
    std::vector<int>nearNeighbour(std::vector<std::vector<float>>distanceMatrix);
    std::vector<int>bruteForce(std::vector<std::vector<float>>distanceMatrix);

};
#endif