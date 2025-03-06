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
    std::vector<int>pathPlan(std::vector<double> startPos, Boxes boxCor);

    private:
    double CalDistance(double dx,double dy);
   
    std::vector<int>nearNeighbour(std::vector<std::vector<double>>distanceMatrix);
    std::vector<int>bruteForce(std::vector<std::vector<double>>distanceMatrix);

};
#endif