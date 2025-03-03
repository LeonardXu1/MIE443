#include "../include/pathPlanning.h"
int const NUMBER_OF_POINT=5;
int const NUMBER_OF_BOXES=5;
double dx;
double dy;
double pathPlanning::CalDistance(double dx, double dy){
    double distance=std::sqrt(std::pow(dx,2)+std::pow(dy,2));
    return distance;
}
std::vector<int>pathPlanning::pathPlan(std::vector<double> startPos, Boxes boxCor){//startPos is 1D: 1*3, boxPos is 2D: 5*3
    std::vector<std::vector<double>>distanceMatrix;
    std::vector<std::vector<double>>combinedData;
    std::vector<std::vector<double>> boxPos;
    boxPos.reserve(boxCor.coords.size());
    for(const auto&row:boxCor.coords){
        std::vector<double>doublerow(row.begin(),row.end());
        boxPos.push_back(doublerow);
    }
    combinedData.push_back(startPos);
    combinedData.insert(combinedData.end(),boxPos.begin(),boxPos.end());
    distanceMatrix.resize(NUMBER_OF_POINT+1,std::vector<double>(NUMBER_OF_POINT+1,0));
    for(int i=0;i<NUMBER_OF_POINT+1;i++){//go through the row
        for(int j=0;j<NUMBER_OF_POINT+1;j++){//go through the col
          if(i==j){
            distanceMatrix[i][j]=std::numeric_limits<double>::infinity();
          }
          else{
            dx=combinedData[i][0]-combinedData[j][0];
            dy=combinedData[i][1]-combinedData[j][1];
            distanceMatrix[i][j]=CalDistance(dx,dy);
          }
        }

    }
    std::vector<int>resultPath=nearNeighbour(distanceMatrix);
    for(int i=0;i<resultPath.size();i++){
        ROS_INFO("Path plan is %i",resultPath[i]);
    }
    return resultPath;



}

std::vector<int>pathPlanning::nearNeighbour(std::vector<std::vector<double>>distanceMatrix){
    std::vector<std::vector<double>>referenceMatrix=distanceMatrix;
    std::vector<int>path={0,0,0,0,0};
    //path.reserve(NUMBER_OF_POINT);
    int currentLocation=0;
    double totalDistance=0;
    for(int i=0;i<path.size();i++){
        int nearestLocation=-1;
        double minDistance=std::numeric_limits<double>::infinity();
        for(int j=0;j<NUMBER_OF_POINT+1;j++){
            if(referenceMatrix[currentLocation][j]<minDistance){
                nearestLocation=j;
                minDistance=referenceMatrix[currentLocation][j];
            }
        }
        path[i]=nearestLocation;
        totalDistance=distanceMatrix[currentLocation][nearestLocation]+totalDistance;
        for(int z=0;z<NUMBER_OF_POINT+1;z++){
            referenceMatrix[currentLocation][z]=std::numeric_limits<double>::infinity();
            referenceMatrix[z][currentLocation]=std::numeric_limits<double>::infinity();

        }
        if(minDistance==std::numeric_limits<double>::infinity()){
            ROS_INFO("finished calculating");
            break;
        }
        currentLocation=nearestLocation;
    }
    //ROS_INFO("Best distance is %d",totalDistance);
    // for(int i=0;i<path.size();i++){
    //    std::cout<<path[i]<<"";
    // }
    // std::cout<<std::endl;
    return path;

}