#include "../include/pathPlanning.h"
int const NUMBER_OF_POINT=5;
int const NUMBER_OF_BOXES=5;
float dx;
float dy;
float pathPlanning::CalDistance(float dx, float dy){
    float distance=std::sqrt(std::pow(dx,2)+std::pow(dy,2));
    return distance;
}
std::vector<int>pathPlanning::pathPlan(std::vector<float> startPos, Boxes boxCor){//startPos is 1D: 1*3, boxPos is 2D: 5*3
    std::vector<std::vector<float>>distanceMatrix;
    std::vector<std::vector<float>>combinedData;
    std::vector<std::vector<float>> boxPos;
    boxPos.reserve(boxCor.coords.size());
    for(const auto&row:boxCor.coords){
        std::vector<float>doublerow(row.begin(),row.end());
        boxPos.push_back(doublerow);
    }
    combinedData.push_back(startPos);
    combinedData.insert(combinedData.end(),boxPos.begin(),boxPos.end());
    distanceMatrix.resize(NUMBER_OF_POINT+1,std::vector<float>(NUMBER_OF_POINT+1,0));
    for(int i=0;i<NUMBER_OF_POINT+1;i++){//go through the row
        for(int j=0;j<NUMBER_OF_POINT+1;j++){//go through the col
          if(i==j){
            distanceMatrix[i][j]=std::numeric_limits<float>::infinity();
          }
          else{
            dx=combinedData[i][0]-combinedData[j][0];
            dy=combinedData[i][1]-combinedData[j][1];
            distanceMatrix[i][j]=CalDistance(dx,dy);
          }
        }

    }
    std::vector<int>resultPath=bruteForce(distanceMatrix);
    for(int i=0;i<resultPath.size();i++){
        ROS_INFO("Path plan is %i",resultPath[i]);
    }
    return resultPath;



}

std::vector<int>pathPlanning::nearNeighbour(std::vector<std::vector<float>>distanceMatrix){
    std::vector<std::vector<float>>referenceMatrix=distanceMatrix;
    std::vector<int>path={0,0,0,0,0};
    //path.reserve(NUMBER_OF_POINT);
    int currentLocation=0;
    float totalDistance=0;
    for(int i=0;i<path.size();i++){
        int nearestLocation=-1;
        float minDistance=std::numeric_limits<float>::infinity();
        for(int j=0;j<NUMBER_OF_POINT+1;j++){
            if(referenceMatrix[currentLocation][j]<minDistance){
                nearestLocation=j;
                minDistance=referenceMatrix[currentLocation][j];
            }
        }
        path[i]=nearestLocation;
        totalDistance=distanceMatrix[currentLocation][nearestLocation]+totalDistance;
        for(int z=0;z<NUMBER_OF_POINT+1;z++){
            referenceMatrix[currentLocation][z]=std::numeric_limits<float>::infinity();
            referenceMatrix[z][currentLocation]=std::numeric_limits<float>::infinity();

        }
        if(minDistance==std::numeric_limits<float>::infinity()){
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
    totalDistance+=distanceMatrix[currentLocation][0];
    std::cout<<totalDistance<<std::endl;
    return path;

}
std::vector<int>pathPlanning::bruteForce(std::vector<std::vector<float>>distanceMatrix){
    std::vector<std::vector<float>>referenceMatrix=distanceMatrix;
    std::vector<int>path={0,0,0,0,0};
    //path.reserve(NUMBER_OF_POINT);
    std::vector<int>boxNumber={1,2,3,4,5};
    

    
    float minDistance=std::numeric_limits<float>::infinity();
    int count=0;
     do{   
        float weight=0;        
        weight+=referenceMatrix[0][boxNumber[0]];//initial point to go
        for(int j=0;j<boxNumber.size()-1;j++){
            weight+=referenceMatrix[boxNumber[j]][boxNumber[j+1]];//from inital point to another point to another point
        }
        weight+=referenceMatrix[boxNumber.back()][0];//last point return to home
        std::cout<<weight<<std::endl;
        if(weight<minDistance){
            minDistance=weight;
            path=boxNumber;
        }
        count++;
    }while(std::next_permutation(boxNumber.begin(),boxNumber.end()));
 
        
        
    ROS_INFO("count %i",count);
    std::cout<<minDistance<<std::endl;
    // for(int i=0;i<path.size();i++){
    //    std::cout<<path[i]<<"";
    // }
    // std::cout<<std::endl;
    return path;

}
