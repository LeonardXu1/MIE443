#pragma once

#include <image_transport/image_transport.h>
#include <std_msgs/String.h>
#include <opencv2/core.hpp>
#include <cv.h>
#include <cv_bridge/cv_bridge.h>
#include <boxes.h>
#include <iostream>
#include "opencv2/calib3d.hpp"
#include "opencv2/highgui.hpp"
#include "opencv2/imgproc.hpp"
#include "opencv2/xfeatures2d.hpp"
#include "opencv2/features2d.hpp"

#define IMAGE_TYPE sensor_msgs::image_encodings::BGR8
#define IMAGE_TOPIC "camera/rgb/image_raw" // kinect:"camera/rgb/image_raw" webcam:"camera/image"

using namespace cv;
using namespace cv::xfeatures2d;
class ImagePipeline {
    private:
        cv::Mat img;
        bool isValid;
        image_transport::Subscriber sub;
    public:
        ImagePipeline(ros::NodeHandle& n);
        void imageCallback(const sensor_msgs::ImageConstPtr& msg);
        int getTemplateID(Boxes& boxes);
        int confidenceLevel(Boxes& boxes);
        int SURFIntitialize(Boxes& boxes);
        cv::Mat drawSceneMatches(
            cv::Mat& imageScene, 
            cv::Mat& imgObject, 
            std::vector<cv::KeyPoint>& keypointsObject,
            std::vector<cv::DMatch>& matches,
            std::vector<cv::KeyPoint>& keypointsScene);
};
