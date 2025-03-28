#pragma once

#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/opencv.hpp>

class ColorDetector
{
public:
    ColorDetector(ros::NodeHandle& nh);

    bool isColorTriggered() const;

private:
    image_transport::ImageTransport it_;
    image_transport::Subscriber sub_;

    bool colorTriggered_;
    int detectionCount_;
    const int detectionThreshold_ = 1;  // Number of consecutive detections required

    void imageCallback(const sensor_msgs::ImageConstPtr& msg);
    bool detectBlue(const cv::Mat& image);
};
