#include "color_detector.h"

ColorDetector::ColorDetector(ros::NodeHandle& nh) : it_(nh), detectionCount_(0), colorTriggered_(false)
{
    sub_ = it_.subscribe("/webcam/image_raw", 1, &ColorDetector::imageCallback, this);
}

void ColorDetector::imageCallback(const sensor_msgs::ImageConstPtr& msg)
{
    try
    {
        cv::Mat bgr = cv_bridge::toCvShare(msg, "bgr8")->image;
        bool detected = detectBlue(bgr);

        if (detected)
        {
            detectionCount_++;
            if (detectionCount_ >= detectionThreshold_)
            {
                colorTriggered_ = true;
            }
        }
        else
        {
            detectionCount_ = 0;
            colorTriggered_ = false;
        }
    }
    catch (cv_bridge::Exception& e)
    {
        ROS_ERROR("cv_bridge exception: %s", e.what());
    }
}

bool ColorDetector::detectBlue(const cv::Mat& image)
{
    cv::Mat hsv, mask;
    cv::cvtColor(image, hsv, cv::COLOR_BGR2HSV);
    cv::Scalar lowerBlue(100, 150, 50);
    cv::Scalar upperBlue(140, 255, 255);
    cv::inRange(hsv, lowerBlue, upperBlue, mask);

    int nonZero = cv::countNonZero(mask);
    return nonZero > 500;  // Can tweak this threshold
}

bool ColorDetector::isColorTriggered() const
{
    return colorTriggered_;
}
