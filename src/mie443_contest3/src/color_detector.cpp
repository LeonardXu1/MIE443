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
    cv::Mat hsv;
    cv::cvtColor(image, hsv, cv::COLOR_BGR2HSV);

    // Define color ranges in HSV
    cv::Scalar lowerBlue(100, 150, 50), upperBlue(140, 255, 255);
    cv::Scalar lowerRed1(0, 100, 100), upperRed1(10, 255, 255);
    cv::Scalar lowerRed2(160, 100, 100), upperRed2(179, 255, 255);
    cv::Scalar lowerGreen(40, 70, 70), upperGreen(80, 255, 255);
    cv::Scalar lowerYellow(20, 100, 100), upperYellow(30, 255, 255);

    // Create masks
    cv::Mat blueMask, redMask1, redMask2, redMask, greenMask, yellowMask;
    cv::inRange(hsv, lowerBlue, upperBlue, blueMask);
    cv::inRange(hsv, lowerRed1, upperRed1, redMask1);
    cv::inRange(hsv, lowerRed2, upperRed2, redMask2);
    redMask = redMask1 | redMask2;
    cv::inRange(hsv, lowerGreen, upperGreen, greenMask);
    cv::inRange(hsv, lowerYellow, upperYellow, yellowMask);

    // Count pixels
    int blueCount = cv::countNonZero(blueMask);
    int redCount = cv::countNonZero(redMask);
    int greenCount = cv::countNonZero(greenMask);
    int yellowCount = cv::countNonZero(yellowMask);

    // Determine the dominant color
    std::string detectedColor = "none";
    int maxCount = 0;

    if (blueCount > maxCount) { maxCount = blueCount; detectedColor = "blue"; }
    if (redCount > maxCount)  { maxCount = redCount;  detectedColor = "red"; }
    if (greenCount > maxCount){ maxCount = greenCount;detectedColor = "green"; }
    if (yellowCount > maxCount){ maxCount = yellowCount; detectedColor = "yellow"; }

    // Print result
    ROS_INFO_STREAM("Detected color: " << detectedColor << " (Pixels: " << maxCount << ")");

    // Return true only for blue (still your main trigger)
    return (blueCount > 500);
}


bool ColorDetector::isColorTriggered() const
{
    return colorTriggered_;
}
