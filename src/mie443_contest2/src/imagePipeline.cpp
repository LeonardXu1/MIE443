#include <imagePipeline.h>

#define IMAGE_TYPE sensor_msgs::image_encodings::BGR8
#define IMAGE_TOPIC "camera/rgb/image_raw" // kinect:"camera/rgb/image_raw" webcam:"camera/image"

ImagePipeline::ImagePipeline(ros::NodeHandle& n) {
    image_transport::ImageTransport it(n);
    sub = it.subscribe(IMAGE_TOPIC, 1, &ImagePipeline::imageCallback, this);
    isValid = false;
}

void ImagePipeline::imageCallback(const sensor_msgs::ImageConstPtr& msg) {
    try {
        if(isValid) {
            img.release();
        }
        img = (cv_bridge::toCvShare(msg, IMAGE_TYPE)->image).clone();
        isValid = true;
    } catch (cv_bridge::Exception& e) {
        std::cout << "ERROR: Could not convert from " << msg->encoding.c_str()
                  << " to " << IMAGE_TYPE.c_str() << "!" << std::endl;
        isValid = false;
    }    
}

int ImagePipeline::getTemplateID(Boxes& boxes) {
    int template_id = -1;
    if(!isValid) {
        std::cout << "ERROR: INVALID IMAGE!" << std::endl;
    } else if(img.empty() || img.rows <= 0 || img.cols <= 0) {
        std::cout << "ERROR: VALID IMAGE, BUT STILL A PROBLEM EXISTS!" << std::endl;
        std::cout << "img.empty():" << img.empty() << std::endl;
        std::cout << "img.rows:" << img.rows << std::endl;
        std::cout << "img.cols:" << img.cols << std::endl;
    } else {
        /***YOUR CODE HERE***/
        // Use: boxes.templates
        cv::imshow("view", img);
        cv::waitKey(10);
    }  
    return template_id;
}
//test


//struct for image data

struct ImageData {
    cv::Mat greyImage;            // Grayscale template image
    std::vector<cv::KeyPoint> keypoints; // Detected keypoints
    cv::Mat descriptors;              // Feature descriptors
};

//vector of image data global variable to access proccessed templates
std::vector<ImageData> processedImages;


int SURFIntitialize(Boxes& boxes)
{
    //STEP 1: configuration of SURF detector for initial template analysis
    Ptr<SURF> detector = SURF::create();
    detector->setHessianThreshold(400);

    //STEP 2: import process templates
    int numTemplates = boxes.templates.size();
    processedImages.clear(); //ensure empty

    //STEP 3: import template123 from boxes
    for (int i = 0; i < numTemplates; i++)
    {
        ImageData imgData;

        //load image
        imgData.greyImage = imread(boxes.templates[i], IMREAD_GRAYSCALE);

        //confirm image loaded
        if (imgData.greyImage.empty())
        {
            std::cout << "ERROR: Could not load template image " << boxes.templates[i] << std::endl;
            return -1;
        }

        //detect keypoints and compute descriptors
        detector->detectAndCompute(imgData.greyImage, noArray(), imgData.keypoints, imgData.descriptors);

        //confirm keypoints and descriptors were detected
        if (imgData.keypoints.empty() || imgData.descriptors.empty())
        {
            std::cout << "ERROR: Could not detect keypoints or compute descriptors for template image " << boxes.templates[i] << std::endl;
            return -1;
        }
        processedImages.push_back(imgData); //store processed image data in global variable
    }
    return 0;// Succesfull in image retrieval and processing
}


#include <imagePipeline.h>
#include <opencv2/xfeatures2d.hpp>
#include <opencv2/features2d.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/highgui.hpp>
#include <filesystem>

#define IMAGE_TYPE sensor_msgs::image_encodings::BGR8
#define IMAGE_TOPIC "camera/rgb/image_raw" // kinect:"camera/rgb/image_raw" webcam:"camera/image"

ImagePipeline::ImagePipeline(ros::NodeHandle& n) {
    image_transport::ImageTransport it(n);
    sub = it.subscribe(IMAGE_TOPIC, 1, &ImagePipeline::imageCallback, this);
    isValid = false;
}

void ImagePipeline::imageCallback(const sensor_msgs::ImageConstPtr& msg) {
    try {
        if(isValid) {
            img.release();
        }
        img = (cv_bridge::toCvShare(msg, IMAGE_TYPE)->image).clone();
        isValid = true;
        // Show the live webcam feed in a window titled "webcam"
        cv::imshow("webcam", img);
        cv::waitKey(1);
    } catch (cv_bridge::Exception& e) {
        std::cout << "ERROR: Could not convert from " << msg->encoding.c_str()
                  << " to " << IMAGE_TYPE.c_str() << "!" << std::endl;
        isValid = false;
    }    
}

int ImagePipeline::getTemplateID(Boxes& boxes) {
    int template_id = -1;
    if(!isValid) {
        std::cout << "ERROR: INVALID IMAGE!" << std::endl;
    } else if(img.empty() || img.rows <= 0 || img.cols <= 0) {
        std::cout << "ERROR: VALID IMAGE, BUT STILL A PROBLEM EXISTS!" << std::endl;
        std::cout << "img.empty():" << img.empty() << std::endl;
        std::cout << "img.rows:" << img.rows << std::endl;
        std::cout << "img.cols:" << img.cols << std::endl;
    } else {
        // Load template images from boxes_database folder
        std::vector<cv::Mat> templates;
        for (const auto& entry : std::filesystem::directory_iterator("boxes_database")) {
            templates.push_back(cv::imread(entry.path().string(), cv::IMREAD_COLOR));
        }

        // Check if templates are loaded
        for (size_t i = 0; i < templates.size(); ++i) {
            if (templates[i].empty()) {
                std::cout << "ERROR: Could not load template " << i + 1 << "!" << std::endl;
                return -1;
            }
        }

        // Initialize the SURF detector
        auto detector = cv::xfeatures2d::SURF::create();
        std::vector<cv::KeyPoint> keypoints_img;
        cv::Mat descriptors_img;

        // Detect keypoints and compute descriptors for the captured image
        detector->detectAndCompute(img, cv::noArray(), keypoints_img, descriptors_img);

        double max_confidence = 0.0;
        for (size_t i = 0; i < templates.size(); ++i) {
            std::vector<cv::KeyPoint> keypoints_template;
            cv::Mat descriptors_template;

            // Detect keypoints and compute descriptors for the template image
            detector->detectAndCompute(templates[i], cv::noArray(), keypoints_template, descriptors_template);

            // Match descriptors using FLANN matcher
            cv::FlannBasedMatcher matcher;
            std::vector<cv::DMatch> matches;
            matcher.match(descriptors_img, descriptors_template, matches);

            // Calculate confidence score based on match distances
            double confidence = 0.0;
            for (const auto& match : matches) {
                confidence += match.distance;
            }
            confidence = 1.0 / (confidence / matches.size());

            if (confidence > max_confidence) {
                max_confidence = confidence;
                template_id = i;
            }
        }

        // Also display the processed image in another window (optional)
        cv::imshow("Processed Image", img);
        cv::waitKey(10);

        // Print confidence score and chosen template ID
        std::cout << "Template ID: " << template_id << ", Confidence: " << max_confidence << std::endl;
    }  
    return template_id;
}