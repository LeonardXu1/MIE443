#include <imagePipeline.h>
#include <opencv2/xfeatures2d.hpp>


#define IMAGE_TYPE sensor_msgs::image_encodings::BGR8
#define IMAGE_TOPIC "camera/rgb/image_raw" // kinect:"camera/rgb/image_raw" webcam:"camera/image"

using namespace cv;
using namespace cv::xfeatures2d;




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

int ImagePipeline::getTemplateID(Boxes& boxes) 
{
    int template_id = -1;
    if(!isValid) {
        std::cout << "ERROR: INVALID IMAGE!" << std::endl;
    } else if(img.empty() || img.rows <= 0 || img.cols <= 0) {
        std::cout << "ERROR: VALID IMAGE, BUT STILL A PROBLEM EXISTS!" << std::endl;
        std::cout << "img.empty():" << img.empty() << std::endl;
        std::cout << "img.rows:" << img.rows << std::endl;
        std::cout << "img.cols:" << img.cols << std::endl;
    } else {
        
        SURFIntitialize(boxes);
        getTemplateID2(boxes);
        cv::imshow("view", img);
        cv::waitKey(10);
    }  
    return template_id;
}




struct ImageData {
    cv::Mat greyImage;            // Grayscale template image
    std::vector<cv::KeyPoint> keypoints; // Detected keypoints
    cv::Mat descriptors;              // Feature descriptors
};

//vector of image data global variable to access proccessed templates
std::vector<ImageData> processedImages;


int ImagePipeline::SURFIntitialize(Boxes& boxes)
{
    //STEP 1: configuration of SURF detector for initial template analysis
    Ptr<SURF> detector = SURF::create();
    detector->setHessianThreshold(400);

    //STEP 2: import process templates
    int numTemplates = boxes.templates.size();
    processedImages.clear(); //ensure empty
    //ImageData imgData;
    //STEP 3: import template123 from boxes
    for (int i = 0; i < numTemplates; i++)
    {
        
        ImageData imgData;
        //load image
        imgData.greyImage = boxes.templates[i];
        //cv::cvtColor(boxes.templates[i], imgData.greyImage, cv::COLOR_RGBA2GRAY,0);
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
        //cv::imshow("grey Image",imgData.greyImage);
        //std::cout<< " image "<< i << std::endl;
        //cv::imshow("stored in globalVariable", processedImages[i].greyImage);
        
    }
    //cv::imshow("stored in globalVariable", processedImages[1].greyImage);
    return 0;// Succesfull in image retrieval and processing
}





int ImagePipeline::getTemplateID2(Boxes& boxes) {
    int template_id = -1;
    int numTemplates = boxes.templates.size();
  

    ImageData liveImage;


    if(!isValid) {
        std::cout << "ERROR: INVALID IMAGE!inside of ID2" << std::endl;
    } else if(img.empty() || img.rows <= 0 || img.cols <= 0) {
        std::cout << "ERROR: VALID IMAGE, BUT STILL A PROBLEM EXISTS! inside of ID2" << std::endl;
        std::cout << "img.empty():" << img.empty() << std::endl;
        std::cout << "img.rows:" << img.rows << std::endl;
        std::cout << "img.cols:" << img.cols << std::endl;
    } else {

        // Check if templates are loaded
        for (size_t i = 0; i < numTemplates; ++i) {
            if (processedImages[i].greyImage.empty()) {
                std::cout << "ERROR: Could not load template " << i + 1 << "!" << std::endl;
                return -1;
            }
        }
        // Check if liveimg is loaded
        if (img.empty()) {
            std::cout << "ERROR: Could not load live img!" << std::endl;
            return -1;
        }
        

        
        // Initialize the SURF detector
        auto detector = cv::xfeatures2d::SURF::create();
        //auto matcher = cv::DescriptorMatcher::create(FLANNBASED);
        // std::vector<cv::KeyPoint> keypoints_img;
        // cv::Mat descriptors_img;

        liveImage.greyImage= img;
       

        // Detect keypoints and compute descriptors for the captured image
        detector->detectAndCompute(liveImage.greyImage, cv::noArray(), liveImage.keypoints, liveImage.descriptors);
        
        //cv::imshow("stored in globalVariable", liveImage.greyImage);
    
        // int numTemplates = boxes.templates.size();
        std::vector<cv::DMatch> bestMatch;

        double max_confidence = 0.0;
        for (size_t i = 0; i < numTemplates; ++i) {


            // Match descriptors using FLANN matcher
            cv::FlannBasedMatcher matcher;
            std::vector<cv::DMatch> matches;
            matcher.match( processedImages[i].descriptors, liveImage.descriptors, matches);

            // Calculate confidence score based on match distances
            double confidence = 0.0;
            for (const auto& match : matches) {
                confidence += match.distance;
            }
            confidence = 1.0 / (confidence / matches.size());

            if (confidence > max_confidence) {
                max_confidence = confidence;
                bestMatch.clear();
                bestMatch = matches;
                template_id = i;
            }
        }

        //Also display the processed image in another window (optional)
      // cv::drawMatches(liveImage.greyImage, liveImage.keypoints, processedImages[template_id].greyImage, processedImages[template_id].keypoints, bestMatch);
        

        // Print confidence score and chosen template ID
        std::cout << "Template ID: " << template_id << ", Confidence: " << max_confidence << std::endl;
    }  
    return template_id;
}




// [  9%] Built target kobuki_msgs_generate_messages_eus
// Scanning dependencies of target contest2
// [ 13%] Building CXX object mie443_contest2/CMakeFiles/contest2.dir/src/imagePipeline.cpp.o
// /home/thursday2023/catkin_ws/src/mie443_contest2/src/imagePipeline.cpp: In member function ‘int ImagePipeline::getTemplateID2(Boxes&)’:
// /home/thursday2023/catkin_ws/src/mie443_contest2/src/imagePipeline.cpp:155:9: error: ‘liveImage’ was not declared in this scope
//          liveImage.greyImage= img;
//          ^
// mie443_contest2/CMakeFiles/contest2.dir/build.make:158: recipe for target 'mie443_contest2/CMakeFiles/contest2.dir/src/imagePipeline.cpp.o' failed
// make[2]: *** [mie443_contest2/CMakeFiles/contest2.dir/src/imagePipeline.cpp.o] Error 1
// CMakeFiles/Makefile2:2267: recipe for target 'mie443_contest2/CMakeFiles/contest2.dir/all' failed
// make[1]: *** [mie443_contest2/CMakeFiles/contest2.dir/all] Error 2
// Makefile:138: recipe for target 'all' failed
// make: *** [all] Error 2
// Invoking "make -j4 -l4" failed
