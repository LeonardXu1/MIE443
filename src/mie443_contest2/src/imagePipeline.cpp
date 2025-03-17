#include <imagePipeline.h>

#define CONFIDENCE_THRESHOLD 0.05
#define MIN_REQUIRED_KEYPOINTS 50
ImagePipeline::ImagePipeline(ros::NodeHandle &n)
{
    image_transport::ImageTransport it(n);
    sub = it.subscribe(IMAGE_TOPIC, 1, &ImagePipeline::imageCallback, this);
    isValid = false;
}

void ImagePipeline::imageCallback(const sensor_msgs::ImageConstPtr &msg)
{

    try
    {

        if (isValid)
        {
            img.release();
        }
        img = (cv_bridge::toCvShare(msg, IMAGE_TYPE)->image).clone();
        isValid = true;
        // Show the live webcam feed in a window titled "webcam"
        cv::imshow("webcam", img);
        cv::waitKey(1);
    }
    catch (cv_bridge::Exception &e)
    {
        std::cout << "ERROR: Could not convert from " << msg->encoding.c_str()
                  << " to " << IMAGE_TYPE.c_str() << "!" << std::endl;
        isValid = false;
    }
}

int ImagePipeline::getTemplateID(Boxes &boxes)
{
    int template_id = -1;
    if (!isValid)
    {
        std::cout << "ERROR: INVALID IMAGE!" << std::endl;
    }
    else if (img.empty() || img.rows <= 0 || img.cols <= 0)
    {
        std::cout << "ERROR: VALID IMAGE, BUT STILL A PROBLEM EXISTS!" << std::endl;
        std::cout << "img.empty():" << img.empty() << std::endl;
        std::cout << "img.rows:" << img.rows << std::endl;
        std::cout << "img.cols:" << img.cols << std::endl;
    }
    else
    {

        SURFIntitialize(boxes);
        template_id=confidenceLevel(boxes);
        cv::imshow("view", img);
        cv::waitKey(10);
    }
    return template_id;
}

struct ImageData
{
    cv::Mat greyImage;                   // Grayscale template image
    std::vector<cv::KeyPoint> keypoints; // Detected keypoints
    cv::Mat descriptors;                 // Feature descriptors
};

// vector of image data global variable to access proccessed templates
std::vector<ImageData> processedImages;

int ImagePipeline::SURFIntitialize(Boxes &boxes)
{
    // STEP 1: configuration of SURF detector for initial template analysis
    Ptr<SURF> detector = SURF::create();
    detector->setHessianThreshold(400);

    // STEP 2: import process templates
    int numTemplates = boxes.templates.size();
    processedImages.clear(); // ensure empty
    // ImageData imgData;
    // STEP 3: import template123 from boxes
    for (int i = 0; i < numTemplates; i++)
    {

        ImageData imgData;
        // load image
        imgData.greyImage = boxes.templates[i];
        // cv::cvtColor(boxes.templates[i], imgData.greyImage, cv::COLOR_RGBA2GRAY,0);
        // confirm image loaded
        if (imgData.greyImage.empty())
        {
            std::cout << "ERROR: Could not load template image " << boxes.templates[i] << std::endl;
            return -1;
        }

        // detect keypoints and compute descriptors
        detector->detectAndCompute(imgData.greyImage, noArray(), imgData.keypoints, imgData.descriptors);

        // confirm keypoints and descriptors were detected
        if (imgData.keypoints.empty() || imgData.descriptors.empty())
        {
            std::cout << "ERROR: Could not detect keypoints or compute descriptors for template image " << boxes.templates[i] << std::endl;
            return -1;
        }
        processedImages.push_back(imgData); // store processed image data in global variable
        // cv::imshow("grey Image",imgData.greyImage);
        // std::cout<< " image "<< i << std::endl;
        // cv::imshow("stored in globalVariable", processedImages[i].greyImage);
    }
    // cv::imshow("stored in globalVariable", processedImages[1].greyImage);
    return 0; // Succesfull in image retrieval and processing
}

int ImagePipeline::confidenceLevel(Boxes &boxes)
{
    int template_id = -1;
    int numTemplates = boxes.templates.size();

    ImageData liveImage;

    if (!isValid)
    {
        std::cout << "ERROR: INVALID IMAGE!inside of ID2" << std::endl;
        return -1;
    }
    else if (img.empty() || img.rows <= 0 || img.cols <= 0)
    {
        std::cout << "ERROR: VALID IMAGE, BUT STILL A PROBLEM EXISTS! inside of ID2" << std::endl;
        std::cout << "img.empty():" << img.empty() << std::endl;
        std::cout << "img.rows:" << img.rows << std::endl;
        std::cout << "img.cols:" << img.cols << std::endl;
        return -1;
    }
    

        // Check if templates are loaded
        for (size_t i = 0; i < numTemplates; ++i)
        {
            if (processedImages[i].greyImage.empty())
            {
                std::cout << "ERROR: Could not load template " << i + 1 << "!" << std::endl;
                return -1;
            }
        }
        // Check if liveimg is loaded
        if (img.empty())
        {
            std::cout << "ERROR: Could not load live img!" << std::endl;
            return -1;
        }

        // Initialize the SURF detector
        auto detector = cv::xfeatures2d::SURF::create();
        // auto matcher = cv::DescriptorMatcher::create(FLANNBASED);
        //  std::vector<cv::KeyPoint> keypoints_img;
        //  cv::Mat descriptors_img;

        liveImage.greyImage = img;

        // Detect keypoints and compute descriptors for the captured image
        detector->detectAndCompute(liveImage.greyImage, cv::noArray(), liveImage.keypoints, liveImage.descriptors);
        //white or blank pic
        // if(liveImage.keypoints.size()<MIN_REQUIRED_KEYPOINTS){
        //     std::cout<<"Prob blank or black image"<<liveImage.keypoints.size()<<"< required matching points"<<std::endl;
        //     return -1;
        // }
        // cv::imshow("stored in globalVariable", liveImage.greyImage);

        // int numTemplates = boxes.templates.size();
        std::vector<cv::DMatch> bestMatches;

        double max_confidence = 0.0;
        for (size_t i = 0; i < numTemplates; ++i)
        {

            // Match descriptors using FLANN matcher
            cv::FlannBasedMatcher matcher;
            std::vector<std::vector<cv::DMatch>> knnMatches;
            if(processedImages[i].descriptors.empty()||liveImage.descriptors.empty()){
                continue;
            }
            matcher.knnMatch(processedImages[i].descriptors, liveImage.descriptors, knnMatches,2);
            const float ratio_thresh=0.75f;
            std::vector<DMatch>goodMatches;
            for(size_t j=0;j<knnMatches.size();j++){
                if(knnMatches[j][0].distance<ratio_thresh*knnMatches[j][1].distance){
                    goodMatches.push_back(knnMatches[j][0]);
                }
            }
            // Calculate confidence score based on match distances
            
            double confidence = 0.0;
            if (goodMatches.size() >= 4) {  // Need at least 4 points for homography
                // Simple ratio of good matches to keypoints
                confidence = (double)goodMatches.size() / 
                            (double)std::min(processedImages[i].keypoints.size(), 
                                            liveImage.keypoints.size());
                
                // Extract point correspondences for geometric verification
                std::vector<cv::Point2f> objPoints, scenePoints;
                for (const auto& match : goodMatches) {
                    objPoints.push_back(processedImages[i].keypoints[match.queryIdx].pt);
                    scenePoints.push_back(liveImage.keypoints[match.trainIdx].pt);
                }
                
                // Find homography and count inliers
                std::vector<uchar> inliersMask;
                cv::Mat H = cv::findHomography(objPoints, scenePoints, cv::RANSAC, 3.0, inliersMask);
                
                // Count inliers (geometric verification)
                int inliers = cv::countNonZero(inliersMask);
                
                // Adjust confidence by inlier ratio
                if (goodMatches.size() > 0) {
                    confidence *= (double)inliers / (double)goodMatches.size();
                }
            }
            
            // Update best match if current confidence is higher
            if (confidence > max_confidence) {
                max_confidence = confidence;
                bestMatches = goodMatches;
                template_id = i;
            }
        }
        
        // draw image
        if (template_id >= 0 && max_confidence > CONFIDENCE_THRESHOLD) {
            std::cout << "Template ID: " << template_id << ", Confidence: " << max_confidence << std::endl;
            
            // Generate and display match visualization
            cv::Mat matchVisualization = drawSceneMatches(
                liveImage.greyImage,
                processedImages[template_id].greyImage,
                processedImages[template_id].keypoints,
                bestMatches,
                liveImage.keypoints
            );
            
            // Display the visualization
           
        } else {
            template_id = -2;  // No good match found
            std::cout << "No confident match found prob blank. Best confidence: " << max_confidence << std::endl;
        }
        
        return template_id;
    }


cv::Mat ImagePipeline::drawSceneMatches(
    cv::Mat& imageScene, 
    cv::Mat& imgObject, 
    std::vector<cv::KeyPoint>& keypointsObject,
    std::vector<cv::DMatch>& matches,
    std::vector<cv::KeyPoint>& keypointsScene) {
    
    // Create output image
    cv::Mat imageMatches;
    
    // Draw matches between object and scene
    cv::drawMatches(
        imgObject, keypointsObject, 
        imageScene, keypointsScene, 
        matches, imageMatches, 
        cv::Scalar::all(-1), cv::Scalar::all(-1),
        std::vector<char>(),
        cv::DrawMatchesFlags::DEFAULT
    );
    
    // Extract matching points for homography calculation
    std::vector<cv::Point2f> obj;
    std::vector<cv::Point2f> scene;
    cv::Mat display_copy=imageMatches.clone();
    for (size_t i = 0; i < matches.size(); i++) {
        // Get the keypoints from the good matches
        obj.push_back(keypointsObject[matches[i].queryIdx].pt);
        scene.push_back(keypointsScene[matches[i].trainIdx].pt);
        cv::Point2f point1=keypointsObject[matches[i].queryIdx].pt;
        cv::Point2f point2=keypointsScene[matches[i].trainIdx].pt;
        
        cv::line(display_copy,point1, point2, cv::Scalar(0,255,0),2);
    }
    
    // Only proceed if we have enough points for homography
    if (obj.size() >= 4) {
        // Find homography matrix
        cv::Mat H = cv::findHomography(obj, scene, cv::RANSAC,3);
        
        // Define corners of object to be detected
        if(!H.empty()){
        std::vector<cv::Point2f> objCorners(4);
        objCorners[0] = cv::Point2f(0, 0);
        objCorners[1] = cv::Point2f((float)imgObject.cols, 0);
        objCorners[2] = cv::Point2f((float)imgObject.cols, (float)imgObject.rows);
        objCorners[3] = cv::Point2f(0, (float)imgObject.rows);
        
        // Transform object corners to scene coordinates
        std::vector<cv::Point2f> sceneCorners(4);
        cv::perspectiveTransform(objCorners, sceneCorners, H);
        //std::cout<<"drawing homography"<<std::endl;
        // Draw lines between corners (object boundaries)
        cv::line(imageMatches, 
                sceneCorners[0] + cv::Point2f((float)imgObject.cols, 0),
                sceneCorners[1] + cv::Point2f((float)imgObject.cols, 0),
                cv::Scalar(0, 255, 0), 4);
        cv::line(imageMatches, 
                sceneCorners[1] + cv::Point2f((float)imgObject.cols, 0),
                sceneCorners[2] + cv::Point2f((float)imgObject.cols, 0),
                cv::Scalar(0, 255, 0), 4);
        cv::line(imageMatches, 
                sceneCorners[2] + cv::Point2f((float)imgObject.cols, 0),
                sceneCorners[3] + cv::Point2f((float)imgObject.cols, 0),
                cv::Scalar(0, 255, 0), 4);
        cv::line(imageMatches, 
                sceneCorners[3] + cv::Point2f((float)imgObject.cols, 0),
                sceneCorners[0] + cv::Point2f((float)imgObject.cols, 0),
                cv::Scalar(0, 255, 0), 4);
    }
    else{
        std::cout<<"not enough point for homography"<<std::endl;

    }
}
else{
    std::cout<<"not enough object for homography"<<std::endl;
}
    cv::namedWindow("Template matching",cv::WINDOW_NORMAL);    
    cv::imshow("Template Matching",imageMatches);
    cv::waitKey(10);
    return imageMatches;
}