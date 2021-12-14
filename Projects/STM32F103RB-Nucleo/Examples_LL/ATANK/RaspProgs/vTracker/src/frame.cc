#include "frame.h"
#include "log.h"

Frame::Frame(cv::Mat image, bool resize, int maxFeatureNum) {
    _image = image;
    _maxFeatureNum = maxFeatureNum;

    if (resize) {
        cv::resize(_image, _image, cv::Size(_image.cols/2, _image.rows/2));
    }
    cv::cvtColor(_image, _image_gray, cv::COLOR_BGR2GRAY);

    cv::Ptr<cv::ORB> detector = cv::ORB::create(_maxFeatureNum);
    detector->detectAndCompute(_image_gray, cv::noArray(), _keypoint, _descriptor);

    FRAME_DBG_PRINT("Detected feature Num = %d", _descriptor.size().height);
}

Frame::~Frame(void) {
}

cv::Mat Frame::getImageFeature(void) {
    cv::Mat img;
    cv::drawKeypoints(_image, _keypoint, img, 
              cv::Scalar(0,0,255), cv::DrawMatchesFlags::DEFAULT);
    return img;
}
