#include "frame.h"
#include "log.h"

Frame::Frame(cv::Mat image, float scale) {
    _image = image;

    if (scale != 1.0) {
        cv::resize(_image, _image, 
                cv::Size(_image.cols * scale, _image.rows * scale));
    }
    cv::cvtColor(_image, _image_gray, cv::COLOR_BGR2GRAY);
}

Frame::~Frame(void) {
}

cv::Mat Frame::getImage(float scale) {
    cv::Mat _tmp;
    cv::resize(_image, _tmp, 
            cv::Size(_image.cols * scale, _image.rows * scale));
    return _tmp;
}

cv::Mat Frame::getEdgeImage(float scale) {
    cv::Mat _tmp, sobelX, sobelY;
    cv::resize(_image_gray, _tmp, 
            cv::Size(_image.cols * scale, _image.rows * scale));
    cv::Sobel(_tmp, sobelX, CV_8U, 1, 0);
    cv::Sobel(_tmp, sobelY, CV_8U, 0, 1);
    return abs(sobelX) + abs(sobelY);
}

int Frame::detectFeatures(int maxFeatureNum) {
    _maxFeatureNum = maxFeatureNum;

    cv::Ptr<cv::ORB> detector = cv::ORB::create(_maxFeatureNum);
    //detector->detectAndCompute(_image_gray, cv::noArray(), _keypoint, _descriptor);
    detector->detect(_image_gray, _keypoint);
    
    //FRAME_DBG_PRINT("Detected feature Num = %d", _descriptor.size().height);

    return _keypoint.size();
}

cv::Mat Frame::getImageWithFeature(void) {
    cv::Mat img;
    cv::drawKeypoints(_image, _keypoint, img, 
              cv::Scalar(0,0,255), cv::DrawMatchesFlags::DEFAULT);
    return img;
}

std::vector<cv::Point2f> Frame::getKeyPoints(int maxKeyPoints) {
    std::vector<cv::Point2f> pts;

    if (_keypoint.size() == 0) {
        detectFeatures(maxKeyPoints);
    }

    for(auto kpt : _keypoint) {
        if (kpt.octave == 0) {
            pts.emplace_back(kpt.pt);
        }
    }

    return pts;
}
