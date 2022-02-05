#include <opencv2/opencv.hpp>

#ifndef __FRAME_H__
#define __FRAME_H__

class Frame {
public:
    Frame(cv::Mat image, float scale = 1.0);
    ~Frame();
    cv::Mat getImage(void) {return _image;}
    cv::Mat getImage(float scale);
    cv::Mat getEdgeImage(float scale);
    cv::Mat getImageWithFeature(void);
    std::vector<cv::Point2f> getKeyPoints(int maxKeyPoints = 500);
    int getFeatureNum(void) {return _keypoint.size();}
    int detectFeatures(int maxFeatureNum);
    void writeIntoFile(std::string fname);
private:
    int _maxFeatureNum;
    cv::Mat _image, _image_gray;           // image frame(color, gray_scale)
    std::vector<cv::KeyPoint>  _keypoint;  // feature Keypoints
    cv::Mat _descriptor;                   // feature Descriptors
    cv::Point2f _gmv;                      // global motion vector
};

#endif  // __FRAME_H__
