#include <string>
#include <opencv2/opencv.hpp>

#include "log.h"
#include "frame.h"

class KeyPoint {
public:
    KeyPoint(std::vector<cv::Point2f> &pts);
    ~KeyPoint(void);
private:
    std::vector<cv::Point2f> _points;
    std::vector<cv::KeyPoint>  _keypoints;  // feature Keypoints
    std::vector<cv::Mat> _descriptors;      // feature Descriptors
};

class Matcher {
public:
    Matcher(int keyFrameCount);
    ~Matcher(void);
    int getImageCount(void) {return _images.size();}
    int getKeyPointCount(void) {return _keyPoints.size();}
    void addImage(std::string fname);
    void addKeyPoint(std::string fname);
private:
    std::vector<cv::Mat> _images;
    std::vector<KeyPoint> _keyPoints;
};
