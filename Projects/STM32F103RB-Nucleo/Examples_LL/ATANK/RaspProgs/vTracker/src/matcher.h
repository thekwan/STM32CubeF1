#include <string>
#include <opencv2/opencv.hpp>

#include "log.h"
#include "frame.h"

class KeyFrame {
public:
    KeyFrame(cv::Mat image, std::vector<cv::Point2f> &kpoints);
    ~KeyFrame(void);
private:
    cv::Mat _image;
    std::vector<cv::Point2f> _kpoints;
    //std::vector<cv::KeyPoint>  _keypoints;  // feature Keypoints
    //std::vector<cv::Mat> _descriptors;      // feature Descriptors
};

class Matcher {
public:
    Matcher(int keyFrameCount);
    ~Matcher(void);
    int getKeyFrameCount(void) {return _keyFrames.size();}
    void addKeyFrame(std::string imageFname, std::string pointFname, 
            std::string indexFname);
private:
    std::vector<KeyFrame> _keyFrames;
};
