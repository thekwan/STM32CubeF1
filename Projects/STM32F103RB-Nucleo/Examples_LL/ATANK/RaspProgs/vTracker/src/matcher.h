#include <string>
#include <opencv2/opencv.hpp>

#include "log.h"
#include "frame.h"

class MatchPoint {
public:
    MatchPoint(void) { }
    ~MatchPoint(void) { }
    std::vector<std::pair<int,int>> index_pair;
private:
};

class KeyFrame {
public:
    KeyFrame(cv::Mat image, std::vector<cv::Point2f> &kpoints);
    ~KeyFrame(void);
    void displayImage(void);
private:
    cv::Mat _image;
    std::vector<cv::Point2f> _kpoints;
    //std::vector<cv::KeyPoint>  _keypoints;  // feature Keypoints
    //std::vector<cv::Mat> _descriptors;      // feature Descriptors
    //std::vector<MatchPoint> _matches;
};

class Matcher {
public:
    Matcher(int keyFrameCount);
    ~Matcher(void);
    int getKeyFrameCount(void) {return _keyFrames.size();}
    void addKeyFrame(std::string imageFname, std::string pointFname);
    void addMatchIndex(int keyFrameIndex, std::string indexFname);
    void checkKeyFrames(void);
private:
    std::vector<KeyFrame> _keyFrames;
    //std::vector<MatchPoint> _matches;
    std::map<std::pair<int,int>,MatchPoint> _matches;
};
