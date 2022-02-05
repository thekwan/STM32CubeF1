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
    Matcher(void);
    ~Matcher(void);
    int getKeyFrameCount(void) {return _keyFrames.size();}
    int addKeyFrame(std::string imageFname, std::string pointFname);
    int addMatchIndex(int keyFrameIndex, std::string indexFname);
    void checkKeyFrames(void);
private:
    std::vector<KeyFrame> _keyFrames;
    //std::vector<MatchPoint> _matches;
    std::map<std::pair<int,int>,MatchPoint> _matches;
};
