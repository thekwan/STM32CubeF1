#include <string>
#include <opencv2/opencv.hpp>

#include "log.h"
#include "frame.h"

class MatchPoint {
public:
    MatchPoint(void) { }
    ~MatchPoint(void) { }
    void addMatchPair(std::pair<int,int> pair) {
        _index_pair.push_back(pair);
    }
    int getMatchPairNum(void) {
        return _index_pair.size();
    }
    const std::vector<std::pair<int,int>> &getPair(void) {
        return _index_pair;
    }
private:
    std::vector<std::pair<int,int>> _index_pair;
};

class KeyFrame {
public:
    KeyFrame(cv::Mat image, std::vector<cv::Point2f> &kpoints);
    ~KeyFrame(void);
    void displayImage(void);
    const cv::Mat& getImage(void);
    const std::vector<cv::Point2f>& getKeyPoints(void);
private:
    cv::Mat _image;
    std::vector<cv::Point2f> _kpoints;
    //std::vector<cv::KeyPoint>  _keypoints;  // feature Keypoints
    //std::vector<cv::Mat> _descriptors;      // feature Descriptors
    //std::vector<int> _wpointId;
};

class Matcher {
public:
    Matcher(void);
    ~Matcher(void);
    int getKeyFrameCount(void) {return _keyFrames.size();}
    int addKeyFrame(std::string imageFname, std::string pointFname);
    int addMatchIndex(int keyFrameIndex, std::string indexFname);
    void checkKeyFrames(void);
    void drawMatchKeyFrames(void);
private:
    std::vector<KeyFrame> _keyFrames;
    std::map<std::pair<int,int>,MatchPoint> _matches;
    //std::vector<cv::Point3f> _wpoints;
};
