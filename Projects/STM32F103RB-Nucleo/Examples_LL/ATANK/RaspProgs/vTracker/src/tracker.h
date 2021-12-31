#include <string>
#include <opencv2/opencv.hpp>

#include "log.h"
#include "frame.h"

class Tracker {
    enum class kfDecision {
        CURRENT,
        PREVIOUS,
        NONE
    };
public:
    Tracker(std::string videoUrl, float scale = 1.0);
    ~Tracker(void) { }
    void tracking(int skip_frame, int maxKeyPoints = 100);
private:
    void saveKeyFrameImagePrev(void);
    void holdKeyFrameImage(cv::Mat &image, std::vector<cv::Point2f> &kpts);
    void saveKeyFrameImage(cv::Mat &image, std::vector<cv::Point2f> &kpts);
    void saveImage(std::string fname, cv::Mat &image);
    void savePoints(std::string fname, std::vector<cv::Point2f> &pts);
    kfDecision selectKeyFrameImage(int tckPtNum, int newPtNum);

    int _frameCount;
    int _keyFrameCount;
    float _scale;
    std::string _videoUrl;
    cv::VideoCapture _cap;
    // for save key frame image.
    cv::Mat _image_prev;
    std::vector<cv::Point2f> _kpts_prev;
};
