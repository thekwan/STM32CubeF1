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
    void saveLastStableKeyPtsImage(cv::Mat &image, 
            std::vector<cv::Point2f> &kpts);
    kfDecision selectKeyFrameImage(void);

    int _tckNum;
    int _kptNum;
    int _tckNum_prev;
    int _kptNum_prev;

    int _frameCount;
    int _keyFrameCount;
    int _keyFrameStopCount;
    float _scale;
    float _trackSuccRate;
    std::string _videoUrl;
    std::string _trackState, _trackState_prev;
    cv::VideoCapture _cap;
    // for save key frame image.
    cv::Mat _image_prev;
    std::vector<cv::Point2f> _kpts_prev;

    cv::Mat _image_last_stable;
    std::vector<cv::Point2f> _kpts_last_stable;
};
