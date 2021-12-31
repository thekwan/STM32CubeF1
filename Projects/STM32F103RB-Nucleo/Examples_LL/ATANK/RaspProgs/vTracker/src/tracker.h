#include <string>
#include <opencv2/opencv.hpp>

#include "log.h"
#include "frame.h"

class Tracker {
public:
    Tracker(std::string videoUrl, float scale = 1.0);
    ~Tracker(void) { }
    void tracking(int skip_frame, int maxKeyPoints = 100);
private:
    int _frameCount;
    float _scale;
    std::string _videoUrl;
    cv::VideoCapture _cap;
};
