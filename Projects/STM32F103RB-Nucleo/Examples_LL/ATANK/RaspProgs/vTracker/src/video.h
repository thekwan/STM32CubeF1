#include <string>
#include <opencv2/opencv.hpp>

#include "log.h"
#include "frame.h"

#ifndef __VIDEO_H__
#define __VIDEO_H__

class Video {
public:
    Video(std::string fname, float scale = 1.0);
    ~Video(void);
    int openVideoFile(std::string fileName, float scale);
    int tracking(int skip_frame = 0, int maxKeyPoints = 100);
    int playVideo(int interval_ms);
private:
    int _frameCount;
    std::vector<Frame> _videoFrames;
    std::string _videoFileName;
};

#endif // __VIDEO_H__
