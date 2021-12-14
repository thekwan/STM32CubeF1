#include <string>
#include <opencv2/opencv.hpp>

#include "log.h"
#include "frame.h"

class Video {
public:
    Video(std::string fname);
    ~Video(void);
    int openVideoFile(std::string fileName);
    int playVideo(int interval_ms);
private:
    int _frameCount;
    std::vector<Frame> _videoFrames;
    std::string _videoFileName;
};
