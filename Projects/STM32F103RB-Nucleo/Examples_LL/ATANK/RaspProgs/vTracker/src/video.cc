#include "video.h"

Video::Video(std::string fname) {
    openVideoFile(fname);
}

Video::~Video(void) {
}

int Video::openVideoFile(std::string fileName) {
    _videoFileName = fileName;

    cv::VideoCapture cap(_videoFileName);

    if (!cap.isOpened()) {
        std::cerr << "[ERROR] Can't open the input video file '" \
            << _videoFileName << "'" << std::endl;
        return 0;
    }

    _frameCount = 0;

    VIDEO_DBG_PRINT("Reading video frames...");

    while(1) {
        cv::Mat frame;
        cap >> frame;

        if (frame.empty()) {
            break;
        }

        _videoFrames.emplace_back(Frame(frame));
        _frameCount++;
    }

    VIDEO_DBG_PRINT("Total video frames: %d", _frameCount);

    cap.release();

    return 1;
}

int Video::playVideo(int interval_ms) {
    int delay_time = interval_ms;
    for(int i = 0; i < _videoFrames.size(); i++) {
        char msg[128];
        cv::Mat image = _videoFrames[i].getImageFeature();
        int kptNum = _videoFrames[i].getFeatureNum();

        // display usage on the image
        std::string usage = "'s':stop, 'c':continue, 'n':next frame, 'p':prev frame";
        std::string text1 = "Frame#: "+ std::to_string(i);
        cv::Scalar textColor = cv::Scalar(0,255,0);
        cv::putText(image, usage, cv::Point(10,10), 1, 1.0, textColor);
        cv::putText(image, text1, cv::Point(10,25), 1, 1.0, textColor);

        std::string text2 = "Kpt#: "+ std::to_string(kptNum);
        cv::putText(image, text2, cv::Point(10,40), 1, 1.0, textColor);

        cv::imshow("Frame", image);
        char c = cv::waitKey(delay_time);
        if (c == 27) {  // 'ESC' to escape the loop
            break;
        } else if (c == 's') {  // 'STOP'
            delay_time = 0;
        } else if (c == 'c') {  // 'CONTINUE'
            delay_time = interval_ms;
        } else if (c == 'n') {  // 'NEXT FRAME'
            delay_time = 0;
        } else if (c == 'b') {  // 'PREVIOUS FRAME'
            delay_time = 0;
            i -= 2;
        } else if (delay_time == 0) {
            i--;
        }
    }
    cv::destroyAllWindows();

    return 1;
}
