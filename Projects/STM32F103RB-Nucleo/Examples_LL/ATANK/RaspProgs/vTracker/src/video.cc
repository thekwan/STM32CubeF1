#include "video.h"

Video::Video(std::string fname, float scale) {
    openVideoFile(fname, scale);
}

Video::~Video(void) {
}

int Video::openVideoFile(std::string fileName, float scale) {
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

        _videoFrames.emplace_back(Frame(frame, scale));
        _frameCount++;

        VIDEO_DBG_PRINT_NNLINE("\r%d", _frameCount);
    }

    VIDEO_DBG_PRINT("Total video frames: %d", _frameCount);

    cap.release();

    return 1;
}

int Video::tracking(int skip_frame, int maxKeyPoints) {
    const int trackerPatchSize = 21;
    const int interval_ms = 10;
    const int Thr = 6;
    const int image_margin = 20;

    int delay_time = 0;

    std::vector<cv::Point2f> kpts_prev, kpts_curr;
    std::vector<unsigned char> track_flags;
    std::vector<float> errors;
    const cv::Size patch(trackerPatchSize,trackerPatchSize);
    const cv::TermCriteria criteria = cv::TermCriteria(cv::TermCriteria::COUNT
            + cv::TermCriteria::EPS, 30, 0.01);

    // get features for 1st image frame.
    cv::Mat image_prev = _videoFrames[skip_frame].getImage();
    kpts_prev = _videoFrames[skip_frame].getKeyPoints(maxKeyPoints);

    for(int i = skip_frame+1; i < _videoFrames.size(); i++) {
        char msg[128];
        cv::Mat image_curr = _videoFrames[i].getImage();

        cv::calcOpticalFlowPyrLK(image_prev, image_curr, kpts_prev, kpts_curr,
                track_flags, errors, patch, 0, 
                criteria, cv::OPTFLOW_LK_GET_MIN_EIGENVALS, 0.01);

        // copy image frame
        image_curr.copyTo(image_prev);

        // display trackable features (RED)
        kpts_prev.clear();
        int tckNum = 0;
        for (int k = 0; k < kpts_curr.size(); k++) {
            if (track_flags[k] == 1 && 
               (kpts_curr[k].x > image_margin) &&
               (kpts_curr[k].y > image_margin) &&
               (kpts_curr[k].x < (image_curr.cols-image_margin)) &&
               (kpts_curr[k].y < (image_curr.rows-image_margin))
               ) {
                cv::circle(image_curr, kpts_curr[k], 3, cv::Scalar(0,0,255));
                kpts_prev.emplace_back(kpts_curr[k]);
                tckNum++;
            }
        }
        int kptNumThrKeyFrame = tckNum / 2;


        // Get new keypoint
        std::vector<cv::Point2f> newKpts;
        for (auto kpt : _videoFrames[i].getKeyPoints(maxKeyPoints)) {
            bool redundant = false;
            for (auto tck : kpts_prev) {
                if ((fabs(kpt.x - tck.x) < Thr && fabs(kpt.y - tck.y) < Thr)) {
                    redundant = true;
                    break;
                }
            }
            if (!redundant) {
                newKpts.emplace_back(kpt);
            }
        }
        int kptNum = newKpts.size();


        if (kptNum > kptNumThrKeyFrame) {
            VIDEO_DBG_PRINT("[%4d] %d, %d  [KEY FRMAE]", i, tckNum, kptNum);
        } else {
            VIDEO_DBG_PRINT("[%4d] %d, %d", i, tckNum, kptNum);
        }

        // display new detected features (BLUE)
        for (int k = 0; k < kptNum; k++) {
            cv::circle(image_curr, newKpts[k], 3, cv::Scalar(255,0,0));
            if (kptNum > kptNumThrKeyFrame) {
                kpts_prev.emplace_back(newKpts[k]);
            }
        }

        // display usage on the image
        std::string usage = "'s':stop, 'c':continue, 'n':next frame";
        std::string text1 = "Frame#: "+ std::to_string(i);
        cv::Scalar textColor = cv::Scalar(0,255,0);
        cv::putText(image_curr, usage, cv::Point(10,10), 1, 1.0, textColor);
        cv::putText(image_curr, text1, cv::Point(10,25), 1, 1.0, textColor);

        std::string text2 = "Kpt#: "+ std::to_string(kptNum);
        text2 += "   Tck#: "+ std::to_string(tckNum);
        cv::putText(image_curr, text2, cv::Point(10,40), 1, 1.0, textColor);

        cv::imshow("Frame", image_curr);

        char c = cv::waitKey(delay_time);
        if (c == 27) {  // 'ESC' to escape the loop
            break;
        } else if (c == 's') {  // 'STOP'
            delay_time = 0;
        } else if (c == 'c') {  // 'CONTINUE'
            delay_time = interval_ms;
        } else if (c == 'n') {  // 'NEXT FRAME'
            delay_time = 0;
        }
    }
    cv::destroyAllWindows();

    return 1;
}

int Video::playVideo(int interval_ms) {
    int delay_time = interval_ms;
    for(int i = 0; i < _videoFrames.size(); i++) {
        char msg[128];
        cv::Mat image = _videoFrames[i].getImageWithFeature();
        int kptNum = _videoFrames[i].getFeatureNum();

        // display usage on the image
        std::string usage = "'s':stop, 'c':continue, 'n':next frame, 'b':prev frame";
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
            if (i < -1) {
                i = -1;
            }
        } else if (delay_time == 0) {
            i--;
        }
    }
    cv::destroyAllWindows();

    return 1;
}
