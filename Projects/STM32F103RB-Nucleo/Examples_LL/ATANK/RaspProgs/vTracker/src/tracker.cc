#include "tracker.h"

Tracker::Tracker(std::string videoUrl, float scale) {
    _videoUrl = videoUrl;
    _scale = scale;
    _cap.open(_videoUrl);
    _frameCount = 0;
}

void Tracker::tracking(int skip_frame, int maxKeyPoints) {
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
    cv::Mat image_curr, image_prev;
    _cap >> image_prev;
    Frame frame(image_prev, _scale);
    kpts_prev = frame.getKeyPoints(maxKeyPoints);

    while(1) {
        char msg[128];
        _cap >> image_curr;

        if (image_curr.empty()) {
            break;
        }
        _frameCount++;


        frame = Frame(image_curr);

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
        for (auto kpt : frame.getKeyPoints(maxKeyPoints)) {
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
            VIDEO_DBG_PRINT("[%4d] %d, %d  [KEY FRMAE]", _frameCount, tckNum, kptNum);
        } else {
            VIDEO_DBG_PRINT("[%4d] %d, %d", _frameCount, tckNum, kptNum);
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
        std::string text1 = "Frame#: "+ std::to_string(_frameCount);
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

    return;
}


