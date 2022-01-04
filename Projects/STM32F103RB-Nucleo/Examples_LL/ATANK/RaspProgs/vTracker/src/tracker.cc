#include "tracker.h"

Tracker::Tracker(std::string videoUrl, float scale) {
    _videoUrl = videoUrl;
    _scale = scale;
    _cap.open(_videoUrl);
    _frameCount = 0;
    _keyFrameCount = 0;
}

void Tracker::saveImage(std::string fname, cv::Mat &image) {
    cv::imwrite(fname, image);
}

void Tracker::savePoints(std::string fname, std::vector<cv::Point2f> &pts) {
    std::ofstream of(fname);
    
    of << pts.size() << std::endl;
    for (auto pt : pts) {
        of << pt.x << " " << pt.y << std::endl;
    }
    of.close();
}

void Tracker::saveIntegers(std::string fname, std::vector<int> &idx) {
    std::ofstream of(fname);
    
    of << idx.size() << std::endl;
    for (auto id : idx) {
        of << id << std::endl;
    }
    of.close();
}

void Tracker::saveKeyFrameImage(cv::Mat &image, std::vector<cv::Point2f> &kpts,
        std::vector<int> &tidx) {
    std::string cntPostFix = "_" + std::to_string(_keyFrameCount);
    std::string imageFileName = "kfImage" + cntPostFix + ".jpg";
    std::string kptFileName = "kfPoint" + cntPostFix + ".txt";
    std::string tidxFileName = "kfTindex" + cntPostFix + ".txt";

    saveImage(imageFileName, image);
    savePoints(kptFileName, kpts);
    saveIntegers(tidxFileName, tidx);

    _keyFrameCount++;
}

void Tracker::saveKeyFrameImagePrev(void) {
    std::string cntPostFix = "_" + std::to_string(_keyFrameCount);
    std::string imageFileName = "kfImage" + cntPostFix + ".jpg";
    std::string kptFileName = "kfPoint" + cntPostFix + ".txt";
    std::string tidxFileName = "kfTindex" + cntPostFix + ".txt";

    saveImage(imageFileName, _image_prev);
    savePoints(kptFileName, _kpts_prev);
    saveIntegers(tidxFileName, _tidx_prev);

    _keyFrameCount++;
}
void Tracker::holdKeyFrameImage(cv::Mat &image, std::vector<cv::Point2f> &kpts,
        std::vector<int> &tidx) {
   image.copyTo(_image_prev);
   _kpts_prev = kpts;
   _tidx_prev = tidx;
}

Tracker::kfDecision Tracker::selectKeyFrameImage() {
#if 0
    if (_tckNum_prev > (_tckNum*2) || (_tckNum_prev-_tckNum)>40) {
        return kfDecision::PREVIOUS;
    }
    else if (_kptNum > (_tckNum/2)) {
        return kfDecision::CURRENT;
    }
    else if (_kptNum > 200) {
        return kfDecision::CURRENT;
    }
    else {
        return kfDecision::NONE;
    }
#elif 1
    if (_frameCount == 1) {
        return kfDecision::CURRENT;
    }
    if (_keyFrameStopCount > 0) {
        _keyFrameStopCount--;
        return kfDecision::NONE;
    }

    if (_trackState.compare("UNSTAB") == 0) {
        if (_trackState_prev.compare("STABLE") == 0) {
            _keyFrameStopCount = 20;
            return kfDecision::PREVIOUS;
        } else {
            return kfDecision::NONE;
        }
    }

    if (_kptNum > (_tckNum/2)) {
        _keyFrameStopCount = 20;
        return kfDecision::CURRENT;
    }
    else if (_kptNum > 170) {
        _keyFrameStopCount = 20;
        return kfDecision::CURRENT;
    }
    else {
        return kfDecision::NONE;
    }
#endif
}

void Tracker::tracking(int skip_frame, int maxKeyPoints) {
    const int trackerPatchSize = 21;
    const int interval_ms = 10;
    const int Thr = 6;
    const int image_margin = 20;

    _trackState = "STABLE";
    _trackState_prev = "STABLE";
    _keyFrameStopCount = 0;
    int stateCount = 0;
    int unstable_counter = 0;

    int delay_time = 1;
    std::vector<cv::Point2f> kpts_prev, kpts_curr;
    std::vector<int> tidx_prev, tidx_curr;
    std::vector<unsigned char> track_flags;
    std::vector<float> errors;
    cv::Mat image_curr, image_prev;
    const cv::Size patch(trackerPatchSize,trackerPatchSize);
    const cv::TermCriteria criteria = cv::TermCriteria(cv::TermCriteria::COUNT
            + cv::TermCriteria::EPS, 30, 0.01);

    for (int i = 0; i < skip_frame; i++) {
        _cap >> image_curr;
    }

    // get features for 1st image frame.
    _cap >> image_prev;
    Frame frame(image_prev, _scale);
    kpts_prev = frame.getKeyPoints(maxKeyPoints);

    _tckNum_prev = 0;
    _kptNum_prev = kpts_prev.size();

    // initialize key-point index.
    for (int i = 0; i < _kptNum_prev; i++) {
        tidx_prev.push_back(i);
    }

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
                //criteria, cv::OPTFLOW_LK_GET_MIN_EIGENVALS, 0.05);

        // copy image frame
        image_curr.copyTo(image_prev);

        // display trackable features (RED)
        kpts_prev.clear();
        tidx_curr.clear();
        _tckNum = 0;
        for (int k = 0; k < kpts_curr.size(); k++) {
            if (track_flags[k] == 1 && 
               (kpts_curr[k].x > image_margin) &&
               (kpts_curr[k].y > image_margin) &&
               (kpts_curr[k].x < (image_curr.cols-image_margin)) &&
               (kpts_curr[k].y < (image_curr.rows-image_margin))
               ) {
                cv::circle(image_curr, kpts_curr[k], 3, cv::Scalar(0,0,255));
                kpts_prev.emplace_back(kpts_curr[k]);
                tidx_curr.push_back(tidx_prev[k]);
                _tckNum++;
            }
        }

        // check the tracking quality by using track_flags counter.
        int trackInNum = kpts_curr.size();
        int trackOutNum = 0;
        for (int i = 0; i < kpts_curr.size(); i++) {
            if (track_flags[i] == 1) {
                trackOutNum++;
            }
        }
        _trackSuccRate = (float)trackOutNum / trackInNum;

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
        _kptNum = newKpts.size();


        int tckNumDiff = _tckNum - _tckNum_prev;
        int kptNumDiff = _kptNum - _kptNum_prev;

#if 0
        if (tckNumDiff < -10) {
            stateCount = 0;
            state = "UNSTAB";
        }
        if (stateCount < 10) {
            stateCount++;
        }
        else if (stateCount >= 10) {
            state = "STABLE";
        }
#else
        if (unstable_counter == 0 && _trackSuccRate > 0.95) {
            _trackState = "STABLE";
        } else {
            _trackState = "UNSTAB";
            if (unstable_counter == 0) {
                unstable_counter = 30;
            }
        }
#endif


        kfDecision kfDecIdx = selectKeyFrameImage();

        // display new detected features (BLUE)
        int tidx_num = tidx_curr.size();
        for (int k = 0; k < _kptNum; k++) {
            cv::circle(image_curr, newKpts[k], 3, cv::Scalar(255,0,0));
            if (kfDecIdx == kfDecision::CURRENT) {
                kpts_prev.emplace_back(newKpts[k]);
                tidx_curr.push_back(tidx_num++);
            }
        }



        // display usage on the image
        std::string usage = "'s':stop, 'c':continue, 'n':next frame";
        std::string text1 = "Frame#: "+ std::to_string(_frameCount);
        cv::Scalar textColor = cv::Scalar(0,255,0);
        cv::putText(image_curr, usage, cv::Point(10,10), 1, 1.0, textColor);
        cv::putText(image_curr, text1, cv::Point(10,25), 1, 1.0, textColor);

        std::string text2 = "Tck#: "+ std::to_string(_tckNum);
        text2 += "   Kpt#: "+ std::to_string(_kptNum);
        cv::putText(image_curr, text2, cv::Point(10,40), 1, 1.0, textColor);

        //cv::imshow("Frame", image_curr);
        //cv::imshow("Edge", frame.getEdgeImage(1.0));


        // Selection of Key Frame
        if (kfDecIdx == kfDecision::CURRENT) {
            saveKeyFrameImage(image_curr, kpts_prev, tidx_curr);
            VIDEO_DBG_PRINT("[%4d] t:%3d(%4d), n:%3d  [%s] tr[%1.2f] [KEY FRMAE(CURRENT):  %d]", 
                    _frameCount, _tckNum, tckNumDiff, _kptNum, _trackState.c_str(),
                    _trackSuccRate, _keyFrameCount);
        } else if (kfDecIdx == kfDecision::PREVIOUS) {
            saveKeyFrameImagePrev();
            VIDEO_DBG_PRINT("[%4d] t:%3d(%4d), n:%3d  [%s] tr[%1.2f] [KEY FRMAE(PREVIOUS): %d]", 
                    _frameCount, _tckNum, tckNumDiff, _kptNum, _trackState.c_str(), 
                    _trackSuccRate, _keyFrameCount);
        } else {
            if (_trackState.compare("STABLE") == 0) {
                holdKeyFrameImage(image_curr, kpts_prev, tidx_curr);
            }
            VIDEO_DBG_PRINT("[%4d] t:%3d(%4d), n:%3d  [%s] tr[%1.2f]", 
                    _frameCount, _tckNum, tckNumDiff, _kptNum, _trackState.c_str(),
                    _trackSuccRate);
        }

        //saveImage("frame_"+std::to_string(_frameCount)+".jpg", image_curr);

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

        _tckNum_prev = _tckNum;
        _kptNum_prev = _kptNum;
        _trackState_prev = _trackState;
        tidx_prev = tidx_curr;
        if (unstable_counter > 0) {
            unstable_counter--;
        }
    }
    cv::destroyAllWindows();

    return;
}


