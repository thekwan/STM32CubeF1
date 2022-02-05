#include "matcher.h"

Matcher::Matcher(int keyFrameCount) {

    for (int i = 0; i < keyFrameCount; i++) {
        std::string cntPostFix = "_" + std::to_string(i);
        std::string imageFileName = "kfImage" + cntPostFix + ".jpg";
        std::string kptFileName = "kfPoint" + cntPostFix + ".txt";
        addImage(imageFileName);
        addKeyPoint(kptFileName);
    }
    TRACKER_DBG_PRINT("keyFrame image count = %d", getImageCount());
    TRACKER_DBG_PRINT("keyFrame point count = %d", getKeyPointCount());

    return;
}

Matcher::~Matcher(void) {
    return;
}

void Matcher::addImage(std::string fname) {
    _images.push_back(cv::imread(fname, cv::IMREAD_COLOR));
}

void Matcher::addKeyPoint(std::string fname) {
    int num;
    float x, y;
    std::ifstream fs(fname);

    if (fs.is_open()) {
        fs >> num;
        std::vector<cv::Point2f> pts;
        for (int i = 0; i < num; i++) {
            fs >> x;
            fs >> y;
            pts.emplace_back(x, y);
        }
        _keyPoints.emplace_back(pts);
    }
    else {
        TRACKER_DBG_PRINT("[ERROR] Can't open the file '%s'", fname.c_str());
    }
}

KeyPoint::KeyPoint(std::vector<cv::Point2f> &pts) {
    _points.swap(pts);
}

KeyPoint::~KeyPoint(void) {
}
