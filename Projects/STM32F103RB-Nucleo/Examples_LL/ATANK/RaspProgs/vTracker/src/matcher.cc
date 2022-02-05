#include "matcher.h"

Matcher::Matcher(int keyFrameCount) {

    // Read KeyFrame data
    for (int i = 0; i < keyFrameCount; i++) {
        std::string cntPostFix = "_" + std::to_string(i);
        std::string imgFileName = "kfImage" + cntPostFix + ".jpg";
        std::string kptFileName = "kfPoint" + cntPostFix + ".txt";
        addKeyFrame(imgFileName, kptFileName);
    }
    TRACKER_DBG_PRINT("Total KeyFrame count = %d", getKeyFrameCount());

    // Read tracking data
    MatchPoint mp;
    for (int i = 1; i < keyFrameCount; i++) {
        _matches[std::pair<int,int>(i-1,i)] = mp;
        std::string cntPostFix = "_" + std::to_string(i);
        std::string idxFileName = "kfTindex" + cntPostFix + ".txt";
        addMatchIndex(i, idxFileName);
    }

    return;
}

Matcher::~Matcher(void) {
    return;
}

void Matcher::addMatchIndex(int keyFrameIndex, std::string indexFname) {
    // add keypoint data
    int num, index;
    std::ifstream fs(indexFname);
    MatchPoint &mp = _matches[std::pair<int,int>(keyFrameIndex-1, keyFrameIndex)];

    if (fs.is_open()) {
        fs >> num;
        for (int i = 0; i < num; i++) {
            fs >> index;
            if (index >= 0) {
                mp.index_pair.push_back(std::pair<int,int>(index, i));
            }
        }
    }
    else {
        TRACKER_DBG_PRINT("[ERROR] Can't open the file '%s'", indexFname.c_str());
    }

    TRACKER_DBG_PRINT("keyFrame[%d,%d].mp.pair = %lu", keyFrameIndex-1, keyFrameIndex, 
            mp.index_pair.size());
}

void Matcher::addKeyFrame(std::string imageFname, std::string pointFname) {
    cv::Mat image;
    std::vector<cv::Point2f> keypoints;

    // add image data 
    image = cv::imread(imageFname, cv::IMREAD_COLOR);
    cv::cvtColor(image, image, cv::COLOR_BGR2GRAY);

    // add keypoint data
    int num;
    float x, y;
    std::ifstream fs(pointFname);

    if (fs.is_open()) {
        fs >> num;
        for (int i = 0; i < num; i++) {
            fs >> x;
            fs >> y;
            keypoints.emplace_back(x, y);
        }
    }
    else {
        TRACKER_DBG_PRINT("[ERROR] Can't open the file '%s'", pointFname.c_str());
    }

    _keyFrames.emplace_back(image, keypoints);
}

void Matcher::checkKeyFrames(void) {
    for (auto kframe : _keyFrames) {
        kframe.displayImage();
    }
    return;
}

KeyFrame::KeyFrame(cv::Mat image, std::vector<cv::Point2f> &kpoints) {
    _image = image;
    _kpoints.swap(kpoints);
}

KeyFrame::~KeyFrame(void) {
}

void KeyFrame::displayImage(void) {
    cv::Mat tmp;
    _image.copyTo(tmp);

    for (auto point : _kpoints) {
        cv::circle(tmp, point, 3, cv::Scalar(0,0,255));
    }
    cv::imshow("KeyFrameImage", tmp);
    cv::waitKey(0);
}
