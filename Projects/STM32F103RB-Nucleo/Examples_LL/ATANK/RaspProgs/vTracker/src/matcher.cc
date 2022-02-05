#include "matcher.h"

Matcher::Matcher(int keyFrameCount) {

    for (int i = 0; i < keyFrameCount; i++) {
        std::string cntPostFix = "_" + std::to_string(i);
        std::string imgFileName = "kfImage" + cntPostFix + ".jpg";
        std::string kptFileName = "kfPoint" + cntPostFix + ".txt";
        std::string idxFileName = "kfTindex" + cntPostFix + ".txt";
        addKeyFrame(imgFileName, kptFileName, idxFileName);
    }
    TRACKER_DBG_PRINT("Total KeyFrame count = %d", getKeyFrameCount());

    return;
}

Matcher::~Matcher(void) {
    return;
}

void Matcher::addKeyFrame(std::string imageFname, std::string pointFname, 
        std::string indexFname) {
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

KeyFrame::KeyFrame(cv::Mat image, std::vector<cv::Point2f> &kpoints) {
    _image = image;
    _kpoints.swap(kpoints);
}

KeyFrame::~KeyFrame(void) {
}
