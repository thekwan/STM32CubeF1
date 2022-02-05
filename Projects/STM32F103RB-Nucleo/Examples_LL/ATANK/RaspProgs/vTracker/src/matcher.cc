#include "matcher.h"

Matcher::Matcher(void) {

    // Read KeyFrame data
    int i = 0;
    while (1) {
        std::string cntPostFix = "_" + std::to_string(i);
        std::string imgFileName = "kfImage" + cntPostFix + ".jpg";
        std::string kptFileName = "kfPoint" + cntPostFix + ".txt";
        if (addKeyFrame(imgFileName, kptFileName) == 0) {
            break;
        }
        i++;
    }

    // Read tracking data
    MatchPoint mp;
    i = 0;
    while (1) {
        _matches[std::pair<int,int>(i-1,i)] = mp;
        std::string cntPostFix = "_" + std::to_string(i);
        std::string idxFileName = "kfTindex" + cntPostFix + ".txt";
        if (addMatchIndex(i, idxFileName) == 0) {
            break;
        }
        i++;
    }

    TRACKER_DBG_PRINT("Total KeyFrame count = %d", getKeyFrameCount());

    return;
}

Matcher::~Matcher(void) {
    return;
}

int Matcher::addMatchIndex(int keyFrameIndex, std::string indexFname) {
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
        return 0;
    }

    TRACKER_DBG_PRINT("keyFrame[%d,%d].mp.pair = %lu", keyFrameIndex-1, keyFrameIndex, 
            mp.index_pair.size());
    return 1;
}

int Matcher::addKeyFrame(std::string imageFname, std::string pointFname) {
    cv::Mat image;
    std::vector<cv::Point2f> keypoints;

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
        return 0;
    }

    // add image data 
    image = cv::imread(imageFname, cv::IMREAD_COLOR);
    cv::cvtColor(image, image, cv::COLOR_BGR2GRAY);

    _keyFrames.emplace_back(image, keypoints);

    return 1;
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
