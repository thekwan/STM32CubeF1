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
    i = 1;
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
                mp.addMatchPair(std::pair<int,int>(index, i));
            }
        }
    }
    else {
        return 0;
    }

    TRACKER_DBG_PRINT("keyFrame[%d,%d].mp.pair = %d", keyFrameIndex-1, 
            keyFrameIndex, mp.getMatchPairNum());
    return 1;
}

int Matcher::addKeyFrame(std::string imageFname, std::string pointFname) {
    cv::Mat image;
    std::vector<cv::Point2f> keypoints;

    // add keypoint data
    int num;
    float x, y;
    std::ifstream fs(pointFname);

    TRACKER_DBG_PRINT("pointFname = %s", pointFname.c_str());

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

void Matcher::drawMatchKeyFrames(void) {
    std::string text;
    int kf_num = _keyFrames.size();

    for (int kf0 = 0; kf0 < kf_num; kf0++) {
        for (int kf1 = kf0+1; kf1 < kf_num; kf1++) {
            MatchPoint &mp = _matches[std::pair<int,int>(kf0,kf1)];
            const cv::Mat img0 = _keyFrames[kf0].getImage();
            const cv::Mat img1 = _keyFrames[kf1].getImage();
            const std::vector<cv::Point2f> kps0 = _keyFrames[kf0].getKeyPoints();
            const std::vector<cv::Point2f> kps1 = _keyFrames[kf1].getKeyPoints();
            const std::vector<std::pair<int,int>> pair = mp.getPair();

            if (pair.size() < 1) {
                continue;
            }

            cv::Mat img;
            cv::hconcat(img0, img1, img);
            cv::cvtColor(img, img, cv::COLOR_GRAY2BGR);

            for (auto pt : kps0) {
                cv::circle(img, pt, 3, cv::Scalar(255,0,0));
            }
            for (auto pt : kps1) {
                cv::circle(img, pt+cv::Point2f(img0.cols,0), 3, cv::Scalar(255,0,0));
            }
            for (auto idx : pair) {
                std::string ta = std::to_string(idx.first);
                std::string tb = std::to_string(idx.second);
                cv::Point2f a = kps0[idx.first];
                cv::Point2f b = kps1[idx.second] + cv::Point2f(img0.cols,0);
                cv::circle(img, a, 3, cv::Scalar(0,0,255));
                cv::circle(img, b, 3, cv::Scalar(0,0,255));
                cv::line(img, a, b, cv::Scalar(0,0,255),1,8,0);
            
#if 0
                cv::Point ax = cv::Point(kps0[idx.first]);
                cv::Point bx = cv::Point(kps1[idx.second]);
                ta += "["+std::to_string(ax.x)+","+std::to_string(ax.y)+"]";
                tb += "["+std::to_string(bx.x)+","+std::to_string(bx.y)+"]";
                cv::putText(img, ta, cv::Point(a)+cv::Point(0,2), 1, 1.0, cv::Scalar(0,255,255));
                cv::putText(img, tb, cv::Point(b)+cv::Point(0,2), 1, 1.0, cv::Scalar(0,255,255));
#endif
            }

            text = "KF#: ["+ std::to_string(kf0) + "," + std::to_string(kf1) + "]";
            cv::putText(img, text, cv::Point(10,15), 1, 1.0, cv::Scalar(0,255,0));
            text = "kp#: ["+ std::to_string(kps0.size()) + "," 
                           + std::to_string(kps1.size()) + "]";
            cv::putText(img, text, cv::Point(10,30), 1, 1.0, cv::Scalar(255,0,0));
            text = "pair#: ["+ std::to_string(pair.size()) + "]";
            cv::putText(img, text, cv::Point(10,45), 1, 1.0, cv::Scalar(0,0,255));

            cv::imshow("drawKeyFrameMatch", img);
            cv::waitKey(0);
        }
    }
}

KeyFrame::KeyFrame(cv::Mat image, std::vector<cv::Point2f> &kpoints) {
    _image = image;
    _kpoints.swap(kpoints);
}

KeyFrame::~KeyFrame(void) {
}

const cv::Mat& KeyFrame::getImage(void) {
    return _image;
}

const std::vector<cv::Point2f>& KeyFrame::getKeyPoints(void) {
    return _kpoints;
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
