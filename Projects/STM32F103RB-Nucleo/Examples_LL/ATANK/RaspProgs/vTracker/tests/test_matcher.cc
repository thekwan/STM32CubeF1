#include <iostream>
#include <string>
#include <opencv2/opencv.hpp>

#include "log.h"
#include "tracker.h"
#include "matcher.h"

void getGlobalMotionVector(
        std::vector<cv::DMatch> &matches, 
        std::vector<cv::KeyPoint> &vkpt_prev, 
        std::vector<cv::KeyPoint> &vkpt_curr, cv::Point2f &gmv);
void getGoodMatches(
        std::vector<cv::DMatch> &matches, 
        std::vector<cv::KeyPoint> &vkpt_prev, std::vector<cv::KeyPoint> &vkpt_curr,
        cv::Point2f &gmv, std::vector<char> &good_flag);

const cv::String keys = 
    "{help h usage ?    |           | print this message              }"
    "{keyFrameCount     | 0         | key frame image count           }";

int main(int argc, char *argv[]) {
    cv::CommandLineParser parser(argc, argv, keys);

    parser.about("Video feature mapper tester v1.0");

    if (parser.has("help")) {
        parser.printMessage();
        return 0;
    }

    int keyFrameCount = parser.get<int>("keyFrameCount");

    Matcher matcher(keyFrameCount);

    return 0;
}
