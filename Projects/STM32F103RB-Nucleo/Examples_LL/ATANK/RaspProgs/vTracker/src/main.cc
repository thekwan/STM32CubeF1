#include <iostream>
#include <string>
#include <opencv2/opencv.hpp>

#include "log.h"
#include "video.h"

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
    "{inFile            | input.avi | video file for input            }";

int main(int argc, char *argv[]) {
    cv::CommandLineParser parser(argc, argv, keys);

    parser.about("Video feature tracking tester v1.0");

    if (parser.has("help")) {
        parser.printMessage();
        return 0;
    }

    std::string inFileName = parser.get<std::string>("inFile");

    std::cout << "Parameter checking...." << std::endl;
    std::cout << "inFile = " << inFileName << std::endl;

    Video video(inFileName, 1.0);

    //video.playVideo(10);
    video.tracking(10, 2000);

    return 0;
}
