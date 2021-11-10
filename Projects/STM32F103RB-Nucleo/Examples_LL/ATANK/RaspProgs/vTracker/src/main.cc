#include <iostream>
#include <string>
#include <opencv2/opencv.hpp>

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

    cv::VideoCapture cap(inFileName);

    if (!cap.isOpened()) {
        std::cerr << "[ERROR] Can't open the input video file '" << inFileName << "'" << std::endl;
    }

    int frameCnt = 0;

    cv::Ptr<cv::ORB> detector;
    cv::Ptr<cv::DescriptorMatcher> matcher;
    std::vector<cv::KeyPoint> vkpt;

    detector = cv::ORB::create();
    matcher = cv::DescriptorMatcher::create("BruteForce-Hamming(2)");

    while(1) {
        cv::Mat frame, frame_gray, frame_kpt;
        cap >> frame;

        if (frame.empty()) {
            break;
        }

        cv::cvtColor(frame, frame_gray, cv::COLOR_BGR2GRAY);

        // ORB feature extractor
        detector->detect(frame_gray, vkpt);

        std::string log = "Frame#: "+ std::to_string(frameCnt);
        cv::putText(frame, log, cv::Point(10,10), 1, 0.8, cv::Scalar::all(255));
        log = std::to_string(frame.cols) + "x" + std::to_string(frame.rows);
        cv::putText(frame, log, cv::Point(10,25), 1, 0.8, cv::Scalar::all(255));
        log = "kpts: " + std::to_string(vkpt.size());
        cv::putText(frame, log, cv::Point(10,40), 1, 0.8, cv::Scalar::all(255));

        cv::drawKeypoints(frame, vkpt, frame_kpt, cv::Scalar::all(-1), cv::DrawMatchesFlags::DEFAULT);

        cv::imshow("Frame", frame_kpt);

        // exit keyboard code
        char c = (char) cv::waitKey(25);
        if (c == 27) {
            break;
        }

        frameCnt++;
    }

    cap.release();
    cv::destroyAllWindows();

    std::cout << "Total frame count = " << frameCnt << std::endl;

    return 0;
}
