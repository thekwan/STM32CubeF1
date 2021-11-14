#include <iostream>
#include <string>
#include <opencv2/opencv.hpp>

void getGoodMatches(std::vector<cv::DMatch> &matches, std::vector<cv::DMatch> &goods, int avgCnt);

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
    int maxFeatureNum = 5000;
    int goodMatchNum = 500;

    cv::Mat frame, frame_gray, frame_prev;
    cv::Mat desc_curr, desc_prev;

    cv::Ptr<cv::ORB> detector = cv::ORB::create(maxFeatureNum);
    cv::Ptr<cv::DescriptorMatcher> matcher = cv::DescriptorMatcher::create("BruteForce-Hamming");

    std::vector<cv::KeyPoint> vkpt_curr, vkpt_prev;
    std::vector<cv::DMatch> matches;

    while(1) {
        cap >> frame;

        if (frame.empty()) {
            break;
        }

        cv::resize(frame, frame, cv::Size(frame.cols/2, frame.rows/2));
        cv::cvtColor(frame, frame_gray, cv::COLOR_BGR2GRAY);

        // ORB feature extractor
        detector->detectAndCompute(frame_gray, cv::noArray(), vkpt_curr, desc_curr);
        std::cout << "[" << frameCnt << "] ";
        std::cout << "features[" << desc_curr.size() << "]";

        // feature matching
        if (vkpt_prev.size() > 0) {
            matcher->match(desc_curr, desc_prev, matches);    // ORB
            // sort matches by distance
            sort(matches.begin(), matches.end());
        }

        cv::Mat frame_draw = (frame*0.6 + frame_prev*0.4);

        std::string log = "Frame#: "+ std::to_string(frameCnt);
        cv::putText(frame_draw, log, cv::Point(10,10), 1, 0.8, cv::Scalar::all(255));
        log = std::to_string(frame_draw.cols) + "x" + std::to_string(frame_draw.rows);
        cv::putText(frame_draw, log, cv::Point(10,25), 1, 0.8, cv::Scalar::all(255));
        log = "kpts: " + std::to_string(vkpt_curr.size());
        cv::putText(frame_draw, log, cv::Point(10,40), 1, 0.8, cv::Scalar::all(255));

        //cv::drawKeypoints(frame, vkpt_curr, frame_kpt, cv::Scalar::all(-1), cv::DrawMatchesFlags::DEFAULT);
        if (vkpt_prev.size() > 0) {
#if 0
            std::vector<cv::DMatch> good_matches(matches.begin(), matches.begin()+goodMatchNum);
            getGoodMatches(matches, good_matches, goodMatchNum);
#else
            std::vector<cv::DMatch> good_matches;
            getGoodMatches(matches, good_matches, goodMatchNum);
#endif
            float dist_avg = 0;
            int dist_cnt = 0;
            for (auto match : good_matches) {
                if (match.distance > 0) {
                    int qIdx = match.queryIdx;
                    int tIdx = match.trainIdx;

                    cv::circle(frame_draw, vkpt_curr[qIdx].pt, 5, cv::Scalar(255,0,0), 1, 8, 0);
                    cv::circle(frame_draw, vkpt_prev[tIdx].pt, 5, cv::Scalar(0,0,255), 1, 8, 0);
                    cv::line(frame_draw, vkpt_curr[qIdx].pt, vkpt_prev[tIdx].pt, cv::Scalar(0,255,0), 1, 8, 0);

                    dist_avg += match.distance;
                    dist_cnt++;
                }
            }

            if (dist_cnt > 0) {
                dist_avg /= dist_cnt;

                std::cout << " Match.dist(cnt, avg)=(" << dist_cnt << " , " << dist_avg << ")";
            }
        }

        std::cout << "\n";

        cv::imshow("Frame", frame_draw);
        //if (frame_prev.rows > 0)
        //    cv::imshow("Frame(prev)", frame_prev);

        // exit keyboard code
        char c = (char) cv::waitKey(0);
        if (c == 27) {
            break;
        }

        vkpt_prev = vkpt_curr;
        desc_prev = desc_curr;
        frame.copyTo(frame_prev);

        frameCnt++;
    }

    cap.release();
    cv::destroyAllWindows();

    std::cout << "Total frame count = " << frameCnt << std::endl;

    return 0;
}

cv::Point2f getMotionVectors(cv::DMatch match) {

}

void getGoodMatches(std::vector<cv::DMatch> &matches, std::vector<cv::DMatch> &goods, int avgCnt) {
    // get averaged motion vector for first 100 
    //std::cout << "---------------------------------------------\n";
    //std::cout << "first 100 matches\n";
    float distAvg = 0;
    for (int i = 0; i < avgCnt; i++) {
        distAvg += matches[i].distance;
    }
    distAvg /= avgCnt;

    std::cout << "distAvg[" << distAvg << "]";

    for (int i = 0; i < matches.size(); i++) {
        if (matches[i].distance < 1.5*distAvg) {
            goods.push_back(matches[i]);
        }
    }

    std::cout << " goods[" << goods.size() << "]last[" << goods[goods.size()-1].distance << "]";
}
