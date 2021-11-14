#include <iostream>
#include <string>
#include <opencv2/opencv.hpp>

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

    cv::VideoCapture cap(inFileName);

    if (!cap.isOpened()) {
        std::cerr << "[ERROR] Can't open the input video file '" \
            << inFileName << "'" << std::endl;
    }

    int frameCnt = 0;
    int maxFeatureNum = 2000;

    cv::Mat frame, frame_gray, frame_prev, frame_cmp;
    cv::Mat desc_curr, desc_prev;

    cv::Ptr<cv::ORB> detector = cv::ORB::create(maxFeatureNum);
    cv::Ptr<cv::DescriptorMatcher> matcher = \
            cv::DescriptorMatcher::create("BruteForce-Hamming");

    std::vector<cv::KeyPoint> vkpt_curr, vkpt_prev;
    std::vector<cv::DMatch> matches;
    cv::Point2f gmv;

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
        std::cout << "features[" << desc_curr.rows << "]";

        // feature matching
        if (vkpt_prev.size() > 0) {
            matcher->match(desc_curr, desc_prev, matches);    // ORB
            // sort matches by distance
            sort(matches.begin(), matches.end());
        }

        cv::Mat frame_draw = (frame*0.7 + frame_prev*0.3);

        std::string text = "Frame#: "+ std::to_string(frameCnt);
        cv::Scalar textColor = cv::Scalar::all(255);
        cv::putText(frame_draw, text, cv::Point(10,10), 1, 0.8, textColor);
        text = std::to_string(frame_draw.cols) + "x" + std::to_string(frame_draw.rows);
        cv::putText(frame_draw, text, cv::Point(10,25), 1, 0.8, textColor);
        text = "kpts: " + std::to_string(vkpt_curr.size());
        cv::putText(frame_draw, text, cv::Point(10,40), 1, 0.8, textColor);

        //cv::drawKeypoints(frame, vkpt_curr, frame_kpt, 
        //          cv::Scalar::all(-1), cv::DrawMatchesFlags::DEFAULT);
        if (vkpt_prev.size() > 0) {
            std::vector<char> good_match_flag;
            getGlobalMotionVector(matches, vkpt_prev, vkpt_curr, gmv);
            getGoodMatches(matches, vkpt_prev, vkpt_curr, gmv, good_match_flag);

            //cv::drawMatches(frame, vkpt_curr, frame_prev, vkpt_prev, matches,
            //        frame_cmp, cv::Scalar(255,0,0), cv::Scalar(0,0,255), good_match_flag);

            float dist_avg = 0;
            int dist_cnt = 0;
            for (int i = 0; i < matches.size(); i++) {
                if (good_match_flag[i] == 1 && matches[i].distance > 0) {
                    int qIdx = matches[i].queryIdx;
                    int tIdx = matches[i].trainIdx;

                    cv::circle(frame_draw, vkpt_prev[tIdx].pt, 5, 
                            cv::Scalar(255,0,0), 1, 8, 0);  // BLUE
                    cv::circle(frame_draw, vkpt_curr[qIdx].pt, 5, 
                            cv::Scalar(0,0,255), 1, 8, 0);  // RED
                    cv::line(frame_draw, vkpt_curr[qIdx].pt, 
                            vkpt_prev[tIdx].pt, cv::Scalar(0,255,0), 1, 8, 0);

                    //std::cout << vkpt_curr[qIdx].pt << "\t" << vkpt_prev[tIdx].pt << "\n";

                    dist_avg += matches[i].distance;
                    dist_cnt++;
                }
            }

            if (dist_cnt > 0) {
                dist_avg /= dist_cnt;

                std::cout << " Match.dist(cnt, avg)=(" << dist_cnt 
                    << " , " << dist_avg << ")";
            }
            //cv::imshow("Frame", frame_cmp);
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

void getGlobalMotionVector(std::vector<cv::DMatch> &matches, 
        std::vector<cv::KeyPoint> &vkpt_prev, 
        std::vector<cv::KeyPoint> &vkpt_curr, cv::Point2f &gmv) {

    // method 1: get global motion vector(GMV) by averaging all MVs.
    cv::Point2f mvs = cv::Point2f(0,0);
    int ptCnt = 0;

    for (auto match : matches) {
        if (match.distance < 30) {
            cv::Point2f cpt = vkpt_curr[match.queryIdx].pt;
            cv::Point2f ppt = vkpt_prev[match.trainIdx].pt;

            //std::cout << " cp[" << cpt.x << "," << cpt.y << "]\t";
            //std::cout << " pp[" << ppt.x << "," << ppt.y << "]\n";

            cv::Point2f mv = cpt - ppt;
            mvs += mv;
            ptCnt++;
        }
    }
    mvs /= ptCnt;

    gmv = -mvs; // motion vector is the inverse of feature movement.

    std::cout << " GMV[" << gmv.x << "," << gmv.y << "]";
}

void getGoodMatches(
        std::vector<cv::DMatch> &matches, 
        std::vector<cv::KeyPoint> &vkpt_prev, 
        std::vector<cv::KeyPoint> &vkpt_curr, cv::Point2f &gmv,
        std::vector<char> &good_flag) 
{
    good_flag.resize(matches.size());

    // check motion vector difference and feature distance.
    int idx = 0;
    for (auto match : matches) {
        cv::Point2f cpt = vkpt_curr[match.queryIdx].pt;
        cv::Point2f ppt = vkpt_prev[match.trainIdx].pt;
        cv::Point2f mv = cpt - ppt;

        // motion vector difference with global mv.
        cv::Point2f diff = mv + gmv;

        float norm_g = norm(gmv);
        float norm_d = norm(diff);
        float norm_dg_scale = norm_d / norm_g;

        //std::cout << " norms[" << norm_dg_scale << "]";
        //std::cout << " normg[" << norm_g << "]\n";
        //std::cout << " diff[" << match.distance << "]\n";

        good_flag[idx] = 0;

        if (norm_g < 1.0) {
            // no movement
            if (match.distance < 20 && norm_dg_scale <= 1.0) {
                good_flag[idx] = 1;
            }
        }
        else {
            if (match.distance <= 25 && norm_dg_scale < 1.0)  {
                good_flag[idx] = 1;
            }
        }

        idx++;
    }
}
