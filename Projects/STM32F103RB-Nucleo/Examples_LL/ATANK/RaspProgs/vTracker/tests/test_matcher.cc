#include <iostream>
#include <string>
#include <opencv2/opencv.hpp>

#include "log.h"
#include "matcher.h"

const cv::String keys = 
    "{help h usage ?    |           | print this message              }";

int main(int argc, char *argv[]) {
    Matcher matcher;
    //matcher.checkKeyFrames();
    matcher.drawMatchKeyFrames();

    return 0;
}
