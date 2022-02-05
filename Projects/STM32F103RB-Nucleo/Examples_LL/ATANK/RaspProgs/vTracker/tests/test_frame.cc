#include <iostream>

#include "gtest/gtest.h"
#include "frame.h"

TEST(FrameCreate, Frame)
{
    cv::Mat test_image;
    test_image = cv::imread("../data/plane.jpg");
    Frame frame(test_image);
    EXPECT_EQ(1, 1);
}
