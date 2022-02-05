cmake_minimum_required(VERSION 2.8)

find_package(OpenCV REQUIRED)

set(CMAKE_CXX_FLAGS "-g -std=c++11")

include_directories(
    "."
    "../src"
    )

add_executable(test_frame.exe "test_frame.cc")
add_executable(test_tracker.exe "test_tracker.cc")
add_executable(test_matcher.exe "test_matcher.cc")
target_link_libraries(test_frame.exe
    gtest_main
    slam 
    "-lpthread"
    ${OpenCV_LIBRARIES})
target_link_libraries(test_tracker.exe
    gtest_main
    slam 
    "-lpthread"
    ${OpenCV_LIBRARIES})
target_link_libraries(test_matcher.exe
    gtest_main
    slam 
    "-lpthread"
    ${OpenCV_LIBRARIES})
