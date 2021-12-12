cmake_minimum_required(VERSION 2.8)

find_package(OpenCV REQUIRED)

set(CMAKE_CXX_FLAGS "-g -std=c++11")

include_directories(
    "."
    "../src"
    )

FILE(GLOB_RECURSE sourcefiles "main.cc")

add_executable(test_main.exe ${sourcefiles})
target_link_libraries(test_main.exe
    gtest_main
    tracker.a
    "-lpthread"
    "-lglut"
    "-lGLU"
    "-lGL"
    ${OpenCV_LIBRARIES})
