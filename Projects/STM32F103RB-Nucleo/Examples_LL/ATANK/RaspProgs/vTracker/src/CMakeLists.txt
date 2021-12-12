cmake_minimum_required(VERSION 2.8)
project(tracker)

find_package(OpenCV REQUIRED)

set(CMAKE_CXX_FLAGS "-g -std=c++11")

include_directories(
    "."
    )

FILE(GLOB_RECURSE sourcefiles 
    "tracker.cc"
    "frame.cc")

add_library(${PROJECT_NAME}.a ${sourcefiles})
add_executable(${PROJECT_NAME}.exe "main.cc")
target_link_libraries(${PROJECT_NAME}.exe
    "-lpthread"
    "-lglut"
    "-lGLU"
    "-lGL"
    ${PROJECT_NAME}.a
    ${OpenCV_LIBRARIES})
