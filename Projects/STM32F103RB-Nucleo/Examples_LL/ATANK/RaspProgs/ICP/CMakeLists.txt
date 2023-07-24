cmake_minimum_required(VERSION 3.10)
project(LidarTest)

find_package(Eigen3 3 REQUIRED NO_MODULE)
#find_package(OpenCV REQUIRED)

set(CMAKE_C_FLAGS "-g -std=c++11")
set(CMAKE_CXX_FLAGS "-g -std=c++11")
#set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${CMAKE_SOURCE_DIR})

include_directories(
    "src"
    "test"
    )

#include (
#    ${wxWidgets_USE_FILE}
#    )

FILE(GLOB_RECURSE sourcefiles 
    "main.cc"
    "map.cc"
    "lidar.cc"
    "ui.cc"
    "test.cc")

add_executable(${PROJECT_NAME}.exe ${sourcefiles})
target_link_libraries(${PROJECT_NAME}.exe "-lpthread -lglut -lGLU -lGL")
target_link_libraries(${PROJECT_NAME}.exe Eigen3::Eigen glog gflags)
#target_link_libraries(${PROJECT_NAME}.exe Eigen3::Eigen glog gflags ${OpenCV_LIBRARIES})
