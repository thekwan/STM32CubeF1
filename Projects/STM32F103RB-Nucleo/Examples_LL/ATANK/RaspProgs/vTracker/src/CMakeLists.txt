cmake_minimum_required(VERSION 2.8)
project(slam)

find_package(OpenCV REQUIRED)

set(CMAKE_CXX_FLAGS "-g -std=c++11")

include_directories(
    "."
    )

FILE(GLOB_RECURSE sourcefiles 
    "matcher.cc"
    "tracker.cc"
    "video.cc"
    "frame.cc")

add_library(slam ${sourcefiles})
#add_executable(${PROJECT_NAME}.exe "main.cc")
#target_link_libraries(${PROJECT_NAME}.exe
#    "-lpthread"
#    #"-lglut"
#    #"-lGLU"
#    #"-lGL"
#    ${PROJECT_NAME}
#    ${OpenCV_LIBRARIES})
