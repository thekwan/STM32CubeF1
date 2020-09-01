cmake_minimum_required(VERSION 2.8)
project(LidarTest)

#find_package(wxWidgets REQUIRED COMPONENTS net core base)

set(CMAKE_C_FLAGS "-g -std=c++11")
set(CMAKE_CXX_FLAGS "-g -std=c++11")
#set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${CMAKE_SOURCE_DIR})

include_directories(
    "src"
    )

#include (
#    ${wxWidgets_USE_FILE}
#    )

FILE(GLOB_RECURSE sourcefiles 
    "src/main.cc"
    "src/map.cc"
    "src/uart.cc")

add_executable(${PROJECT_NAME}.exe ${sourcefiles})
#target_link_libraries(${PROJECT_NAME}.exe "-lpthread" ${wxWidgets_LIBRARIES})
target_link_libraries(${PROJECT_NAME}.exe "-lpthread -lglut -lGLU -lGL")
