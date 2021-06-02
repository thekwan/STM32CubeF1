cmake_minimum_required(VERSION 2.8)
project(spiTester)


set(CMAKE_C_FLAGS "-g")
#set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${CMAKE_SOURCE_DIR})

include_directories(
    "."
    )

FILE(GLOB_RECURSE sourcefiles 
    "main.c")

add_executable(${PROJECT_NAME}.exe ${sourcefiles})
#target_link_libraries(${PROJECT_NAME}.exe "-lpthread")
#add_executable(vpipe_test ${sourcefiles} )
#target_link_libraries(vpipe_test ${OpenCV_LIBS})
