cmake_minimum_required(VERSION 2.8)
project(TankCtrl)

find_package(wxWidgets REQUIRED COMPONENTS net core base)

set(CMAKE_C_FLAGS "-g")
set(CMAKE_CXX_FLAGS "-g")
#set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${CMAKE_SOURCE_DIR})

include_directories(
    "imgs"
    "inc"
    )

include (
    ${wxWidgets_USE_FILE}
    )

FILE(GLOB_RECURSE sourcefiles 
    "src/main.cpp"
    "src/rosClient.cpp")

add_executable(${PROJECT_NAME}.exe ${sourcefiles})
target_link_libraries(${PROJECT_NAME}.exe "-lpthread" ${wxWidgets_LIBRARIES})
