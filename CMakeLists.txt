cmake_minimum_required(VERSION 2.8.3)
project(navigation)

SET(CMAKE_BUILD_TYPE Debug)

ADD_DEFINITIONS(
    -std=c++11 # Or -std=c++0x
    -Wno-deprecated
)

find_package (
    OpenCV REQUIRED
)

add_executable (
    ${PROJECT_NAME}
    src/navigation.cpp
    src/sensors_drivers/logparser.cpp
    src/utils/gui.cpp
    src/data_types/pole.cpp
)

target_link_libraries (
    ${PROJECT_NAME}
    ${OpenCV_LIBS}
)
