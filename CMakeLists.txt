cmake_minimum_required(VERSION 2.8.3)
project(navigation)

SET(CMAKE_BUILD_TYPE Debug)

ADD_DEFINITIONS(
    -std=c++11 # Or -std=c++0x
    -Wno-deprecated
)

find_package(
    OpenCV REQUIRED
)
find_package(PCL 1.3 REQUIRED COMPONENTS common kdtree octree search sample_consensus segmentation io)

include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

add_executable(
    ${PROJECT_NAME}
    src/navigation.cpp
    src/sensors_drivers/logparser.cpp
    src/utils/gui.cpp
    src/utils/ransac.cpp
    src/data_types/pole.cpp
    src/data_types/line.cpp
    src/data_manipulation/poleextractor.cpp
    src/data_manipulation/lineextractor.cpp
    src/data_manipulation/ekfstateestimator.cpp
    src/motion_planners/linefollowermp.cpp
    src/motion_planners/endlineturnmp.cpp
)

target_link_libraries(
    ${PROJECT_NAME}
    ${OpenCV_LIBS}
    ${PCL_COMMON_LIBRARIES}
    ${PCL_SEARCH_LIBRARIES}
    ${PCL_SAMPLE_CONSENSUS_LIBRARIES}
    ${PCL_KDTREE_LIBRARIES}
    ${PCL_OCTREE_LIBRARIES}
    ${PCL_SEGMENTATION_LIBRARIES}
    ${PCL_IO_LIBRARIES}
)
