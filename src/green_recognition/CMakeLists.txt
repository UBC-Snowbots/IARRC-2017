cmake_minimum_required(VERSION 2.8.3)
project(green_recognition)

## Find catkin macros and libraries
find_package(catkin REQUIRED COMPONENTS
        roscpp
        rospy
        image_transport
        cv_bridge
        )

find_package(OpenCV REQUIRED)
# find_package(sb_utils REQUIRED)

add_definitions(-std=c++14)

catkin_package(
        #  INCLUDE_DIRS include
)


###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
# include_directories(include)
include_directories(
        ${catkin_INCLUDE_DIRS}
        ${sb_utils_INCLUDE_DIRS}
        ${OpenCV_INCLUDE_DIRS}
        vector
        ./include
)

## Declare a C++ executable
# add_executable(sample_package_node src/sample_package_node.cpp)
add_executable(green_recognition
        src/green_recognition.cpp
        src/GreenRecognition.cpp
        include/GreenRecognition.h
        )

add_executable(green_filter
        src/green_filter.cpp
        src/GreenFilter.cpp
        include/GreenFilter.h
        )

## Specify libraries to link a library or executable target against
# target_link_libraries(sample_package_node
#   ${catkin_LIBRARIES}
# )
target_link_libraries(green_recognition
        ${catkin_LIBRARIES}
        ${OpenCV_LIBS}
        ${sb_utils_LIBRARIES}
        )

target_link_libraries(green_filter
        ${catkin_LIBRARIES}
        ${OpenCV_LIBS}
        ${sb_utils_LIBRARIES}
        )

#############
## Testing ##
#############

## Add gtest based cpp test target and link libraries
# catkin_add_gtest(${PROJECT_NAME}-test test/test_utils_lib.cpp)
# if(TARGET ${PROJECT_NAME}-test)
#   target_link_libraries(${PROJECT_NAME}-test ${PROJECT_NAME})
# endif()