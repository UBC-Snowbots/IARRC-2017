cmake_minimum_required(VERSION 2.8.3)
project(vision)

add_definitions(-std=c++14)

find_package(catkin REQUIRED COMPONENTS
        roscpp
        rospy
        image_transport
        cv_bridge
        )
find_package(OpenCV REQUIRED)
find_package(sb_utils REQUIRED)


include_directories(
        ${catkin_INCLUDE_DIRS}
        ${sb_utils_INCLUDE_DIRS}
        ${OpenCV_INCLUDE_DIRS}
        ./include
)

catkin_package()

add_executable(HSVFilter
        src/HSVFilter.cpp
        src/hsv_filter.cpp
        src/HSVFilterNode.cpp
        include/HSVFilter.h
        include/HSVFilterNode.h
        )

add_executable(IPMFilter
        src/IPMFilter.cpp
        src/ipm_filter.cpp
        src/IPMFilterNode.cpp
        src/IPM.cpp
        include/IPMFilter.h
        include/IPMFilterNode.h
        include/IPM.h
        )

add_executable(camera
        src/camera_init.cpp)

add_executable(circle_detection
        src/circle_detection.cpp
        src/CircleDetection.cpp
        include/CircleDetection.h
        )

target_link_libraries(HSVFilter
        ${catkin_LIBRARIES}
        ${OpenCV_LIBS}
        ${sb_utils_LIBRARIES}
        )

target_link_libraries(IPMFilter
        ${catkin_LIBRARIES}
        ${OpenCV_LIBS}
        ${sb_utils_LIBRARIES}
        )

target_link_libraries(circle_detection
        ${catkin_LIBRARIES}
        ${OpenCV_LIBS}
        ${sb_utils_LIBRARIES}
        )

target_link_libraries(camera
        ${catkin_LIBRARIES}
        ${OpenCV_LIBS}
        )

if (CATKIN_ENABLE_TESTING)
    # gTest
    catkin_add_gtest(green-recognition-test test/green-recognition-test.cpp src/HSVFilter.cpp src/HSVFilterNode.cpp src/CircleDetection.cpp WORKING_DIRECTORY ${PROJECT_SOURCE_DIR}/test)
    target_link_libraries(green-recognition-test ${catkin_LIBRARIES})
endif()

