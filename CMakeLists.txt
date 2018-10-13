cmake_minimum_required(VERSION 2.8.3)
project(lidar_perception)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS}  -std=c++0x")


find_package(catkin REQUIRED COMPONENTS  roscpp roslib std_msgs sensor_msgs pcl_conversions
        pcl_ros cv_bridge )

find_package(Boost REQUIRED COMPONENTS )
find_package(OpenCV REQUIRED)

catkin_package( )

include_directories( ${catkin_INCLUDE_DIRS}  ${OpenCV_INCLUDE_DIRS} )

add_executable(groundDetectNode src/main.cpp 
                        src/ground_detect.h 
                        src/ground_detect.cpp
                        src/cluster.cpp
                        src/cluster.h
                        src/boxer.h
                        src/boxer.cpp
                        src/tracker.h
                        src/tracker.cpp
                        src/hungarian_bigraph_matcher.h
                        src/hungarian_bigraph_matcher.cc
                        src/fusion.h
                        src/fusion.cpp
                        src/tracker_object.h )
target_link_libraries(groundDetectNode ${catkin_LIBRARIES} ${OpenCV_LIBRARIES} )