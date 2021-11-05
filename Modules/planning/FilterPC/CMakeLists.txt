cmake_minimum_required( VERSION 2.8 )

PROJECT(FilterPC)

set( CMAKE_BUILD_TYPE "Release" )
set(CMAKE_CXX_FLAGS "-std=c++11")
set(CMAKE_CXX_FLAGS_RELEASE "-O3 -Wall -g")

find_package(Eigen3 REQUIRED)
find_package(PCL 1.7 REQUIRED)
find_package(catkin REQUIRED COMPONENTS
        roscpp
        rospy
        std_msgs
        sensor_msgs
        nav_msgs
        pcl_ros
        visualization_msgs)
catkin_package()

include_directories(
    include 
    SYSTEM
    third_party
    ${catkin_INCLUDE_DIRS} 
    ${Eigen3_INCLUDE_DIRS} 
    ${PCL_INCLUDE_DIRS})

link_directories(${PCL_LIBRARY_DIRS})

add_definitions(${PCL_DEFINITIONS})


add_executable(filter
  src/filter.cpp
)

target_link_libraries(filter
  ${catkin_LIBRARIES}
  ${PCL_LIBRARIES}
)

