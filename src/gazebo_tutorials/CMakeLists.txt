cmake_minimum_required(VERSION 2.8.3)
project(gazebo_tutorials)

set(CMAKE_CXX_FLAGS "-std=c++11 ${CMAKE_CXX_FLAGS}")


find_package(catkin REQUIRED COMPONENTS 
    roscpp
    rospy
    gazebo_ros
    gazebo_plugins
    gazebo_msgs
    std_msgs
)

find_package(gazebo REQUIRED)

link_directories(${GAZEBO_LIBRARY_DIRS})
include_directories(include ${Boost_INCLUDE_DIR} ${catkin_INCLUDE_DIRS} ${GAZEBO_INCLUDE_DIRS} ${SDFormat_INCLUDE_DIRS})


catkin_package(
  DEPENDS roscpp rospy gazebo_ros gazebo_plugins gazebo_msgs std_msgs 
)
add_subdirectory(src)
