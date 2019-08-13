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
    message_generation
    robosub
    robosub_msgs
)

find_package(gazebo REQUIRED)

link_directories(${GAZEBO_LIBRARY_DIRS})
include_directories(include ${Boost_INCLUDE_DIR} ${catkin_INCLUDE_DIRS} ${GAZEBO_INCLUDE_DIRS} ${SDFormat_INCLUDE_DIRS})


catkin_package(
  DEPENDS gazebo_ros gazebo_plugins roscpp
)
add_subdirectory(src)
