cmake_minimum_required(VERSION 2.8.3)
project(robot_description)

find_package(catkin REQUIRED COMPONENTS
  urdf
  roscpp
  rospy
  sensor_msgs
  std_msgs
  tf
)
catkin_package(CATKIN_DEPENDS urdf roscpp rospy std_msgs tf)
include_directories(${catkin_INCLUDE_DIRS})
include_directories(include ${catkin_INCLUDE_DIRS})

add_executable(state_publisher src/state_publisher.cpp)
add_executable(serial_publisher src/serial_publisher.cpp)
target_link_libraries(state_publisher ${catkin_LIBRARIES})
target_link_libraries(serial_publisher ${catkin_LIBRARIES} serial)
