cmake_minimum_required(VERSION 2.8.3)
project(perception_neuron_gesture_recognition)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  roscpp
  rospy
  std_msgs
  std_srvs
  tf
)

###################################
## catkin specific configuration ##
###################################
catkin_package(
	CATKIN_DEPENDS geometry_msgs roscpp rospy std_msgs std_srvs tf
)

###########
## Build ##
###########

include_directories(
  ${catkin_INCLUDE_DIRS}
  include
)

## Declare a C++ executable
add_executable(perception_neuron_gesture_recognition_node src/PerceptionGestures.cpp)

## Specify libraries to link a library or executable target against
target_link_libraries(perception_neuron_gesture_recognition_node
   ${catkin_LIBRARIES}
)