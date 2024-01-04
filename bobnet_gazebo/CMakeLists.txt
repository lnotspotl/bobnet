cmake_minimum_required(VERSION 3.0.2)
project(bobnet_gazebo)

## C++17 is a requirement
set (CMAKE_CXX_STANDARD 17)
add_compile_options(-std=c++17)


find_package(catkin REQUIRED COMPONENTS
  controller_interface
  hardware_interface
  pluginlib
  roscpp
  urdf
  realtime_tools
  bobnet_msgs
  bobnet_config
  bobnet_core
)

find_package(gazebo REQUIRED)
find_package(pinocchio REQUIRED)
find_package(yaml-cpp REQUIRED)

catkin_package(
 CATKIN_DEPENDS bobnet_msgs bobnet_config bobnet_core
 DEPENDS pinocchio
)



include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${GAZEBO_INCLUDE_DIRS}
  ${pinocchio_INCLUDE_DIRS}
)

add_library(JointController src/JointController.cpp)
target_link_libraries(JointController ${catkin_LIBRARIES} yaml-cpp) 

add_library(StateEstimator src/StateEstimator.cpp)
target_link_libraries(StateEstimator ${catkin_LIBRARIES} ${gazebo_LIBRARIES} ${pinocchio_LIBRARIES} yaml-cpp)