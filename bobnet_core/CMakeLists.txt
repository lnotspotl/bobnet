cmake_minimum_required(VERSION 3.0.2)
project(bobnet_core)

## C++17 is a requirement
add_compile_options(-std=c++17)


find_package(catkin REQUIRED COMPONENTS
    bobnet_msgs
)
catkin_package(
 INCLUDE_DIRS include
 LIBRARIES bobnet_core
  CATKIN_DEPENDS bobnet_msgs 
  DEPENDS Eigen
)

## Build
include_directories(
    include
    ${catkin_INCLUDE_DIRS}
)

add_library(bobnet_core
  src/State.cpp
)

## Install
install(TARGETS bobnet_core
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION}
)

install(
  DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)