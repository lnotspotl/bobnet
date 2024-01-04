cmake_minimum_required(VERSION 3.0.2)
project(bobnet_config)

## C++17 is a requirement
add_compile_options(-std=c++17)

## Find packages
find_package(catkin REQUIRED COMPONENTS 
    roscpp
    bobnet_core
)

find_package(yaml-cpp REQUIRED)

catkin_package(
    INCLUDE_DIRS include
    CATKIN_DEPENDS bobnet_core
    DEPENDS yaml-cpp
)

## Install
install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  FILES_MATCHING PATTERN "*.h"
  PATTERN ".svn" EXCLUDE
)