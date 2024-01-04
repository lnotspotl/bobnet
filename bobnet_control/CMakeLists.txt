cmake_minimum_required(VERSION 3.0.2)
project(bobnet_control)

## C++17 is a requirement
add_compile_options(-std=c++17)

## Find packages
find_package(catkin REQUIRED COMPONENTS
roscpp
bobnet_msgs
bobnet_reference
bobnet_gridmap
bobnet_core
bobnet_visualization
bobnet_config
)
find_package(pinocchio REQUIRED)
find_package(Torch REQUIRED)
find_package(yaml-cpp REQUIRED)

catkin_package(
 INCLUDE_DIRS include
 LIBRARIES bobnet_control 
 CATKIN_DEPENDS roscpp bobnet_msgs bobnet_reference bobnet_core bobnet_visualization bobnet_config
)

## Build
include_directories(
    include
    ${catkin_INCLUDE_DIRS}
    ${TORCH_INCLUDE_DIRS}
    ${pinocchio_INCLUDE_DIRS}
)

add_library(bobnet_control
  src/CentralController.cpp
  src/StaticController.cpp
  src/JointPID.cpp
  src/StateSubscriber.cpp
  src/BobController.cpp
  src/InverseKinematics.cpp
  src/CentralPatternGenerator.cpp
)
target_link_libraries(bobnet_control ${catkin_LIBRARIES} ${TORCH_LIBRARIES} ${pinocchio_LIBRARIES} yaml-cpp)

install(TARGETS bobnet_control
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION}
)

install(
  DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)