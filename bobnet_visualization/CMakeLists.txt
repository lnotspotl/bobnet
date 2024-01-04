cmake_minimum_required(VERSION 3.0.2)
project(bobnet_visualization)

## C++17 is a requirement
add_compile_options(-std=c++17)

find_package(catkin REQUIRED COMPONENTS
    roscpp
    tf
    urdf
    kdl_parser
    robot_state_publisher
    geometry_msgs
    bobnet_core
    bobnet_config
)
find_package(Torch REQUIRED)

find_package(catkin REQUIRED COMPONENTS
  ${CATKIN_PACKAGE_DEPENDENCIES}
)

find_package(yaml-cpp REQUIRED)

catkin_package(
    INCLUDE_DIRS include
    LIBRARIES bobnet_visualization 
    CATKIN_DEPENDS roscpp bobnet_config
)

include_directories(
    include
    ${catkin_INCLUDE_DIRS}
    ${TORCH_INCLUDE_DIRS}
)

add_library(bobnet_visualization
    src/RvizVisualizer.cpp
    src/anymal_c/AnymalCVisualizer.cpp
    src/Visualizers.cpp
)
target_link_libraries(bobnet_visualization ${catkin_LIBRARIES}
${TORCH_LIBRARIES}
yaml-cpp
)

install(
  TARGETS bobnet_visualization
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION}
)
install(
  DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)