cmake_minimum_required(VERSION 3.0.2)
project(bobnet_gridmap)

## C++17 is a requirement
add_compile_options(-std=c++17)


find_package(catkin REQUIRED COMPONENTS
    roscpp
    grid_map_ros
    grid_map_pcl
    bobnet_config
    bobnet_core
)

find_package(yaml-cpp REQUIRED)
find_package(OpenCV REQUIRED)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES bobnet_gridmap 
 CATKIN_DEPENDS bobnet_config bobnet_core
 DEPENDS OpenCV
)

include_directories(
 include
 ${catkin_INCLUDE_DIRS}
 ${OpenCV_INCLUDE_DIRS}
)

add_library(bobnet_gridmap
  src/GridmapInterface.cpp
)
target_link_libraries(bobnet_gridmap ${catkin_LIBRARIES} ${OpenCV_LIBRARIES} yaml-cpp)

add_executable(load_map
 src/load_map.cpp
 src/StaticLoader.cpp
 )
target_link_libraries(load_map ${catkin_LIBRARIES} ${OpenCV_LIBRARIES} yaml-cpp)