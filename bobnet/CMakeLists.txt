cmake_minimum_required(VERSION 3.0.2)
project(bobnet)

## C++17 is a requirement
add_compile_options(-std=c++17)

## Find packages
find_package(catkin REQUIRED COMPONENTS 
    roscpp
    bobnet_control
    bobnet_gridmap
    robot_state_publisher
)

find_package(Torch REQUIRED)
find_package(yaml-cpp REQUIRED)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${TORCH_CXX_FLAGS}")

catkin_package(
    CATKIN_DEPENDS bobnet_control roscpp bobnet_config
)


## Build
include_directories(
    ${catkin_INCLUDE_DIRS}
    ${TORCH_INCLUDE_DIRS}
)

add_executable(main src/main.cpp)
target_link_libraries(main ${catkin_LIBRARIES} ${TORCH_LIBRARIES} yaml-cpp)