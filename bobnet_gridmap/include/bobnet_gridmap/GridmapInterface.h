#pragma once

#include <ros/ros.h>

#include <bobnet_controllers/Types.h>
#include <grid_map_ros/grid_map_ros.hpp>

namespace bobnet_gridmap {

using namespace bobnet_controllers;

class GridmapInterface {
   public:
    GridmapInterface(ros::NodeHandle &nh, const std::string &topic);
    inline bool isInitialized() { return initialized_; }
    void waitTillInitialized();

    inline scalar_t atPosition(scalar_t x, scalar_t y) {
        grid_map::Position position(x, y);
        return map_.atPosition("elevation", position);
    }

    vs atPositions(vs &xs, vs &ys, scalar_t offset_x, scalar_t offset_y);

   private:
    void callback(const grid_map_msgs::GridMap &msg);

    ros::Subscriber subscriber_;
    grid_map::GridMap map_;
    bool initialized_ = false;
}

}  // namespace bobnet_gridmap