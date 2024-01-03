#pragma once

#include <ros/ros.h>
#include <vector>


#include <memory>
#include <Eigen/Dense>

#include <grid_map_ros/grid_map_ros.hpp>

#include <bobnet_core/Types.h>

namespace bobnet_gridmap {

using namespace bobnet_core;

class GridmapInterface {
   public:
    GridmapInterface(ros::NodeHandle &nh, const std::string &topic);
    inline bool isInitialized() { return initialized_; }
    void waitTillInitialized();

    inline scalar_t atPosition(scalar_t x, scalar_t y) {
        grid_map::Position position(x, y);
        if (!map_.isInside(position)) {
            return 0.0;
        }
        return map_.atPosition("elevation", position);
    }

    void atPositions(matrix_t &sampled);
    matrix_t samplingPositions_;

   private:
    void callback(const grid_map_msgs::GridMap &msg);

    ros::Subscriber subscriber_;
    grid_map::GridMap map_;
    bool initialized_ = false;

    void generateSamplingPositions();
};

std::unique_ptr<GridmapInterface> getGridmapInterfaceUnique();

std::shared_ptr<GridmapInterface> getGridmapInterfaceShared();

}  // namespace bobnet_gridmap