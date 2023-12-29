#pragma once

#include <ros/ros.h>
#include <vector>

#include <Eigen/Dense>

#include <grid_map_ros/grid_map_ros.hpp>

namespace bobnet_gridmap {

using scalar_t = double;
using vs = std::vector<scalar_t>;

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

    void atPositions(Eigen::MatrixXd &sampled);
    Eigen::MatrixXd samplingPositions_;

   private:
    void callback(const grid_map_msgs::GridMap &msg);

    ros::Subscriber subscriber_;
    grid_map::GridMap map_;
    bool initialized_ = false;

    void generateSamplingPositions();
};

}  // namespace bobnet_gridmap