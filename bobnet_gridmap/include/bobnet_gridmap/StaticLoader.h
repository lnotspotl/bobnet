#pragma once

#include <ros/ros.h>
#include <string>
#include <grid_map_ros/grid_map_ros.hpp>

namespace bobnet_gridmap {

using scalar_t = double;

class StaticLoader {
   public:
    StaticLoader(scalar_t resolution, const std::string &map_frame, const std::string &topic);
    StaticLoader(const std::string &path, scalar_t resolution, const std::string &map_frame, const std::string &topic);
    void load(const std::string &path);
    void publish();
    void publishPeriodically(scalar_t rate);

    inline bool isInitialized() { return initialized_; }

   private:
    bool initialized_ = false;
    grid_map::GridMap map_;

    scalar_t resolution_;
    std::string map_frame_;

    ros::Publisher publisher_;
};

}  // namespace bobnet_gridmap