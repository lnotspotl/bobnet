#include <bobnet_gridmap/GridmapInterface.h>

#include <numeric>

namespace bobnet_gridmap {

/**********************************************************************************************************************/
/**********************************************************************************************************************/
/**********************************************************************************************************************/
GridmapInterface::GridmapInterface(ros::NodeHandle &nh, const std::string &topic) : initialized_(false) {
    subscriber_ = nh.subscribe(topic, 1, &GridmapInterface::callback, this);
    generateSamplingPositions();
}

/**********************************************************************************************************************/
/**********************************************************************************************************************/
/**********************************************************************************************************************/
void GridmapInterface::atPositions(Eigen::MatrixXd &sampled) {
    for (int i = 0; i < sampled.cols(); ++i) {
        scalar_t x = sampled(0, i);
        scalar_t y = sampled(1, i);
        sampled(2, i) = atPosition(x, y);
    }
}

/**********************************************************************************************************************/
/**********************************************************************************************************************/
/**********************************************************************************************************************/
void GridmapInterface::callback(const grid_map_msgs::GridMap &msg) {
    grid_map::GridMapRosConverter::fromMessage(msg, map_);
    initialized_ = true;
}

/**********************************************************************************************************************/
/**********************************************************************************************************************/
/**********************************************************************************************************************/
void GridmapInterface::waitTillInitialized() {
    while (!initialized_) {
        ROS_INFO("[Bobnet gridmap] Waiting for gridmap to be initialized.");
        ros::Duration(0.5).sleep();
        ros::spinOnce();
    }
}

/**********************************************************************************************************************/
/**********************************************************************************************************************/
/**********************************************************************************************************************/
void GridmapInterface::generateSamplingPositions() {
    std::vector<scalar_t> Ns = {6, 8, 10, 12, 16};
    std::vector<scalar_t> rs = {0.1, 0.3, 0.5, 0.7, 0.9};

    samplingPositions_ = Eigen::MatrixXd::Zero(3, std::accumulate(Ns.begin(), Ns.end(), 0));

    size_t idx = 0;
    for (int i = 0; i < Ns.size(); ++i) {
        scalar_t r = rs[i];
        scalar_t N = Ns[i];
        for (int j = 0; j < N; ++j) {
            scalar_t theta = 2 * M_PI * j / N;
            scalar_t x = r * std::cos(theta);
            scalar_t y = r * std::sin(theta);
            samplingPositions_(0, idx) = x;
            samplingPositions_(1, idx) = y;
            ++idx;
        }
    }
}

}  // namespace bobnet_gridmap