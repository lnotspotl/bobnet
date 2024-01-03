#pragma once

#include <array>
#include <ros/ros.h>
#include <bobnet_core/Types.h>
#include <bobnet_core/State.h>
#include <vector>
#include <string>

#include <functional>

#include <Eigen/Dense>

#include <robot_state_publisher/robot_state_publisher.h>
#include <tf/transform_broadcaster.h>
#include <torch/script.h>

namespace bobnet_visualization {

using namespace bobnet_core;

class StateVisualizer {
   public:
    StateVisualizer();
    void visualize(const bobnet_core::State &state);

   private:
    void publishOdomTransform(const ros::Time &timeStamp, const State &state);
    void publishJointAngles(const ros::Time &timeStamp, const State &state);

    std::string odomFrame_;
    std::string baseFrame_;
    std::unique_ptr<robot_state_publisher::RobotStatePublisher> robotStatePublisherPtr_;
    tf::TransformBroadcaster tfBroadcaster_;
    std::vector<std::string> jointNames_;
};

class HeightsReconstructedVisualizer {
   public:
    HeightsReconstructedVisualizer();
    void visualize(const State &state, const matrix_t &sampled, const at::Tensor &nnPointsReconstructed);

   private:
    void publishMarkers(const ros::Time &timeStamp, const matrix_t &sampled, const std::array<float, 3> &rgb,
                        const std::string &markerNamePrefix, std::function<scalar_t(size_t)> heightFunction);

    std::string odomFrame_;
    ros::Publisher markerPublisher_;
};

}  // namespace bobnet_visualization