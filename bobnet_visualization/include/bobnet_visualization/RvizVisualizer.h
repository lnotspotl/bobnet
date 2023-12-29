#pragma once

#include <string>
#include <memory>

#include <ros/ros.h>
#include <robot_state_publisher/robot_state_publisher.h>
#include <tf/transform_broadcaster.h>
#include <torch/script.h>

#include <bobnet_core/State.h>

namespace bobnet_visualization {

class RvizVisualizer {
   public:
    RvizVisualizer();

   protected:
    std::string worldFrame_;
    std::unique_ptr<robot_state_publisher::RobotStatePublisher> robotStatePublisherPtr_;
    tf::TransformBroadcaster tfBroadcaster_;
};

}  // namespace bobnet_visualization