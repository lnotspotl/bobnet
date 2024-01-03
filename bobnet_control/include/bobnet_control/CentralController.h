#pragma once

#include <ros/ros.h>
#include <memory>

#include <bobnet_control/Controller.h>
#include <bobnet_control/StaticController.h>
#include <bobnet_control/BobController.h>
#include <bobnet_control/StateSubscriber.h>
#include <bobnet_control/JointPID.h>

#include <std_msgs/String.h>

// Visualization
#include <robot_state_publisher/robot_state_publisher.h>
#include <tf/transform_broadcaster.h>
#include <bobnet_visualization/Visualizers.h>

namespace bobnet_control {

class CentralController {
   public:
    CentralController();

    void step(const scalar_t dt);

    void visualize();

   private:
   
    ros::Subscriber changeControllerSubscriber_;
    void changeControllerCallback(const std_msgs::String::ConstPtr &msg);

    std::unique_ptr<Controller> staticControllerPtr_;
    std::unique_ptr<Controller> bobControllerPtr_;
    Controller *currentControllerPtr_;


    /* Visualization */
    bool visualizeCommon_;   // transform to odom, joint angles
    bool visualizeControllerSpecific_;  // Controller specific

    std::string odomFrame_;
    std::unique_ptr<robot_state_publisher::RobotStatePublisher> robotStatePublisherPtr_;
    tf::TransformBroadcaster tfBroadcaster_;

    std::shared_ptr<StateSubscriber> stateSubscriberPtr_;
    bobnet_visualization::StateVisualizer stateVisualizer_;
};

} // namespace bobnet_control