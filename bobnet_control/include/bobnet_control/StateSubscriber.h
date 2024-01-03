#pragma once

#include <string>
#include <bobnet_msgs/RobotState.h>
#include <bobnet_core/Types.h>
#include <bobnet_core/State.h>

#include <ros/ros.h>

namespace bobnet_control {

using namespace bobnet_core;

class StateSubscriber {
   public:
    StateSubscriber(ros::NodeHandle &nh, const std::string &topic);
    const State &getState();
    void waitTillInitialized();

   private:
    void callback(const bobnet_msgs::RobotState::Ptr &stateMsgPtr);
    ros::Subscriber subscriber_;

    bobnet_msgs::RobotState::Ptr statePtr_;
    bool stateReady_;
    State state_;
};

}  // namespace bobnet_control