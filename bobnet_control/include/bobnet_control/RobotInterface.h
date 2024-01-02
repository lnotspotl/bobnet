#pragma once

#include <string>
#include <vector>
#include <memory>
#include <ros/ros.h>
#include <bobnet_msgs/RobotState.h>
#include <std_msgs/String.h>

#include <bobnet_control/Controllers.h>
#include <Eigen/Dense>

namespace bobnet_control {

class RobotInterface {
   public:
    RobotInterface(const std::string &stateTopic, const std::string &changeControllerTopic,
                   const std::string &commandTopic, const scalar_t rate,
                   std::unique_ptr<StandController> &&standControllerPtr,
                   std::unique_ptr<RlController> &&rlControllerPtr);
    void setup();
    void run();

   private:
    void stateCallback(const bobnet_msgs::RobotState::ConstPtr stateMsg) { stateMsg_ = stateMsg; }
    void changeControllerCallback(const std_msgs::String &msg) {
        controllerIndex_ = static_cast<size_t>(string2ControllerType(msg.data));
    }

    void publishCommand(scalar_t dt);

    void waitForStateMessage();

    ros::Subscriber stateSubscriber_;
    ros::Subscriber changeControllerSubscriber_;
    ros::Publisher commandPublisher_;

    std::array<std::unique_ptr<Controller>, static_cast<size_t>(ControllerType::NUM_CONTROLLER_TYPES)> controllers_;

    size_t controllerIndex_;

    bobnet_msgs::RobotState::ConstPtr stateMsg_;

    ros::Rate rate_;
};

}  // namespace bobnet_control
