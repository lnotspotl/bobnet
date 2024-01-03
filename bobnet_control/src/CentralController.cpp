#include "bobnet_control/CentralController.h"

#include <bobnet_config/utils.h>

namespace bobnet_control {

CentralController::CentralController() {
    ros::NodeHandle nh;

    // Setup state subscriber
    auto stateTopic = bobnet_config::fromRosConfigFile<std::string>("state_topic");
    stateSubscriberPtr_ = std::make_shared<StateSubscriber>(nh, stateTopic);
    stateSubscriberPtr_->waitTillInitialized();

    // Setup joint PID controller
    auto commandTopic = bobnet_config::fromRosConfigFile<std::string>("command_topic");
    auto jointNames = bobnet_config::fromRosConfigFile<std::vector<std::string>>("joint_names");
    auto jointPIDPtr = std::make_shared<JointPID>(nh, commandTopic, jointNames);

    // Setup static controller
    staticControllerPtr_.reset(new StaticController(jointPIDPtr, stateSubscriberPtr_));
    bobControllerPtr_.reset(new BobController(jointPIDPtr, stateSubscriberPtr_));

    // change controller subscriber
    auto changeControllerTopic = bobnet_config::fromRosConfigFile<std::string>("change_controller_topic");
    changeControllerSubscriber_ = nh.subscribe(changeControllerTopic, 1, &CentralController::changeControllerCallback, this);

    visualizeCommon_ = true;
    visualizeControllerSpecific_ = true;
    currentControllerPtr_ = staticControllerPtr_.get();


    // Visualization
    odomFrame_ = bobnet_config::fromRosConfigFile<std::string>("odom_frame");
    std::string urdfString;
    if(!nh.getParam("/robot_description", urdfString)) {
        ROS_ERROR("Failed to get param /robot_description");
        return;
    }
    
    ROS_INFO("CentralController initialized");
}

void CentralController::step(const scalar_t dt) {
    currentControllerPtr_->sendCommand(dt);

    if (visualizeCommon_) {
        visualize();
    }

    if (visualizeControllerSpecific_) {
        currentControllerPtr_->visualize();
    }
}

void CentralController::visualize() {
    stateVisualizer_.visualize(stateSubscriberPtr_->getState());
}

void CentralController::changeControllerCallback(const std_msgs::String::ConstPtr &msg) {
    auto &msgContent = msg->data;

    if (staticControllerPtr_->isSupported(msgContent)) {
        currentControllerPtr_ = staticControllerPtr_.get();
    } else if (bobControllerPtr_->isSupported(msgContent)) {
        currentControllerPtr_ = bobControllerPtr_.get();
    } else {
        throw std::runtime_error("Controller type " + msgContent + " is not supported");
    }

    currentControllerPtr_->changeController(msg->data);
}

}  // namespace bobnet_control