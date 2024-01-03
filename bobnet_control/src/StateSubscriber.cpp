#include <bobnet_control/StateSubscriber.h>

#include <ros/ros.h>

namespace bobnet_control {

StateSubscriber::StateSubscriber(ros::NodeHandle &nh, const std::string &topic) : stateReady_(false) {
    subscriber_ = nh.subscribe(topic, 1, &StateSubscriber::callback, this);
}

const State &StateSubscriber::getState() {
    if (!stateReady_) {
        state_ = State::fromMessage(statePtr_);
        stateReady_ = true;
    }
    return state_;
}

void StateSubscriber::waitTillInitialized() {
    while (!statePtr_) {
        ros::spinOnce();
        ros::Duration(0.05).sleep();
    }
}

void StateSubscriber::callback(const bobnet_msgs::RobotState::Ptr &msg) {
    stateReady_ = false;
    statePtr_ = msg;
}

}  // namespace bobnet_control