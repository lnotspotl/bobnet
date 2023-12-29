#include <bobnet_controllers/RobotInterface.h>

namespace bobnet_controllers {

/**********************************************************************************************************************/
/**********************************************************************************************************************/
/**********************************************************************************************************************/
RobotInterface::RobotInterface(const std::string &stateTopic, const std::string &changeControllerTopic,
                               const std::string &commandTopic, const scalar_t rate,
                               std::unique_ptr<StandController> &&standControllerPtr,
                               std::unique_ptr<RlController> &&rlControllerPtr)
    : rate_(rate) {
    ros::NodeHandle nh;

    // Setup subscribers
    stateSubscriber_ = nh.subscribe(stateTopic, 1, &RobotInterface::stateCallback, this);
    changeControllerSubscriber_ =
        nh.subscribe(changeControllerTopic, 1, &RobotInterface::changeControllerCallback, this);

    // setup publishers
    commandPublisher_ = nh.advertise<bobnet_msgs::JointCommandArray>(commandTopic, 1, true);

    // stand controller
    controllers_[static_cast<size_t>(ControllerType::STAND)] = std::move(standControllerPtr);

    // rl controller
    controllers_[static_cast<size_t>(ControllerType::RL)] = std::move(rlControllerPtr);

    // Use stand controller initially
    controllerIndex_ = static_cast<size_t>(ControllerType::STAND);
}

/**********************************************************************************************************************/
/**********************************************************************************************************************/
/**********************************************************************************************************************/
void RobotInterface::setup() { waitForStateMessage(); }

/**********************************************************************************************************************/
/**********************************************************************************************************************/
/**********************************************************************************************************************/
void RobotInterface::run() {
    ros::Time timeLast = ros::Time::now();
    while (ros::ok()) {
        ros::spinOnce();
        ros::Time timeNow = ros::Time::now();
        scalar_t dt = (timeNow - timeLast).toSec();
        publishCommand(dt);
        rate_.sleep();
        timeLast = timeNow;
    }
}

/**********************************************************************************************************************/
/**********************************************************************************************************************/
/**********************************************************************************************************************/
void RobotInterface::waitForStateMessage() {
    const std::string &topic = stateSubscriber_.getTopic();
    while (ros::ok()) {
        ros::spinOnce();
        if (stateMsg_) {
            return;
        }
        ROS_INFO_STREAM("Waiting for state message on topic: " << topic);
        ros::Duration(0.1).sleep();
    }
}

/**********************************************************************************************************************/
/**********************************************************************************************************************/
/**********************************************************************************************************************/
void RobotInterface::publishCommand(scalar_t dt) {
    if (!stateMsg_) {
        ROS_WARN_STREAM("No state message received yet");
        return;
    }
    auto commandMsg = controllers_[controllerIndex_]->getCommandMessage(State::fromMessage(stateMsg_), dt);
    commandPublisher_.publish(commandMsg);
}

}  // namespace bobnet_controllers