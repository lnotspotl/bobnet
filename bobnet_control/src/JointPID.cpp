#include "bobnet_control/JointPID.h"

namespace bobnet_control {

/***********************************************************************************************************************/
/***********************************************************************************************************************/
/***********************************************************************************************************************/
JointPID::JointPID(ros::NodeHandle &nh, const std::string &topic, const std::vector<std::string> &jointNames)
    : jointNames_(jointNames) {
    commandPublisher_ = nh.advertise<bobnet_msgs::JointCommandArray>(topic, 1);
}

/***********************************************************************************************************************/
/***********************************************************************************************************************/
/***********************************************************************************************************************/
void JointPID::sendCommand(const vector_t &joint_angles, scalar_t kp, scalar_t kd) {
    bobnet_msgs::JointCommandArray commandArray;
    commandArray.joint_commands.resize(joint_angles.size());

    for (size_t i = 0; i < joint_angles.size(); ++i) {
        bobnet_msgs::JointCommand &command = commandArray.joint_commands[i];
        command.joint_name = jointNames_[i];
        command.position_desired = joint_angles[i];
        command.velocity_desired = 0;
        command.kp = kp;
        command.kd = kd;
        command.torque_ff = 0;
    }

    commandPublisher_.publish(commandArray);
}

}  // namespace bobnet_control