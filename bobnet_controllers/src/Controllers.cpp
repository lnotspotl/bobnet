#include <bobnet_controllers/Controllers.h>

namespace bobnet_controllers {

/*******************************************************************************/
/*******************************************************************************/
/*******************************************************************************/
StandController::StandController(std::vector<std::string> jointNames, std::vector<double> jointAngles, scalar_t kp,
                                 scalar_t kd)
    : jointNames_(jointNames), jointAngles_(jointAngles), kp_(kp), kd_(kd) {}

/*******************************************************************************/
/*******************************************************************************/
/*******************************************************************************/
bobnet_msgs::JointCommandArray StandController::getCommandMessage() {
    bobnet_msgs::JointCommandArray commandArray;
    commandArray.joint_commands.resize(jointNames_.size());

    for (size_t i = 0; i < jointNames_.size(); ++i) {
        commandArray.joint_commands[i].joint_name = jointNames_[i];
        commandArray.joint_commands[i].position_desired = jointAngles_[i];
        commandArray.joint_commands[i].velocity_desired = 0.0;
        commandArray.joint_commands[i].kp = kp_;
        commandArray.joint_commands[i].kd = kd_;
        commandArray.joint_commands[i].torque_ff = 0.0;
    }

    return commandArray;
}

}  // namespace bobnet_controllers