#pragma once

#include <ros/ros.h>
#include <bobnet_msgs/JointCommandArray.h>
#include <bobnet_msgs/JointCommand.h>

#include <string>
#include <vector>
#include <bobnet_core/Types.h>

namespace bobnet_control {

using namespace bobnet_core;

class JointPID {
   public:
    JointPID(ros::NodeHandle &nh, const std::string &topic, const std::vector<std::string> &jointNames);
    void sendCommand(const vector_t &joint_angles, scalar_t kp, scalar_t kd);

   private:
    ros::Publisher commandPublisher_;

    std::vector<std::string> jointNames_;
};

}  // namespace bobnet_control