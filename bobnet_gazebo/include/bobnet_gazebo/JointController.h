#pragma once

#include <unordered_map>
#include <utility>

#include <controller_interface/controller.h>
#include <hardware_interface/joint_command_interface.h>
#include <pluginlib/class_list_macros.h>
#include <realtime_tools/realtime_buffer.h>
#include <ros/ros.h>
#include <urdf/model.h>

#include "bobnet_msgs/JointCommand.h"
#include "bobnet_msgs/JointCommandArray.h"

#include <bobnet_config/utils.h>

#include <bobnet_core/Types.h>
namespace bobnet_gazebo {

using namespace bobnet_core;
class JointController : public controller_interface::Controller<hardware_interface::EffortJointInterface> {
   public:
    // initialize joint controller
    bool init(hardware_interface::EffortJointInterface *hw, ros::NodeHandle &n);

    // update joint torques, this function gets called every simulation timestep
    void update(const ros::Time &time, const ros::Duration &period);

    // WBC controller command callback
    void command_callback(const bobnet_msgs::JointCommandArrayConstPtr &command_ptr);

   private:
    // WBC controller command subscriber
    ros::Subscriber command_subscriber;

    // joint name -> JointHandle map
    std::unordered_map<std::string, hardware_interface::JointHandle> joint_map;

    // joint_name -> joint_limits map
    std::unordered_map<std::string, std::pair<scalar_t, scalar_t>> joint_limits;

    // command message realtime buffer
    typedef bobnet_msgs::JointCommandArray BufferType;
    realtime_tools::RealtimeBuffer<BufferType> command_buffer;

    std::vector<scalar_t> last_joint_angles_;
    size_t num_joints_;
    std::unordered_map<std::string, size_t> joint_idxs_;
};

}  // namespace bobnet_gazebo