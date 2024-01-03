#include <string>
#include <vector>
#include <algorithm>

#include "bobnet_gazebo/JointController.h"

namespace bobnet_gazebo {
bool JointController::init(hardware_interface::EffortJointInterface *hw, ros::NodeHandle &n) {
    ROS_INFO("[Bobnet gazebo joint controller] Initializing joint controller.");

    urdf::Model urdf;
    if (!urdf.initParam("robot_description")) {
        ROS_ERROR("Could not parse urdf file! Failed to initialize.");
        return false;
    }

    ROS_INFO("[Bobnet gazebo joint controller] Loading joint names from config file.");
    auto joint_names = bobnet_config::fromRosConfigFile<std::vector<std::string>>("joint_names");

    ROS_INFO("[Bobnet gazebo joint controller] Loading joint limits from config file.");

    for (int i = 0; i < joint_names.size(); ++i) {
        auto joint_name = joint_names[i];
        try {
            joint_map[joint_name] = hw->getHandle(joint_name);
        } catch (const hardware_interface::HardwareInterfaceException &e) {
            ROS_ERROR("Exception thrown %s", e.what());
            return false;
        }

        std::pair<scalar_t, scalar_t> joint_limit;
        urdf::JointConstSharedPtr joint_urdf = urdf.getJoint(joint_name);
        if (!joint_urdf) {
            ROS_ERROR("Could not find joint '%s' in urdf", joint_name.c_str());
            return false;
        }
        joint_limit.first = joint_urdf->limits->lower;
        joint_limit.second = joint_urdf->limits->upper;
        joint_limits[joint_name] = joint_limit;
        joint_idxs_[joint_name] = i;
    }

    ROS_INFO("[Bobnet gazebo joint controller] Subscribing to command topic.");

    // subscribe to WBC control command topic
    auto baseName = bobnet_config::fromRosConfigFile<std::string>("base_name");
    ROS_INFO_STREAM("Base name: " << baseName);
    auto topic = bobnet_config::fromRosConfigFile<std::string>("command_topic");


    ROS_INFO_STREAM("Subscribing to " << topic);
    command_subscriber =
        ros::NodeHandle().subscribe<bobnet_msgs::JointCommandArray>(topic, 1, &JointController::command_callback, this);

    // Initialize real-time buffer
    command_buffer.writeFromNonRT(BufferType());

    num_joints_ = joint_names.size();

    return true;
}  // namespace bobnet_gazebo

void JointController::update(const ros::Time &time, const ros::Duration &period) {
    BufferType &command_array = *command_buffer.readFromRT();

    if (last_joint_angles_.size() != num_joints_) {
        last_joint_angles_.resize(num_joints_);
        for (int i = 0; i < num_joints_; ++i) {
            last_joint_angles_[i] = joint_map[joint_map.begin()->first].getPosition();
        }
    }

    const int n_commands = command_array.joint_commands.size();
    for (int i = 0; i < n_commands; ++i) {
        bobnet_msgs::JointCommand &command = command_array.joint_commands[i];

        // get command values
        std::string &joint_name = command.joint_name;
        scalar_t position_desired = command.position_desired;
        scalar_t velocity_desired = command.velocity_desired;
        scalar_t kp = command.kp;
        scalar_t kd = command.kd;
        scalar_t torque_ff = command.torque_ff;

        // get current values
        hardware_interface::JointHandle &joint = joint_map[joint_name];
        scalar_t position_current = joint.getPosition();

        scalar_t dt = period.toSec();
        size_t joint_index = joint_idxs_[joint_name];
        scalar_t velocity_current = (position_current - last_joint_angles_[joint_index]) / dt;
        last_joint_angles_[joint_index] = position_current;
        // scalar_t velocity_current = joint.getVelocity();

        // compute position and velocity error
        scalar_t position_error = position_desired - position_current;
        scalar_t velocity_error = velocity_desired - velocity_current;

        // compute torque command
        scalar_t torque = torque_ff + kp * position_error + kd * velocity_error;

        // saturate torque command
        constexpr scalar_t minTorque = -100.0;
        constexpr scalar_t maxTorque = 100.0;
        torque = std::max(minTorque, std::min(maxTorque, torque));

        // set joint torque
        joint.setCommand(torque);

        // Todo: enforce joint limits
    }
}

void JointController::command_callback(const bobnet_msgs::JointCommandArrayConstPtr &command_ptr) {
    command_buffer.writeFromNonRT(*command_ptr);
}

}  // namespace bobnet_gazebo

PLUGINLIB_EXPORT_CLASS(bobnet_gazebo::JointController, controller_interface::ControllerBase);