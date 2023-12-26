#include <string>
#include <vector>
#include <algorithm>

#include "bobnet_gazebo/JointController.h"

namespace bobnet_gazebo {
JointController::~JointController() {
    // unsubscriber from WBC control command topic
    command_subscriber.shutdown();
}

bool JointController::init(hardware_interface::EffortJointInterface *hw, ros::NodeHandle &n) {
    ROS_INFO("[Bobnet gazebo joint controller] Initializing joint controller.");

    // get JointConstSharedPtrs
    const std::string joints_tag = "joints";
    std::vector<std::string> joint_names;
    if (!n.getParam(joints_tag, joint_names)) {
        ROS_ERROR("Could not get joint names! Failed to initialize.");
        return false;
    }

    urdf::Model urdf;
    if (!urdf.initParam("robot_description")) {
        ROS_ERROR("Could not parse urdf file! Failed to initialize.");
        return false;
    }

    for (int i = 0; i < joint_names.size(); ++i) {
        auto joint_name = joint_names[i];
        try {
            joint_map[joint_name] = hw->getHandle(joint_name);
        } catch (const hardware_interface::HardwareInterfaceException &e) {
            ROS_ERROR("Exception thrown %s", e.what());
            return false;
        }

        std::pair<double, double> joint_limit;
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

    // subscribe to WBC control command topic
    const std::string topic = "bobnet_gazebo/joint_controller/command";
    command_subscriber =
        n.subscribe<bobnet_msgs::JointCommandArray>(topic, 1, &JointController::command_callback, this);

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
        double position_desired = command.position_desired;
        double velocity_desired = command.velocity_desired;
        double kp = command.kp;
        double kd = command.kd;
        double torque_ff = command.torque_ff;

        // get current values
        hardware_interface::JointHandle &joint = joint_map[joint_name];
        double position_current = joint.getPosition();

        double dt = period.toSec();
        size_t joint_index = joint_idxs_[joint_name];
        double velocity_current = (position_current - last_joint_angles_[joint_index]) / dt;

        last_joint_angles_[joint_index] = position_current;
        // double velocity_current = joint.getVelocity();

        // compute position and velocity error
        double position_error = position_desired - position_current;
        double velocity_error = velocity_desired - velocity_current;

        // compute torque command
        double torque = torque_ff + kp * position_error + kd * velocity_error;
        torque = std::max(-100.0, std::min(100.0, torque));

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
