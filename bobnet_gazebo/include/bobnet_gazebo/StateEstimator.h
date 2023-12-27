#pragma once

// STL
#include <vector>
#include <string>

// ROS
#include <ros/ros.h>
#include <bobnet_msgs/RobotState.h>

// Gazebo plugin
#include <gazebo/gazebo.hh>
#include <gazebo/common/common.hh>
#include <gazebo/physics/physics.hh>

// Pinocchio
#include "pinocchio/algorithm/joint-configuration.hpp"
#include "pinocchio/parsers/urdf.hpp"

// Eigen
#include <Eigen/Dense>

namespace gazebo {

class StateEstimator : public ModelPlugin {
   public:
    void Load(physics::ModelPtr _parent, sdf::ElementPtr sdf);
    void OnUpdate();

   private:
    void fillStateMsg(bobnet_msgs::RobotState &msg, double dt);

    /* Convert a rotation matrix to its angle-axis representation */
    Eigen::Vector3d mat2aa(const Eigen::Matrix3d &R);

    // callback function for Gazebo
    event::ConnectionPtr updateConnection;

    // robot state publisher
    ros::Publisher statePublisher_;

    physics::ModelPtr robot_;

    // robot base link ptr
    physics::LinkPtr baseLinkPtr_;

    // joint pointers
    std::vector<physics::JointPtr> joints_;
    std::vector<double> lastJointAngles_;

    // pinocchio model
    pinocchio::Model model_;
    pinocchio::Data data_;

    std::string lf_name_;
    std::string rf_name_;
    std::string lh_name_;
    std::string rh_name_;

    // update rate
    double updateRate_;

    bool firstUpdate_ = true;

    // last yaw angle
    Eigen::Matrix3d lastBaseOrientationMat_;
    common::Time lastSimTime_;
};

}  // namespace gazebo